#include "manymove_planner/move_group_planner.hpp"

MoveGroupPlanner::MoveGroupPlanner(
    const rclcpp::Node::SharedPtr &node,
    const std::string &planning_group,
    const std::string &base_frame,
    const std::string &tcp_frame,
    const std::string &traj_controller)
    : node_(node), logger_(node->get_logger()),
      planning_group_(planning_group),
      base_frame_(base_frame),
      tcp_frame_(tcp_frame),
      traj_controller_(traj_controller)
{
    // Initialize MoveGroupInterface with the shared node
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, planning_group_);
    move_group_interface_->setPlanningTime(0.5);

    RCLCPP_INFO(logger_, "MoveGroupPlanner initialized with group: %s", planning_group_.c_str());

    // Create a PlanningSceneMonitor

    // "robot_description" is typically the parameter name for the robot’s URDF
    // If you have a different param or an SRDF param, adjust accordingly
    const std::string robot_description_param = "robot_description";

    // Construct the monitor
    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        node_, robot_description_param);

    if (!psm_->getPlanningScene())
    {
        RCLCPP_ERROR(logger_, "PlanningSceneMonitor did not load a valid robot model");
        // handle error if needed
    }
    else
    {
        // Optionally request the full scene once from the service
        // psm_->requestPlanningSceneState("/get_planning_scene");

        // Start listening to diffs / updates from move_group
        psm_->startSceneMonitor("/monitored_planning_scene");

        // Optional: start listening to the robot state topic
        // psm_->startStateMonitor();

        // If you want world geometry updates (collision objects) via /collision_object, etc.
        psm_->startWorldGeometryMonitor();

        RCLCPP_INFO(logger_, "PlanningSceneMonitor started: listening to /monitored_planning_scene");
    }

    // Initialize FollowJointTrajectory action client
    follow_joint_traj_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(node_, "/" + traj_controller_ + "/follow_joint_trajectory");
    if (!follow_joint_traj_client_->wait_for_action_server(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(logger_, "FollowJointTrajectory action server not available after waiting");
    }
    else
    {
        RCLCPP_INFO(logger_, "FollowJointTrajectory action server available");
    }

    // /joint_states subscriber to map positions and velocities
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::SensorDataQoS(),
        std::bind(&MoveGroupPlanner::jointStateCallback, this, std::placeholders::_1));

    // For safety, initialize the maps as empty
    current_positions_.clear();
    current_velocities_.clear();
}

// functions to let ExecuteTrajectory in action_server.cpp handle the collision check feedback
rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr MoveGroupPlanner::getFollowJointTrajClient() const
{
    return follow_joint_traj_client_;
}

moveit::core::RobotModelConstPtr MoveGroupPlanner::getRobotModel() const
{
    return move_group_interface_->getRobotModel();
}

std::string MoveGroupPlanner::getPlanningGroup() const
{
    return planning_group_;
}

double MoveGroupPlanner::computePathLength(const moveit_msgs::msg::RobotTrajectory &trajectory) const
{
    const auto &joint_trajectory = trajectory.joint_trajectory;
    double total_length = 0.0;

    // Compute joint-space path length
    auto computeJointPathLength = [&](const trajectory_msgs::msg::JointTrajectory &traj)
    {
        double length = 0.0;
        for (size_t i = 1; i < traj.points.size(); ++i)
        {
            double segment_length = 0.0;
            for (size_t j = 0; j < traj.points[i].positions.size(); ++j)
            {
                double diff = traj.points[i].positions[j] - traj.points[i - 1].positions[j];
                segment_length += diff * diff;
            }
            length += std::sqrt(segment_length);
        }
        return length;
    };

    // Compute Cartesian path length
    auto computeCartesianPathLength = [&]()
    {
        if (trajectory.multi_dof_joint_trajectory.joint_names.empty())
            return 0.0;

        double length = 0.0;
        for (size_t i = 1; i < trajectory.multi_dof_joint_trajectory.points.size(); ++i)
        {
            const auto &prev_point = trajectory.multi_dof_joint_trajectory.points[i - 1];
            const auto &curr_point = trajectory.multi_dof_joint_trajectory.points[i];

            if (prev_point.transforms.empty() || curr_point.transforms.empty())
                continue;

            const auto &prev_transform = prev_point.transforms[0];
            const auto &curr_transform = curr_point.transforms[0];

            Eigen::Vector3d prev_pos(prev_transform.translation.x, prev_transform.translation.y, prev_transform.translation.z);
            Eigen::Vector3d curr_pos(curr_transform.translation.x, curr_transform.translation.y, curr_transform.translation.z);

            double dist = (curr_pos - prev_pos).norm();
            length += dist;
        }
        return length;
    };

    double joint_length = computeJointPathLength(joint_trajectory);
    double cart_length = computeCartesianPathLength();

    total_length = (2 * cart_length) + (joint_length / 2.0);
    return total_length;
}

double MoveGroupPlanner::computeMaxCartesianSpeed(const robot_trajectory::RobotTrajectoryPtr &trajectory) const
{
    // Compute max Cartesian speed by analyzing consecutive waypoints
    if (trajectory->getWayPointCount() < 2)
        return 0.0;
    double max_speed = 0.0;
    for (size_t i = 1; i < trajectory->getWayPointCount(); i++)
    {
        Eigen::Isometry3d prev_pose = trajectory->getWayPoint(i - 1).getGlobalLinkTransform(tcp_frame_);
        Eigen::Isometry3d curr_pose = trajectory->getWayPoint(i).getGlobalLinkTransform(tcp_frame_);

        double dist = (curr_pose.translation() - prev_pose.translation()).norm();
        double dt = trajectory->getWayPointDurationFromPrevious(i);
        double speed = dist / dt;
        if (speed > max_speed)
            max_speed = speed;
    }
    return max_speed;
}

// Apply Time Parameterization
bool MoveGroupPlanner::applyTimeParameterization(
    robot_trajectory::RobotTrajectoryPtr &trajectory,
    const manymove_planner::msg::MovementConfig &config)
{
    double velocity_scaling_factor = config.velocity_scaling_factor;
    double acceleration_scaling_factor = config.acceleration_scaling_factor;

    const int max_iterations = 32;
    for (int iteration = 0; iteration < max_iterations; iteration++)
    {
        // Reset durations
        for (size_t i = 1; i < trajectory->getWayPointCount(); i++)
            trajectory->setWayPointDurationFromPrevious(i, 0.0);

        bool time_param_success = false;
        if (config.smoothing_type == "time_optimal")
        {
            trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
            time_param_success = time_param.computeTimeStamps(*trajectory, velocity_scaling_factor, acceleration_scaling_factor);
        }
        else if (config.smoothing_type == "iterative" || config.smoothing_type == "iterative_parabolic")
        {
            trajectory_processing::IterativeSplineParameterization time_param;
            time_param_success = time_param.computeTimeStamps(*trajectory, velocity_scaling_factor, acceleration_scaling_factor);
        }
        else if (config.smoothing_type == "ruckig")
        {
            // Note: Ruckig smoothing requires calling applySmoothing on the trajectory directly
            trajectory_processing::IterativeSplineParameterization time_param;
            time_param_success = trajectory_processing::RuckigSmoothing::applySmoothing(
                *trajectory, velocity_scaling_factor, acceleration_scaling_factor);
        }
        else
        {
            // Default fallback to time_optimal
            trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
            time_param_success = time_param.computeTimeStamps(*trajectory, velocity_scaling_factor, acceleration_scaling_factor);
        }

        if (!time_param_success)
        {
            RCLCPP_ERROR(logger_, "Failed to compute time stamps using '%s'", config.smoothing_type.c_str());

            // try fallback approach:
            RCLCPP_WARN(logger_, "Fallback to time-optimal smoothing...");
            for (size_t i = 1; i < trajectory->getWayPointCount(); i++)
                trajectory->setWayPointDurationFromPrevious(i, 0.0);

            trajectory_processing::TimeOptimalTrajectoryGeneration fallback_param;
            time_param_success = fallback_param.computeTimeStamps(*trajectory, velocity_scaling_factor, acceleration_scaling_factor);

            if (!time_param_success)
            {
                RCLCPP_ERROR(logger_, "Fallback time-optimal smoothing also failed.");
                return false;
            }
        }

        double max_speed = computeMaxCartesianSpeed(trajectory);
        if (max_speed <= config.max_cartesian_speed)
        {
            return true; // success
        }
        else
        {
            RCLCPP_WARN(logger_, "Adjusting cartesian within limits: reducing scaling factors...");
            double scale = (config.max_cartesian_speed * 0.99) / max_speed;
            velocity_scaling_factor *= scale;
            // Adjust acceleration similarly (heuristic)
            // acceleration_scaling_factor = (acceleration_scaling_factor * scale + acceleration_scaling_factor) / 2.0;

            if (velocity_scaling_factor < 0.01 || acceleration_scaling_factor < 0.01)
            {
                RCLCPP_ERROR(logger_, "Scaling factors too small to limit Cartesian speed.");
                return false;
            }
        }
    }

    RCLCPP_ERROR(logger_, "Failed to limit Cartesian speed after iterations.");
    return false;
}

std::pair<bool, moveit_msgs::msg::RobotTrajectory> MoveGroupPlanner::plan(const manymove_planner::action::MoveManipulator::Goal &goal_msg)
{
    std::vector<std::pair<moveit_msgs::msg::RobotTrajectory, double>> trajectories;

    // Handle start state
    if (!goal_msg.goal.start_joint_values.empty())
    {
        moveit::core::RobotState start_state(*move_group_interface_->getCurrentState());
        const moveit::core::JointModelGroup *joint_model_group = start_state.getJointModelGroup(move_group_interface_->getName());

        start_state.setJointGroupPositions(joint_model_group, goal_msg.goal.start_joint_values);
        move_group_interface_->setStartState(start_state);
    }
    else
    {
        move_group_interface_->setStartStateToCurrentState();
    }

    // Set scaling factors
    move_group_interface_->setMaxVelocityScalingFactor(goal_msg.goal.config.velocity_scaling_factor);
    move_group_interface_->setMaxAccelerationScalingFactor(goal_msg.goal.config.acceleration_scaling_factor);

    // Set movement targets
    if ((goal_msg.goal.movement_type == "pose") || (goal_msg.goal.movement_type == "joint") || (goal_msg.goal.movement_type == "named"))
    {
        if (goal_msg.goal.movement_type == "pose")
        {
            move_group_interface_->setPoseTarget(goal_msg.goal.pose_target, tcp_frame_);
        }
        else if (goal_msg.goal.movement_type == "joint")
        {
            move_group_interface_->setJointValueTarget(goal_msg.goal.joint_values);
        }
        else if (goal_msg.goal.movement_type == "named")
        {
            move_group_interface_->setNamedTarget(goal_msg.goal.named_target);
        }

        // Plan multiple trajectories
        int attempts = 0;
        while (attempts < goal_msg.goal.config.plan_number_limit && static_cast<int>(trajectories.size()) < goal_msg.goal.config.plan_number_target)
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
            {
                double length = computePathLength(plan.trajectory_);
                trajectories.emplace_back(plan.trajectory_, length);
            }
            else
            {
                RCLCPP_WARN(logger_, "%s target planning attempt %d failed.",
                            goal_msg.goal.movement_type.c_str(), attempts + 1);
            }
            attempts++;
        }
    }
    else if (goal_msg.goal.movement_type == "cartesian")
    {
        // Cartesian movement
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(goal_msg.goal.pose_target);

        int attempts = 0;
        while (attempts < goal_msg.goal.config.plan_number_limit && static_cast<int>(trajectories.size()) < goal_msg.goal.config.plan_number_target)
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            RCLCPP_INFO_STREAM(logger_, "Cartesian path planning attempt with step size " << goal_msg.goal.config.step_size << ", jump threshold " << goal_msg.goal.config.jump_threshold);
            double fraction = move_group_interface_->computeCartesianPath(
                waypoints, goal_msg.goal.config.step_size, goal_msg.goal.config.jump_threshold, plan.trajectory_);

            if (fraction >= 1.0)
            {
                double length = computePathLength(plan.trajectory_);
                trajectories.emplace_back(plan.trajectory_, length);
            }
            else
            {
                RCLCPP_WARN(logger_, "Cartesian path planning attempt %d failed (%.2f%% achieved)", attempts + 1, fraction * 100.0);
            }
            attempts++;
        }
    }
    else
    {
        RCLCPP_ERROR(logger_, "Unknown movement_type: %s", goal_msg.goal.movement_type.c_str());
        return {false, moveit_msgs::msg::RobotTrajectory()};
    }

    if (trajectories.empty())
    {
        RCLCPP_ERROR(logger_, "No valid trajectory found for movement_type: %s", goal_msg.goal.movement_type.c_str());
        return {false, moveit_msgs::msg::RobotTrajectory()};
    }

    // Select the shortest trajectory
    auto shortest = std::min_element(trajectories.begin(), trajectories.end(),
                                     [](const auto &a, const auto &b)
                                     { return a.second < b.second; });

    return {true, shortest->first};
}

bool MoveGroupPlanner::executeTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory)
{
    // 1) Basic checks
    if (trajectory.joint_trajectory.points.empty())
    {
        RCLCPP_ERROR(logger_, "Received an empty trajectory. Execution aborted.");
        return false;
    }

    if (!follow_joint_traj_client_)
    {
        RCLCPP_ERROR(logger_, "FollowJointTrajectory action client not initialized!");
        return false;
    }

    // 2) Wait for the action server (optional if you've done it earlier)
    if (!follow_joint_traj_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(logger_, "FollowJointTrajectory action server not available after waiting.");
        return false;
    }

    // 3) Construct the FollowJointTrajectory goal
    control_msgs::action::FollowJointTrajectory::Goal goal_msg;
    goal_msg.trajectory = trajectory.joint_trajectory;

    RCLCPP_INFO(logger_, "Sending FollowJointTrajectory goal (MoveGroupPlanner) ...");

    // 4) Make a shared promise to communicate success/failure
    auto result_promise = std::make_shared<std::promise<bool>>();
    std::future<bool> result_future = result_promise->get_future();

    // 5) Setup send_goal_options with result callback (and optional feedback)
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions options;

    // (a) Optional feedback callback
    options.feedback_callback =
        [this](auto /*unused_goal_handle*/,
               const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
    {
        // You can log partial progress or time
        RCLCPP_DEBUG(logger_, "Partial execution, time_from_start: %.2f",
                     rclcpp::Duration(feedback->actual.time_from_start).seconds());
    };

    // (b) Result callback: set the promise
    options.result_callback =
        [this, result_promise](const auto &wrapped_result)
    {
        bool success = false;
        switch (wrapped_result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(logger_, "FollowJointTrajectory succeeded (MoveGroupPlanner).");
            success = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(logger_, "FollowJointTrajectory was aborted.");
            success = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(logger_, "FollowJointTrajectory was canceled.");
            success = false;
            break;
        default:
            RCLCPP_ERROR(logger_, "Unknown result from FollowJointTrajectory.");
            success = false;
            break;
        }
        // fulfill the promise with success/failure
        result_promise->set_value(success);
    };

    // 6) Send the goal asynchronously
    auto goal_handle_future = follow_joint_traj_client_->async_send_goal(goal_msg, options);

    // 7) Wait for the goal handle
    if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
        RCLCPP_ERROR(logger_, "Failed to get goal handle from FollowJointTrajectory within 5 seconds.");
        return false;
    }
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(logger_, "FollowJointTrajectory goal was rejected by the server.");
        return false;
    }

    // 8) Wait for the result asynchronously with the promise (non-spinning)
    RCLCPP_INFO(logger_, "Waiting for FollowJointTrajectory result (MoveGroupPlanner) ...");

    // We'll allow up to 300s for the motion to complete. Adjust to your needs.
    auto status = result_future.wait_for(std::chrono::seconds(300));
    if (status != std::future_status::ready)
    {
        RCLCPP_ERROR(logger_, "Trajectory execution timed out (MoveGroupPlanner).");
        return false;
    }

    bool exec_success = result_future.get();
    if (!exec_success)
    {
        RCLCPP_ERROR(logger_, "Trajectory execution failed (MoveGroupPlanner).");
        return false;
    }

    RCLCPP_INFO(logger_, "Trajectory execution succeeded (MoveGroupPlanner).");
    return true;
}

bool MoveGroupPlanner::executeTrajectoryWithFeedback(
    const moveit_msgs::msg::RobotTrajectory &trajectory,
    const std::vector<size_t> &sizes,
    const std::shared_ptr<GoalHandleMoveManipulatorSequence> &goal_handle)
{
    if (!follow_joint_traj_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(logger_, "FollowJointTrajectory action server not available");
        return false;
    }

    FollowJointTrajectory::Goal follow_goal;
    follow_goal.trajectory = trajectory.joint_trajectory;

    auto result_promise = std::make_shared<std::promise<bool>>();
    std::future<bool> result_future = result_promise->get_future();
    const auto &points = follow_goal.trajectory.points;

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

    // Helper function to compute Euclidean distance in joint space
    auto computeDistance = [](const std::vector<double> &a, const std::vector<double> &b) -> double
    {
        if (a.size() != b.size())
            return std::numeric_limits<double>::max();

        double distance = 0.0;
        for (size_t i = 0; i < a.size(); ++i)
        {
            double diff = a[i] - b[i];
            distance += diff * diff;
        }
        return std::sqrt(distance);
    };

    // Variable to track the last found waypoint index
    size_t last_found_index = 0;

    // Updated helper function to find the closest waypoint index
    auto findClosestWaypoint = [&](const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &waypoints,
                                   const std::vector<double> &actual_positions) -> size_t
    {
        // Define a search window around the last found index
        // Start 2 points before the last found index, but not less than 0
        int start_search = static_cast<int>(last_found_index) - 2;
        if (start_search < 0)
            start_search = 0;

        // End the search at last_found_index + 3 or the end of the waypoint list
        size_t end_search = std::min(last_found_index + 3, waypoints.size());

        double tolerance = 1e-2; // Increased tolerance from 1e-4 to 1e-2
        size_t best_idx = last_found_index;
        double min_distance = std::numeric_limits<double>::max();

        // Search the defined window for the closest waypoint
        for (size_t i = static_cast<size_t>(start_search); i < end_search; ++i)
        {
            double distance = computeDistance(actual_positions, waypoints[i].positions);

            // Debugging: Log the distance for each waypoint in the search window
            RCLCPP_DEBUG(logger_, "Waypoint %zu distance: %.6f", i, distance);

            if (distance < min_distance)
            {
                min_distance = distance;
                best_idx = i;
            }

            // If within tolerance, prioritize this waypoint
            if (distance <= tolerance)
            {
                best_idx = i;
                min_distance = distance;
                break; // Exit early if within tolerance
            }
        }

        // Update last_found_index to the best found index if it has advanced
        if (best_idx > last_found_index)
        {
            RCLCPP_DEBUG(logger_, "Updating last_found_index from %zu to %zu", last_found_index, best_idx);
            last_found_index = best_idx;
        }
        else if (best_idx == last_found_index && last_found_index < waypoints.size() - 1)
        {
            // If best_idx hasn't advanced and it's not at the end, try to increment it
            last_found_index = std::min(last_found_index + 1, waypoints.size() - 1);
            RCLCPP_DEBUG(logger_, "Incrementing last_found_index to %zu", last_found_index);
        }

        // Debugging: Log the selected closest index and its distance
        RCLCPP_DEBUG(logger_, "Selected closest_idx: %zu with distance: %.6f", best_idx, min_distance);

        return best_idx;
    };

    // Updated feedback callback
    send_goal_options.feedback_callback =
        [this, sizes, goal_handle, points, computeDistance, findClosestWaypoint](auto, const auto &feedback)
    {
        if (!goal_handle)
            return;

        // Extract actual positions from feedback
        std::vector<double> actual_positions = feedback->actual.positions;

        // Debugging: Print actual positions
        std::stringstream pos_stream;
        pos_stream << "Actual Positions: ";
        for (const auto &pos : actual_positions)
            pos_stream << pos << " ";
        RCLCPP_DEBUG(logger_, "%s", pos_stream.str().c_str());

        // Find the closest waypoint index
        size_t closest_idx = findClosestWaypoint(points, actual_positions);

        // Debugging: Print closest waypoint index
        RCLCPP_DEBUG(logger_, "Closest Waypoint Index: %zu", closest_idx);

        // Determine current segment based on sizes
        size_t segment_index = 0;
        size_t cumulative = 0;
        for (size_t s = 0; s < sizes.size(); ++s)
        {
            if (closest_idx < cumulative + sizes[s])
            {
                segment_index = s;
                break;
            }
            cumulative += sizes[s];
        }

        // Calculate progress as the ratio of closest_idx to total waypoints
        double total_waypoints = static_cast<double>(points.size());
        double progress = static_cast<double>(closest_idx) / total_waypoints;

        // Clamp progress to [0.0, 1.0]
        progress = std::min(std::max(progress, 0.0), 1.0);

        // Publish or log progress
        auto feedback_msg = std::make_shared<MoveManipulatorSequence::Feedback>();
        feedback_msg->progress = static_cast<float>(progress);
        goal_handle->publish_feedback(feedback_msg);

        RCLCPP_INFO(logger_, "Sequence execution progress: %.6f (segment %zu of %zu)",
                    progress, segment_index + 1, sizes.size());
    };

    send_goal_options.result_callback =
        [this, result_promise, goal_handle](const auto &result)
    {
        bool success = false;
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(logger_, "FollowJointTrajectory succeeded");
            success = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(logger_, "FollowJointTrajectory aborted");
            success = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(logger_, "FollowJointTrajectory canceled");
            success = false;
            break;
        default:
            RCLCPP_ERROR(logger_, "Unknown result code from FollowJointTrajectory");
            success = false;
            break;
        }
        result_promise->set_value(success);
    };

    auto goal_handle_future = follow_joint_traj_client_->async_send_goal(follow_goal, send_goal_options);

    // Wait for the goal handle to be ready without spinning again
    if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
        RCLCPP_ERROR(logger_, "Failed to send goal to FollowJointTrajectory action server within the timeout");
        return false;
    }

    auto goal_handle_result = goal_handle_future.get();
    if (!goal_handle_result)
    {
        RCLCPP_ERROR(logger_, "FollowJointTrajectory action server rejected the goal");
        return false;
    }

    // Wait for the result via the promise set in the result callback
    if (result_future.wait_for(std::chrono::seconds(120)) == std::future_status::timeout)
    {
        RCLCPP_ERROR(logger_, "FollowJointTrajectory action did not complete within the timeout");
        return false;
    }

    bool exec_success = result_future.get();
    if (exec_success)
    {
        RCLCPP_INFO(logger_, "Trajectory execution succeeded for sequence");
        return true;
    }
    else
    {
        RCLCPP_ERROR(logger_, "Trajectory execution failed for sequence");
        return false;
    }
}

bool MoveGroupPlanner::areSameJointTargets(const std::vector<double> &j1, const std::vector<double> &j2, double tolerance) const
{
    if (j1.size() != j2.size())
    {
        return false;
    }

    for (size_t i = 0; i < j1.size(); i++)
    {
        if (std::abs(j1[i] - j2[i]) > tolerance)
        {
            return false;
        }
    }

    return true;
}

std::pair<std::vector<moveit_msgs::msg::RobotTrajectory>, std::vector<manymove_planner::msg::MovementConfig>> MoveGroupPlanner::planSequence(
    const manymove_planner::action::MoveManipulatorSequence::Goal &sequence_goal)
{
    std::vector<moveit_msgs::msg::RobotTrajectory> planned_trajectories;
    std::vector<manymove_planner::msg::MovementConfig> planned_configs;

    // Initialize previous end joint targets
    std::vector<double> previous_end_joint_targets;

    if (sequence_goal.goals.empty())
    {
        RCLCPP_WARN(logger_, "planSequence called with an empty goals vector.");
        return {planned_trajectories, planned_configs};
    }

    const double pose_tolerance = 1e-3; // Define a suitable tolerance

    for (size_t i = 0; i < sequence_goal.goals.size(); ++i)
    {
        manymove_planner::action::MoveManipulator::Goal modified_goal;

        modified_goal.goal = sequence_goal.goals[i];

        if (planned_trajectories.empty())
        {
            // If there isn't a previous planned trajectory, check if the first goal has start_joint_values
            const auto &first_goal = sequence_goal.goals[0];
            if (!first_goal.start_joint_values.empty())
            {
                previous_end_joint_targets = first_goal.start_joint_values;
                RCLCPP_INFO(logger_, "Initialized previous_end_joint_targets from start_joint_values of the first goal.");
            }
            else
            {
                previous_end_joint_targets = move_group_interface_->getCurrentJointValues();
                RCLCPP_INFO(logger_, "Initialized previous_end_joint_targets from the current robot pose.");
            }
        }
        else
        {
            // There is a previously planned trajectory, we'll use its joint targets as start pose
            previous_end_joint_targets = planned_trajectories.back().joint_trajectory.points.back().positions;
        }

        // Assigning the right joint targets to the start_joint_values to compute the trajectory
        modified_goal.goal.start_joint_values = previous_end_joint_targets;

        // Plan the trajectory for the current move
        auto [success, trajectory] = plan(modified_goal);
        if (!success)
        {
            // if one trajectory planning fails, abort the planning sequence
            RCLCPP_ERROR(logger_, "Planning failed for move %zu. Aborting sequence.", i + 1);
            // to evaluate if return some indicator of which trajectory failed
            return {{}, {}};
        }

        if (areSameJointTargets(trajectory.joint_trajectory.points.back().positions, previous_end_joint_targets, pose_tolerance))
        {
            // if the pose at the end of the latest trajectory is the same as the previous one, there is no movement to do, skip the trajectory
            RCLCPP_ERROR(logger_, "Resulting trajectory too small, skipping");
        }
        else
        {
            // Append the planned trajectory and its config to the lists
            planned_trajectories.push_back(trajectory);
            planned_configs.push_back(modified_goal.goal.config);
        }
    }

    // the number of planned trajectories can vary from the lenght of the input vectors due to duplicated points removal!
    return {planned_trajectories, planned_configs};
}

std::pair<bool, moveit_msgs::msg::RobotTrajectory> MoveGroupPlanner::applyTimeParametrizationSequence(
    const std::vector<moveit_msgs::msg::RobotTrajectory> &trajectories,
    const std::vector<manymove_planner::msg::MovementConfig> &configs,
    std::vector<size_t> &sizes)
{
    if (trajectories.size() != configs.size())
    {
        RCLCPP_ERROR(logger_, "Mismatched trajectories and configs size in applyTimeParametrizationSequence.");
        return {false, moveit_msgs::msg::RobotTrajectory()};
    }

    sizes.clear();
    moveit_msgs::msg::RobotTrajectory concatenated;
    concatenated.joint_trajectory.joint_names = move_group_interface_->getActiveJoints();

    double cumulative_time = 0.0; // Track total time

    for (size_t i = 0; i < trajectories.size(); ++i)
    {
        const auto &traj = trajectories[i];
        const auto &conf = configs[i];

        auto robot_traj_ptr = std::make_shared<robot_trajectory::RobotTrajectory>(
            move_group_interface_->getRobotModel(), move_group_interface_->getName());
        robot_traj_ptr->setRobotTrajectoryMsg(*move_group_interface_->getCurrentState(), traj);

        // Apply time parameterization
        if (!applyTimeParameterization(robot_traj_ptr, conf))
        {
            RCLCPP_ERROR(logger_, "Time parameterization failed for segment %zu", i + 1);
            return {false, moveit_msgs::msg::RobotTrajectory()};
        }

        moveit_msgs::msg::RobotTrajectory segment;
        robot_traj_ptr->getRobotTrajectoryMsg(segment);

        // Remove the first point of the segment if it's identical to the last point of the concatenated trajectory
        if (!concatenated.joint_trajectory.points.empty() && !segment.joint_trajectory.points.empty())
        {
            const auto &prev_last = concatenated.joint_trajectory.points.back().positions;
            const auto &current_first = segment.joint_trajectory.points.front().positions;
            bool identical = true;
            for (size_t idx = 0; idx < prev_last.size(); ++idx)
            {
                if (std::abs(prev_last[idx] - current_first[idx]) > 1e-6)
                {
                    identical = false;
                    break;
                }
            }
            if (identical)
            {
                segment.joint_trajectory.points.erase(segment.joint_trajectory.points.begin());
            }
        }

        // Offset the time_from_start for each point in the segment
        for (auto &point : segment.joint_trajectory.points)
        {
            double original_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
            double new_time = original_time + cumulative_time;
            point.time_from_start.sec = static_cast<int>(new_time);
            point.time_from_start.nanosec = static_cast<int>((new_time - static_cast<int>(new_time)) * 1e9);
        }

        // Update cumulative_time based on the last point of the segment
        if (!segment.joint_trajectory.points.empty())
        {
            const auto &last_point = segment.joint_trajectory.points.back();
            cumulative_time = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;
        }

        // Append the segment to the concatenated trajectory
        size_t added_size = segment.joint_trajectory.points.size();
        concatenated.joint_trajectory.points.insert(
            concatenated.joint_trajectory.points.end(),
            segment.joint_trajectory.points.begin(),
            segment.joint_trajectory.points.end());

        sizes.push_back(added_size);
    }

    RCLCPP_INFO(logger_, "Successfully applied time parametrization sequence over %zu segments.", trajectories.size());
    return {true, concatenated};
}

bool MoveGroupPlanner::sendControlledStop(double deceleration_time)
{
    RCLCPP_INFO(logger_, "Constructing a single-point 'controlled stop' trajectory (%.2fs).",
                deceleration_time);

    // 1) Make sure the FollowJointTrajectory action server is up
    if (!follow_joint_traj_client_->wait_for_action_server(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(logger_, "Cannot send stop trajectory, FollowJointTrajectory server is not available.");
        return false;
    }

    // 2) Retrieve current joint positions
    std::vector<double> positions = move_group_interface_->getCurrentJointValues();
    if (positions.empty())
    {
        RCLCPP_ERROR(logger_, "Failed to retrieve current joint values.");
        return false;
    }

    // (Optional) Current joint velocities are not directly accessible.
    // We'll assume zero velocities for safety.
    std::vector<double> velocities(positions.size(), 0.0);

    // 3) Build a single-point trajectory
    //    We'll place the time_from_start at `deceleration_time` so the controller
    //    sees a short time window in which to decelerate.
    control_msgs::action::FollowJointTrajectory::Goal stop_goal;
    stop_goal.trajectory.joint_names = move_group_interface_->getJointNames();

    trajectory_msgs::msg::JointTrajectoryPoint stop_point;
    stop_point.positions = positions;
    stop_point.velocities = velocities; // Assume 0 if not available
    stop_point.accelerations.resize(positions.size(), 0.0);

    // Use deceleration_time to define how long we give the controller to ramp to zero velocity.
    // A larger deceleration_time will produce a smoother (but slower) stop.
    stop_point.time_from_start = rclcpp::Duration::from_seconds(deceleration_time);

    stop_goal.trajectory.points.push_back(stop_point);

    RCLCPP_INFO(logger_, "Sending single-point stop trajectory [time_from_start=%.2fs].", deceleration_time);

    // 4) Send the goal
    auto send_goal_future = follow_joint_traj_client_->async_send_goal(stop_goal);
    if (send_goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
        RCLCPP_ERROR(logger_, "Timeout while sending stop trajectory goal.");
        return false;
    }

    auto goal_handle = send_goal_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(logger_, "Stop trajectory goal was rejected by the controller.");
        return false;
    }

    // 5) Optionally wait for result to confirm execution
    auto result_future = follow_joint_traj_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
        RCLCPP_ERROR(logger_, "Controlled stop goal did not finish before timeout.");
        return false;
    }

    auto wrapped_result = result_future.get();
    switch (wrapped_result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(logger_, "Single-point controlled stop completed successfully.");
        return true;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(logger_, "Stop goal was aborted by the controller.");
        return false;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(logger_, "Stop goal was canceled by the controller.");
        return false;
    default:
        RCLCPP_ERROR(logger_, "Stop goal ended with unknown result code %d.", (int)wrapped_result.code);
        return false;
    }
}

bool MoveGroupPlanner::sendControlledStopLinear(double deceleration_time)
{
    RCLCPP_INFO(logger_, "[MoveGroupPlanner] Constructing a short 'controlled stop' trajectory (%.2fs).", deceleration_time);

    // 1) Wait for the FollowJointTrajectory action server
    if (!follow_joint_traj_client_->wait_for_action_server(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(logger_, "[MoveGroupPlanner] Cannot send stop trajectory, FollowJointTrajectory server is not available.");
        return false;
    }

    // 2) Get the current state of the robot
    const moveit::core::RobotState &current_state = *move_group_interface_->getCurrentState();
    if (!current_state.getJointModelGroup(planning_group_))
    {
        RCLCPP_ERROR(logger_, "[MoveGroupPlanner] JointModelGroup '%s' not found.", planning_group_.c_str());
        return false;
    }

    const moveit::core::JointModelGroup *jmg = current_state.getJointModelGroup(planning_group_);
    if (!jmg)
    {
        RCLCPP_ERROR(logger_, "[MoveGroupPlanner] JointModelGroup '%s' not found.", planning_group_.c_str());
        return false;
    }

    // Get the joint names in the group
    std::vector<std::string> joint_names = jmg->getVariableNames();
    std::vector<double> q0(joint_names.size(), 0.0);
    std::vector<double> dq0(joint_names.size(), 0.0);

    {
        // Lock to safely read from current_positions_/velocities_
        std::lock_guard<std::mutex> lock(js_mutex_);

        for (size_t i = 0; i < joint_names.size(); ++i)
        {
            const auto &jn = joint_names[i];

            // If not found in maps, default to 0.0
            if (current_positions_.find(jn) != current_positions_.end())
                q0[i] = current_positions_[jn];
            if (current_velocities_.find(jn) != current_velocities_.end())
                dq0[i] = current_velocities_[jn];
        }
    }

    // 3) Estimate final positions after linear deceleration:
    //    qf = q0 + 0.5 * dq0 * T_dec
    //    (0.5 is because the average velocity during uniform decel from dq0 to 0 is dq0/2)
    std::vector<double> qf(q0.size());
    for (size_t i = 0; i < q0.size(); ++i)
    {
        double delta = 0.5 * dq0[i] * deceleration_time;
        qf[i] = q0[i] + delta;
    }

    // 4) Build a single-point trajectory
    control_msgs::action::FollowJointTrajectory::Goal stop_goal;
    stop_goal.trajectory.joint_names = joint_names;

    // We'll only specify the final point, time_from_start = deceleration_time
    trajectory_msgs::msg::JointTrajectoryPoint p_final;
    p_final.positions = qf;
    // velocities -> zero
    p_final.velocities.resize(q0.size(), 0.0);
    p_final.accelerations.resize(q0.size(), 0.0);
    // deceleration_time is the total time we allow for the motion
    p_final.time_from_start = rclcpp::Duration::from_seconds(deceleration_time);

    stop_goal.trajectory.points.push_back(p_final);

    // 5) Send the new "stop" goal
    RCLCPP_INFO(logger_, "[MoveGroupPlanner] Sending single-point 'stop' trajectory with final time %.2fs.", deceleration_time);

    auto send_goal_future = follow_joint_traj_client_->async_send_goal(stop_goal);
    if (send_goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
        RCLCPP_ERROR(logger_, "[MoveGroupPlanner] Timeout while sending stop trajectory goal.");
        return false;
    }

    auto goal_handle = send_goal_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(logger_, "[MoveGroupPlanner] Stop trajectory goal was rejected by the joint trajectory server.");
        return false;
    }

    // 6) Optionally wait for the result
    auto result_future = follow_joint_traj_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
        RCLCPP_ERROR(logger_, "[MoveGroupPlanner] Controlled stop goal did not finish before timeout.");
        return false;
    }

    auto wrapped_result = result_future.get();
    switch (wrapped_result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(logger_, "[MoveGroupPlanner] Single-point stop completed successfully.");
        return true;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(logger_, "[MoveGroupPlanner] Stop goal was aborted by the controller.");
        return false;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(logger_, "[MoveGroupPlanner] Stop goal was canceled by the controller.");
        return false;
    default:
        RCLCPP_ERROR(logger_, "[MoveGroupPlanner] Stop goal ended with unknown result code %d.", (int)wrapped_result.code);
        return false;
    }
}

bool MoveGroupPlanner::isStateValid(const moveit::core::RobotState *state,
                                    const moveit::core::JointModelGroup *group) const
{
    if (!psm_)
    {
        RCLCPP_ERROR(logger_, "PlanningSceneMonitor is null in isStateValid()");
        return true; // fallback: allow
    }

    // Lock the scene for read-only
    planning_scene_monitor::LockedPlanningSceneRO locked_scene(psm_);
    if (!locked_scene)
    {
        RCLCPP_ERROR(logger_, "Failed to lock the PlanningScene. isStateValid returns true.");
        return true; // fallback
    }

    // Create a copy of the state for collision checking
    // Or we can just pass *state directly, see below
    moveit::core::RobotState temp_state(*state);
    temp_state.update(); // Make sure transforms are up to date

    // Prepare the collision request
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.contacts = true;
    collision_request.max_contacts = 10;
    collision_request.group_name = group->getName();

    // Check collision
    locked_scene->checkCollision(collision_request, collision_result, temp_state);

    if (collision_result.collision)
    {
        RCLCPP_WARN(logger_,
                    "[MoveGroupPlanner] Collision detected in custom isStateValid() (group='%s').",
                    group->getName().c_str());
    }

    // Return true if no collision
    return !collision_result.collision;
}

void MoveGroupPlanner::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Lock mutex for thread-safe access
    std::lock_guard<std::mutex> lock(js_mutex_);

    // Update position/velocity for each joint in the message
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        const std::string &joint_name = msg->name[i];

        // Safety checks (avoid out of range)
        double pos = 0.0;
        double vel = 0.0;
        if (i < msg->position.size())
            pos = msg->position[i];
        if (i < msg->velocity.size())
            vel = msg->velocity[i];

        current_positions_[joint_name] = pos;
        current_velocities_[joint_name] = vel;
    }
}