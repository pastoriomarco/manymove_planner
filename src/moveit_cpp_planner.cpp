#include "manymove_planner/moveit_cpp_planner.hpp"

using manymove_planner::msg::MovementConfig;
using namespace std::chrono_literals;
using GoalHandleMoveManipulatorSequence = rclcpp_action::ServerGoalHandle<manymove_planner::action::MoveManipulatorSequence>;

MoveItCppPlanner::MoveItCppPlanner(
    const rclcpp::Node::SharedPtr &node,
    const std::string &planning_group,
    const std::string &base_frame,
    const std::string &tcp_frame,
    const std::string &traj_controller)
    : node_(node), logger_(node->get_logger()), base_frame_(base_frame), tcp_frame_(tcp_frame), traj_controller_(traj_controller)
{
    moveit_cpp_ptr_ = std::make_shared<moveit_cpp::MoveItCpp>(node_);
    moveit_cpp_ptr_->getPlanningSceneMonitor()->providePlanningSceneService();

    planning_components_ = std::make_shared<moveit_cpp::PlanningComponent>(planning_group, moveit_cpp_ptr_);
    RCLCPP_INFO(logger_, "MoveItCppPlanner initialized with group: %s", planning_group.c_str());

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
}

// Compute Path Length
double MoveItCppPlanner::computePathLength(const moveit_msgs::msg::RobotTrajectory &trajectory) const
{
    if (trajectory.joint_trajectory.points.empty())
    {
        RCLCPP_WARN(logger_, "Joint trajectory is empty. Path length is zero.");
        return 0.0;
    }

    // Helper to compute joint-space path length
    auto computeJointPathLength = [&](const moveit_msgs::msg::RobotTrajectory &traj) -> double
    {
        double length = 0.0;
        for (size_t i = 1; i < traj.joint_trajectory.points.size(); ++i)
        {
            const auto &prev_point = traj.joint_trajectory.points[i - 1];
            const auto &curr_point = traj.joint_trajectory.points[i];

            // Ensure joint positions are valid
            if (prev_point.positions.size() != curr_point.positions.size())
            {
                RCLCPP_ERROR(logger_, "Mismatch in joint positions size at trajectory points %zu and %zu.", i - 1, i);
                return 0.0;
            }

            double segment_length = 0.0;
            for (size_t j = 0; j < prev_point.positions.size(); ++j)
            {
                double diff = curr_point.positions[j] - prev_point.positions[j];
                segment_length += diff * diff;
            }
            length += std::sqrt(segment_length);
        }
        return length;
    };

    // Helper to compute Cartesian path length
    auto computeCartesianPathLength = [&](const moveit_msgs::msg::RobotTrajectory &traj) -> double
    {
        if (traj.multi_dof_joint_trajectory.points.empty())
        {
            RCLCPP_WARN(logger_, "Cartesian path cannot be computed as multi-DOF trajectory is empty.");
            return 0.0;
        }

        double length = 0.0;
        for (size_t i = 1; i < traj.multi_dof_joint_trajectory.points.size(); ++i)
        {
            const auto &prev_transform = traj.multi_dof_joint_trajectory.points[i - 1].transforms.front();
            const auto &curr_transform = traj.multi_dof_joint_trajectory.points[i].transforms.front();

            // Convert transforms to Eigen for Cartesian computation
            Eigen::Vector3d prev_position(prev_transform.translation.x, prev_transform.translation.y, prev_transform.translation.z);
            Eigen::Vector3d curr_position(curr_transform.translation.x, curr_transform.translation.y, curr_transform.translation.z);

            length += (curr_position - prev_position).norm();
        }
        return length;
    };

    // Compute both joint and Cartesian path lengths
    double joint_length = computeJointPathLength(trajectory);
    double cart_length = computeCartesianPathLength(trajectory);

    // Combined length calculation
    double total_length = (2 * cart_length) + (joint_length / 2.0);

    return total_length;
}

// Compute Max Cartesian Speed
double MoveItCppPlanner::computeMaxCartesianSpeed(const robot_trajectory::RobotTrajectoryPtr &trajectory) const
{
    if (trajectory->getWayPointCount() < 2)
        return 0.0;
    double max_speed = 0.0;
    for (size_t i = 1; i < trajectory->getWayPointCount(); i++)
    {
        Eigen::Isometry3d prev_pose = trajectory->getWayPoint(i - 1).getGlobalLinkTransform(tcp_frame_);
        Eigen::Isometry3d curr_pose = trajectory->getWayPoint(i).getGlobalLinkTransform(tcp_frame_);
        double dist = (curr_pose.translation() - prev_pose.translation()).norm();
        double dt = trajectory->getWayPointDurationFromPrevious(i);
        if (dt > 0.0)
        {
            double speed = dist / dt;
            if (speed > max_speed)
                max_speed = speed;
        }
    }
    return max_speed;
}

// Apply Time Parameterization
bool MoveItCppPlanner::applyTimeParameterization(moveit_msgs::msg::RobotTrajectory &trajectory, const manymove_planner::msg::MovementConfig &config)
{
    // Convert moveit_msgs::msg::RobotTrajectory to robot_trajectory::RobotTrajectory
    auto robot_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(
        moveit_cpp_ptr_->getRobotModel(), planning_components_->getPlanningGroupName());
    robot_trajectory->setRobotTrajectoryMsg(*moveit_cpp_ptr_->getCurrentState(), trajectory);

    double velocity_scaling_factor = config.velocity_scaling_factor;
    double acceleration_scaling_factor = config.acceleration_scaling_factor;

    const int max_iterations = 5;
    for (int iteration = 0; iteration < max_iterations; iteration++)
    {
        // Reset durations
        for (size_t i = 1; i < robot_trajectory->getWayPointCount(); i++)
            robot_trajectory->setWayPointDurationFromPrevious(i, 0.0);

        bool time_param_success = false;

        // Apply smoothing based on the specified type
        if (config.smoothing_type == "time_optimal")
        {
            trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
            time_param_success = time_param.computeTimeStamps(*robot_trajectory, velocity_scaling_factor, acceleration_scaling_factor);
        }
        else if (config.smoothing_type == "iterative" || config.smoothing_type == "iterative_parabolic")
        {
            trajectory_processing::IterativeSplineParameterization time_param;
            time_param_success = time_param.computeTimeStamps(*robot_trajectory, velocity_scaling_factor, acceleration_scaling_factor);
        }
        else if (config.smoothing_type == "ruckig")
        {
            time_param_success = trajectory_processing::RuckigSmoothing::applySmoothing(*robot_trajectory, velocity_scaling_factor, acceleration_scaling_factor);
        }
        else
        {
            // Default fallback
            trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
            time_param_success = time_param.computeTimeStamps(*robot_trajectory, velocity_scaling_factor, acceleration_scaling_factor);
        }

        if (!time_param_success)
        {
            RCLCPP_ERROR(logger_, "Failed to compute time stamps using '%s'", config.smoothing_type.c_str());

            // Try fallback approach: time-optimal smoothing
            for (size_t i = 1; i < robot_trajectory->getWayPointCount(); i++)
                robot_trajectory->setWayPointDurationFromPrevious(i, 0.0);

            trajectory_processing::TimeOptimalTrajectoryGeneration fallback_param;
            time_param_success = fallback_param.computeTimeStamps(*robot_trajectory, velocity_scaling_factor, acceleration_scaling_factor);

            if (!time_param_success)
            {
                RCLCPP_ERROR(logger_, "Fallback time-optimal smoothing also failed.");
                return false;
            }
        }

        double max_speed = computeMaxCartesianSpeed(robot_trajectory);
        if (max_speed <= config.max_cartesian_speed)
        {
            // Convert updated robot_trajectory back to moveit_msgs::msg::RobotTrajectory
            trajectory = convertToMsg(*robot_trajectory);
            return true; // Success
        }
        else
        {
            double scale = config.max_cartesian_speed / max_speed;
            velocity_scaling_factor *= scale;
            acceleration_scaling_factor = (acceleration_scaling_factor * scale + acceleration_scaling_factor) / 2.0;

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

bool MoveItCppPlanner::executeTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory)
{
    if (!follow_joint_traj_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(logger_, "FollowJointTrajectory action server not available");
        return false;
    }

    control_msgs::action::FollowJointTrajectory::Goal follow_goal;
    follow_goal.trajectory = trajectory.joint_trajectory;

    auto result_promise = std::make_shared<std::promise<bool>>();
    std::future<bool> result_future = result_promise->get_future();

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();

    // No feedback needed here
    send_goal_options.feedback_callback = [](auto, const auto &) {};

    send_goal_options.result_callback =
        [this, result_promise](const auto &result)
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

    // Wait for the goal handle to be ready
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

    // Wait for the result future
    if (result_future.wait_for(std::chrono::seconds(120)) == std::future_status::timeout)
    {
        RCLCPP_ERROR(logger_, "FollowJointTrajectory action did not complete within the timeout");
        return false;
    }

    bool exec_success = result_future.get();
    if (exec_success)
    {
        RCLCPP_INFO(logger_, "Trajectory execution succeeded");
        return true;
    }
    else
    {
        RCLCPP_ERROR(logger_, "Trajectory execution failed");
        return false;
    }
}

bool MoveItCppPlanner::areSameJointTargets(const std::vector<double> &j1, const std::vector<double> &j2, double tolerance) const
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

std::pair<bool, moveit_msgs::msg::RobotTrajectory> MoveItCppPlanner::plan(const manymove_planner::action::MoveManipulator::Goal &goal_msg)
{
    std::vector<std::pair<moveit_msgs::msg::RobotTrajectory, double>> trajectories;

    // Handle start state
    if (!goal_msg.goal.start_joint_values.empty())
    {
        moveit::core::RobotState start_state(*moveit_cpp_ptr_->getCurrentState());
        const moveit::core::JointModelGroup *joint_model_group = start_state.getJointModelGroup(planning_components_->getPlanningGroupName());
        start_state.setJointGroupPositions(joint_model_group, goal_msg.goal.start_joint_values);
        planning_components_->setStartState(start_state);
    }
    else
    {
        planning_components_->setStartStateToCurrentState();
    }

    // Set movement targets
    if ((goal_msg.goal.movement_type == "pose") || (goal_msg.goal.movement_type == "joint") || (goal_msg.goal.movement_type == "named"))
    {
        if (goal_msg.goal.movement_type == "pose")
        {
            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.pose = goal_msg.goal.pose_target;
            target_pose.header.frame_id = tcp_frame_;
            planning_components_->setGoal(target_pose, tcp_frame_);
        }
        else if (goal_msg.goal.movement_type == "joint")
        {
            moveit::core::RobotState goal_state(*moveit_cpp_ptr_->getCurrentState());
            goal_state.setJointGroupPositions(planning_components_->getPlanningGroupName(), goal_msg.goal.joint_values);
            planning_components_->setGoal(goal_state);
        }
        else if (goal_msg.goal.movement_type == "named")
        {
            planning_components_->setGoal(goal_msg.goal.named_target);
        }

        // Plan multiple trajectories
        int attempts = 0;
        while (attempts < goal_msg.goal.config.plan_number_limit &&
               static_cast<int>(trajectories.size()) < goal_msg.goal.config.plan_number_target)
        {
            auto solution = planning_components_->plan();
            if (solution)
            {
                moveit_msgs::msg::RobotTrajectory traj_msg;
                solution.trajectory->getRobotTrajectoryMsg(traj_msg);
                double length = computePathLength(traj_msg);
                trajectories.emplace_back(traj_msg, length);
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
        while (attempts < goal_msg.goal.config.plan_number_limit &&
               static_cast<int>(trajectories.size()) < goal_msg.goal.config.plan_number_target)
        {
            RCLCPP_INFO(logger_, "Cartesian path planning attempt %d with step size %.3f, jump threshold %.3f",
                        attempts + 1, goal_msg.goal.config.step_size, goal_msg.goal.config.jump_threshold);

            std::vector<moveit::core::RobotStatePtr> trajectory_states;
            auto start_state = planning_components_->getStartState();
            Eigen::Isometry3d target_isometry;
            tf2::fromMsg(goal_msg.goal.pose_target, target_isometry);

            double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
                start_state.get(),
                start_state->getJointModelGroup(planning_components_->getPlanningGroupName()),
                trajectory_states,
                start_state->getJointModelGroup(planning_components_->getPlanningGroupName())->getLinkModel(tcp_frame_),
                target_isometry,
                true,
                moveit::core::MaxEEFStep(goal_msg.goal.config.step_size),
                moveit::core::JumpThreshold(goal_msg.goal.config.jump_threshold),
                moveit::core::GroupStateValidityCallbackFn(),
                kinematics::KinematicsQueryOptions(),
                nullptr);

            if (fraction >= 1.0)
            {
                robot_trajectory::RobotTrajectory robot_trajectory(moveit_cpp_ptr_->getRobotModel(), planning_components_->getPlanningGroupName());
                for (const auto &state : trajectory_states)
                {
                    robot_trajectory.addSuffixWayPoint(*state, 0.1);
                }
                moveit_msgs::msg::RobotTrajectory traj_msg = convertToMsg(robot_trajectory);
                double length = computePathLength(traj_msg);
                trajectories.emplace_back(traj_msg, length);
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

bool MoveItCppPlanner::executeTrajectoryWithFeedback(
    const moveit_msgs::msg::RobotTrajectory &trajectory,
    const std::vector<size_t> &sizes,
    const std::shared_ptr<GoalHandleMoveManipulatorSequence> &goal_handle)
{
    if (!follow_joint_traj_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(logger_, "FollowJointTrajectory action server not available");
        return false;
    }

    control_msgs::action::FollowJointTrajectory::Goal follow_goal;
    follow_goal.trajectory = trajectory.joint_trajectory;

    auto result_promise = std::make_shared<std::promise<bool>>();
    std::future<bool> result_future = result_promise->get_future();
    const auto &points = follow_goal.trajectory.points;

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();

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
        int start_search = static_cast<int>(last_found_index) - 2;
        if (start_search < 0)
            start_search = 0;

        size_t end_search = std::min(last_found_index + 16, waypoints.size());
        double tolerance = 1e-2;
        size_t best_idx = last_found_index;
        double min_distance = std::numeric_limits<double>::max();

        for (size_t i = start_search; i < end_search; ++i)
        {
            double distance = computeDistance(actual_positions, waypoints[i].positions);

            if (distance < min_distance)
            {
                min_distance = distance;
                best_idx = i;
            }

            if (distance <= tolerance)
            {
                break;
            }
        }

        if (best_idx > last_found_index)
        {
            last_found_index = best_idx;
        }
        else if (best_idx == last_found_index && last_found_index < waypoints.size() - 1)
        {
            last_found_index = std::min(last_found_index + 1, waypoints.size() - 1);
        }

        return best_idx;
    };

    // Feedback callback
    send_goal_options.feedback_callback =
        [this, sizes, goal_handle, points, computeDistance, findClosestWaypoint](auto /*goal_handle*/, const auto &feedback)
    {
        if (!goal_handle)
            return;

        const std::vector<double> &actual_positions = feedback->actual.positions;
        size_t closest_idx = findClosestWaypoint(points, actual_positions);

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

        double progress = static_cast<double>(closest_idx) / points.size();
        progress = std::clamp(progress, 0.0, 1.0);

        auto feedback_msg = std::make_shared<manymove_planner::action::MoveManipulatorSequence::Feedback>();
        feedback_msg->progress = static_cast<float>(progress);
        goal_handle->publish_feedback(feedback_msg);

        RCLCPP_INFO(logger_, "Sequence execution progress: %.6f (segment %zu of %zu)",
                    progress, segment_index + 1, sizes.size());
    };

    // Result callback
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


std::pair<bool, moveit_msgs::msg::RobotTrajectory> MoveItCppPlanner::applyTimeParametrizationSequence(
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
    concatenated.joint_trajectory.joint_names = moveit_cpp_ptr_->getRobotModel()
                                                    ->getJointModelGroup(planning_components_->getPlanningGroupName())
                                                    ->getJointModelNames();

    double cumulative_time = 0.0;

    for (size_t i = 0; i < trajectories.size(); ++i)
    {
        const auto &traj = trajectories[i];
        const auto &conf = configs[i];

        auto robot_traj_ptr = std::make_shared<robot_trajectory::RobotTrajectory>(
            moveit_cpp_ptr_->getRobotModel(), planning_components_->getPlanningGroupName());
        robot_traj_ptr->setRobotTrajectoryMsg(*moveit_cpp_ptr_->getCurrentState(), traj);

        moveit_msgs::msg::RobotTrajectory converted_traj = convertToMsg(*robot_traj_ptr);

        if (!applyTimeParameterization(converted_traj, conf))
        {
            RCLCPP_ERROR(logger_, "Time parameterization failed for segment %zu", i + 1);
            return {false, moveit_msgs::msg::RobotTrajectory()};
        }

        // Remove duplicate points
        if (!concatenated.joint_trajectory.points.empty() && !converted_traj.joint_trajectory.points.empty())
        {
            const auto &prev_last = concatenated.joint_trajectory.points.back().positions;
            const auto &current_first = converted_traj.joint_trajectory.points.front().positions;

            bool identical = std::equal(prev_last.begin(), prev_last.end(), current_first.begin(), current_first.end(),
                                        [](double a, double b)
                                        { return std::abs(a - b) < 1e-6; });

            if (identical)
            {
                converted_traj.joint_trajectory.points.erase(converted_traj.joint_trajectory.points.begin());
            }
        }

        // Offset time_from_start
        for (auto &point : converted_traj.joint_trajectory.points)
        {
            double original_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
            double new_time = original_time + cumulative_time;
            point.time_from_start.sec = static_cast<int>(new_time);
            point.time_from_start.nanosec = static_cast<int>((new_time - static_cast<int>(new_time)) * 1e9);
        }

        if (!converted_traj.joint_trajectory.points.empty())
        {
            const auto &last_point = converted_traj.joint_trajectory.points.back();
            cumulative_time = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;
        }

        size_t added_size = converted_traj.joint_trajectory.points.size();
        concatenated.joint_trajectory.points.insert(
            concatenated.joint_trajectory.points.end(),
            converted_traj.joint_trajectory.points.begin(),
            converted_traj.joint_trajectory.points.end());

        sizes.push_back(added_size);
    }

    RCLCPP_INFO(logger_, "Successfully applied time parametrization sequence over %zu segments.", trajectories.size());
    return {true, concatenated};
}

std::pair<std::vector<moveit_msgs::msg::RobotTrajectory>, std::vector<manymove_planner::msg::MovementConfig>>
MoveItCppPlanner::planSequence(const manymove_planner::action::MoveManipulatorSequence::Goal &sequence_goal)
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
            // Use start_joint_values or the current state for the first goal
            const auto &first_goal = sequence_goal.goals[0];
            if (!first_goal.start_joint_values.empty())
            {
                previous_end_joint_targets = first_goal.start_joint_values;
                RCLCPP_INFO(logger_, "Initialized previous_end_joint_targets from start_joint_values of the first goal.");
            }
            else
            {
                previous_end_joint_targets = std::vector<double>(
                    moveit_cpp_ptr_->getCurrentState()->getVariablePositions(),
                    moveit_cpp_ptr_->getCurrentState()->getVariablePositions() + moveit_cpp_ptr_->getCurrentState()->getVariableCount());

                RCLCPP_INFO(logger_, "Initialized previous_end_joint_targets from the current robot pose.");
            }
        }
        else
        {
            // Use the last planned trajectory's end state as the starting state
            previous_end_joint_targets = planned_trajectories.back().joint_trajectory.points.back().positions;
        }

        // Set start_joint_values for the current goal
        modified_goal.goal.start_joint_values = previous_end_joint_targets;

        // Plan the trajectory for the current goal
        auto [success, trajectory] = plan(modified_goal);
        if (!success)
        {
            RCLCPP_ERROR(logger_, "Planning failed for move %zu. Aborting sequence.", i + 1);
            return {{}, {}};
        }

        // Skip if the new trajectory's end is too close to the previous end
        if (areSameJointTargets(trajectory.joint_trajectory.points.back().positions, previous_end_joint_targets, pose_tolerance))
        {
            RCLCPP_WARN(logger_, "Resulting trajectory too small for move %zu, skipping.", i + 1);
        }
        else
        {
            // Add the planned trajectory and its config to the result
            planned_trajectories.push_back(trajectory);
            planned_configs.push_back(modified_goal.goal.config);
        }
    }

    RCLCPP_INFO(logger_, "Successfully planned sequence with %zu steps.", planned_trajectories.size());
    return {planned_trajectories, planned_configs};
}

// Helper method to convert RobotTrajectory to moveit_msgs::msg::RobotTrajectory
moveit_msgs::msg::RobotTrajectory MoveItCppPlanner::convertToMsg(const robot_trajectory::RobotTrajectory &trajectory) const
{
    moveit_msgs::msg::RobotTrajectory traj_msg;
    trajectory.getRobotTrajectoryMsg(traj_msg);
    return traj_msg;
}
