#include "manymove_planner/manymove_planner.hpp"
#include <algorithm>
#include <cmath>
#include <future>
#include <map>
#include <tf2/LinearMath/Quaternion.h>

#include <moveit/robot_model/robot_model.h>

ManyMovePlanner::ManyMovePlanner(
    const rclcpp::Node::SharedPtr &node,
    const std::string &planning_group,
    const std::string &base_frame,
    const std::string &tcp_frame,
    const std::string &traj_controller)
    : node_(node), logger_(node->get_logger()), base_frame_(base_frame), tcp_frame_(tcp_frame), traj_controller_(traj_controller)
{
    // Initialize MoveGroupInterface with the shared node
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, planning_group);
    move_group_interface_->setPlanningTime(5.0);

    RCLCPP_INFO(logger_, "ManyMovePlanner initialized with group: %s", planning_group.c_str());

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
double ManyMovePlanner::computePathLength(const moveit_msgs::msg::RobotTrajectory &trajectory) const
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

double ManyMovePlanner::computeMaxCartesianSpeed(const robot_trajectory::RobotTrajectoryPtr &trajectory) const
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

bool ManyMovePlanner::applyTimeParameterization(robot_trajectory::RobotTrajectoryPtr &trajectory, const manymove_planner::msg::MovementConfig &config)
{
    double velocity_scaling_factor = config.velocity_scaling_factor;
    double acceleration_scaling_factor = config.acceleration_scaling_factor;

    const int max_iterations = 5;
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
        else
        {
            // Default fallback
            trajectory_processing::IterativeSplineParameterization time_param;
            time_param_success = time_param.computeTimeStamps(*trajectory, velocity_scaling_factor, acceleration_scaling_factor);
        }

        if (!time_param_success)
        {
            RCLCPP_ERROR(logger_, "Failed to compute time stamps using '%s'", config.smoothing_type.c_str());
            return false;
        }

        double max_speed = computeMaxCartesianSpeed(trajectory);
        if (max_speed <= config.max_cartesian_speed)
        {
            return true; // success
        }
        else
        {
            double scale = config.max_cartesian_speed / max_speed;
            velocity_scaling_factor *= scale;
            // Adjust acceleration similarly (heuristic)
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

std::pair<bool, moveit_msgs::msg::RobotTrajectory> ManyMovePlanner::plan(const manymove_planner::action::MoveManipulator::Goal &goal)
{
    std::vector<std::pair<moveit_msgs::msg::RobotTrajectory, double>> trajectories;

    // Handle start state
    if (!goal.goal.start_joint_values.empty())
    {
        moveit::core::RobotState start_state(*move_group_interface_->getCurrentState());
        const moveit::core::JointModelGroup *joint_model_group = start_state.getJointModelGroup(move_group_interface_->getName());

        start_state.setJointGroupPositions(joint_model_group, goal.goal.start_joint_values);
        move_group_interface_->setStartState(start_state);
    }
    else
    {
        move_group_interface_->setStartStateToCurrentState();
    }

    // Set scaling factors
    move_group_interface_->setMaxVelocityScalingFactor(goal.goal.config.velocity_scaling_factor);
    move_group_interface_->setMaxAccelerationScalingFactor(goal.goal.config.acceleration_scaling_factor);

    // Set movement targets
    if ((goal.goal.movement_type == "pose") || (goal.goal.movement_type == "joint") || (goal.goal.movement_type == "named"))
    {
        if (goal.goal.movement_type == "pose")
        {
            move_group_interface_->setPoseTarget(goal.goal.pose_target, tcp_frame_);
        }
        else if (goal.goal.movement_type == "joint")
        {
            move_group_interface_->setJointValueTarget(goal.goal.joint_values);
        }
        else if (goal.goal.movement_type == "named")
        {
            move_group_interface_->setNamedTarget(goal.goal.named_target);
        }

        // Plan multiple trajectories
        int attempts = 0;
        while (attempts < goal.goal.config.plan_number_limit && static_cast<int>(trajectories.size()) < goal.goal.config.plan_number_target)
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
                            goal.goal.movement_type.c_str(), attempts + 1);
            }
            attempts++;
        }
    }
    else if (goal.goal.movement_type == "cartesian")
    {
        // Cartesian movement
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(goal.goal.pose_target);

        int attempts = 0;
        while (attempts < goal.goal.config.plan_number_limit && static_cast<int>(trajectories.size()) < goal.goal.config.plan_number_target)
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            double fraction = move_group_interface_->computeCartesianPath(
                waypoints, goal.goal.config.step_size, goal.goal.config.jump_threshold, plan.trajectory_);

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
        RCLCPP_ERROR(logger_, "Unknown movement_type: %s", goal.goal.movement_type.c_str());
        return {false, moveit_msgs::msg::RobotTrajectory()};
    }

    if (trajectories.empty())
    {
        RCLCPP_ERROR(logger_, "No valid trajectory found for movement_type: %s", goal.goal.movement_type.c_str());
        return {false, moveit_msgs::msg::RobotTrajectory()};
    }

    // Select the shortest trajectory
    auto shortest = std::min_element(trajectories.begin(), trajectories.end(),
                                     [](const auto &a, const auto &b)
                                     { return a.second < b.second; });

    return {true, shortest->first};
}

bool ManyMovePlanner::executeTrajectoryWithFeedback(
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

        // End the search at last_found_index + 16 or the end of the waypoint list
        size_t end_search = std::min(last_found_index + 16, waypoints.size());

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

bool ManyMovePlanner::executeTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory)
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

bool ManyMovePlanner::isAtPoseTarget(const geometry_msgs::msg::Pose &target_pose, double tolerance) const
{
    geometry_msgs::msg::Pose current_pose = move_group_interface_->getCurrentPose(tcp_frame_).pose;

    double position_diff = std::sqrt(
        std::pow(current_pose.position.x - target_pose.position.x, 2) +
        std::pow(current_pose.position.y - target_pose.position.y, 2) +
        std::pow(current_pose.position.z - target_pose.position.z, 2));

    double orientation_diff = std::abs(current_pose.orientation.x - target_pose.orientation.x) +
                              std::abs(current_pose.orientation.y - target_pose.orientation.y) +
                              std::abs(current_pose.orientation.z - target_pose.orientation.z) +
                              std::abs(current_pose.orientation.w - target_pose.orientation.w);

    return (position_diff < tolerance) && (orientation_diff < tolerance);
}

bool ManyMovePlanner::isAtJointTarget(const std::vector<double> &joint_values, double tolerance) const
{
    const std::vector<double> &current_joint_values = move_group_interface_->getCurrentJointValues();
    if (current_joint_values.size() != joint_values.size())
        return false;

    for (size_t i = 0; i < joint_values.size(); ++i)
    {
        if (std::abs(current_joint_values[i] - joint_values[i]) > tolerance)
            return false;
    }

    return true;
}

bool ManyMovePlanner::isAtNamedTarget(const std::string &target_name, double tolerance) const
{
    std::map<std::string, double> target_joint_values_map;
    try
    {
        target_joint_values_map = move_group_interface_->getNamedTargetValues(target_name);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(logger_, "Exception retrieving named target '%s': %s", target_name.c_str(), e.what());
        return false;
    }

    const std::vector<std::string> &joint_names = move_group_interface_->getJoints();
    std::vector<double> target_joint_values;
    target_joint_values.reserve(joint_names.size());

    for (const auto &joint_name : joint_names)
    {
        auto it = target_joint_values_map.find(joint_name);
        if (it != target_joint_values_map.end())
        {
            target_joint_values.push_back(it->second);
        }
        else
        {
            RCLCPP_ERROR(logger_, "Joint '%s' not found in named target '%s'", joint_name.c_str(), target_name.c_str());
            return false;
        }
    }

    std::vector<double> current_joint_values = move_group_interface_->getCurrentJointValues();
    if (current_joint_values.size() != target_joint_values.size())
    {
        RCLCPP_ERROR(logger_, "Current joint values size doesn't match target joint values size.");
        return false;
    }

    for (size_t i = 0; i < current_joint_values.size(); ++i)
    {
        if (std::abs(current_joint_values[i] - target_joint_values[i]) > tolerance)
            return false;
    }

    return true;
}

bool ManyMovePlanner::moveToPoseTarget(const geometry_msgs::msg::Pose &target_pose, const manymove_planner::msg::MovementConfig &config)
{
    if (isAtPoseTarget(target_pose))
    {
        RCLCPP_INFO(logger_, "Already at Pose Target. No movement needed.");
        return true;
    }

    manymove_planner::action::MoveManipulator::Goal goal;
    goal.goal.movement_type = "pose";
    goal.goal.pose_target = target_pose;
    goal.goal.start_joint_values = {};
    goal.goal.config = config;

    auto [success, trajectory] = plan(goal);
    if (!success)
    {
        RCLCPP_ERROR(logger_, "Planning failed for Pose Target.");
        return false;
    }

    std::vector<moveit_msgs::msg::RobotTrajectory> single_traj_vec = {trajectory};
    std::vector<manymove_planner::msg::MovementConfig> single_config_vec = {config};
    std::vector<size_t> sizes;
    auto [param_success, final_trajectory] = applyTimeParametrizationSequence(single_traj_vec, single_config_vec, sizes);

    if (!param_success)
    {
        RCLCPP_ERROR(logger_, "Time parameterization failed for Pose Target trajectory.");
        return false;
    }

    if (!executeTrajectory(final_trajectory))
    {
        RCLCPP_ERROR(logger_, "Execution failed for Pose Target trajectory.");
        return false;
    }

    RCLCPP_INFO(logger_, "Successfully executed Pose Target trajectory.");
    return true;
}

bool ManyMovePlanner::moveToJointTarget(const std::vector<double> &joint_values, const manymove_planner::msg::MovementConfig &config)
{
    if (isAtJointTarget(joint_values))
    {
        RCLCPP_INFO(logger_, "Already at Joint Target. No movement needed.");
        return true;
    }

    manymove_planner::action::MoveManipulator::Goal goal;
    goal.goal.movement_type = "joint";
    goal.goal.joint_values = joint_values;
    goal.goal.start_joint_values = {};
    goal.goal.config = config;

    auto [success, trajectory] = plan(goal);
    if (!success)
    {
        RCLCPP_ERROR(logger_, "Planning failed for Joint Target.");
        return false;
    }

    std::vector<moveit_msgs::msg::RobotTrajectory> single_traj_vec = {trajectory};
    std::vector<manymove_planner::msg::MovementConfig> single_config_vec = {config};
    std::vector<size_t> sizes;
    auto [param_success, final_trajectory] = applyTimeParametrizationSequence(single_traj_vec, single_config_vec, sizes);

    if (!param_success)
    {
        RCLCPP_ERROR(logger_, "Time parameterization failed for Joint Target trajectory.");
        return false;
    }

    if (!executeTrajectory(final_trajectory))
    {
        RCLCPP_ERROR(logger_, "Execution failed for Joint Target trajectory.");
        return false;
    }

    RCLCPP_INFO(logger_, "Successfully executed Joint Target trajectory.");
    return true;
}

bool ManyMovePlanner::moveToNamedTarget(const std::string &target_name, const manymove_planner::msg::MovementConfig &config)
{
    if (isAtNamedTarget(target_name))
    {
        RCLCPP_INFO(logger_, "Already at the Named Target '%s'. No movement needed.", target_name.c_str());
        return true;
    }

    manymove_planner::action::MoveManipulator::Goal goal;
    goal.goal.movement_type = "named";
    goal.goal.named_target = target_name;
    goal.goal.start_joint_values = {};
    goal.goal.config = config;

    auto [success, trajectory] = plan(goal);
    if (!success)
    {
        RCLCPP_ERROR(logger_, "Planning failed for Named Target.");
        return false;
    }

    std::vector<moveit_msgs::msg::RobotTrajectory> single_traj_vec = {trajectory};
    std::vector<manymove_planner::msg::MovementConfig> single_config_vec = {config};
    std::vector<size_t> sizes;
    auto [param_success, final_trajectory] = applyTimeParametrizationSequence(single_traj_vec, single_config_vec, sizes);

    if (!param_success)
    {
        RCLCPP_ERROR(logger_, "Time parameterization failed for Named Target trajectory.");
        return false;
    }

    if (!executeTrajectory(final_trajectory))
    {
        RCLCPP_ERROR(logger_, "Execution failed for Named Target trajectory.");
        return false;
    }

    RCLCPP_INFO(logger_, "Successfully executed Named Target trajectory.");
    return true;
}

bool ManyMovePlanner::moveCartesianPath(const geometry_msgs::msg::Pose &target_pose, const manymove_planner::msg::MovementConfig &config)
{
    if (isAtPoseTarget(target_pose))
    {
        RCLCPP_INFO(logger_, "Already at the Pose Target. No movement needed.");
        return true;
    }

    manymove_planner::action::MoveManipulator::Goal goal;
    goal.goal.movement_type = "cartesian";
    goal.goal.pose_target = target_pose;
    goal.goal.start_joint_values = {};
    goal.goal.config = config;

    auto [success, trajectory] = plan(goal);
    if (!success)
    {
        RCLCPP_ERROR(logger_, "Planning failed for Cartesian Path.");
        return false;
    }

    std::vector<moveit_msgs::msg::RobotTrajectory> single_traj_vec = {trajectory};
    std::vector<manymove_planner::msg::MovementConfig> single_config_vec = {config};
    std::vector<size_t> sizes;
    auto [param_success, final_trajectory] = applyTimeParametrizationSequence(single_traj_vec, single_config_vec, sizes);

    if (!param_success)
    {
        RCLCPP_ERROR(logger_, "Time parameterization failed for Cartesian Path trajectory.");
        return false;
    }

    if (!executeTrajectory(final_trajectory))
    {
        RCLCPP_ERROR(logger_, "Execution failed for Cartesian Path trajectory.");
        return false;
    }

    RCLCPP_INFO(logger_, "Successfully executed Cartesian Path trajectory.");
    return true;
}

bool ManyMovePlanner::findCollisionObject(const std::string &partial_id, moveit_msgs::msg::CollisionObject &found_object)
{
    auto client = node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");

    if (!client->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(logger_, "Service /get_planning_scene not available");
        return false;
    }

    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components = 0;

    auto future = client->async_send_request(request);

    auto status = future.wait_for(std::chrono::seconds(5));
    if (status != std::future_status::ready)
    {
        RCLCPP_ERROR(logger_, "Service /get_planning_scene not responded in time");
        return false;
    }

    auto response = future.get();
    if (!response)
    {
        RCLCPP_ERROR(logger_, "Empty response from /get_planning_scene");
        return false;
    }

    const auto &objects = response->scene.world.collision_objects;
    for (const auto &obj : objects)
    {
        if (obj.id.find(partial_id) != std::string::npos)
        {
            found_object = obj;
            return true;
        }
    }

    return false;
}

geometry_msgs::msg::Pose ManyMovePlanner::computeEndPoseFromJoints(const std::vector<double> &joint_values) const
{
    moveit::core::RobotState state(*move_group_interface_->getCurrentState());
    const moveit::core::JointModelGroup *jmg = state.getJointModelGroup(move_group_interface_->getName());
    state.setJointGroupPositions(jmg, joint_values);
    state.update();

    const Eigen::Isometry3d &eef_transform = state.getGlobalLinkTransform(tcp_frame_);
    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = eef_transform.translation().x();
    pose_msg.position.y = eef_transform.translation().y();
    pose_msg.position.z = eef_transform.translation().z();
    Eigen::Quaterniond q(eef_transform.rotation());
    pose_msg.orientation.x = q.x();
    pose_msg.orientation.y = q.y();
    pose_msg.orientation.z = q.z();
    pose_msg.orientation.w = q.w();
    return pose_msg;
}

std::vector<double> ManyMovePlanner::getNamedTargetJoints(const std::string &name)
{
    std::map<std::string, double> named_map = move_group_interface_->getNamedTargetValues(name);
    const auto &joint_names = move_group_interface_->getJoints();
    std::vector<double> values;
    values.reserve(joint_names.size());
    for (const auto &jn : joint_names)
    {
        auto it = named_map.find(jn);
        if (it == named_map.end())
        {
            return std::vector<double>();
        }
        values.push_back(it->second);
    }
    return values;
}

bool ManyMovePlanner::areSamePoses(const geometry_msgs::msg::Pose &p1, const geometry_msgs::msg::Pose &p2, double tolerance)
{
    double dx = p1.position.x - p2.position.x;
    double dy = p1.position.y - p2.position.y;
    double dz = p1.position.z - p2.position.z;
    double pos_dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    double ox = std::abs(p1.orientation.x - p2.orientation.x);
    double oy = std::abs(p1.orientation.y - p2.orientation.y);
    double oz = std::abs(p1.orientation.z - p2.orientation.z);
    double ow = std::abs(p1.orientation.w - p2.orientation.w);
    double ori_diff = ox + oy + oz + ow;

    return (pos_dist < tolerance && ori_diff < tolerance);
}

bool ManyMovePlanner::areSameJointTargets(const std::vector<double> &j1, const std::vector<double> &j2, double tolerance) const
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

// Updated planSequence function in manymove_planner.cpp

std::pair<std::vector<moveit_msgs::msg::RobotTrajectory>, std::vector<manymove_planner::msg::MovementConfig>> ManyMovePlanner::planSequence(
    const manymove_planner::action::MoveManipulatorSequence::Goal &sequence_goal)
{
    std::vector<moveit_msgs::msg::RobotTrajectory> planned_trajectories;
    std::vector<manymove_planner::msg::MovementConfig> planned_configs;

    // Initialize previous end pose and joint targets
    std::vector<double> previous_end_joint_targets;
    // geometry_msgs::msg::Pose previous_end_pose;

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
            // Check if the first goal has start_joint_values
            const auto &first_goal = sequence_goal.goals[0];
            if (!first_goal.start_joint_values.empty())
            {
                previous_end_joint_targets = first_goal.start_joint_values;
                // previous_end_pose = computeEndPoseFromJoints(previous_end_joint_targets);
                RCLCPP_INFO(logger_, "Initialized previous_end_pose from start_joint_values of the first goal.");
            }
            else
            {
                previous_end_joint_targets = move_group_interface_->getCurrentJointValues();
                // previous_end_pose = move_group_interface_->getCurrentPose(tcp_frame_).pose;
                RCLCPP_INFO(logger_, "Initialized previous_end_pose from the current robot pose.");
            }
        }
        else
        {
            previous_end_joint_targets = planned_trajectories.back().joint_trajectory.points.back().positions;
            // previous_end_pose = computeEndPoseFromJoints(previous_end_joint_targets);
        }

        modified_goal.goal.start_joint_values = previous_end_joint_targets;

        // Plan the trajectory for the current move
        auto [success, trajectory] = plan(modified_goal);
        if (!success)
        {
            RCLCPP_ERROR(logger_, "Planning failed for move %zu. Aborting sequence.", i + 1);
            // Optionally, you can choose to continue instead of aborting
            return {{}, {}};
        }

        if (areSameJointTargets(trajectory.joint_trajectory.points.back().positions, previous_end_joint_targets, pose_tolerance))
        {
            RCLCPP_ERROR(logger_, "Resulting trajectory too small, skipping");
        }
        else
        {
            // Append the planned trajectory and its config to the lists
            planned_trajectories.push_back(trajectory);
            planned_configs.push_back(modified_goal.goal.config);
        }
    }

    return {planned_trajectories, planned_configs};
}

std::pair<bool, moveit_msgs::msg::RobotTrajectory> ManyMovePlanner::applyTimeParametrizationSequence(
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

    double cumulative_time = 0.0;        // Track total time
    std::vector<double> last_velocities; // Track last velocities for continuity

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

            // Extract last velocities for continuity (if needed)
            last_velocities = last_point.velocities;
        }

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
