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
    const std::string &tcp_frame)
    : node_(node), logger_(node->get_logger()), base_frame_(base_frame), tcp_frame_(tcp_frame)
{
    // Initialize MoveGroupInterface with the shared node
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, planning_group);
    move_group_interface_->setPlanningTime(5.0);

    RCLCPP_INFO(logger_, "ManyMovePlanner initialized with group: %s", planning_group.c_str());

    // Initialize FollowJointTrajectory action client
    follow_joint_traj_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(node_, "/lite6_traj_controller/follow_joint_trajectory");
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

    for (size_t i = 1; i < joint_trajectory.points.size(); ++i)
    {
        double segment_length = 0.0;
        for (size_t j = 0; j < joint_trajectory.points[i].positions.size(); ++j)
        {
            double diff = joint_trajectory.points[i].positions[j] - joint_trajectory.points[i - 1].positions[j];
            segment_length += diff * diff;
        }
        total_length += std::sqrt(segment_length);
    }

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

    // Helper function to find the closest waypoint index based on actual positions
    auto findClosestWaypoint = [&](const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &waypoints,
                                   const std::vector<double> &actual_positions) -> size_t
    {
        size_t closest_idx = 0;
        double min_distance = std::numeric_limits<double>::max();

        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            double distance = computeDistance(actual_positions, waypoints[i].positions);
            if (distance < min_distance)
            {
                min_distance = distance;
                closest_idx = i;
            }
        }

        return closest_idx;
    };

    // Updated feedback callback
    send_goal_options.feedback_callback =
        [this, sizes, goal_handle, points, computeDistance, findClosestWaypoint](auto, const auto &feedback)
    {
        if (!goal_handle)
            return;

        // Extract actual positions from feedback
        std::vector<double> actual_positions = feedback->actual.positions;

        // // Debugging: Print actual positions
        // std::stringstream pos_stream;
        // pos_stream << "Actual Positions: ";
        // for (const auto &pos : actual_positions)
        //     pos_stream << pos << " ";
        // RCLCPP_INFO(logger_, "%s", pos_stream.str().c_str());

        // Find the closest waypoint index
        size_t closest_idx = findClosestWaypoint(points, actual_positions);

        // // Debugging: Print closest waypoint index
        // RCLCPP_INFO(logger_, "Closest Waypoint Index: %zu", closest_idx);

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

std::pair<std::vector<moveit_msgs::msg::RobotTrajectory>, std::vector<manymove_planner::msg::MovementConfig>> ManyMovePlanner::planSequence(
    const manymove_planner::action::MoveManipulatorSequence::Goal &sequence_goal)
{
    std::vector<moveit_msgs::msg::RobotTrajectory> trajectories;
    std::vector<manymove_planner::msg::MovementConfig> configs;

    if (sequence_goal.goals.empty())
    {
        RCLCPP_WARN(logger_, "planSequence called with an empty goals vector.");
        return {trajectories, configs};
    }

    auto areSamePoses = [&](const geometry_msgs::msg::Pose &p1, const geometry_msgs::msg::Pose &p2, double tolerance)
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
    };

    auto getNamedTargetJoints = [&](const std::string &name) -> std::vector<double>
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
                return std::vector<double>(); // Error, missing joint
            }
            values.push_back(it->second);
        }
        return values;
    };

    geometry_msgs::msg::Pose last_request_pose;
    std::vector<double> last_request_joints;
    std::string last_request_named;
    bool have_last_request = false;
    bool last_was_pose = false;
    bool last_was_joint = false;
    bool last_was_named = false;
    bool last_was_cartesian = false;

    std::vector<bool> duplicates(sequence_goal.goals.size(), false);

    for (size_t i = 0; i < sequence_goal.goals.size(); ++i)
    {
        const auto g = sequence_goal.goals[i]; // Access the nested goal
        if (i == 0)
        {
            // No previous goal
            have_last_request = true;
            if (g.movement_type == "pose" || g.movement_type == "cartesian")
            {
                last_request_pose = g.pose_target;
                last_was_pose = (g.movement_type == "pose");
                last_was_cartesian = (g.movement_type == "cartesian");
                last_was_joint = false;
                last_was_named = false;
            }
            else if (g.movement_type == "joint")
            {
                last_request_joints = g.joint_values;
                last_was_joint = true;
                last_was_pose = false;
                last_was_named = false;
                last_was_cartesian = false;
            }
            else if (g.movement_type == "named")
            {
                last_request_named = g.named_target;
                last_was_named = true;
                last_was_joint = false;
                last_was_pose = false;
                last_was_cartesian = false;
            }
            continue;
        }

        bool dup = false;
        if (g.movement_type == "pose" || g.movement_type == "cartesian")
        {
            if ((last_was_pose || last_was_cartesian) && areSamePoses(last_request_pose, g.pose_target, 1e-3))
            {
                dup = true;
            }
        }
        else if (g.movement_type == "joint")
        {
            if (last_was_joint && last_request_joints.size() == g.joint_values.size())
            {
                bool same = true;
                for (size_t idx = 0; idx < g.joint_values.size(); ++idx)
                {
                    if (std::abs(last_request_joints[idx] - g.joint_values[idx]) > 1e-3)
                    {
                        same = false;
                        break;
                    }
                }
                if (same)
                    dup = true;
            }
        }
        else if (g.movement_type == "named")
        {
            if (last_was_named && (last_request_named == g.named_target))
            {
                dup = true;
            }
        }

        duplicates[i] = dup;

        if (!dup)
        {
            // Update last request
            if (g.movement_type == "pose" || g.movement_type == "cartesian")
            {
                last_request_pose = g.pose_target;
                last_was_pose = (g.movement_type == "pose");
                last_was_cartesian = (g.movement_type == "cartesian");
                last_was_joint = false;
                last_was_named = false;
            }
            else if (g.movement_type == "joint")
            {
                last_request_joints = g.joint_values;
                last_was_joint = true;
                last_was_pose = false;
                last_was_named = false;
                last_was_cartesian = false;
            }
            else if (g.movement_type == "named")
            {
                last_request_named = g.named_target;
                last_was_named = true;
                last_was_pose = false;
                last_was_joint = false;
                last_was_cartesian = false;
            }
        }
    }

    std::vector<moveit_msgs::msg::RobotTrajectory> planned_trajectories(sequence_goal.goals.size());
    std::vector<manymove_planner::msg::MovementConfig> planned_configs(sequence_goal.goals.size());

    for (size_t i = 0; i < sequence_goal.goals.size(); ++i)
    {
        if (duplicates[i])
        {
            continue;
        }

        manymove_planner::action::MoveManipulator::Goal modified_goal;
        modified_goal.goal = sequence_goal.goals[i];
        if (i > 0)
        {
            int j = static_cast<int>(i) - 1;
            std::vector<double> last_valid_joints;
            while (j >= 0)
            {
                if (!duplicates[j] && !planned_trajectories[j].joint_trajectory.points.empty())
                {
                    last_valid_joints = planned_trajectories[j].joint_trajectory.points.back().positions;
                    break;
                }
                j--;
            }
            if (!last_valid_joints.empty())
            {
                modified_goal.goal.start_joint_values = last_valid_joints;
            }
        }

        auto [success, trajectory] = plan(modified_goal);
        if (!success)
        {
            RCLCPP_ERROR(logger_, "Planning failed for goal %zu. Returning empty.", i + 1);
            return {{}, {}};
        }

        planned_trajectories[i] = trajectory;
        planned_configs[i] = modified_goal.goal.config;
    }

    for (size_t i = 0; i < sequence_goal.goals.size(); ++i)
    {
        if (duplicates[i])
        {
            moveit_msgs::msg::RobotTrajectory single_point_traj;
            single_point_traj.joint_trajectory.joint_names = move_group_interface_->getActiveJoints();

            trajectory_msgs::msg::JointTrajectoryPoint pt;
            std::vector<double> last_jv;
            {
                int j = static_cast<int>(i) - 1;
                while (j >= 0)
                {
                    if (!duplicates[j] && !planned_trajectories[j].joint_trajectory.points.empty())
                    {
                        last_jv = planned_trajectories[j].joint_trajectory.points.back().positions;
                        break;
                    }
                    j--;
                }
                if (last_jv.empty())
                {
                    last_jv = move_group_interface_->getCurrentJointValues();
                }
            }

            pt.positions = last_jv;
            pt.time_from_start.sec = 0;
            pt.time_from_start.nanosec = 0;
            single_point_traj.joint_trajectory.points.push_back(pt);
            // For std::thread
            planned_trajectories[i] = single_point_traj;
            planned_configs[i] = sequence_goal.goals[i].config;
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

        // If not the first segment, set initial velocities based on last velocities of previous segment
        if (i > 0 && !last_velocities.empty())
        {
            robot_traj_ptr->setRobotTrajectoryMsg(*move_group_interface_->getCurrentState(), traj);
            // Optionally, set initial velocities if your time parametrization method supports it
            // This depends on the capabilities of the trajectory_processing library
            // For example:
            // robot_traj_ptr->getWayPoint(0).setVariableVelocity(last_velocities);
        }

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
            point.time_from_start.nanosec = static_cast<int>((new_time - point.time_from_start.sec) * 1e9);
        }

        // Update cumulative_time based on the last point of the segment
        if (!segment.joint_trajectory.points.empty())
        {
            const auto &last_point = segment.joint_trajectory.points.back();
            cumulative_time = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;

            // Extract last velocities for continuity
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