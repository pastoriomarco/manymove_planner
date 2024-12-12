// manymove_planner.cpp

#include "manymove_planner/manymove_planner.hpp"
#include <algorithm>
#include <cmath>
#include <future>

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

        double max_cartesian_speed = computeMaxCartesianSpeed(trajectory);
        if (max_cartesian_speed <= config.max_cartesian_speed)
        {
            return true; // success
        }
        else
        {
            double scale = config.max_cartesian_speed / max_cartesian_speed;
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
    // Initialize variables
    std::vector<std::pair<moveit_msgs::msg::RobotTrajectory, double>> trajectories;

    // Configure the move_group_interface_ based on movement_type
    // Handle start state
    if (!goal.start_joint_values.empty())
    {
        moveit::core::RobotState start_state(*move_group_interface_->getCurrentState());
        const moveit::core::JointModelGroup *joint_model_group = start_state.getJointModelGroup(move_group_interface_->getName());

        // Set joint group positions; no return value to capture
        start_state.setJointGroupPositions(joint_model_group, goal.start_joint_values);
        move_group_interface_->setStartState(start_state);
    }
    else
    {
        move_group_interface_->setStartStateToCurrentState();
    }

    // Set scaling factors
    move_group_interface_->setMaxVelocityScalingFactor(goal.config.velocity_scaling_factor);
    move_group_interface_->setMaxAccelerationScalingFactor(goal.config.acceleration_scaling_factor);

    // Set movement targets
    if ((goal.movement_type == "pose") || (goal.movement_type == "joint") || (goal.movement_type == "named"))
    {
        if (goal.movement_type == "pose")
        {
            move_group_interface_->setPoseTarget(goal.pose_target, tcp_frame_);
        }
        else if (goal.movement_type == "joint")
        {
            move_group_interface_->setJointValueTarget(goal.joint_values);
        }
        else if (goal.movement_type == "named")
        {
            move_group_interface_->setNamedTarget(goal.named_target);
        }

        // Plan multiple trajectories
        int attempts = 0;
        while (attempts < goal.config.plan_number_limit && static_cast<int>(trajectories.size()) < goal.config.plan_number_target)
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
            {
                double length = computePathLength(plan.trajectory_);
                trajectories.emplace_back(plan.trajectory_, length);
            }
            else
            {
                RCLCPP_WARN_STREAM(logger_, goal.movement_type << " target planning attempt " << (attempts + 1) << " failed.");
            }
            attempts++;
        }
    }
    else if (goal.movement_type == "cartesian")
    {
        // For cartesian movement, use pose_target as the single waypoint
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(goal.pose_target);

        // Plan multiple trajectories
        int attempts = 0;
        while (attempts < goal.config.plan_number_limit && static_cast<int>(trajectories.size()) < goal.config.plan_number_target)
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            double fraction = move_group_interface_->computeCartesianPath(
                waypoints,
                goal.config.step_size,      // eef_step
                goal.config.jump_threshold, // jump_threshold
                plan.trajectory_);

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
        RCLCPP_ERROR(logger_, "Unknown movement_type: %s", goal.movement_type.c_str());
        return std::make_pair(false, moveit_msgs::msg::RobotTrajectory());
    }

    // Check if any trajectory was found
    if (trajectories.empty())
    {
        RCLCPP_ERROR(logger_, "No valid trajectory found for movement_type: %s", goal.movement_type.c_str());
        return std::make_pair(false, moveit_msgs::msg::RobotTrajectory());
    }

    // Select the shortest trajectory
    auto shortest = std::min_element(trajectories.begin(), trajectories.end(),
                                     [](const std::pair<moveit_msgs::msg::RobotTrajectory, double> &a,
                                        const std::pair<moveit_msgs::msg::RobotTrajectory, double> &b) -> bool
                                     {
                                         return a.second < b.second;
                                     });

    // Return the trajectory and success flag
    return std::make_pair(true, shortest->first);
}

bool ManyMovePlanner::executeTrajectory(const robot_trajectory::RobotTrajectoryPtr &trajectory)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    trajectory->getRobotTrajectoryMsg(plan.trajectory_);

    bool success = (move_group_interface_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success)
    {
        RCLCPP_ERROR(logger_, "Trajectory execution failed.");
    }
    return success;
}

bool ManyMovePlanner::isAtPoseTarget(const geometry_msgs::msg::Pose &target_pose, double tolerance) const
{
    geometry_msgs::msg::Pose current_pose = move_group_interface_->getCurrentPose(tcp_frame_).pose;

    double position_diff = std::sqrt(
        std::pow(current_pose.position.x - target_pose.position.x, 2) +
        std::pow(current_pose.position.y - target_pose.position.y, 2) +
        std::pow(current_pose.position.z - target_pose.position.z, 2));

    // For orientation, compute the quaternion distance
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
    // Retrieve the joint values for the named target
    std::map<std::string, double> target_joint_values_map;
    target_joint_values_map = move_group_interface_->getNamedTargetValues(target_name);

    // Convert the map to a vector of joint values in the correct order
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

    // Get current joint values
    std::vector<double> current_joint_values = move_group_interface_->getCurrentJointValues();

    if (current_joint_values.size() != target_joint_values.size())
    {
        RCLCPP_ERROR(logger_, "Current joint values size does not match target joint values size.");
        return false;
    }

    // Compare joint values within the specified tolerance
    for (size_t i = 0; i < current_joint_values.size(); ++i)
    {
        if (std::abs(current_joint_values[i] - target_joint_values[i]) > tolerance)
            return false;
    }

    return true;
}

bool ManyMovePlanner::moveToPoseTarget(const geometry_msgs::msg::Pose &target_pose, const manymove_planner::msg::MovementConfig &config)
{
    // Check if already at the target pose
    if (isAtPoseTarget(target_pose))
    {
        RCLCPP_INFO(logger_, "Already at the Pose Target. No movement needed.");
        return true;
    }

    // Construct the Goal message
    manymove_planner::action::MoveManipulator::Goal goal;
    goal.movement_type = "pose";
    goal.pose_target = target_pose;
    // No joint values or named target needed for pose movements
    goal.start_joint_values = {}; // Empty to use current state
    goal.config = config;

    // Call the centralized plan() function
    auto [success, trajectory] = plan(goal);

    if (!success)
    {
        RCLCPP_ERROR(logger_, "Planning failed for Pose Target.");
        return false;
    }

    // Create RobotTrajectory for time parameterization
    auto robot_traj = std::make_shared<robot_trajectory::RobotTrajectory>(
        move_group_interface_->getRobotModel(), move_group_interface_->getName());
    robot_traj->setRobotTrajectoryMsg(*move_group_interface_->getCurrentState(), trajectory);

    // Apply time parameterization
    if (!applyTimeParameterization(robot_traj, config))
    {
        RCLCPP_ERROR(logger_, "Time parameterization failed for Pose Target trajectory.");
        return false;
    }

    // Execute the trajectory
    if (!executeTrajectory(robot_traj))
    {
        RCLCPP_ERROR(logger_, "Execution failed for Pose Target trajectory.");
        return false;
    }

    RCLCPP_INFO(logger_, "Successfully executed Pose Target trajectory.");
    return true;
}

bool ManyMovePlanner::moveToJointTarget(const std::vector<double> &joint_values, const manymove_planner::msg::MovementConfig &config)
{
    // Check if already at the target joint positions
    if (isAtJointTarget(joint_values))
    {
        RCLCPP_INFO(logger_, "Already at the Joint Target. No movement needed.");
        return true;
    }

    // Construct the Goal message
    manymove_planner::action::MoveManipulator::Goal goal;
    goal.movement_type = "joint";
    goal.joint_values = joint_values;
    // No pose target or named target needed for joint movements
    goal.start_joint_values = {}; // Empty to use current state
    goal.config = config;

    // Call the centralized plan() function
    auto [success, trajectory] = plan(goal);

    if (!success)
    {
        RCLCPP_ERROR(logger_, "Planning failed for Joint Target.");
        return false;
    }

    // Create RobotTrajectory for time parameterization
    auto robot_traj = std::make_shared<robot_trajectory::RobotTrajectory>(
        move_group_interface_->getRobotModel(), move_group_interface_->getName());
    robot_traj->setRobotTrajectoryMsg(*move_group_interface_->getCurrentState(), trajectory);

    // Apply time parameterization
    if (!applyTimeParameterization(robot_traj, config))
    {
        RCLCPP_ERROR(logger_, "Time parameterization failed for Joint Target trajectory.");
        return false;
    }

    // Execute the trajectory
    if (!executeTrajectory(robot_traj))
    {
        RCLCPP_ERROR(logger_, "Execution failed for Joint Target trajectory.");
        return false;
    }

    RCLCPP_INFO(logger_, "Successfully executed Joint Target trajectory.");
    return true;
}

bool ManyMovePlanner::moveToNamedTarget(const std::string &target_name, const manymove_planner::msg::MovementConfig &config)
{
    // Check if already at the named target
    if (isAtNamedTarget(target_name))
    {
        RCLCPP_INFO(logger_, "Already at the Named Target '%s'. No movement needed.", target_name.c_str());
        return true;
    }

    // Construct the Goal message
    manymove_planner::action::MoveManipulator::Goal goal;
    goal.movement_type = "named";
    goal.named_target = target_name;
    // No pose target or joint values needed for named movements
    goal.start_joint_values = {}; // Empty to use current state
    goal.config = config;

    // Call the centralized plan() function
    auto [success, trajectory] = plan(goal);

    if (!success)
    {
        RCLCPP_ERROR(logger_, "Planning failed for Named Target.");
        return false;
    }

    // Create RobotTrajectory for time parameterization
    auto robot_traj = std::make_shared<robot_trajectory::RobotTrajectory>(
        move_group_interface_->getRobotModel(), move_group_interface_->getName());
    robot_traj->setRobotTrajectoryMsg(*move_group_interface_->getCurrentState(), trajectory);

    // Apply time parameterization
    if (!applyTimeParameterization(robot_traj, config))
    {
        RCLCPP_ERROR(logger_, "Time parameterization failed for Named Target trajectory.");
        return false;
    }

    // Execute the trajectory
    if (!executeTrajectory(robot_traj))
    {
        RCLCPP_ERROR(logger_, "Execution failed for Named Target trajectory.");
        return false;
    }

    RCLCPP_INFO(logger_, "Successfully executed Named Target trajectory.");
    return true;
}

bool ManyMovePlanner::moveCartesianPath(const geometry_msgs::msg::Pose &target_pose, const manymove_planner::msg::MovementConfig &config)
{
    // Check if already at the target pose
    if (isAtPoseTarget(target_pose))
    {
        RCLCPP_INFO(logger_, "Already at the Pose Target. No movement needed.");
        return true;
    }
    
    // Construct the Goal message
    manymove_planner::action::MoveManipulator::Goal goal;
    goal.movement_type = "cartesian";
    goal.pose_target = target_pose; // Using pose_target as the single waypoint
    // No joint values or named target needed for Cartesian movements
    goal.start_joint_values = {}; // Empty to use current state
    goal.config = config;

    // Call the centralized plan() function
    auto [success, trajectory] = plan(goal);

    if (!success)
    {
        RCLCPP_ERROR(logger_, "Planning failed for Cartesian Path.");
        return false;
    }

    // Create RobotTrajectory for time parameterization
    auto robot_traj = std::make_shared<robot_trajectory::RobotTrajectory>(
        move_group_interface_->getRobotModel(), move_group_interface_->getName());
    robot_traj->setRobotTrajectoryMsg(*move_group_interface_->getCurrentState(), trajectory);

    // Apply time parameterization
    if (!applyTimeParameterization(robot_traj, config))
    {
        RCLCPP_ERROR(logger_, "Time parameterization failed for Cartesian Path trajectory.");
        return false;
    }

    // Execute the trajectory
    if (!executeTrajectory(robot_traj))
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

std::pair<std::vector<moveit_msgs::msg::RobotTrajectory>, std::vector<manymove_planner::msg::MovementConfig>> ManyMovePlanner::planSequence(const std::vector<manymove_planner::action::MoveManipulator::Goal> &goals)
{
    std::vector<moveit_msgs::msg::RobotTrajectory> trajectories;
    std::vector<manymove_planner::msg::MovementConfig> configs;

    if (goals.empty())
    {
        RCLCPP_WARN(logger_, "planSequence called with an empty goals vector.");
        return {trajectories, configs};
    }

    std::vector<double> last_joint_values;

    for (size_t i = 0; i < goals.size(); ++i)
    {
        // Create a mutable copy of the current goal
        manymove_planner::action::MoveManipulator::Goal current_goal = goals[i];

        // If not the first goal, set the start_joint_values to the last joint values of the previous trajectory
        if (i > 0 && !last_joint_values.empty())
        {
            current_goal.start_joint_values = last_joint_values;
        }

        // Log the planning for the current goal
        RCLCPP_INFO(logger_, "Planning trajectory for goal %zu with movement_type: %s", i + 1, current_goal.movement_type.c_str());

        // Plan the trajectory using the existing plan() function
        auto [success, trajectory] = plan(current_goal);

        if (!success)
        {
            RCLCPP_ERROR(logger_, "Planning failed for goal %zu with movement_type: %s", i + 1, current_goal.movement_type.c_str());
            break; // Optionally, continue to next goal or stop planning
        }

        // Store the planned trajectory and config
        trajectories.emplace_back(trajectory);
        configs.emplace_back(current_goal.config);

        // Extract the last joint values from the trajectory to set as start for next goal
        if (!trajectory.joint_trajectory.points.empty())
        {
            last_joint_values = trajectory.joint_trajectory.points.back().positions;
            RCLCPP_INFO(logger_, "Extracted last joint values for goal %zu.", i + 1);
        }
        else
        {
            RCLCPP_WARN(logger_, "Planned trajectory for goal %zu has no joint points.", i + 1);
            last_joint_values = {}; // Reset or handle as needed
        }
    }

    return {trajectories, configs};
}