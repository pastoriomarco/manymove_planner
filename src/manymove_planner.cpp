// manymove_planner.cpp

#include "manymove_planner/manymove_planner.hpp"
#include <algorithm>
#include <cmath>
#include <future>
#include <map>

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
    try
    {
        target_joint_values_map = move_group_interface_->getNamedTargetValues(target_name);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(logger_, "Exception while retrieving named target '%s': %s", target_name.c_str(), e.what());
        return false;
    }

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
    const std::vector<manymove_planner::action::MoveManipulator::Goal> &goals)
{
    std::vector<moveit_msgs::msg::RobotTrajectory> trajectories;
    std::vector<manymove_planner::msg::MovementConfig> configs;

    if (goals.empty())
    {
        RCLCPP_WARN(logger_, "planSequence called with an empty goals vector.");
        return {trajectories, configs};
    }

    // Helper functions
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
                return std::vector<double>(); // Error, missing joint
            values.push_back(it->second);
        }
        return values;
    };

    // First pass: Determine duplicates.
    // We need the final state of the previous trajectory to check duplicates, but at this stage we haven't planned yet.
    // Instead, we will simulate planning by "predicting" the end state. Actually, for duplicates, we need to rely on previous move's end state.
    // For the first goal, there's no previous end state, so no duplicate check.
    // For subsequent goals, we check if the current goal leads to the same final state as the last planned final state.

    // We'll store the predicted final joint state after each successfully planned goal (on second phase).
    // But we have a chicken-and-egg problem: duplicates should be identified before planning.
    // A trick: The only reason goals are duplicates is if their final state (pose, joint values, or named target) matches the previous final state.
    // But we don't know the previous final state's last_joint_values yet until we plan the first non-duplicate goal.

    // Simplify the logic:
    // 1. We'll plan the first goal no matter what.
    // 2. Then we know its end state and can decide duplicates for subsequent goals on-the-fly before planning them.

    // Revised approach:
    // We'll do a single pass, planning as we go, but separating duplicate detection from planning:
    // - For each goal:
    //   - If first or no previous valid move: Just plan it.
    //   - Else: Check if duplicate by comparing current goal's target end state with last_joint_values end state.
    //     - If duplicate: record as duplicate and insert a placeholder after finishing planning all.
    //     - If not duplicate: plan and update end state.

    // Wait, this still intermixes planning and duplicate detection.
    // The suggestion was to first identify duplicates, then plan.

    // Let's try a two-pass approach that doesn't need exact final joint states for duplicates:
    // Actually, we do need final joint states to accurately determine duplicates for pose/cartesian and named (which rely on actual end joints).
    // For "joint" and "named" goals, we can deduce final joint states easily. For "pose"/"cartesian", we need to plan or at least know the final joint state from previous goal.

    // Another strategy:
    // Assume duplicates are checked based on the final requested target, not the actual planned end state.
    // If the user tries the same final pose/joint/named target as the last target, it's a duplicate.
    // This is slightly different logic than before but acceptable given the instructions. The user expects duplicates if they request the same final condition.

    // We'll store the "final request" of the previous goal:
    geometry_msgs::msg::Pose last_request_pose;
    std::vector<double> last_request_joints;
    std::string last_request_named;
    bool have_last_request = false;
    bool last_was_pose = false;
    bool last_was_joint = false;
    bool last_was_named = false;
    bool last_was_cartesian = false;

    std::vector<bool> duplicates(goals.size(), false);

    for (size_t i = 0; i < goals.size(); ++i)
    {
        const auto &g = goals[i];
        if (i == 0)
        {
            // No previous goal, can't be duplicate
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

        // For subsequent goals:
        if (!have_last_request)
        {
            // This would mean previous planning failed or we never got a valid previous request.
            // That can't happen if we fail early on plan. So assume we always have last_request after the first.
            have_last_request = true; // Just a safety net, should never hit.
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
                if (same) dup = true;
            }
        }
        else if (g.movement_type == "named")
        {
            if (last_was_named && (last_request_named == g.named_target))
            {
                dup = true;
            }
        }

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

        duplicates[i] = dup;
    }

    // Now we have a duplicates vector marking which goals are duplicates.
    // Next step: Plan only for non-duplicates.
    // We'll store planned trajectories and configs in temporary vectors, skipping duplicates.
    // After planning, we insert single-point for duplicates.

    std::vector<moveit_msgs::msg::RobotTrajectory> planned_trajectories(goals.size());
    std::vector<manymove_planner::msg::MovementConfig> planned_configs(goals.size());

    for (size_t i = 0; i < goals.size(); ++i)
    {
        if (duplicates[i])
        {
            // Skip planning now, handle later
            continue;
        }

        // Plan for this goal
        manymove_planner::action::MoveManipulator::Goal modified_goal = goals[i];

        // If not first and we want to set start_joint_values from previous planned if needed:
        // Actually, the old logic set start_joint_values from last_joint_values of the last planned trajectory.
        // Wait, we don't have last_joint_values from actual planning now since we decided to first identify duplicates.
        // Actually, we still can do a step-by-step approach:
        // The suggestion was to separate logic, but we need the last joint values from the previously planned trajectory to chain them.
        // We'll do a slight adjustment:
        // We'll keep track of the last planned trajectory's end joint state as we plan along non-duplicates.
        // For duplicates, we do not plan, so we do not update last_joint_values.

        // Find the last non-duplicate trajectory planned before i to set start_joint_values:
        if (i > 0)
        {
            // Search backwards for a planned non-duplicate trajectory
            int j = (int)i - 1;
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
                modified_goal.start_joint_values = last_valid_joints;
            }
        }

        auto [success, trajectory] = plan(modified_goal);
        if (!success)
        {
            RCLCPP_ERROR(logger_, "Planning failed for goal %zu. Returning empty.", i+1);
            return {{}, {}};
        }

        planned_trajectories[i] = trajectory;
        planned_configs[i] = modified_goal.config;
    }

    // Now insert single-point trajectories for duplicates:
    for (size_t i = 0; i < goals.size(); ++i)
    {
        if (duplicates[i])
        {
            // single-point trajectory
            moveit_msgs::msg::RobotTrajectory single_point_traj;
            single_point_traj.joint_trajectory.joint_names = move_group_interface_->getActiveJoints();

            trajectory_msgs::msg::JointTrajectoryPoint pt;
            // Use last known end state of previous planned trajectory:
            std::vector<double> last_jv;
            {
                int j = (int)i - 1;
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
                    // If we didn't find any previous planned non-duplicate,
                    // fallback to current joint state.
                    last_jv = move_group_interface_->getCurrentJointValues();
                }
            }

            pt.positions = last_jv;
            pt.time_from_start.sec = 0;
            pt.time_from_start.nanosec = 0;
            single_point_traj.joint_trajectory.points.push_back(pt);

            planned_trajectories[i] = single_point_traj;
            planned_configs[i] = goals[i].config; // Just use goal config
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

    bool first = true;
    std::vector<size_t> segment_sizes; // temporary

    for (size_t i = 0; i < trajectories.size(); ++i)
    {
        const auto &traj = trajectories[i];
        const auto &conf = configs[i];

        // Convert to RobotTrajectory for parameterization
        auto robot_traj_ptr = std::make_shared<robot_trajectory::RobotTrajectory>(
            move_group_interface_->getRobotModel(), move_group_interface_->getName());
        robot_traj_ptr->setRobotTrajectoryMsg(*move_group_interface_->getCurrentState(), traj);

        // Apply segment-level time parameterization
        if (!applyTimeParameterization(robot_traj_ptr, conf))
        {
            RCLCPP_ERROR(logger_, "Time parameterization failed for segment %zu", i+1);
            return {false, moveit_msgs::msg::RobotTrajectory()};
        }

        // Get the parameterized segment
        moveit_msgs::msg::RobotTrajectory segment;
        robot_traj_ptr->getRobotTrajectoryMsg(segment);

        // If not the first segment, remove the first waypoint if it duplicates the last of concatenated
        if (!first)
        {
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
                    // Remove first point of this segment
                    segment.joint_trajectory.points.erase(segment.joint_trajectory.points.begin());
                }
            }
        }

        // Append to concatenated
        size_t added_size = segment.joint_trajectory.points.size();
        concatenated.joint_trajectory.points.insert(
            concatenated.joint_trajectory.points.end(),
            segment.joint_trajectory.points.begin(),
            segment.joint_trajectory.points.end()
        );

        segment_sizes.push_back(added_size);
        if (first) first = false;
    }

    // Optional: Final smoothing over the entire concatenated trajectory if needed.
    // For now, assume the per-segment parameterization and joining is sufficient.

    sizes = segment_sizes;

    RCLCPP_INFO(logger_, "Successfully applied time parametrization sequence over %zu segments.", trajectories.size());
    return {true, concatenated};
}

