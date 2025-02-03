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
    : node_(node), logger_(node->get_logger()),
      planning_group_(planning_group),
      base_frame_(base_frame), tcp_frame_(tcp_frame),
      traj_controller_(traj_controller)
{
    moveit_cpp_ptr_ = std::make_shared<moveit_cpp::MoveItCpp>(node_);
    moveit_cpp_ptr_->getPlanningSceneMonitor()->providePlanningSceneService();

    /**
     * The following functions were used to publish the standard topics and services needed by the manymove_object_manager package.
     * Correctly configuring /config/moveit_cpp.yaml and the package launchers seems to offer the same results, while adding
     * these lines seems to compromise the package functionality in some contexts: for example, inside the current
     * NVIDIA Isaac ROS 3.1/3.2 docker container for Isaac Manipulator.
     */
    // // moveit_cpp_ptr_->getPlanningSceneMonitor()->requestPlanningSceneState("get_planning_scene");
    // moveit_cpp_ptr_->getPlanningSceneMonitor()->startSceneMonitor();
    // moveit_cpp_ptr_->getPlanningSceneMonitor()->startStateMonitor();
    // moveit_cpp_ptr_->getPlanningSceneMonitor()->startWorldGeometryMonitor("/collision_object", "/attached_collision_object", true);
    // // moveit_cpp_ptr_->getPlanningSceneMonitor()->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType::UPDATE_GEOMETRY);
    // moveit_cpp_ptr_->getPlanningSceneMonitor()->monitorDiffs(true);

    planning_components_ = std::make_shared<moveit_cpp::PlanningComponent>(planning_group_, moveit_cpp_ptr_);
    RCLCPP_INFO(logger_, "===================================================");
    RCLCPP_INFO(logger_, "MoveItCppPlanner initialized with group: %s", planning_group_.c_str());

    plan_parameters_.load(node_);

    RCLCPP_INFO(logger_, "===================================================");
    RCLCPP_INFO_STREAM(logger_, "plan_parameters_.max_acceleration_scaling_factor: " << plan_parameters_.max_acceleration_scaling_factor);
    RCLCPP_INFO_STREAM(logger_, "plan_parameters_.max_velocity_scaling_factor: " << plan_parameters_.max_velocity_scaling_factor);
    RCLCPP_INFO_STREAM(logger_, "plan_parameters_.planner_id: " << plan_parameters_.planner_id);
    RCLCPP_INFO_STREAM(logger_, "plan_parameters_.planning_attempts: " << plan_parameters_.planning_attempts);
    RCLCPP_INFO_STREAM(logger_, "plan_parameters_.planning_pipeline: " << plan_parameters_.planning_pipeline);
    RCLCPP_INFO_STREAM(logger_, "plan_parameters_.planning_time: " << plan_parameters_.planning_time);
    RCLCPP_INFO(logger_, "===================================================");

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
        std::bind(&MoveItCppPlanner::jointStateCallback, this, std::placeholders::_1));

    // For safety, initialize the maps as empty
    current_positions_.clear();
    current_velocities_.clear();
}

// functions to let ExecuteTrajectory in action_server.cpp handle the collision check feedback
rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr MoveItCppPlanner::getFollowJointTrajClient() const
{
    return follow_joint_traj_client_;
}

moveit::core::RobotModelConstPtr MoveItCppPlanner::getRobotModel() const
{
    return moveit_cpp_ptr_->getRobotModel();
}

std::string MoveItCppPlanner::getPlanningGroup() const
{
    return planning_group_;
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

    // Helper to compute Cartesian path length using TCP pose
    auto computeCartesianPathLength = [&](const moveit_msgs::msg::RobotTrajectory &traj) -> double
    {
        double length = 0.0;

        // Access the robot model
        auto robot_model = moveit_cpp_ptr_->getRobotModel();
        if (!robot_model)
        {
            RCLCPP_ERROR(logger_, "Robot model is null.");
            return 0.0;
        }

        // Create a robot state
        moveit::core::RobotState robot_state(robot_model);
        const auto &joint_model_group = robot_model->getJointModelGroup(planning_group_);
        if (!joint_model_group)
        {
            RCLCPP_ERROR(logger_, "Invalid joint model group.");
            return 0.0;
        }

        for (size_t i = 1; i < traj.joint_trajectory.points.size(); ++i)
        {
            // Set the previous and current joint values
            const auto &prev_point = traj.joint_trajectory.points[i - 1];
            const auto &curr_point = traj.joint_trajectory.points[i];

            robot_state.setJointGroupPositions(joint_model_group, prev_point.positions);
            const Eigen::Isometry3d prev_tcp_pose = robot_state.getGlobalLinkTransform(tcp_frame_);

            robot_state.setJointGroupPositions(joint_model_group, curr_point.positions);
            const Eigen::Isometry3d curr_tcp_pose = robot_state.getGlobalLinkTransform(tcp_frame_);

            // Calculate the Cartesian distance
            Eigen::Vector3d prev_position = prev_tcp_pose.translation();
            Eigen::Vector3d curr_position = curr_tcp_pose.translation();

            length += (curr_position - prev_position).norm();
        }

        return length;
    };

    // Compute both joint and Cartesian path lengths
    double joint_length = computeJointPathLength(trajectory);
    double cart_length = computeCartesianPathLength(trajectory);

    // Combine lengths (example: weight them as desired)
    double total_length = joint_length + (2 * cart_length);

    return total_length;
}

// Function to get a geometry_msgs::msg::Pose from a RobotState and frame
geometry_msgs::msg::Pose MoveItCppPlanner::getPoseFromRobotState(const moveit::core::RobotState &robot_state,
                                                                 const std::string &link_frame)
{

    // Clone the state to ensure the original state isn't modified
    moveit::core::RobotState state(robot_state);

    // Update link transforms to ensure they are valid
    state.updateLinkTransforms();

    geometry_msgs::msg::Pose pose;

    // Get the transform of the frame
    const Eigen::Isometry3d &pose_eigen = state.getGlobalLinkTransform(link_frame);

    // Extract position
    pose.position.x = pose_eigen.translation().x();
    pose.position.y = pose_eigen.translation().y();
    pose.position.z = pose_eigen.translation().z();

    // Extract orientation as a quaternion
    Eigen::Quaterniond quat(pose_eigen.rotation());
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();

    return pose;
}

// Function to compute the Euclidean distance between the start pose and the target pose
double MoveItCppPlanner::computeCartesianDistance(const geometry_msgs::msg::Pose &start_pose,
                                                  const geometry_msgs::msg::Pose &target_pose)
{
    // Compute the Euclidean distance to the target pose
    double dx = target_pose.position.x - start_pose.position.x;
    double dy = target_pose.position.y - start_pose.position.y;
    double dz = target_pose.position.z - start_pose.position.z;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Function to get a pose from a trajectory and TCP frame
geometry_msgs::msg::Pose MoveItCppPlanner::getPoseFromTrajectory(const moveit_msgs::msg::RobotTrajectory &traj_msg,
                                                                 const moveit::core::RobotState &robot_state,
                                                                 const std::string &link_frame,
                                                                 bool use_last_point)
{
    geometry_msgs::msg::Pose pose;

    // Ensure the trajectory is not empty
    if (traj_msg.joint_trajectory.points.empty())
    {
        throw std::runtime_error("Trajectory is empty, cannot extract pose.");
    }

    // Select the point to use (first or last)
    const auto &point = use_last_point ? traj_msg.joint_trajectory.points.back() : traj_msg.joint_trajectory.points.front();
    const auto &joint_names = traj_msg.joint_trajectory.joint_names;
    std::vector<double> joint_positions(point.positions.begin(), point.positions.end());

    // Clone the robot state to avoid modifying the original
    moveit::core::RobotState state(robot_state);

    // Update link transforms to ensure they are valid
    state.updateLinkTransforms();

    // Set the joint positions in the cloned state
    state.setVariablePositions(joint_names, joint_positions);

    // Get the pose of the TCP frame
    pose = getPoseFromRobotState(state, link_frame);

    return pose;
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

moveit_msgs::msg::RobotTrajectory MoveItCppPlanner::convertToMsg(const robot_trajectory::RobotTrajectory &trajectory) const
{
    moveit_msgs::msg::RobotTrajectory traj_msg;
    trajectory.getRobotTrajectoryMsg(traj_msg);
    return traj_msg;
}

std::pair<bool, moveit_msgs::msg::RobotTrajectory> MoveItCppPlanner::plan(const manymove_planner::action::MoveManipulator::Goal &goal_msg)
{
    std::vector<std::pair<moveit_msgs::msg::RobotTrajectory, double>> trajectories;
    auto robot_model_ptr = moveit_cpp_ptr_->getRobotModel();
    auto robot_start_state_ptr = planning_components_->getStartState();
    auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(planning_group_);

    // Handle start state
    if (!goal_msg.goal.start_joint_values.empty())
    {
        moveit::core::RobotState start_state(*moveit_cpp_ptr_->getCurrentState());
        start_state.setJointGroupPositions(joint_model_group_ptr, goal_msg.goal.start_joint_values);
        planning_components_->setStartState(start_state);
    }
    else
    {
        planning_components_->setStartStateToCurrentState();
    }

    /** Create PlanRequestParameters and set scaling factors
     * moveit_cpp::PlanningComponent::PlanRequestParameters params;
     * params.planning_pipeline = "ompl"; // or the name of your pipeline from your moveit_cpp.yaml
     * params.max_velocity_scaling_factor = goal_msg.goal.config.velocity_scaling_factor;
     * params.max_acceleration_scaling_factor = goal_msg.goal.config.acceleration_scaling_factor;
     * IMPORTANT NOTE: You can set other parameters in `params` as needed, but remember that if you
     * set even one parameter you will have to set all of them. When I last tried, setting one
     * parameter had as consequence that none of the default parameters is used anymore.
     */

    // Set movement targets
    if ((goal_msg.goal.movement_type == "joint") || (goal_msg.goal.movement_type == "named"))
    {
        if (goal_msg.goal.movement_type == "joint")
        {
            RCLCPP_INFO_STREAM(logger_, "Setting joint target");
            moveit::core::RobotState goal_state(robot_model_ptr);
            goal_state.setJointGroupPositions(joint_model_group_ptr, goal_msg.goal.joint_values);
            planning_components_->setGoal(goal_state);
        }
        else if (goal_msg.goal.movement_type == "named")
        {
            RCLCPP_INFO_STREAM(logger_, "Setting named target " << goal_msg.goal.named_target);
            planning_components_->setGoal(goal_msg.goal.named_target);
        }

        // Plan multiple trajectories using the plan(params) method
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
                RCLCPP_INFO_STREAM(logger_, "Calculated traj length: " << length);
            }
            else
            {
                RCLCPP_WARN(logger_, "%s target planning attempt %d failed.",
                            goal_msg.goal.movement_type.c_str(), attempts + 1);
            }
            attempts++;
        }
    }
    else if (goal_msg.goal.movement_type == "pose")
    {

        RCLCPP_INFO_STREAM(logger_, "Setting pose target for " << tcp_frame_);
        // geometry_msgs::msg::PoseStamped target_pose;
        // target_pose.pose = goal_msg.goal.pose_target;
        // target_pose.header.frame_id = tcp_frame_;
        // planning_components_->setGoal(target_pose, tcp_frame_);

        // Plan multiple trajectories using the plan(params) method
        int attempts = 0;
        while (attempts < goal_msg.goal.config.plan_number_limit &&
               static_cast<int>(trajectories.size()) < goal_msg.goal.config.plan_number_target)
        {

            auto target_state = *robot_start_state_ptr;
            target_state.setFromIK(joint_model_group_ptr, goal_msg.goal.pose_target);
            planning_components_->setGoal(target_state);

            auto solution = planning_components_->plan();

            if (solution && solution.error_code == moveit::core::MoveItErrorCode::SUCCESS)
            {
                moveit_msgs::msg::RobotTrajectory traj_msg;
                solution.trajectory->getRobotTrajectoryMsg(traj_msg);
                double length = computePathLength(traj_msg);

                if (traj_msg.joint_trajectory.points.empty())
                {
                    RCLCPP_WARN(logger_, "%s target planning attempt %d failed: trajectory is empty.",
                                goal_msg.goal.movement_type.c_str(), attempts + 1);
                    break; // the traj is empty
                }

                // I'm having trouble with plans succeeding with empty or almost empty trajectories
                // The following checks are to verify that the trajectory makes sense

                // auto start_pose = getPoseFromRobotState(*robot_start_state_ptr, tcp_frame_);
                // auto traj_start_pose = getPoseFromTrajectory(traj_msg, *robot_start_state_ptr, tcp_frame_, false);
                auto traj_end_pose = getPoseFromTrajectory(traj_msg, *robot_start_state_ptr, tcp_frame_, true);

                // double min_euclidean_distance = computeCartesianDistance(start_pose, goal_msg.goal.pose_target);
                // double traj_euclidean_distance = computeCartesianDistance(traj_start_pose, traj_end_pose);
                // double starts_euclidean_distance = computeCartesianDistance(start_pose, traj_start_pose);
                double targets_euclidean_distance = computeCartesianDistance(traj_end_pose, goal_msg.goal.pose_target);

                // RCLCPP_INFO_STREAM(logger_, "Minimum theoretical eclidean distance between start pose and target pose: " << min_euclidean_distance);
                // RCLCPP_INFO_STREAM(logger_, "Euclidean distance between trajectory first point and trajectory last point: " << traj_euclidean_distance);
                // RCLCPP_INFO_STREAM(logger_, "Eclidean distance between theoretical start and calculated trajectory first point: " << starts_euclidean_distance);

                double traj_tolerance = 0.001;

                if (targets_euclidean_distance < traj_tolerance)
                {

                    trajectories.emplace_back(traj_msg, length);
                    RCLCPP_INFO_STREAM(logger_, "Calculated pose traj length: " << length);
                    RCLCPP_INFO(logger_, "moveit::core::MoveItErrorCode = %d.",
                                solution.error_code.val);
                }
                else
                {
                    RCLCPP_WARN_STREAM(logger_, "Eclidean distance between theoretical target and calculated trajectory last point: " << targets_euclidean_distance);

                    RCLCPP_WARN_STREAM(logger_, "The planner was not able to calculate trajectory with end point within tolerance." << targets_euclidean_distance);
                    RCLCPP_WARN(logger_, "%s target planning attempt %d failed: trajectory is empty.",
                                goal_msg.goal.movement_type.c_str(), attempts + 1);
                }
            }
            else
            {
                RCLCPP_WARN(logger_, "%s target planning attempt %d failed.",
                            goal_msg.goal.movement_type.c_str(), attempts + 1);
                RCLCPP_WARN(logger_, "moveit::core::MoveItErrorCode = %d.",
                            solution.error_code.val);
            }
            attempts++;
        }
    }
    else if (goal_msg.goal.movement_type == "cartesian")
    {

        int attempts = 0;
        while (attempts < goal_msg.goal.config.plan_number_limit &&
               static_cast<int>(trajectories.size()) < goal_msg.goal.config.plan_number_target)
        {
            RCLCPP_INFO(logger_, "Cartesian path planning attempt %d with step size %.3f, jump threshold %.3f",
                        attempts + 1, goal_msg.goal.config.step_size, goal_msg.goal.config.jump_threshold);

            // Handle start state
            if (!goal_msg.goal.start_joint_values.empty())
            {
                moveit::core::RobotState start_state(*moveit_cpp_ptr_->getCurrentState());
                start_state.setJointGroupPositions(joint_model_group_ptr, goal_msg.goal.start_joint_values);
                planning_components_->setStartState(start_state);
            }
            else
            {
                planning_components_->setStartStateToCurrentState();
            }

            // Get the initial robot state
            auto start_state = planning_components_->getStartState();

            // Get the end-effector link model
            const moveit::core::LinkModel *ee_link = joint_model_group_ptr->getLinkModel(tcp_frame_);

            // Retrieve the global pose of the end-effector at the start
            Eigen::Isometry3d start_pose = start_state->getGlobalLinkTransform(ee_link);
            // Assume that waypoints is filled with one or more geometry_msgs::msg::Pose
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(goal_msg.goal.pose_target);

            // Convert geometry_msgs::msg::Pose to Eigen::Isometry3d
            EigenSTL::vector_Isometry3d eigen_waypoints;
            eigen_waypoints.push_back(start_pose);

            for (const auto &wp : waypoints)
            {
                Eigen::Isometry3d eigen_pose;
                tf2::fromMsg(wp, eigen_pose);
                eigen_waypoints.push_back(eigen_pose);
            }

            std::vector<moveit::core::RobotStatePtr> trajectory_states;

            // // Use the waypoint-based overload
            // double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
            //     start_state.get(),
            //     joint_model_group_ptr,
            //     trajectory_states,
            //     ee_link,
            //     eigen_waypoints,
            //     true,
            //     moveit::core::MaxEEFStep(goal_msg.goal.config.step_size),
            //     moveit::core::JumpThreshold(goal_msg.goal.config.jump_threshold),
            //     moveit::core::GroupStateValidityCallbackFn(),
            //     kinematics::KinematicsQueryOptions(),
            //     nullptr);
            moveit::core::GroupStateValidityCallbackFn custom_callback =
                std::bind(&MoveItCppPlanner::isStateValid, this, std::placeholders::_1, std::placeholders::_2);

            double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
                start_state.get(),
                joint_model_group_ptr,
                trajectory_states,
                ee_link,
                eigen_waypoints,
                true,
                moveit::core::MaxEEFStep(goal_msg.goal.config.step_size),
                moveit::core::JumpThreshold(goal_msg.goal.config.jump_threshold),
                custom_callback,
                kinematics::KinematicsQueryOptions(),
                nullptr);

            if (fraction >= 1.0)
            {

                RCLCPP_INFO_STREAM(logger_, "trajectory_states length: " << trajectory_states.size());
                // Construct a RobotTrajectory from the computed states
                robot_trajectory::RobotTrajectory robot_trajectory(robot_model_ptr, planning_group_);
                for (const auto &state : trajectory_states)
                {
                    robot_trajectory.addSuffixWayPoint(*state, 0.0);
                }

                // Convert to message
                moveit_msgs::msg::RobotTrajectory traj_msg = convertToMsg(robot_trajectory);
                double length = computePathLength(traj_msg);
                trajectories.emplace_back(traj_msg, length);
                RCLCPP_INFO_STREAM(logger_, "Calculated traj length: " << length);
            }
            else
            {
                RCLCPP_WARN(logger_, "Cartesian path planning attempt %d failed (%.2f%% achieved)",
                            attempts + 1, fraction * 100.0);
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

// Apply Time Parameterization
bool MoveItCppPlanner::applyTimeParameterization(
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
            // Adjust acceleration similarly
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

bool MoveItCppPlanner::executeTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory)
{
    // 1) Basic checks
    if (trajectory.joint_trajectory.points.empty())
    {
        RCLCPP_ERROR(logger_, "Empty trajectory, aborting (MoveItCppPlanner).");
        return false;
    }

    if (!follow_joint_traj_client_)
    {
        RCLCPP_ERROR(logger_, "FollowJointTrajectory action client not initialized! (MoveItCppPlanner)");
        return false;
    }

    // 2) Optionally wait for server
    if (!follow_joint_traj_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(logger_, "FollowJointTrajectory server not available (MoveItCppPlanner).");
        return false;
    }

    // 3) Create goal
    control_msgs::action::FollowJointTrajectory::Goal goal_msg;
    goal_msg.trajectory = trajectory.joint_trajectory;

    RCLCPP_INFO(logger_, "Sending FollowJointTrajectory goal (MoveItCppPlanner) ...");

    // 4) Prepare promise/future
    auto result_promise = std::make_shared<std::promise<bool>>();
    std::future<bool> result_future = result_promise->get_future();

    // 5) SendGoalOptions
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions options;

    // (a) Optional feedback
    options.feedback_callback =
        [this](auto, auto feedback)
    {
        RCLCPP_DEBUG(logger_, "Partial execution (MoveItCppPlanner): time_from_start %.2f",
                     rclcpp::Duration(feedback->actual.time_from_start).seconds());
    };

    // (b) Result callback
    options.result_callback =
        [this, result_promise](const auto &wrapped_result)
    {
        bool success = false;
        switch (wrapped_result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(logger_, "FollowJointTrajectory succeeded (MoveItCppPlanner).");
            success = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(logger_, "FollowJointTrajectory was aborted (MoveItCppPlanner).");
            success = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(logger_, "FollowJointTrajectory canceled (MoveItCppPlanner).");
            success = false;
            break;
        default:
            RCLCPP_ERROR(logger_, "Unknown result from FollowJointTrajectory (MoveItCppPlanner).");
            success = false;
            break;
        }
        result_promise->set_value(success);
    };

    // 6) Send the goal
    auto goal_handle_future = follow_joint_traj_client_->async_send_goal(goal_msg, options);

    // 7) Wait for the goal handle
    if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
        RCLCPP_ERROR(logger_, "Goal handle not received in time (MoveItCppPlanner).");
        return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(logger_, "FollowJointTrajectory goal rejected (MoveItCppPlanner).");
        return false;
    }

    // 8) Wait for result via future
    RCLCPP_INFO(logger_, "Waiting for FollowJointTrajectory result (MoveItCppPlanner) ...");
    auto status = result_future.wait_for(std::chrono::seconds(300));
    if (status != std::future_status::ready)
    {
        RCLCPP_ERROR(logger_, "Trajectory execution timed out (MoveItCppPlanner).");
        return false;
    }

    bool exec_success = result_future.get();
    if (!exec_success)
    {
        RCLCPP_ERROR(logger_, "Trajectory execution failed (MoveItCppPlanner).");
        return false;
    }

    RCLCPP_INFO(logger_, "Trajectory execution succeeded (MoveItCppPlanner).");
    return true;
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
    // Verify that the number of trajectories matches the number of configurations
    if (trajectories.size() != configs.size())
    {
        RCLCPP_ERROR(logger_, "Mismatched trajectories and configs size in applyTimeParametrizationSequence.");
        return {false, moveit_msgs::msg::RobotTrajectory()};
    }

    // Clear any existing sizes
    sizes.clear();

    // Check if trajectories are empty
    if (trajectories.empty())
    {
        RCLCPP_ERROR(logger_, "No trajectories provided to applyTimeParametrizationSequence.");
        return {false, moveit_msgs::msg::RobotTrajectory()};
    }

    // Extract the standard joint names from the first trajectory
    std::vector<std::string> standard_joint_names = trajectories[0].joint_trajectory.joint_names;

    // Initialize the concatenated trajectory with the standard joint names
    moveit_msgs::msg::RobotTrajectory concatenated;
    concatenated.joint_trajectory.joint_names = standard_joint_names;

    // Initialize cumulative time to adjust time_from_start for concatenated trajectory points
    double cumulative_time = 0.0;

    // Iterate through each trajectory segment and apply time parameterization
    for (size_t i = 0; i < trajectories.size(); ++i)
    {
        const auto &traj = trajectories[i];
        const auto &conf = configs[i];

        // Verify that the current trajectory's joint names match the standard joint names
        if (traj.joint_trajectory.joint_names != standard_joint_names)
        {
            RCLCPP_ERROR(logger_,
                         "Joint names mismatch in trajectory segment %zu. Expected %zu joints, got %zu joints.",
                         i + 1, standard_joint_names.size(), traj.joint_trajectory.joint_names.size());
            return {false, moveit_msgs::msg::RobotTrajectory()};
        }

        // Convert the trajectory message to a RobotTrajectory object for manipulation
        auto robot_traj_ptr = std::make_shared<robot_trajectory::RobotTrajectory>(
            moveit_cpp_ptr_->getRobotModel(), planning_components_->getPlanningGroupName());
        robot_traj_ptr->setRobotTrajectoryMsg(*moveit_cpp_ptr_->getCurrentState(), traj);

        // Apply time parameterization based on the provided configuration
        if (!applyTimeParameterization(robot_traj_ptr, conf))
        {
            RCLCPP_ERROR(logger_, "Time parameterization failed for segment %zu", i + 1);
            return {false, moveit_msgs::msg::RobotTrajectory()};
        }

        // Convert the time-parameterized RobotTrajectory back to a message
        moveit_msgs::msg::RobotTrajectory segment;
        robot_traj_ptr->getRobotTrajectoryMsg(segment);

        // Verify that the joint names in the segment match the standard joint names
        if (segment.joint_trajectory.joint_names != standard_joint_names)
        {
            RCLCPP_ERROR(logger_,
                         "After time parameterization, joint names in segment %zu do not match the standard joint names.",
                         i + 1);
            return {false, moveit_msgs::msg::RobotTrajectory()};
        }

        // Remove the first point of the segment if it's identical to the last point of the concatenated trajectory
        if (!concatenated.joint_trajectory.points.empty() && !segment.joint_trajectory.points.empty())
        {
            const auto &prev_last = concatenated.joint_trajectory.points.back().positions;
            const auto &current_first = segment.joint_trajectory.points.front().positions;

            bool identical = true;
            if (prev_last.size() == current_first.size())
            {
                for (size_t idx = 0; idx < prev_last.size(); ++idx)
                {
                    if (std::abs(prev_last[idx] - current_first[idx]) > 1e-6)
                    {
                        identical = false;
                        break;
                    }
                }
            }
            else
            {
                identical = false;
            }

            if (identical)
            {
                RCLCPP_INFO(logger_, "Removing duplicate first point of segment %zu.", i + 1);
                segment.joint_trajectory.points.erase(segment.joint_trajectory.points.begin());
            }
        }

        // Iterate through each point in the segment and ensure only standard joints are included
        std::vector<trajectory_msgs::msg::JointTrajectoryPoint> filtered_points;
        for (const auto &point : segment.joint_trajectory.points)
        {
            // Verify that the number of positions matches the number of joint names
            if (point.positions.size() != standard_joint_names.size())
            {
                RCLCPP_ERROR(logger_,
                             "Mismatch in number of positions and joint names in segment %zu: "
                             "%zu positions vs %zu joint names.",
                             i + 1, point.positions.size(), standard_joint_names.size());
                return {false, moveit_msgs::msg::RobotTrajectory()};
            }

            // Create a new JointTrajectoryPoint with only the standard joint positions
            trajectory_msgs::msg::JointTrajectoryPoint filtered_point;
            filtered_point.positions = point.positions;

            // Adjust time_from_start by adding cumulative_time to maintain correct timing
            double original_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
            double new_time = original_time + cumulative_time;
            filtered_point.time_from_start.sec = static_cast<int>(new_time);
            filtered_point.time_from_start.nanosec = static_cast<int>((new_time - static_cast<int>(new_time)) * 1e9);

            filtered_points.push_back(filtered_point);
        }

        // Update cumulative_time based on the last point of the filtered segment
        if (!filtered_points.empty())
        {
            const auto &last_point = filtered_points.back();
            cumulative_time = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;
        }

        // Append the filtered points to the concatenated trajectory
        size_t added_size = filtered_points.size();
        concatenated.joint_trajectory.points.insert(
            concatenated.joint_trajectory.points.end(),
            filtered_points.begin(),
            filtered_points.end());

        // Record the number of points added for each segment
        sizes.push_back(added_size);
    }

    // Log the successful application of the time parameterization sequence
    RCLCPP_INFO(logger_, "Successfully applied time parametrization sequence over %zu segments.", trajectories.size());

    // Return the concatenated trajectory containing only the standard joints
    return {true, concatenated};
}

std::pair<std::vector<moveit_msgs::msg::RobotTrajectory>, std::vector<manymove_planner::msg::MovementConfig>>
MoveItCppPlanner::planSequence(const manymove_planner::action::MoveManipulatorSequence::Goal &sequence_goal)
{
    std::vector<moveit_msgs::msg::RobotTrajectory> planned_trajectories;
    std::vector<manymove_planner::msg::MovementConfig> planned_configs;

    if (sequence_goal.goals.empty())
    {
        RCLCPP_WARN(logger_, "planSequence called with an empty goals vector.");
        return {planned_trajectories, planned_configs};
    }

    std::vector<double> previous_end_joint_targets;
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
                auto current_state = moveit_cpp_ptr_->getCurrentState();
                previous_end_joint_targets = std::vector<double>(
                    current_state->getVariablePositions(),
                    current_state->getVariablePositions() + current_state->getVariableCount());

                RCLCPP_INFO(logger_, "Initialized previous_end_joint_targets from the current robot pose.");
            }
        }
        else
        {
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

        // Check if the trajectory is too small to be meaningful
        if (!trajectory.joint_trajectory.points.empty() &&
            areSameJointTargets(trajectory.joint_trajectory.points.back().positions, previous_end_joint_targets, pose_tolerance))
        {
            RCLCPP_WARN(logger_, "Resulting trajectory too small for move %zu, skipping.", i + 1);
        }
        else
        {
            planned_trajectories.push_back(trajectory);
            planned_configs.push_back(modified_goal.goal.config);
        }
    }

    RCLCPP_INFO(logger_, "Successfully planned sequence with %zu steps.", planned_trajectories.size());
    return {planned_trajectories, planned_configs};
}

bool MoveItCppPlanner::isStateValid(const moveit::core::RobotState *state,
                                    const moveit::core::JointModelGroup *group) const
{
    // 1. Set up collision request & result
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    // Enable contact checks (if you need detailed contact info)
    collision_request.contacts = true;
    collision_request.max_contacts = 10;

    // **Key Line**: Limit collision checks to the JointModelGroup
    collision_request.group_name = group->getName();

    // 2. Access the planning scene via the PlanningSceneMonitor
    auto psm = moveit_cpp_ptr_->getPlanningSceneMonitor();
    if (!psm)
    {
        RCLCPP_ERROR(logger_, "PlanningSceneMonitor is null. Cannot perform collision checking.");
        return false;
    }

    planning_scene_monitor::LockedPlanningSceneRO locked_scene(psm);
    if (!locked_scene)
    {
        RCLCPP_ERROR(logger_, "LockedPlanningSceneRO is null. Cannot perform collision checking.");
        return false;
    }

    // 3. Clone the given RobotState to update transforms if needed
    moveit::core::RobotState temp_state(*state);
    // Make sure transforms in temp_state are up-to-date
    temp_state.update(); // or temp_state.updateLinkTransforms();

    // 4. Perform the collision check
    locked_scene->checkCollision(collision_request, collision_result, temp_state);

    // 5. Log if a collision is detected
    if (collision_result.collision)
    {
        RCLCPP_WARN(logger_, "Collision detected in custom validity callback for group '%s'.",
                    group->getName().c_str());
    }

    // Return true if no collision, false otherwise
    return !collision_result.collision;
}

bool MoveItCppPlanner::sendControlledStop(double deceleration_time)
{
    RCLCPP_INFO(logger_, "Constructing a short 'controlled stop' trajectory (%.2fs) [SINGLE-POINT].",
                deceleration_time);

    // 1) Make sure the FollowJointTrajectory action server is up
    if (!follow_joint_traj_client_->wait_for_action_server(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(logger_, "Cannot send stop trajectory, FollowJointTrajectory server is not available.");
        return false;
    }

    // 2) Get current robot state
    auto current_state = moveit_cpp_ptr_->getCurrentState();
    if (!current_state)
    {
        RCLCPP_ERROR(logger_, "Failed to get current robot state for stop trajectory.");
        return false;
    }
    const auto &joint_model_group = current_state->getJointModelGroup(planning_group_);
    if (!joint_model_group)
    {
        RCLCPP_ERROR(logger_, "JointModelGroup '%s' not found.", planning_group_.c_str());
        return false;
    }

    // Grab joint names and positions
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
    std::vector<double> positions;
    current_state->copyJointGroupPositions(joint_model_group, positions);

    // If we can’t read actual velocities, just assume 0
    std::vector<double> velocities(positions.size(), 0.0);

    // 3) Build a SINGLE-point trajectory
    control_msgs::action::FollowJointTrajectory::Goal stop_goal;
    stop_goal.trajectory.joint_names = joint_names;

    // Only Point: current position, zero velocity
    // time_from_start can be 0.0 if your controller is okay with that,
    // or something small like 0.1. Right now it is left to the user to
    // decide, but the default is 0.25
    trajectory_msgs::msg::JointTrajectoryPoint p0;
    p0.positions = positions;
    p0.velocities = velocities;
    p0.accelerations.resize(positions.size(), 0.0);

    // Use deceleration_time to define how long we give the controller to ramp to zero velocity.
    // A larger deceleration_time will produce a smoother (but slower) stop.
    p0.time_from_start = rclcpp::Duration::from_seconds(deceleration_time);

    stop_goal.trajectory.points.push_back(p0);

    // 4) Send the new "stop" goal
    RCLCPP_INFO(logger_, "Sending single-point 'stop' trajectory.");

    auto send_goal_future = follow_joint_traj_client_->async_send_goal(stop_goal);
    if (send_goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
        RCLCPP_ERROR(logger_, "Timeout while sending stop trajectory goal.");
        return false;
    }

    auto goal_handle = send_goal_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(logger_, "Stop trajectory goal was rejected by the joint trajectory server.");
        return false;
    }

    // Wait for the result (optional)
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
        RCLCPP_INFO(logger_, "Single-point stop completed successfully.");
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

bool MoveItCppPlanner::sendControlledStopLinear(double deceleration_time)
{
    RCLCPP_INFO(logger_, "[MoveItCppPlanner] Constructing a short 'controlled stop' trajectory (%.2fs).", deceleration_time);

    // 1) Wait for the FollowJointTrajectory action server
    if (!follow_joint_traj_client_->wait_for_action_server(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(logger_, "[MoveItCppPlanner] Cannot send stop trajectory, FollowJointTrajectory server is not available.");
        return false;
    }

    // 2) Collect current joint positions & velocities for the planning_group_
    //    from the stored joint_states data
    auto current_state = moveit_cpp_ptr_->getCurrentState();
    if (!current_state)
    {
        RCLCPP_ERROR(logger_, "[MoveItCppPlanner] No current robot state available!");
        return false;
    }

    const moveit::core::JointModelGroup *jmg = current_state->getJointModelGroup(planning_group_);
    if (!jmg)
    {
        RCLCPP_ERROR(logger_, "[MoveItCppPlanner] JointModelGroup '%s' not found.", planning_group_.c_str());
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
    RCLCPP_INFO(logger_, "[MoveItCppPlanner] Sending single-point 'stop' trajectory with final time %.2fs.", deceleration_time);

    auto send_goal_future = follow_joint_traj_client_->async_send_goal(stop_goal);
    if (send_goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
        RCLCPP_ERROR(logger_, "[MoveItCppPlanner] Timeout while sending stop trajectory goal.");
        return false;
    }

    auto goal_handle = send_goal_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(logger_, "[MoveItCppPlanner] Stop trajectory goal was rejected by the joint trajectory server.");
        return false;
    }

    // 6) Optionally wait for the result
    auto result_future = follow_joint_traj_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    {
        RCLCPP_ERROR(logger_, "[MoveItCppPlanner] Controlled stop goal did not finish before timeout.");
        return false;
    }

    auto wrapped_result = result_future.get();
    switch (wrapped_result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(logger_, "[MoveItCppPlanner] Single-point stop completed successfully.");
        return true;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(logger_, "[MoveItCppPlanner] Stop goal was aborted by the controller.");
        return false;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(logger_, "[MoveItCppPlanner] Stop goal was canceled by the controller.");
        return false;
    default:
        RCLCPP_ERROR(logger_, "[MoveItCppPlanner] Stop goal ended with unknown result code %d.", (int)wrapped_result.code);
        return false;
    }
}

void MoveItCppPlanner::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
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