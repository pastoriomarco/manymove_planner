#pragma once

#include "planner_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <future>
#include <map>
#include <tf2/LinearMath/Quaternion.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include "manymove_planner/msg/move_manipulator_goal.hpp"

#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <moveit/robot_model/robot_model.h>

/**
 * @class MoveGroupPlanner
 * @brief A planner class integrating with MoveGroupInterface for motion planning in ROS2.
 *
 * This class implements the PlannerInterface using MoveGroupInterface as the
 * underlying mechanism to plan and optionally execute trajectories via
 * FollowJointTrajectory. It provides utilities to handle multiple movement
 * types (joint, pose, named, cartesian).
 */
class MoveGroupPlanner : public PlannerInterface
{
public:
    using MoveManipulator = manymove_planner::action::MoveManipulator;
    using MoveManipulatorSequence = manymove_planner::action::MoveManipulatorSequence;
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
    using GoalHandleMoveManipulator = rclcpp_action::ServerGoalHandle<MoveManipulator>;
    using GoalHandleMoveManipulatorSequence = rclcpp_action::ServerGoalHandle<MoveManipulatorSequence>;

    /**
     * @brief Constructor for MoveGroupPlanner.
     * @param node Shared pointer to the ROS2 node.
     * @param planning_group Name of the planning group (as configured in MoveIt).
     * @param base_frame The base frame of the robot.
     * @param tcp_frame The tool center point (TCP) frame for the manipulator.
     * @param traj_controller Name of the trajectory controller to use for execution.
     */
    MoveGroupPlanner(
        const rclcpp::Node::SharedPtr &node,
        const std::string &planning_group,
        const std::string &base_frame,
        const std::string &tcp_frame,
        const std::string &traj_controller);

    /**
     * @brief Default destructor.
     */
    ~MoveGroupPlanner() override = default;

    /**
     * @brief Plan a trajectory for a single goal.
     * @param goal The MoveManipulator goal containing movement type and parameters.
     * @return A pair containing a success flag and the planned robot trajectory.
     */
    std::pair<bool, moveit_msgs::msg::RobotTrajectory> plan(const manymove_planner::action::MoveManipulator::Goal &goal) override;

    /**
     * @brief Plan a sequence of trajectories for multiple goals.
     * @param sequence_goal A list of MoveManipulator goals to plan sequentially.
     * @return A pair of vectors containing planned trajectories and their respective movement configs.
     */
    std::pair<std::vector<moveit_msgs::msg::RobotTrajectory>, std::vector<manymove_planner::msg::MovementConfig>> planSequence(
        const manymove_planner::action::MoveManipulatorSequence::Goal &sequence_goal) override;

    /**
     * @brief Execute a planned trajectory on the robot.
     * @param trajectory The trajectory to execute.
     * @return True if execution succeeded, false otherwise.
     */
    bool executeTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory) override;

    /**
     * @brief Execute a planned trajectory with feedback to an action server.
     * @param trajectory The trajectory to execute.
     * @param sizes The sizes (number of points) for each segment of the trajectory.
     * @param goal_handle The action server goal handle for providing feedback.
     * @return True if execution succeeded, false otherwise.
     */
    bool executeTrajectoryWithFeedback(
        const moveit_msgs::msg::RobotTrajectory &trajectory,
        const std::vector<size_t> &sizes,
        const std::shared_ptr<GoalHandleMoveManipulatorSequence> &goal_handle) override;

    /**
     * @brief Apply time parameterization to a sequence of trajectories.
     * @param trajectories The input trajectories to be time-parameterized.
     * @param configs Movement configurations for each trajectory segment.
     * @param sizes Output vector to store the number of points in each segment of the final combined trajectory.
     * @return A pair containing a success flag and the final time-parameterized, concatenated trajectory.
     */
    std::pair<bool, moveit_msgs::msg::RobotTrajectory> applyTimeParametrizationSequence(
        const std::vector<moveit_msgs::msg::RobotTrajectory> &trajectories,
        const std::vector<manymove_planner::msg::MovementConfig> &configs,
        std::vector<size_t> &sizes) override;

private:
    /**
     * @brief Compute the total path length of a trajectory.
     * @param trajectory The input trajectory message.
     * @return The computed path length, combining joint and Cartesian distance.
     */
    double computePathLength(const moveit_msgs::msg::RobotTrajectory &trajectory) const;

    /**
     * @brief Compute the maximum Cartesian speed within a trajectory.
     * @param trajectory A pointer to the robot trajectory to analyze.
     * @return The maximum speed in meters/second found along the trajectory.
     */
    double computeMaxCartesianSpeed(const robot_trajectory::RobotTrajectoryPtr &trajectory) const;

    /**
     * @brief Compare two sets of joint targets for equality within a tolerance.
     * @param j1 The first joint target vector.
     * @param j2 The second joint target vector.
     * @param tolerance The maximum allowed difference for each joint.
     * @return True if all joints match within the tolerance, false otherwise.
     */
    bool areSameJointTargets(const std::vector<double> &j1, const std::vector<double> &j2, double tolerance) const;

    /**
     * @brief Apply time parameterization to a single trajectory.
     * @param trajectory Pointer to the robot trajectory.
     * @param config Movement configuration specifying scaling factors and smoothing type.
     * @return True if time parameterization succeeded, false otherwise.
     */
    bool applyTimeParameterization(robot_trajectory::RobotTrajectoryPtr &trajectory, const manymove_planner::msg::MovementConfig &config);

    rclcpp::Node::SharedPtr node_; ///< Shared pointer to the ROS2 node.
    rclcpp::Logger logger_;        ///< Logger for output messages.
    std::string planning_group_;   ///< The planning group name in MoveIt.
    std::string base_frame_;       ///< Base frame of the robot.
    std::string tcp_frame_;        ///< Tool center point (TCP) frame for the manipulator.
    std::string traj_controller_;  ///< Name of the trajectory controller to be used.

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_; ///< Shared pointer to MoveGroupInterface.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;          ///< Interface to the planning scene.

    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr follow_joint_traj_client_; ///< Action client for FollowJointTrajectory.
};
