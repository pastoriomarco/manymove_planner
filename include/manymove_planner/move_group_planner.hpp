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
 * @class MoveItCppPlanner
 * @brief A planner class that integrates with MoveGroup for motion planning in ROS2.
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
     * @brief Constructor for MoveGroupPlanner
     * @param node Shared pointer to the ROS2 node
     * @param planning_group Name of the planning group
     * @param base_frame The base frame of the robot
     * @param tcp_frame Tool Center Point (TCP) frame for the manipulator
     * @param traj_controller Name of the trajectory controller
     */
    MoveGroupPlanner(
        const rclcpp::Node::SharedPtr &node,
        const std::string &planning_group,
        const std::string &base_frame,
        const std::string &tcp_frame,
        const std::string &traj_controller);

    ~MoveGroupPlanner() override = default;

    /**
     * @brief Plan a trajectory for a given goal
     * @param goal The goal for the manipulator
     * @return A pair containing a success flag and the planned robot trajectory
     */
    std::pair<bool, moveit_msgs::msg::RobotTrajectory> plan(const manymove_planner::action::MoveManipulator::Goal &goal) override;

    /**
     * @brief Plan a sequence of trajectories
     * @param sequence_goal The sequence of goals for the manipulator
     * @return A pair containing the planned trajectories and associated movement configurations
     */
    std::pair<std::vector<moveit_msgs::msg::RobotTrajectory>, std::vector<manymove_planner::msg::MovementConfig>> planSequence(
        const manymove_planner::action::MoveManipulatorSequence::Goal &sequence_goal) override;

    /**
     * @brief Execute a planned trajectory
     * @param trajectory The trajectory to execute
     * @return True if execution was successful, false otherwise
     */
    bool executeTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory) override;

    /**
     * @brief Execute a planned trajectory with feedback
     * @param trajectory The trajectory to execute
     * @param sizes Vector of trajectory segment sizes
     * @param goal_handle The goal handle for the action server
     * @return True if execution was successful, false otherwise
     */
    bool executeTrajectoryWithFeedback(
        const moveit_msgs::msg::RobotTrajectory &trajectory,
        const std::vector<size_t> &sizes,
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<manymove_planner::action::MoveManipulatorSequence>> &goal_handle) override;

    /**
     * @brief Apply time parameterization to a sequence of trajectories
     * @param trajectories The input trajectories
     * @param configs Movement configurations for each trajectory
     * @param sizes Output sizes of each trajectory segment
     * @return A pair containing a success flag and the time-parameterized trajectory
     */
    std::pair<bool, moveit_msgs::msg::RobotTrajectory> applyTimeParametrizationSequence(
        const std::vector<moveit_msgs::msg::RobotTrajectory> &trajectories,
        const std::vector<manymove_planner::msg::MovementConfig> &configs,
        std::vector<size_t> &sizes) override;

private:
    /**
     * @brief Compute the total path length of a trajectory
     * @param trajectory The robot trajectory
     * @return The computed path length
     */
    double computePathLength(const moveit_msgs::msg::RobotTrajectory &trajectory) const;

    /**
     * @brief Compute the maximum Cartesian speed of a trajectory
     * @param trajectory The robot trajectory
     * @return The maximum Cartesian speed
     */
    double computeMaxCartesianSpeed(const robot_trajectory::RobotTrajectoryPtr &trajectory) const;

    /**
     * @brief Compare two joint targets for equality within a tolerance
     * @param j1 First joint target
     * @param j2 Second joint target
     * @param tolerance The tolerance for comparison
     * @return True if the joint targets are the same within the tolerance, false otherwise
     */
    bool areSameJointTargets(const std::vector<double> &j1, const std::vector<double> &j2, double tolerance) const;

    /**
     * @brief Apply time parameterization to a single trajectory
     * @param trajectory The robot trajectory to parameterize
     * @param config The movement configuration for time parameterization
     * @return True if successful, false otherwise
     */
    bool applyTimeParameterization(robot_trajectory::RobotTrajectoryPtr &trajectory, const manymove_planner::msg::MovementConfig &config);

    rclcpp::Node::SharedPtr node_; ///< Shared pointer to the ROS2 node
    rclcpp::Logger logger_; ///< Logger instance for logging messages
    std::string base_frame_; ///< The base frame of the robot
    std::string tcp_frame_; ///< The tool center point (TCP) frame
    std::string traj_controller_; ///< Name of the trajectory controller

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_; ///< Shared pointer to the MoveGroupInterface instance
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;///< Shared pointer to the PlanningSceneInterface instance

    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr follow_joint_traj_client_; ///< Action client for FollowJointTrajectory
};
