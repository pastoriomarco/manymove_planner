// manymove_planner.hpp

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <vector>
#include <utility> // For std::pair

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

// Trajectory processing includes
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

// Action message include
#include "manymove_planner/action/move_manipulator.hpp"
#include "manymove_planner/msg/movement_config.hpp"

class ManyMovePlanner
{
public:
    ManyMovePlanner(
        const rclcpp::Node::SharedPtr &node,
        const std::string &planning_group,
        const std::string &base_frame,
        const std::string &tcp_frame);

    bool isAtPoseTarget(const geometry_msgs::msg::Pose &target_pose, double tolerance = 1e-2) const;
    bool isAtJointTarget(const std::vector<double> &joint_values, double tolerance = 1e-2) const;
    bool isAtNamedTarget(const std::string &target_name, double tolerance = 1e-2) const;

    bool moveToPoseTarget(const geometry_msgs::msg::Pose &target_pose, const manymove_planner::msg::MovementConfig &config);
    bool moveToJointTarget(const std::vector<double> &joint_values, const manymove_planner::msg::MovementConfig &config);
    bool moveToNamedTarget(const std::string &target_name, const manymove_planner::msg::MovementConfig &config);
    bool moveCartesianPath(const geometry_msgs::msg::Pose &target_pose, const manymove_planner::msg::MovementConfig &config);

    bool findCollisionObject(const std::string &partial_id, moveit_msgs::msg::CollisionObject &found_object);

    /**
     * @brief Plans a trajectory based on the MoveManipulator action goal.
     *
     * This function centralizes the planning logic for different movement types.
     * It does not perform time parameterization or execution.
     *
     * @param goal The MoveManipulator action goal containing movement details.
     * @return A pair where the first element is a boolean indicating success,
     *         and the second element is the planned RobotTrajectory.
     */
    std::pair<bool, moveit_msgs::msg::RobotTrajectory> plan(const manymove_planner::action::MoveManipulator::Goal &goal);

    /**
     * @brief Plans a sequence of trajectories based on a vector of MoveManipulator action goals.
     *
     * This function plans each goal in sequence, using the last position of the previous trajectory as the start state for the next.
     * It collects all planned trajectories and their corresponding configurations.
     *
     * @param goals A vector of MoveManipulator::Goal, each containing movement type, config, and target.
     * @return A pair where the first element is a vector of RobotTrajectories,
     *         and the second element is a vector of their corresponding MovementConfigs.
     */
    std::pair<std::vector<moveit_msgs::msg::RobotTrajectory>, std::vector<manymove_planner::msg::MovementConfig>> planSequence(const std::vector<manymove_planner::action::MoveManipulator::Goal> &goals);

private:
    bool applyTimeParameterization(robot_trajectory::RobotTrajectoryPtr &trajectory, const manymove_planner::msg::MovementConfig &config);
    bool executeTrajectory(const robot_trajectory::RobotTrajectoryPtr &trajectory);
    double computePathLength(const moveit_msgs::msg::RobotTrajectory &trajectory) const;
    double computeMaxCartesianSpeed(const robot_trajectory::RobotTrajectoryPtr &trajectory) const;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_;
    std::string base_frame_;
    std::string tcp_frame_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};