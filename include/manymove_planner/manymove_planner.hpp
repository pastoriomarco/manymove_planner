#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <vector>
#include <utility>

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

    // Check if at targets
    bool isAtPoseTarget(const geometry_msgs::msg::Pose &target_pose, double tolerance = 1e-2) const;
    bool isAtJointTarget(const std::vector<double> &joint_values, double tolerance = 1e-2) const;
    bool isAtNamedTarget(const std::string &target_name, double tolerance = 1e-2) const;

    // Single movement commands
    bool moveToPoseTarget(const geometry_msgs::msg::Pose &target_pose, const manymove_planner::msg::MovementConfig &config);
    bool moveToJointTarget(const std::vector<double> &joint_values, const manymove_planner::msg::MovementConfig &config);
    bool moveToNamedTarget(const std::string &target_name, const manymove_planner::msg::MovementConfig &config);
    bool moveCartesianPath(const geometry_msgs::msg::Pose &target_pose, const manymove_planner::msg::MovementConfig &config);

    bool findCollisionObject(const std::string &partial_id, moveit_msgs::msg::CollisionObject &found_object);

    // Plan a single move
    std::pair<bool, moveit_msgs::msg::RobotTrajectory> plan(const manymove_planner::action::MoveManipulator::Goal &goal);

    // Plan a sequence of moves
    std::pair<std::vector<moveit_msgs::msg::RobotTrajectory>, std::vector<manymove_planner::msg::MovementConfig>> planSequence(
        const std::vector<manymove_planner::action::MoveManipulator::Goal> &goals);

    // Apply time parametrization and combine into one smoothed trajectory
    std::pair<bool, moveit_msgs::msg::RobotTrajectory> applyTimeParametrizationSequence(
        const std::vector<moveit_msgs::msg::RobotTrajectory> &trajectories,
        const std::vector<manymove_planner::msg::MovementConfig> &configs,
        std::vector<size_t> &sizes);

private:
    bool applyTimeParameterization(robot_trajectory::RobotTrajectoryPtr &trajectory, const manymove_planner::msg::MovementConfig &config);
    bool executeTrajectory(const robot_trajectory::RobotTrajectoryPtr &trajectory);
    double computePathLength(const moveit_msgs::msg::RobotTrajectory &trajectory) const;
    double computeMaxCartesianSpeed(const robot_trajectory::RobotTrajectoryPtr &trajectory) const;

    geometry_msgs::msg::Pose computeEndPoseFromJoints(const std::vector<double> &joint_values) const;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_;
    std::string base_frame_;
    std::string tcp_frame_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};
