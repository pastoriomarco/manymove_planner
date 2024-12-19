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

class MoveGroupPlanner : public PlannerInterface
{
public:
    using MoveManipulator = manymove_planner::action::MoveManipulator;
    using MoveManipulatorSequence = manymove_planner::action::MoveManipulatorSequence;
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
    using GoalHandleMoveManipulator = rclcpp_action::ServerGoalHandle<MoveManipulator>;
    using GoalHandleMoveManipulatorSequence = rclcpp_action::ServerGoalHandle<MoveManipulatorSequence>;

    MoveGroupPlanner(
        const rclcpp::Node::SharedPtr &node,
        const std::string &planning_group,
        const std::string &base_frame,
        const std::string &tcp_frame,
        const std::string &traj_controller);

    ~MoveGroupPlanner() override = default;

    // Planning methods
    std::pair<bool, moveit_msgs::msg::RobotTrajectory> plan(const manymove_planner::action::MoveManipulator::Goal &goal) override;
    std::pair<std::vector<moveit_msgs::msg::RobotTrajectory>, std::vector<manymove_planner::msg::MovementConfig>> planSequence(
        const manymove_planner::action::MoveManipulatorSequence::Goal &sequence_goal) override;

    // Execution methods
    bool executeTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory) override;
    bool executeTrajectoryWithFeedback(
        const moveit_msgs::msg::RobotTrajectory &trajectory,
        const std::vector<size_t> &sizes,
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<manymove_planner::action::MoveManipulatorSequence>> &goal_handle) override;

    // Time parametrization
    std::pair<bool, moveit_msgs::msg::RobotTrajectory> applyTimeParametrizationSequence(
        const std::vector<moveit_msgs::msg::RobotTrajectory> &trajectories,
        const std::vector<manymove_planner::msg::MovementConfig> &configs,
        std::vector<size_t> &sizes) override;

private:
    double computePathLength(const moveit_msgs::msg::RobotTrajectory &trajectory) const;
    double computeMaxCartesianSpeed(const robot_trajectory::RobotTrajectoryPtr &trajectory) const;

    bool areSameJointTargets(const std::vector<double> &j1, const std::vector<double> &j2, double tolerance) const;

    bool applyTimeParameterization(robot_trajectory::RobotTrajectoryPtr &trajectory, const manymove_planner::msg::MovementConfig &config);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_;
    std::string base_frame_;
    std::string tcp_frame_;
    std::string traj_controller_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr follow_joint_traj_client_;
};
