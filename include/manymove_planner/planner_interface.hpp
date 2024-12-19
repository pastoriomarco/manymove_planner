#pragma once

#include <vector>
#include <string>
#include <utility>
#include <memory>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <manymove_planner/msg/movement_config.hpp>
#include <manymove_planner/action/move_manipulator.hpp>
#include <manymove_planner/action/move_manipulator_sequence.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>

class PlannerInterface {
public:
    virtual ~PlannerInterface() = default;

    // Planning methods
    virtual std::pair<bool, moveit_msgs::msg::RobotTrajectory> plan(const manymove_planner::action::MoveManipulator::Goal &goal) = 0;
    virtual std::pair<std::vector<moveit_msgs::msg::RobotTrajectory>, std::vector<manymove_planner::msg::MovementConfig>> planSequence(
        const manymove_planner::action::MoveManipulatorSequence::Goal &sequence_goal) = 0;

    // Execution methods
    virtual bool executeTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory) = 0;
    virtual bool executeTrajectoryWithFeedback(
        const moveit_msgs::msg::RobotTrajectory &trajectory,
        const std::vector<size_t> &sizes,
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<manymove_planner::action::MoveManipulatorSequence>> &goal_handle) = 0;

    // Time parametrization
    virtual std::pair<bool, moveit_msgs::msg::RobotTrajectory> applyTimeParametrizationSequence(
        const std::vector<moveit_msgs::msg::RobotTrajectory>& trajectories,
        const std::vector<manymove_planner::msg::MovementConfig>& configs,
        std::vector<size_t>& sizes) = 0;

protected:
    PlannerInterface() = default;
};
