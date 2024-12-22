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

/**
 * @class PlannerInterface
 * @brief Abstract interface for motion planners.
 *
 * This class defines the essential functions that any motion planner of manymove_planner package must implement,
 * including methods for planning, execution, and time parametrization.
 */
class PlannerInterface {
public:
    /**
     * @brief Virtual destructor for PlannerInterface.
     */
    virtual ~PlannerInterface() = default;

    /**
     * @brief Plan a trajectory to achieve a specific goal.
     * @param goal The target goal for the manipulator.
     * @return A pair containing a success flag and the planned robot trajectory.
     */
    virtual std::pair<bool, moveit_msgs::msg::RobotTrajectory> plan(const manymove_planner::action::MoveManipulator::Goal &goal) = 0;

    /**
     * @brief Plan a sequence of trajectories to achieve multiple goals.
     * @param sequence_goal A sequence of goals for the manipulator.
     * @return A pair containing the planned trajectories and the associated movement configurations.
     */
    virtual std::pair<std::vector<moveit_msgs::msg::RobotTrajectory>, std::vector<manymove_planner::msg::MovementConfig>> planSequence(
        const manymove_planner::action::MoveManipulatorSequence::Goal &sequence_goal) = 0;

    /**
     * @brief Execute a given trajectory.
     * @param trajectory The trajectory to execute.
     * @return True if execution was successful, false otherwise.
     */
    virtual bool executeTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory) = 0;

    /**
     * @brief Execute a given trajectory while providing feedback to the action server.
     * @param trajectory The trajectory to execute.
     * @param sizes A vector containing the sizes of each segment in the trajectory.
     * @param goal_handle The goal handle for the action server.
     * @return True if execution was successful, false otherwise.
     */
    virtual bool executeTrajectoryWithFeedback(
        const moveit_msgs::msg::RobotTrajectory &trajectory,
        const std::vector<size_t> &sizes,
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<manymove_planner::action::MoveManipulatorSequence>> &goal_handle) = 0;

    /**
     * @brief Apply time parameterization to a sequence of trajectories.
     * @param trajectories The input trajectories.
     * @param configs The movement configurations for each trajectory.
     * @param sizes Output sizes of each trajectory segment.
     * @return A pair containing a success flag and the time-parameterized trajectory.
     */
    virtual std::pair<bool, moveit_msgs::msg::RobotTrajectory> applyTimeParametrizationSequence(
        const std::vector<moveit_msgs::msg::RobotTrajectory>& trajectories,
        const std::vector<manymove_planner::msg::MovementConfig>& configs,
        std::vector<size_t>& sizes) = 0;

protected:
    /**
     * @brief Protected constructor to prevent direct instantiation.
     */
    PlannerInterface() = default;
};
