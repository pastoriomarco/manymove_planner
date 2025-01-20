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
 * @brief Abstract interface for motion planners in the manymove_planner package.
 *
 * This class defines the essential functions that any motion planner in the
 * manymove_planner package must implement. It includes methods for planning,
 * execution, and time parameterization of trajectories.
 */
class PlannerInterface
{
public:
    /**
     * @brief Virtual destructor for PlannerInterface.
     */
    virtual ~PlannerInterface() = default;

    /**
     * @brief Plan a trajectory to achieve a specified goal.
     * @param goal The target goal for the manipulator.
     * @return A pair containing a success flag (true if planning succeeded) and the planned robot trajectory.
     */
    virtual std::pair<bool, moveit_msgs::msg::RobotTrajectory> plan(const manymove_planner::action::MoveManipulator::Goal &goal) = 0;

    /**
     * @brief Plan a sequence of trajectories to achieve multiple goals.
     * @param sequence_goal The sequence of goals for the manipulator.
     * @return A pair containing a vector of planned trajectories and a vector of the associated movement configurations.
     */
    virtual std::pair<std::vector<moveit_msgs::msg::RobotTrajectory>, std::vector<manymove_planner::msg::MovementConfig>> planSequence(
        const manymove_planner::action::MoveManipulatorSequence::Goal &sequence_goal) = 0;

    /**
     * @brief Execute a given trajectory on the manipulator.
     * @param trajectory The trajectory to execute.
     * @return True if execution was successful, false otherwise.
     */
    virtual bool executeTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory) = 0;

    /**
     * @brief Execute a given trajectory while providing feedback to an action server.
     * @param trajectory The trajectory to execute.
     * @param sizes A vector containing the sizes (number of points) of each segment in the trajectory.
     * @param goal_handle The goal handle for the action server, used to provide feedback.
     * @return True if execution was successful, false otherwise.
     */
    virtual bool executeTrajectoryWithFeedback(
        const moveit_msgs::msg::RobotTrajectory &trajectory,
        const std::vector<size_t> &sizes,
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<manymove_planner::action::MoveManipulatorSequence>> &goal_handle) = 0;

    /**
     * @brief Apply time parameterization to a sequence of trajectories.
     * @param trajectories The input trajectories to be time-parameterized.
     * @param configs The movement configurations (e.g., velocity and acceleration scaling) for each trajectory.
     * @param sizes Output vector indicating the number of points (or segments) for each trajectory within the time-parameterized result.
     * @return A pair containing a success flag (true if parameterization succeeded) and the combined time-parameterized trajectory.
     */
    virtual std::pair<bool, moveit_msgs::msg::RobotTrajectory> applyTimeParametrizationSequence(
        const std::vector<moveit_msgs::msg::RobotTrajectory> &trajectories,
        const std::vector<manymove_planner::msg::MovementConfig> &configs,
        std::vector<size_t> &sizes) = 0;

    /**
     * @brief The stop_motion action servers takes as input any traj and just stops the motion of the manipulator
     * by overriding the current trajectory execution by traj_controller with the current position,
     * zero velocity, and deceleration time. The robot will try to "spring back" to the position it was
     * when the stop command is issued within the deceleration time. The higher the time, the smoother
     * the stop, but the higher the move lenght to decelerate and come back to the stop point.
     *
     * @param deceleration_time seconds over which to ramp velocities down to 0
     * @return true if the goal was sent and completed successfully
     * @return false if something failed
     */
    virtual bool sendControlledStop(double deceleration_time = 0.25) = 0;

protected:
    /**
     * @brief Protected constructor to prevent direct instantiation of the interface.
     */
    PlannerInterface() = default;
};
