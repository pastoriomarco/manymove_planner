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
 * underlying planning framework. It provides functionalities for trajectory
 * generation, time parameterization, and optionally execution through a
 * FollowJointTrajectory action server.It provides utilities to handle multiple
 * movement types (joint, pose, named, cartesian).
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
     *
     * @details This function doesn't use the internal functionalities of its implementation (MoveGroup or MoveItCPP):
     * instead, it uses a standard MoveIt functionality as the controller's trajectory execution action server, which
     * in this context is usually <robot_name>_traj_controller/follow_joint_trajectory. Older versions of this package
     * did use the specific functions of its implementation, for example the execute() and asyncExecute() funcions, but
     * this created differences in if and how parallel planning and execution would be handled dependin on which implementation
     * is used. Using the traj_controller separates the concerns of planning and execution and makes the implementation more
     * consistant.
     */
    bool executeTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory) override;

    /**
     * @brief Execute a planned trajectory with feedback to an action server.
     * @param trajectory The trajectory to execute.
     * @param sizes The sizes (number of points) for each segment of the trajectory.
     * @param goal_handle The action server goal handle for providing feedback.
     * @return True if execution succeeded, false otherwise.
     *
     * @details This version of the implementation leverages the traj_controller's feedback to get info about the current
     * state of execution. ManymovePlanner is designed to let the user parallelize the planning and the execution of the
     * moves, and while constructing the package I tried different approaches: in an execution made by several chained moves,
     * this function could let the logic builder package (for example manymove_cpp_trees or manymove_py_trees) get a feedback
     * on where the execution of the trajectory was. This could be useful to handle a failed execution, but in later stages
     * of the design of the logic builders I decided to chain each move directly in the logic builder package, thus making this
     * function only used on some functions of this package not currently used in the logic builder packages. This package still
     * publishes the sequence_action_server that lets the user plan and execute directly a sequence of moves, and may be used
     * to implement different logic bulider packages from those offered in ManyMove project.
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

    /**
     * @brief Send a short trajectory to the FollowJointTrajectory controller that decelerates
     *        the robot to zero velocity from its current state.
     *
     * @param deceleration_time seconds over which to ramp velocities down to 0
     * @return true if the goal was sent and completed successfully
     * @return false if something failed
     */
    bool sendControlledStop(double deceleration_time = 1.0);

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
     *
     * @details Most of the industrial and collaborative robots have a maximum cartesian speed over which the robot will perform
     * and emergency stop. Moreover, safety regulations in collaborative applications require the enforcement of maximum cartesian
     * speed limits. While this package is not meant to provide functionalities compliant with safety regulations, most robots
     * will come with such functionalities from factory, and they can't (or shouldn't) be overruled or removed.
     * This function not only applies the time parametrization required for the trajectory to be executed with a smooth motion,
     * but also reduces the velocity scaling if the calculated cartesian speed at any segment of the trajectory exceeds the
     * cartesian limit set on the @p config parameter. Currently this function only limits the velocity scaling factor, not the
     * acceleration scaling factor: this allows for faster movements as the acceleration is not reduced together with the
     * velocity, but try to keep velocities and accelerations coherent with the cartesian speed you want to obtain. Having really
     * slow moves with high accelerations may cause jerky and instable moves, so when you set the @p config param always try to
     * keep the velocity and acceleration scaling factors coherent with the maximum cartesian speed you set.
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
