#pragma once

#include "planner_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <future>
#include <map>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>

#include "manymove_planner/msg/move_manipulator_goal.hpp"

#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/moveit_cpp/planning_component.h>

/**
 * @class MoveItCppPlanner
 * @brief A planner class integrating with MoveItCpp for motion planning in ROS2.
 *
 * This class implements the PlannerInterface methods using MoveItCpp as the
 * underlying planning framework. It provides functionalities for trajectory
 * generation, time parameterization, and optionally execution through a
 * FollowJointTrajectory action server.
 */
class MoveItCppPlanner : public PlannerInterface
{
public:
    using MoveManipulator = manymove_planner::action::MoveManipulator;
    using MoveManipulatorSequence = manymove_planner::action::MoveManipulatorSequence;
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
    using GoalHandleMoveManipulator = rclcpp_action::ServerGoalHandle<MoveManipulator>;
    using GoalHandleMoveManipulatorSequence = rclcpp_action::ServerGoalHandle<MoveManipulatorSequence>;

    /**
     * @brief Constructor for MoveItCppPlanner.
     * @param node Shared pointer to the ROS2 node.
     * @param planning_group Name of the planning group (defined in MoveIt).
     * @param base_frame The base frame of the robot.
     * @param tcp_frame The tool center point (TCP) frame for the manipulator.
     * @param traj_controller Name of the trajectory controller to use for execution.
     */
    MoveItCppPlanner(
        const rclcpp::Node::SharedPtr &node,
        const std::string &planning_group,
        const std::string &base_frame,
        const std::string &tcp_frame,
        const std::string &traj_controller);

    /**
     * @brief Default destructor.
     */
    ~MoveItCppPlanner() override = default;

    /**
     * @brief Plan a trajectory for a given goal.
     * @param goal The goal for the manipulator.
     * @return A pair containing a success flag and the planned robot trajectory.
     */
    std::pair<bool, moveit_msgs::msg::RobotTrajectory> plan(const manymove_planner::action::MoveManipulator::Goal &goal) override;

    /**
     * @brief Plan a sequence of trajectories for multiple goals.
     * @param sequence_goal The sequence of goals for the manipulator.
     * @return A pair containing a list of planned trajectories and associated movement configurations.
     */
    std::pair<std::vector<moveit_msgs::msg::RobotTrajectory>, std::vector<manymove_planner::msg::MovementConfig>> planSequence(
        const manymove_planner::action::MoveManipulatorSequence::Goal &sequence_goal) override;

    /**
     * @brief Execute a planned trajectory on the robot.
     * @param trajectory The trajectory to be executed.
     * @return True if execution was successful, false otherwise.
     */
    bool executeTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory) override;

    /**
     * @brief Execute a planned trajectory with feedback provided to an action server.
     * @param trajectory The trajectory to be executed.
     * @param sizes Vector of trajectory segment sizes (number of points per segment).
     * @param goal_handle The goal handle for providing feedback to an action server.
     * @return True if execution was successful, false otherwise.
     */
    bool executeTrajectoryWithFeedback(
        const moveit_msgs::msg::RobotTrajectory &trajectory,
        const std::vector<size_t> &sizes,
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<manymove_planner::action::MoveManipulatorSequence>> &goal_handle) override;

    /**
     * @brief Apply time parameterization to a list of trajectories.
     * @param trajectories The input trajectories to be parameterized.
     * @param configs Movement configurations (containing velocity/acceleration factors, etc.).
     * @param sizes Output vector indicating how many points are in each segment after concatenation.
     * @return A pair containing a success flag and the final time-parameterized, concatenated trajectory.
     */
    std::pair<bool, moveit_msgs::msg::RobotTrajectory> applyTimeParametrizationSequence(
        const std::vector<moveit_msgs::msg::RobotTrajectory> &trajectories,
        const std::vector<manymove_planner::msg::MovementConfig> &configs,
        std::vector<size_t> &sizes) override;

private:
    /**
     * @brief Compute the total path length of a given trajectory.
     * @param trajectory The MoveIt robot trajectory message.
     * @return The computed path length in joint/Cartesian space.
     */
    double computePathLength(const moveit_msgs::msg::RobotTrajectory &trajectory) const;

    /**
     * @brief Compute the maximum Cartesian speed found in a trajectory.
     * @param trajectory A pointer to the robot trajectory object.
     * @return The maximum Cartesian speed (m/s) found in the trajectory.
     */
    double computeMaxCartesianSpeed(const robot_trajectory::RobotTrajectoryPtr &trajectory) const;

    /**
     * @brief Check if two joint targets (vectors of joint values) are equal within a specified tolerance.
     * @param j1 First joint target.
     * @param j2 Second joint target.
     * @param tolerance The acceptable tolerance for each joint's difference.
     * @return True if all corresponding joints match within the tolerance, false otherwise.
     */
    bool areSameJointTargets(const std::vector<double> &j1, const std::vector<double> &j2, double tolerance) const;

    /**
     * @brief Apply time parameterization to a single trajectory using the given movement config.
     * @param trajectory Pointer to a RobotTrajectory to parameterize.
     * @param config The movement configuration specifying velocity/acceleration factors, etc.
     * @return True if the time parameterization succeeded, false otherwise.
     */
    bool applyTimeParameterization(robot_trajectory::RobotTrajectoryPtr &trajectory, const manymove_planner::msg::MovementConfig &config);

    /**
     * @brief Convert a RobotTrajectory object to a corresponding message.
     * @param trajectory The input RobotTrajectory reference.
     * @return A moveit_msgs::msg::RobotTrajectory representation of the same trajectory.
     */
    moveit_msgs::msg::RobotTrajectory convertToMsg(const robot_trajectory::RobotTrajectory &trajectory) const;

    rclcpp::Node::SharedPtr node_; ///< Shared pointer to the ROS2 node.
    rclcpp::Logger logger_;        ///< Logger instance for logging messages.
    std::string planning_group_;   ///< Name of the planning group.
    std::string base_frame_;       ///< The base frame of the robot.
    std::string tcp_frame_;        ///< The tool center point (TCP) frame.
    std::string traj_controller_;  ///< Name of the trajectory controller.

    std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_ptr_;                ///< Shared pointer to the MoveItCpp instance.
    std::shared_ptr<moveit_cpp::PlanningComponent> planning_components_;   ///< Shared pointer to the PlanningComponent instance.
    moveit_cpp::PlanningComponent::PlanRequestParameters plan_parameters_; ///< Planning parameters loaded at startup.

    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr follow_joint_traj_client_; ///< Action client for FollowJointTrajectory.
};
