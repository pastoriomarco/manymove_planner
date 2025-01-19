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
 * FollowJointTrajectory action server.It provides utilities to handle multiple
 * movement types (joint, pose, named, cartesian).
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
     * @brief Execute a planned trajectory with feedback provided to an action server.
     * @param trajectory The trajectory to be executed.
     * @param sizes Vector of trajectory segment sizes (number of points per segment).
     * @param goal_handle The goal handle for providing feedback to an action server.
     * @return True if execution was successful, false otherwise.
     *
     * @details This version of the executeTrajectory implementation leverages the traj_controller's feedback to get info about
     * the current state of execution. ManymovePlanner is designed to let the user parallelize the planning and the execution
     * of the moves, and while constructing the package I tried different approaches: in an execution made by several chained moves,
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
     * @brief Compute the total path length of a given trajectory.
     * @param trajectory The MoveIt robot trajectory message.
     * @return The computed path length in joint/Cartesian space.
     */
    double computePathLength(const moveit_msgs::msg::RobotTrajectory &trajectory) const;

    /**
     * @brief Calculate the pose relative to a frame from a robot state.
     * @param robot_state The robot state to get the joint positions from.
     * @param link_frame The reference frame to calculate the pose from the joint positions of the robot state.
     * @return The computed distance between the two poses.
     */
    geometry_msgs::msg::Pose getPoseFromRobotState(const moveit::core::RobotState &robot_state,
                                                   const std::string &link_frame);

    /**
     * @brief Compute the euclidean distance between two poses.
     * @param start_pose The start pose to calculate the distance from.
     * @param target_pose The target pose to calculate the distance to.
     * @return The computed distance between the two poses.
     */
    double computeCartesianDistance(const geometry_msgs::msg::Pose &start_pose,
                                    const geometry_msgs::msg::Pose &target_pose);

    /**
     * @brief Calculate the pose relative to a frame from the first or the last point of a trajectory.
     * @param traj_msg The trajectory from which to get the point to calculate from.
     * @param robot_state Contains the info about the robot to calculate the pose.
     * @param link_frame The reference frame to calculate the pose.
     * @param use_last_point If true it uses the last point of the trajectory, if false the first point.
     * @return The computed distance between the two poses.
     */
    geometry_msgs::msg::Pose getPoseFromTrajectory(const moveit_msgs::msg::RobotTrajectory &traj_msg,
                                                   const moveit::core::RobotState &robot_state,
                                                   const std::string &link_frame,
                                                   bool use_last_point = true);

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

    /**
     * @brief Convert a RobotTrajectory object to a corresponding message.
     * @param trajectory The input RobotTrajectory reference.
     * @return A moveit_msgs::msg::RobotTrajectory representation of the same trajectory.
     */
    moveit_msgs::msg::RobotTrajectory convertToMsg(const robot_trajectory::RobotTrajectory &trajectory) const;

    /**
     * @brief Collision-check callback used by Cartesian path computations.
     * @param state Pointer to the RobotState being tested for collisions.
     * @param group Pointer to the JointModelGroup for which collision checks should be performed.
     * @return True if the state is free of collisions, false otherwise.
     *
     * @details This function is provided as a custom validity checker for
     * moveit::core::CartesianInterpolator::computeCartesianPath(), ensuring that
     * intermediate states do not collide with any known obstacles or the robot
     * itself. Without isStateValid() used as a callback function, computeCartesianPath()
     * wouldn't check for collisions just for mesh objects, while it did for primitive objects.
     * I couldn't figure if I missed some setting to avoid this problem, if I found some edge
     * cases that resulted in this abnormal behavior, or if it is in fact the intended behavior
     * of computeCartesianPath(). Regardless, this seems to solve the issue.
     * The function performs a collision check on the given @p state for the
     * specified joint model group. It uses a read only locked version of the current
     * Planning Scene to verify whether the @p state is in collision. If a collision is
     * detected, the function logs a warning and returns false. Otherwise, it returns true.
     */
    bool isStateValid(const moveit::core::RobotState *state, const moveit::core::JointModelGroup *group);

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
