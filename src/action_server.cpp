#include "manymove_planner/planner_interface.hpp"
#include "manymove_planner/action/plan_manipulator.hpp"
#include "manymove_planner/action/move_manipulator.hpp"
#include "manymove_planner/action/move_manipulator_sequence.hpp"
#include "manymove_planner/action/execute_trajectory.hpp"
#include "manymove_planner/action/reset_traj_controller.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <controller_manager_msgs/srv/load_controller.hpp>
#include <controller_manager_msgs/srv/unload_controller.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

#include <rclcpp_action/rclcpp_action.hpp>

class MoveManipulatorActionServer
{
public:
    using MoveManipulator = manymove_planner::action::MoveManipulator;
    using MoveManipulatorSequence = manymove_planner::action::MoveManipulatorSequence;
    using GoalHandleMoveManipulator = rclcpp_action::ServerGoalHandle<MoveManipulator>;
    using GoalHandleMoveManipulatorSequence = rclcpp_action::ServerGoalHandle<MoveManipulatorSequence>;

    using PlanManipulator = manymove_planner::action::PlanManipulator;
    using GoalHandlePlanManipulator = rclcpp_action::ServerGoalHandle<PlanManipulator>;

    using ExecuteTrajectory = manymove_planner::action::ExecuteTrajectory;
    using GoalHandleExecuteTrajectory = rclcpp_action::ServerGoalHandle<ExecuteTrajectory>;

    using ResetTrajController = manymove_planner::action::ResetTrajController;
    using GoalHandleResetTrajController = rclcpp_action::ServerGoalHandle<ResetTrajController>;

    MoveManipulatorActionServer(const rclcpp::Node::SharedPtr &node,
                                const std::shared_ptr<PlannerInterface> &planner,
                                const std::string &planner_prefix = "")
        : node_(node), planner_(planner), planner_prefix_(planner_prefix)
    {
        single_action_server_ = rclcpp_action::create_server<MoveManipulator>(
            node_,
            planner_prefix_ + "move_manipulator",
            std::bind(&MoveManipulatorActionServer::handle_single_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveManipulatorActionServer::handle_single_cancel, this, std::placeholders::_1),
            std::bind(&MoveManipulatorActionServer::handle_single_accepted, this, std::placeholders::_1));

        sequence_action_server_ = rclcpp_action::create_server<MoveManipulatorSequence>(
            node_,
            planner_prefix_ + "move_manipulator_sequence",
            std::bind(&MoveManipulatorActionServer::handle_sequence_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveManipulatorActionServer::handle_sequence_cancel, this, std::placeholders::_1),
            std::bind(&MoveManipulatorActionServer::handle_sequence_accepted, this, std::placeholders::_1));

        plan_action_server_ = rclcpp_action::create_server<PlanManipulator>(
            node_,
            planner_prefix_ + "plan_manipulator",
            std::bind(&MoveManipulatorActionServer::handle_plan_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveManipulatorActionServer::handle_plan_cancel, this, std::placeholders::_1),
            std::bind(&MoveManipulatorActionServer::handle_plan_accepted, this, std::placeholders::_1));

        execute_action_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
            node_,
            planner_prefix_ + "execute_manipulator_traj",
            std::bind(&MoveManipulatorActionServer::handle_execute_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveManipulatorActionServer::handle_execute_cancel, this, std::placeholders::_1),
            std::bind(&MoveManipulatorActionServer::handle_execute_accepted, this, std::placeholders::_1));

        /**
         * The stop_motion action server is used to stop the current execution of exection_action_server if needed.
         * By itself, the traj_controller keeps executing the trajectory even it received the cancel command. But if it
         * receives a new trajectory to execute it switches to it. The stop server just sends a new trajectory with
         * the current position and the velocity at zero. This is not very elegant, but it seems not to cause extreme
         * move reactions in the manipulator moves.
         */
        stop_motion_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
            node_,
            planner_prefix_ + "stop_motion",
            std::bind(&MoveManipulatorActionServer::handle_stop_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveManipulatorActionServer::handle_stop_cancel, this, std::placeholders::_1),
            std::bind(&MoveManipulatorActionServer::handle_stop_accept, this, std::placeholders::_1));

        reset_controller_server_ = rclcpp_action::create_server<ResetTrajController>(
            node_,
            planner_prefix_ + "reset_trajectory_controller", // e.g. "reset_trajectory_controller"
            std::bind(&MoveManipulatorActionServer::handle_reset_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveManipulatorActionServer::handle_reset_cancel, this, std::placeholders::_1),
            std::bind(&MoveManipulatorActionServer::handle_reset_accepted, this, std::placeholders::_1));
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<PlannerInterface> planner_;
    std::string planner_prefix_;

    rclcpp_action::Server<MoveManipulator>::SharedPtr single_action_server_;
    rclcpp_action::Server<MoveManipulatorSequence>::SharedPtr sequence_action_server_;

    rclcpp_action::Server<PlanManipulator>::SharedPtr plan_action_server_;
    rclcpp_action::Server<ExecuteTrajectory>::SharedPtr execute_action_server_;
    rclcpp_action::Server<ExecuteTrajectory>::SharedPtr stop_motion_server_;

    rclcpp_action::Server<ResetTrajController>::SharedPtr reset_controller_server_;

    // -------------------------------------
    // MoveManipulator callbacks
    // -------------------------------------

    rclcpp_action::GoalResponse handle_single_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MoveManipulator::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        RCLCPP_INFO(node_->get_logger(), "Received single MoveManipulator goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_single_cancel(
        const std::shared_ptr<GoalHandleMoveManipulator> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel single move goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_single_accepted(const std::shared_ptr<GoalHandleMoveManipulator> goal_handle)
    {
        // Run in a separate thread
        std::thread{std::bind(&MoveManipulatorActionServer::execute_single, this, goal_handle)}.detach();
    }

    void execute_single(const std::shared_ptr<GoalHandleMoveManipulator> &goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Executing single MoveManipulator goal");
        auto result = std::make_shared<MoveManipulator::Result>();
        const auto &goal_msg = goal_handle->get_goal();

        // Plan single move
        auto [success, trajectory] = planner_->plan(*goal_msg);
        if (!success)
        {
            RCLCPP_ERROR(node_->get_logger(), "Planning failed for single move.");
            result->success = false;
            result->message = "Planning failed";
            goal_handle->abort(result);
            return;
        }

        // Apply time parameterization
        std::vector<moveit_msgs::msg::RobotTrajectory> single_traj_vec = {trajectory};
        std::vector<manymove_planner::msg::MovementConfig> single_config_vec = {goal_msg->goal.config};
        std::vector<size_t> sizes;
        auto [param_success, final_trajectory] = planner_->applyTimeParametrizationSequence(single_traj_vec, single_config_vec, sizes);

        if (!param_success)
        {
            RCLCPP_ERROR(node_->get_logger(), "Time parameterization failed for single move.");
            result->success = false;
            result->message = "Time parametrization failed";
            goal_handle->abort(result);
            return;
        }

        // Execute WITHOUT feedback since it's a single move
        bool exec_success = planner_->executeTrajectory(final_trajectory);
        if (exec_success)
        {
            RCLCPP_INFO(node_->get_logger(), "Single move succeeded");
            result->success = true;
            result->message = "Motion complete";
            goal_handle->succeed(result);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Single move failed");
            result->success = false;
            result->message = "Motion failed";
            goal_handle->abort(result);
        }
    }

    // -------------------------------------
    // MoveManipulatorSequence callbacks
    // -------------------------------------

    rclcpp_action::GoalResponse handle_sequence_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MoveManipulatorSequence::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(node_->get_logger(), "Received MoveManipulatorSequence goal with %zu moves", goal->goals.size());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_sequence_cancel(
        const std::shared_ptr<GoalHandleMoveManipulatorSequence> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel sequence goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_sequence_accepted(const std::shared_ptr<GoalHandleMoveManipulatorSequence> goal_handle)
    {
        // Run in a separate thread
        std::thread{std::bind(&MoveManipulatorActionServer::execute_sequence, this, goal_handle)}.detach();
    }

    void execute_sequence(std::shared_ptr<GoalHandleMoveManipulatorSequence> &goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Executing sequence goal");
        auto result = std::make_shared<MoveManipulatorSequence::Result>();
        const auto &goals_msg = goal_handle->get_goal();

        // Plan the sequence
        RCLCPP_INFO(node_->get_logger(), "Planning Sequence");
        auto [trajectories, configs] = planner_->planSequence(*goals_msg);

        if (trajectories.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "Planning sequence failed.");
            result->success = false;
            result->message = "Planning sequence failed.";
            goal_handle->abort(result);
            return;
        }

        // Time parameterization
        std::vector<size_t> sizes;
        auto [param_success, final_trajectory] = planner_->applyTimeParametrizationSequence(trajectories, configs, sizes);

        if (!param_success)
        {
            RCLCPP_ERROR(node_->get_logger(), "Time parametrization sequence failed.");
            result->success = false;
            result->message = "Time parametrization sequence failed.";
            goal_handle->abort(result);
            return;
        }

        // Execute WITH feedback for the sequence
        bool exec_success = planner_->executeTrajectoryWithFeedback(final_trajectory, sizes, goal_handle);
        if (exec_success)
        {
            RCLCPP_INFO(node_->get_logger(), "Sequence goal succeeded");
            result->success = true;
            result->message = "Motion complete";
            goal_handle->succeed(result);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Sequence goal failed");
            result->success = false;
            result->message = "Motion failed";
            goal_handle->abort(result);
        }
    }

    // -------------------------------------
    // PlanManipulator callbacks
    // -------------------------------------

    rclcpp_action::GoalResponse handle_plan_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const PlanManipulator::Goal> goal_msg)
    {
        RCLCPP_INFO(node_->get_logger(), "Received PlanManipulator action goal request");
        (void)uuid;
        (void)goal_msg;
        // Optionally validate goal_msg->goal ...
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_plan_cancel(
        const std::shared_ptr<GoalHandlePlanManipulator> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel PlanManipulator goal");
        // For a pure planning action, you might accept or reject cancellation
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_plan_accepted(
        const std::shared_ptr<GoalHandlePlanManipulator> goal_handle)
    {
        // Spin off a new thread to do planning so we don't block the executor
        std::thread{std::bind(&MoveManipulatorActionServer::execute_plan_goal, this, goal_handle)}.detach();
    }

    void execute_plan_goal(
        const std::shared_ptr<GoalHandlePlanManipulator> goal_handle)
    {
        auto result = std::make_shared<PlanManipulator::Result>();
        const auto &goal_msg = goal_handle->get_goal(); // plan_manipulator::Goal

        // 1) Actually plan
        manymove_planner::action::MoveManipulator::Goal internal_goal;
        internal_goal.goal = goal_msg->goal; // reusing the existing "MoveManipulatorGoal"

        auto [success, trajectory] = planner_->plan(internal_goal);
        if (!success)
        {
            RCLCPP_ERROR(node_->get_logger(), "Planning failed for plan_manipulator action");
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        // 2) Time parameterization
        std::vector<moveit_msgs::msg::RobotTrajectory> single_traj_vec = {trajectory};
        std::vector<manymove_planner::msg::MovementConfig> single_config_vec = {goal_msg->goal.config};
        std::vector<size_t> sizes;

        auto [param_success, final_trajectory] =
            planner_->applyTimeParametrizationSequence(single_traj_vec, single_config_vec, sizes);

        if (!param_success)
        {
            RCLCPP_ERROR(node_->get_logger(), "Time param failed for plan_manipulator action");
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        // If you want to send feedback in real-time, you can do so in applyTimeParameterization() or
        //   break it into steps. For simplicity, let's just finalize the result:
        result->success = true;
        result->trajectory = final_trajectory;

        RCLCPP_INFO(node_->get_logger(), "PlanManipulator action Succeeded");
        goal_handle->succeed(result);
    }

    // -------------------------------------
    // ExecuteTrajectory callbacks
    // -------------------------------------

    rclcpp_action::GoalResponse handle_execute_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ExecuteTrajectory::Goal> goal)
    {
        (void)uuid;
        if (goal->trajectory.joint_trajectory.points.empty())
        {
            RCLCPP_WARN(node_->get_logger(), "Received an empty trajectory");
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(node_->get_logger(), "Received ExecuteTrajectory goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_execute_cancel(
        const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel trajectory execution");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_execute_accepted(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        // Run in a separate thread
        std::thread{std::bind(&MoveManipulatorActionServer::execute_trajectory, this, goal_handle)}.detach();
    }

    void execute_trajectory(const std::shared_ptr<GoalHandleExecuteTrajectory> &goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Executing trajectory");
        auto result = std::make_shared<ExecuteTrajectory::Result>();
        const auto &goal_msg = goal_handle->get_goal();

        // Execute the trajectory
        bool exec_success = planner_->executeTrajectory(goal_msg->trajectory);

        if (exec_success)
        {
            RCLCPP_INFO(node_->get_logger(), "Trajectory execution succeeded");
            result->success = true;
            result->message = "Trajectory execution complete";
            goal_handle->succeed(result);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Trajectory execution failed");
            result->success = false;
            result->message = "Trajectory execution failed";
            goal_handle->abort(result);
        }
    }

    /**
     * The stop_motion action servers takes as input any traj and just stops the motion of the manipulator
     * by overriding the current trajectory execution by traj_controller with the current position,
     * zero velocity, and deceleration time. The robot will try to "spring back" to the position it was
     * when the stop command is issued within the deceleration time. The higher the time, the smoother
     * the stop, but the higher the move lenght to decelerate and come back to the stop point.
     */

    // -------------------------------------
    // StopMotion callbacks
    // -------------------------------------

    rclcpp_action::GoalResponse handle_stop_goal(
        [[maybe_unused]] const rclcpp_action::GoalUUID &uuid,
        [[maybe_unused]] std::shared_ptr<const ExecuteTrajectory::Goal> goal)
    {
        RCLCPP_INFO(node_->get_logger(), "[ExecuteTrajectory] Received request to STOP");
        // Always accept the request:
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_stop_cancel(
        [[maybe_unused]] const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "[ExecuteTrajectory] Received request to cancel STOP motion");
        // Typically we just accept the cancel, though there's not much sense in "canceling a stop."
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_stop_accept(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        // Execute in a separate thread so as not to block the executor
        std::thread{std::bind(&MoveManipulatorActionServer::execute_stop, this, goal_handle)}.detach();
    }

    void execute_stop(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "[ExecuteTrajectory] Executing STOP motion...");
        auto result = std::make_shared<ExecuteTrajectory::Result>();

        // 1) Check if canceled before we start
        if (goal_handle->is_canceling())
        {
            RCLCPP_WARN(node_->get_logger(), "[ExecuteTrajectory] STOP goal canceled before execution.");
            result->success = false;
            result->message = "Canceled before execution";
            goal_handle->canceled(result);
            return;
        }

        // 2) Actually send the short stop trajectory
        double dec_time = 0.5;
        bool ok = planner_->sendControlledStop(dec_time);

        // 3) If canceled mid-stop
        if (goal_handle->is_canceling())
        {
            RCLCPP_WARN(node_->get_logger(), "[ExecuteTrajectory] STOP goal canceled mid‐execution.");
            result->success = false;
            result->message = "Canceled mid‐execution";
            goal_handle->canceled(result);
            return;
        }

        // 4) Finalize
        if (ok)
        {
            RCLCPP_INFO(node_->get_logger(), "[ExecuteTrajectory] STOP completed successfully.");
            result->success = true;
            result->message = "Stop complete";
            goal_handle->succeed(result);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "[ExecuteTrajectory] STOP motion failed or was aborted by the controller.");
            result->success = false;
            result->message = "Stop motion failed";
            goal_handle->abort(result);
        }
    }

    // -------------------------------------
    // ResetTrajController callbacks
    // -------------------------------------

    rclcpp_action::GoalResponse handle_reset_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ResetTrajController::Goal> goal)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "Received ResetTrajController goal request for controller: %s",
                    goal->controller_name.c_str());

        (void)uuid; // not used
        // Optionally validate the controller name
        if (goal->controller_name.empty())
        {
            RCLCPP_WARN(node_->get_logger(), "Controller name is empty. Rejecting.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_reset_cancel(
        const std::shared_ptr<GoalHandleResetTrajController> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "Received request to CANCEL ResetTrajController goal");
        // For this operation, we can either allow or deny cancel.
        // We'll accept for convenience:
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_reset_accepted(
        const std::shared_ptr<GoalHandleResetTrajController> goal_handle)
    {
        // Execute in a separate thread
        std::thread{
            std::bind(&MoveManipulatorActionServer::execute_reset_traj_controller, this, goal_handle)}
            .detach();
    }

    void execute_reset_traj_controller(
        const std::shared_ptr<GoalHandleResetTrajController> &goal_handle)
    {
        auto result = std::make_shared<ResetTrajController::Result>();
        auto feedback = std::make_shared<ResetTrajController::Feedback>();

        // 1) Check if goal is canceled right away
        if (goal_handle->is_canceling())
        {
            RCLCPP_WARN(node_->get_logger(), "ResetTrajController goal canceled before execution started.");
            result->success = false;
            result->message = "Canceled before execution";
            goal_handle->canceled(result);
            return;
        }

        // 2) Extract controller name
        std::string controller_name = goal_handle->get_goal()->controller_name;
        RCLCPP_INFO(node_->get_logger(), "Executing reset for controller: %s", controller_name.c_str());

        // Feedback can be updated in small increments. For example:
        feedback->progress = 0.1f;
        goal_handle->publish_feedback(feedback);

        // 3) Retrieve parameters from the running controller
        auto param_vector = getControllerParams(node_, controller_name);
        if (param_vector.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to retrieve params from controller: %s", controller_name.c_str());
            result->success = false;
            result->message = "Failed to get params";
            goal_handle->abort(result);
            return;
        }
        feedback->progress = 0.3f;
        goal_handle->publish_feedback(feedback);

        // 4) Unload controller
        if (!unloadController(node_, controller_name))
        {
            RCLCPP_ERROR(node_->get_logger(), "Could not unload controller: %s", controller_name.c_str());
            result->success = false;
            result->message = "Unload failed";
            goal_handle->abort(result);
            return;
        }
        feedback->progress = 0.5f;
        goal_handle->publish_feedback(feedback);

        // 5) Load controller
        if (!loadController(node_, controller_name))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load controller: %s", controller_name.c_str());
            result->success = false;
            result->message = "Load failed";
            goal_handle->abort(result);
            return;
        }
        feedback->progress = 0.6f;
        goal_handle->publish_feedback(feedback);

        // 6) Reapply the saved parameters
        if (!configureControllerParams(node_, controller_name, param_vector))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to set parameters on controller: %s", controller_name.c_str());
            result->success = false;
            result->message = "Set params failed";
            goal_handle->abort(result);
            return;
        }
        feedback->progress = 0.8f;
        goal_handle->publish_feedback(feedback);

        // 7) Activate the controller
        if (!activateController(node_, controller_name))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to activate controller: %s", controller_name.c_str());
            result->success = false;
            result->message = "Activate failed";
            goal_handle->abort(result);
            return;
        }
        feedback->progress = 1.0f;
        goal_handle->publish_feedback(feedback);

        // 8) Succeed
        result->success = true;
        result->message = "Controller reset successful!";
        RCLCPP_INFO(node_->get_logger(), "Controller %s was reset successfully.", controller_name.c_str());
        goal_handle->succeed(result);
    }

    // -------------------------------------
    // Helpers:
    // -------------------------------------

    // A structure to store parameter name-value pairs.
    struct ParamData
    {
        std::string name;
        rcl_interfaces::msg::ParameterValue value;
    };

    // Retrieve all parameters from the trajectory controller
    std::vector<ParamData> getControllerParams(
        rclcpp::Node::SharedPtr node,
        const std::string &controller_name)
    {
        // Name of the service: <controller_name>/get_parameters
        auto client = node->create_client<rcl_interfaces::srv::GetParameters>(
            controller_name + "/get_parameters");

        // We might want to fetch all known param names;
        // or if you know them, pass them. Using wildcard "names = {*}" is not standard,
        // but we'll do a demonstration for each param we suspect.
        // In practice, you'd specify the parameter names or gather them from `ros2 param list`.
        auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();

        // Here, fill with known param names. For brevity, let's pick
        // "allow_partial_joints_goal", "gains.joint1.p", etc.
        // Or you can gather them from command-line queries if necessary.
        request->names = {
            // Basic
            "action_monitor_rate",
            "allow_integration_in_goal_trajectories",
            "allow_nonzero_velocity_at_trajectory_end",
            "allow_partial_joints_goal",
            "cmd_timeout",
            "command_interfaces",
            "command_joints",
            "joints",
            "interface_name",
            "interpolation_method",
            "open_loop_control",
            "robot_description",
            "set_last_command_interface_value_as_state_on_activation",
            "state_publish_rate",
            "update_rate",
            "use_sim_time",

            // Constraints
            "constraints.goal_time",
            "constraints.stopped_velocity_tolerance",
            "constraints.joint1.goal",
            "constraints.joint1.trajectory",
            "constraints.joint2.goal",
            "constraints.joint2.trajectory",
            "constraints.joint3.goal",
            "constraints.joint3.trajectory",
            "constraints.joint4.goal",
            "constraints.joint4.trajectory",
            "constraints.joint5.goal",
            "constraints.joint5.trajectory",
            "constraints.joint6.goal",
            "constraints.joint6.trajectory",

            // Gains
            "gains.joint1.p",
            "gains.joint1.i",
            "gains.joint1.d",
            "gains.joint1.i_clamp",
            "gains.joint1.ff_velocity_scale",
            "gains.joint1.normalize_error",
            "gains.joint1.angle_wraparound",
            "gains.joint2.p",
            "gains.joint2.i",
            "gains.joint2.d",
            "gains.joint2.i_clamp",
            "gains.joint2.ff_velocity_scale",
            "gains.joint2.normalize_error",
            "gains.joint2.angle_wraparound",
            "gains.joint3.p",
            "gains.joint3.i",
            "gains.joint3.d",
            "gains.joint3.i_clamp",
            "gains.joint3.ff_velocity_scale",
            "gains.joint3.normalize_error",
            "gains.joint3.angle_wraparound",
            "gains.joint4.p",
            "gains.joint4.i",
            "gains.joint4.d",
            "gains.joint4.i_clamp",
            "gains.joint4.ff_velocity_scale",
            "gains.joint4.normalize_error",
            "gains.joint4.angle_wraparound",
            "gains.joint5.p",
            "gains.joint5.i",
            "gains.joint5.d",
            "gains.joint5.i_clamp",
            "gains.joint5.ff_velocity_scale",
            "gains.joint5.normalize_error",
            "gains.joint5.angle_wraparound",
            "gains.joint6.p",
            "gains.joint6.i",
            "gains.joint6.d",
            "gains.joint6.i_clamp",
            "gains.joint6.ff_velocity_scale",
            "gains.joint6.normalize_error",
            "gains.joint6.angle_wraparound",
        };
        std::vector<ParamData> retrieved_params;

        if (client->wait_for_service(std::chrono::seconds(5)))
        {
            auto future = client->async_send_request(request);
            if (rclcpp::spin_until_future_complete(node, future) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = future.get();
                // Store them in a vector
                for (size_t i = 0; i < response->values.size(); ++i)
                {
                    ParamData pd;
                    pd.name = request->names[i];
                    pd.value = response->values[i];
                    retrieved_params.push_back(pd);
                }
            }
            else
            {
                RCLCPP_ERROR(
                    node->get_logger(),
                    "Failed to retrieve parameters from controller: %s",
                    controller_name.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "Service %s/get_parameters not available",
                controller_name.c_str());
        }

        return retrieved_params;
    }

    bool unloadController(
        rclcpp::Node::SharedPtr node,
        const std::string &controller_name)
    {
        // Service: /controller_manager/unload_controller
        auto client = node->create_client<controller_manager_msgs::srv::UnloadController>(
            "/controller_manager/unload_controller");

        auto request = std::make_shared<controller_manager_msgs::srv::UnloadController::Request>();
        request->name = controller_name;

        if (client->wait_for_service(std::chrono::seconds(5)))
        {
            auto future = client->async_send_request(request);
            if (rclcpp::spin_until_future_complete(node, future) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                if (future.get()->ok)
                {
                    RCLCPP_INFO(node->get_logger(), "Controller %s unloaded successfully.", controller_name.c_str());
                    return true;
                }
                else
                {
                    RCLCPP_ERROR(node->get_logger(), "Failed to unload controller %s.", controller_name.c_str());
                }
            }
            else
            {
                RCLCPP_ERROR(node->get_logger(), "Failed to call unload_controller service.");
            }
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Service unload_controller not available.");
        }
        return false;
    }

    bool loadController(
        rclcpp::Node::SharedPtr node,
        const std::string &controller_name)
    {
        // Service: /controller_manager/load_controller
        auto client = node->create_client<controller_manager_msgs::srv::LoadController>(
            "/controller_manager/load_controller");

        auto request = std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
        request->name = controller_name;

        if (client->wait_for_service(std::chrono::seconds(5)))
        {
            auto future = client->async_send_request(request);
            if (rclcpp::spin_until_future_complete(node, future) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                if (future.get()->ok)
                {
                    RCLCPP_INFO(node->get_logger(), "Loaded controller %s successfully.", controller_name.c_str());
                    return true;
                }
                else
                {
                    RCLCPP_ERROR(node->get_logger(), "Failed to load controller %s.", controller_name.c_str());
                }
            }
            else
            {
                RCLCPP_ERROR(node->get_logger(), "Failed to call load_controller service.");
            }
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Service load_controller not available.");
        }
        return false;
    }

    bool configureControllerParams(
        rclcpp::Node::SharedPtr node,
        const std::string &controller_name,
        const std::vector<ParamData> &params)
    {
        // Service: /<controller_name>/set_parameters
        auto client = node->create_client<rcl_interfaces::srv::SetParameters>(
            controller_name + "/set_parameters");

        // Transform ParamData into SetParameters::Request
        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        for (const auto &pd : params)
        {
            rclcpp::Parameter rcl_param(pd.name);

            // Convert rcl_interfaces::msg::ParameterValue -> rclcpp::Parameter
            switch (pd.value.type)
            {
            case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
                rcl_param = rclcpp::Parameter(pd.name, pd.value.bool_value);
                break;
            case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
                rcl_param = rclcpp::Parameter(pd.name, static_cast<int64_t>(pd.value.integer_value));
                break;
            case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
                rcl_param = rclcpp::Parameter(pd.name, pd.value.double_value);
                break;
            case rcl_interfaces::msg::ParameterType::PARAMETER_STRING:
                rcl_param = rclcpp::Parameter(pd.name, pd.value.string_value);
                break;
            // Handle arrays if needed...
            default:
                RCLCPP_WARN(node->get_logger(), "Parameter type for '%s' not handled.", pd.name.c_str());
                continue;
            }
            request->parameters.push_back(rcl_param.to_parameter_msg());
        }

        if (client->wait_for_service(std::chrono::seconds(5)))
        {
            auto future = client->async_send_request(request);
            if (rclcpp::spin_until_future_complete(node, future) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO(
                    node->get_logger(),
                    "Controller %s configured with saved parameters.",
                    controller_name.c_str());
                return true;
            }
            else
            {
                RCLCPP_ERROR(node->get_logger(), "Failed to set parameters for %s.", controller_name.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "Service %s/set_parameters not available.",
                controller_name.c_str());
        }
        return false;
    }

    bool activateController(
        rclcpp::Node::SharedPtr node,
        const std::string &controller_name)
    {
        // /controller_manager/switch_controller
        auto client = node->create_client<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller");

        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request->activate_controllers.push_back(controller_name);
        request->deactivate_controllers.clear();
        // If there's an old controller to stop, add it to stop_controllers
        // request->stop_controllers.push_back("old_controller_name");
        request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
        // Options:
        // BEST_EFFORT = tries to switch but won't fail if there's an issue
        // STRICT = must succeed or the call fails

        // If you want the controller to start immediately:
        request->start_asap = false; 
        request->timeout.sec = 0;
        request->timeout.nanosec = 0;

        if (client->wait_for_service(std::chrono::seconds(5)))
        {
            auto future = client->async_send_request(request);
            if (rclcpp::spin_until_future_complete(node, future) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                if (future.get()->ok)
                {
                    RCLCPP_INFO(node->get_logger(), "Controller %s activated.", controller_name.c_str());
                    return true;
                }
                else
                {
                    RCLCPP_ERROR(node->get_logger(), "Failed to activate controller %s.", controller_name.c_str());
                }
            }
            else
            {
                RCLCPP_ERROR(node->get_logger(), "Failed to call switch_controller service.");
            }
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Service switch_controller not available.");
        }
        return false;
    }
};
