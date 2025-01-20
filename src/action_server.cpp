#include "manymove_planner/planner_interface.hpp"
#include "manymove_planner/action/plan_manipulator.hpp"
#include "manymove_planner/action/move_manipulator.hpp"
#include "manymove_planner/action/move_manipulator_sequence.hpp"
#include "manymove_planner/action/execute_trajectory.hpp"
#include <rclcpp/rclcpp.hpp>
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

    MoveManipulatorActionServer(const rclcpp::Node::SharedPtr &node,
                                const std::shared_ptr<PlannerInterface> &planner)
        : node_(node), planner_(planner)
    {
        single_action_server_ = rclcpp_action::create_server<MoveManipulator>(
            node_,
            "move_manipulator",
            std::bind(&MoveManipulatorActionServer::handle_single_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveManipulatorActionServer::handle_single_cancel, this, std::placeholders::_1),
            std::bind(&MoveManipulatorActionServer::handle_single_accepted, this, std::placeholders::_1));

        sequence_action_server_ = rclcpp_action::create_server<MoveManipulatorSequence>(
            node_,
            "move_manipulator_sequence",
            std::bind(&MoveManipulatorActionServer::handle_sequence_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveManipulatorActionServer::handle_sequence_cancel, this, std::placeholders::_1),
            std::bind(&MoveManipulatorActionServer::handle_sequence_accepted, this, std::placeholders::_1));

        plan_action_server_ = rclcpp_action::create_server<PlanManipulator>(
            node_,
            "plan_manipulator",
            std::bind(&MoveManipulatorActionServer::handle_plan_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveManipulatorActionServer::handle_plan_cancel, this, std::placeholders::_1),
            std::bind(&MoveManipulatorActionServer::handle_plan_accepted, this, std::placeholders::_1));

        execute_action_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
            node_,
            "execute_manipulator_traj",
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
            "stop_motion",
            std::bind(&MoveManipulatorActionServer::handle_stop_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveManipulatorActionServer::handle_stop_cancel, this, std::placeholders::_1),
            std::bind(&MoveManipulatorActionServer::handle_stop_accept, this, std::placeholders::_1));
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<PlannerInterface> planner_;

    rclcpp_action::Server<MoveManipulator>::SharedPtr single_action_server_;
    rclcpp_action::Server<MoveManipulatorSequence>::SharedPtr sequence_action_server_;

    rclcpp_action::Server<PlanManipulator>::SharedPtr plan_action_server_;
    rclcpp_action::Server<ExecuteTrajectory>::SharedPtr execute_action_server_;
    rclcpp_action::Server<ExecuteTrajectory>::SharedPtr stop_motion_server_;

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
    // StopMotion callbacks:
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
        double dec_time = 0.25;
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
};
