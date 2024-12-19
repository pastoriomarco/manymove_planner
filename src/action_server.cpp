#include "manymove_planner/planner_interface.hpp"
#include "manymove_planner/action/move_manipulator.hpp"
#include "manymove_planner/action/move_manipulator_sequence.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class MoveManipulatorActionServer
{
public:
    using MoveManipulator = manymove_planner::action::MoveManipulator;
    using MoveManipulatorSequence = manymove_planner::action::MoveManipulatorSequence;
    using GoalHandleMoveManipulator = rclcpp_action::ServerGoalHandle<MoveManipulator>;
    using GoalHandleMoveManipulatorSequence = rclcpp_action::ServerGoalHandle<MoveManipulatorSequence>;

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
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<PlannerInterface> planner_;

    rclcpp_action::Server<MoveManipulator>::SharedPtr single_action_server_;
    rclcpp_action::Server<MoveManipulatorSequence>::SharedPtr sequence_action_server_;

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
};
