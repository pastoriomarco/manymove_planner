#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "manymove_planner/manymove_planner.hpp"
#include "manymove_planner/action/move_manipulator.hpp"

class MoveManipulatorActionServer
{
public:
    using MoveManipulator = manymove_planner::action::MoveManipulator;
    using GoalHandleMoveManipulator = rclcpp_action::ServerGoalHandle<MoveManipulator>;

    MoveManipulatorActionServer(const rclcpp::Node::SharedPtr &node,
                                const std::shared_ptr<ManyMovePlanner> &planner)
        : node_(node), planner_(planner)
    {
        action_server_ = rclcpp_action::create_server<MoveManipulator>(
            node_,
            "move_manipulator",
            std::bind(&MoveManipulatorActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveManipulatorActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&MoveManipulatorActionServer::handle_accepted, this, std::placeholders::_1));
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<ManyMovePlanner> planner_;
    rclcpp_action::Server<MoveManipulator>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MoveManipulator::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(node_->get_logger(), "Received MoveManipulator goal: %s", goal->movement_type.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveManipulator> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveManipulator> goal_handle)
    {
        std::thread{std::bind(&MoveManipulatorActionServer::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMoveManipulator> goal_handle)
    {
        RCLCPP_INFO(node_->get_logger(), "Executing MoveManipulator goal");
        auto result = std::make_shared<MoveManipulator::Result>();
        manymove_planner::msg::MovementConfig config;
        const auto goal = goal_handle->get_goal();
        config = goal->config;

        bool success = false;
        if (goal->movement_type == "pose") {
            success = planner_->moveToPoseTarget(goal->pose_target, config);
        } else if (goal->movement_type == "joint") {
            success = planner_->moveToJointTarget(goal->joint_values, config);
        } else if (goal->movement_type == "named") {
            success = planner_->moveToNamedTarget(goal->named_target, config);
        } else if (goal->movement_type == "cartesian") {
            success = planner_->moveCartesianPath(goal->pose_target, config);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Unknown movement_type: %s", goal->movement_type.c_str());
            result->success = false;
            result->message = "Unknown movement_type";
            goal_handle->abort(result);
            return;
        }

        if (goal_handle->is_canceling()) {
            RCLCPP_INFO(node_->get_logger(), "Goal canceled");
            result->success = false;
            result->message = "Canceled by client";
            goal_handle->canceled(result);
            return;
        }

        if (success) {
            RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
            result->success = true;
            result->message = "Motion complete";
            goal_handle->succeed(result);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Goal failed");
            result->success = false;
            result->message = "Motion failed";
            goal_handle->abort(result);
        }
    }
};