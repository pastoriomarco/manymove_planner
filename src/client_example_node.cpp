#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "manymove_planner/action/move_manipulator.hpp"
#include "manymove_planner/action/move_manipulator_sequence.hpp"
#include "manymove_planner/msg/move_manipulator_goal.hpp"
#include "geometry_msgs/msg/pose.hpp"

using MoveManipulator = manymove_planner::action::MoveManipulator;
using MoveManipulatorSequence = manymove_planner::action::MoveManipulatorSequence;
using MoveManipulatorGoalMsg = manymove_planner::msg::MoveManipulatorGoal;

class MoveManipulatorClient : public rclcpp::Node
{
public:
    MoveManipulatorClient() : Node("move_manipulator_client")
    {
        single_client_ = rclcpp_action::create_client<MoveManipulator>(this, "move_manipulator");
        sequence_client_ = rclcpp_action::create_client<MoveManipulatorSequence>(this, "move_manipulator_sequence");

        // Wait for servers
        if (!single_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_WARN(this->get_logger(), "Single-move action server not available");
        }

        if (!sequence_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_WARN(this->get_logger(), "Sequence action server not available");
        }

        // // Example: Send a single move
        // sendSingleMove();

        // Example: Send a sequence of moves
        sendSequenceOfMoves();
    }

private:
    rclcpp_action::Client<MoveManipulator>::SharedPtr single_client_;
    rclcpp_action::Client<MoveManipulatorSequence>::SharedPtr sequence_client_;

    void fillConfig(manymove_planner::msg::MovementConfig &config, double vel, double accel, double step, double jump, double max_speed, const std::string &smoothing)
    {
        config.velocity_scaling_factor = vel;
        config.acceleration_scaling_factor = accel;
        config.step_size = step;
        config.jump_threshold = jump;
        config.max_cartesian_speed = max_speed;
        config.max_exec_tries = 5;
        config.plan_number_target = 8;
        config.plan_number_limit = 16;
        config.smoothing_type = smoothing;
    }

    void sendSingleMove()
    {
        if (!single_client_)
        {
            RCLCPP_ERROR(this->get_logger(), "No single_client_ available");
            return;
        }

        MoveManipulator::Goal goal;
        // Setup a pose move
        goal.goal.movement_type = "pose";
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = 0.2;
        target_pose.position.y = 0.0;
        target_pose.position.z = 0.2;
        target_pose.orientation.x = 1.0;
        target_pose.orientation.y = 0.0;
        target_pose.orientation.z = 0.0;
        target_pose.orientation.w = 0.0;
        goal.goal.pose_target = target_pose;
        goal.goal.start_joint_values = {};
        fillConfig(goal.goal.config, 0.5, 0.5, 0.01, 0.02, 0.5, "time_optimal");

        auto send_goal_options = rclcpp_action::Client<MoveManipulator>::SendGoalOptions();
        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<MoveManipulator>::WrappedResult &result)
            {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                {
                    RCLCPP_INFO(this->get_logger(), "Single move succeeded: %s", result.result->message.c_str());
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Single move failed with code %d", (int)result.code);
                }
            };

        RCLCPP_INFO(this->get_logger(), "Sending single move goal");
        single_client_->async_send_goal(goal, send_goal_options);
    }

    void sendSequenceOfMoves()
    {
        if (!sequence_client_)
        {
            RCLCPP_ERROR(this->get_logger(), "No sequence_client_ available");
            return;
        }

        // Create a sequence of moves
        std::vector<MoveManipulatorGoalMsg> moves;

        // Joint move
        MoveManipulatorGoalMsg g1;
        g1.movement_type = "joint";
        g1.joint_values = {0.0, -0.785, 0.785, 0.0, 1.57, 0.0};
        fillConfig(g1.config, 0.9, 0.9, 0.01, 0.02, 0.2, "time_optimal");
        moves.push_back(g1);

        // Pose move
        MoveManipulatorGoalMsg g2;
        g2.movement_type = "pose";
        g2.pose_target.position.x = 0.2;
        g2.pose_target.position.y = 0.1;
        g2.pose_target.position.z = 0.2;
        g2.pose_target.orientation.x = 1.0;
        g2.pose_target.orientation.y = 0.0;
        g2.pose_target.orientation.z = 0.0;
        g2.pose_target.orientation.w = 0.0;
        fillConfig(g2.config, 0.1, 0.1, 0.01, 0.02, 0.1, "time_optimal");
        moves.push_back(g2);

        // Named move
        MoveManipulatorGoalMsg g3;
        g3.movement_type = "named";
        g3.named_target = "home";
        fillConfig(g3.config, 0.5, 0.5, 0.01, 0.02, 0.5, "time_optimal");
        moves.push_back(g3);

        MoveManipulatorSequence::Goal seq_goal;
        seq_goal.goals = moves;

        auto send_goal_options = rclcpp_action::Client<MoveManipulatorSequence>::SendGoalOptions();
        send_goal_options.feedback_callback =
            [this](rclcpp_action::ClientGoalHandle<MoveManipulatorSequence>::SharedPtr,
                   const std::shared_ptr<const MoveManipulatorSequence::Feedback> feedback)
            {
                RCLCPP_INFO(this->get_logger(), "Sequence execution progress: %.2f", feedback->progress);
            };

        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<MoveManipulatorSequence>::WrappedResult &result)
            {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                {
                    RCLCPP_INFO(this->get_logger(), "Sequence succeeded: %s", result.result->message.c_str());
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Sequence failed with code %d", (int)result.code);
                }
                // After testing, we can shutdown
                rclcpp::shutdown();
            };

        RCLCPP_INFO(this->get_logger(), "Sending sequence of %zu moves", moves.size());
        sequence_client_->async_send_goal(seq_goal, send_goal_options);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveManipulatorClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
