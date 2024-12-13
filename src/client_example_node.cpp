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

        // Define possible configs
        manymove_planner::msg::MovementConfig max_move_config;
        max_move_config.velocity_scaling_factor = 0.5;
        max_move_config.acceleration_scaling_factor = 0.5;
        max_move_config.step_size = 0.05;
        max_move_config.jump_threshold = 0.0;
        max_move_config.max_cartesian_speed = 0.5;
        max_move_config.max_exec_tries = 5;
        max_move_config.plan_number_target = 8;
        max_move_config.plan_number_limit = 32;
        max_move_config.smoothing_type = "time_optimal";

        manymove_planner::msg::MovementConfig mid_move_config = max_move_config;
        mid_move_config.velocity_scaling_factor = max_move_config.velocity_scaling_factor / 2.0;
        mid_move_config.acceleration_scaling_factor = max_move_config.acceleration_scaling_factor / 2.0;
        mid_move_config.max_cartesian_speed = 0.2;

        manymove_planner::msg::MovementConfig slow_move_config = max_move_config;
        slow_move_config.velocity_scaling_factor = max_move_config.velocity_scaling_factor / 4.0;
        slow_move_config.acceleration_scaling_factor = max_move_config.acceleration_scaling_factor / 4.0;
        slow_move_config.max_cartesian_speed = 0.05;

        // Define possible moves:
        std::vector<double> rest_joint_values = {0.0, -0.785, 0.785, 0.0, 1.57, 0.0};
        std::vector<double> scan_sx_joint_values = {-0.175, -0.419, 1.378, 0.349, 1.535, -0.977};
        std::vector<double> scan_dx_joint_values = {0.733, -0.297, 1.378, -0.576, 1.692, 1.291};

        // Create a sequence of moves
        std::vector<MoveManipulatorGoalMsg> moves;

        // Joint move
        MoveManipulatorGoalMsg joint_rest;
        joint_rest.movement_type = "joint";
        joint_rest.joint_values = rest_joint_values;
        joint_rest.config = mid_move_config;
        moves.push_back(joint_rest);

        MoveManipulatorGoalMsg joint_scan_sx;
        joint_scan_sx.movement_type = "joint";
        joint_scan_sx.joint_values = scan_sx_joint_values;
        joint_scan_sx.config = max_move_config;
        moves.push_back(joint_scan_sx);

        // joint move copying part of the previous move:
        MoveManipulatorGoalMsg joint_scan_dx {joint_scan_sx};
        joint_scan_dx.joint_values = scan_dx_joint_values;
        moves.push_back(joint_scan_dx);

        // Pose move
        MoveManipulatorGoalMsg pose_test;
        pose_test.movement_type = "pose";
        pose_test.pose_target.position.x = 0.2;
        pose_test.pose_target.position.y = -0.1;
        pose_test.pose_target.position.z = 0.3;
        pose_test.pose_target.orientation.x = 1.0;
        pose_test.pose_target.orientation.y = 0.0;
        pose_test.pose_target.orientation.z = 0.0;
        pose_test.pose_target.orientation.w = 0.0;
        pose_test.config = mid_move_config;
        moves.push_back(pose_test);

        // Cartesian move using previous move as start
        pose_test.movement_type = "cartesian";
        pose_test.pose_target.position.z -= 0.1;
        pose_test.config = slow_move_config;
        moves.push_back(pose_test);

        // Named move
        MoveManipulatorGoalMsg named_home;
        named_home.movement_type = "named";
        named_home.named_target = "home";
        named_home.config = mid_move_config;
        moves.push_back(named_home);

        // Repeating initial move to get back to start:
        moves.push_back(joint_rest);

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
