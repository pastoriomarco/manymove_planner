#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "manymove_planner/action/move_manipulator.hpp"
#include "manymove_planner/msg/movement_config.hpp"
#include <geometry_msgs/msg/pose.hpp>

using MoveManipulator = manymove_planner::action::MoveManipulator;

bool sendAndExecuteGoal(
    const rclcpp::Node::SharedPtr &node,
    const rclcpp_action::Client<MoveManipulator>::SharedPtr &client,
    MoveManipulator::Goal &goal)
{
    auto goal_handle_future = client->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(node, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "send_goal call failed");
        return false;
    }
    auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
        return false;
    }

    auto result_future = client->async_get_result(goal_handle);
    RCLCPP_INFO(node->get_logger(), "Goal accepted, waiting for result...");

    if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "get_result call failed");
        return false;
    }

    auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(node->get_logger(), "Result: success=%d, message=%s", result.result->success, result.result->message.c_str());
        return true;
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Motion failed with result code %d", (int)result.code);
        return false;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("move_client_example_node");

    auto client = rclcpp_action::create_client<MoveManipulator>(node, "move_manipulator");
    if (!client->wait_for_action_server(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(node->get_logger(), "Action server not available");
        return 1;
    }

    // Building different movement configs for testing:
    manymove_planner::msg::MovementConfig max_move_config;
    node->get_parameter_or<double>("velocity_scaling_factor", max_move_config.velocity_scaling_factor, 0.5);
    node->get_parameter_or<double>("acceleration_scaling_factor", max_move_config.acceleration_scaling_factor, 0.5);
    node->get_parameter_or<double>("step_size", max_move_config.step_size, 0.05);
    node->get_parameter_or<double>("jump_threshold", max_move_config.jump_threshold, 0.0);
    node->get_parameter_or<double>("max_cartesian_speed", max_move_config.max_cartesian_speed, 0.5);
    node->get_parameter_or<int>("max_exec_tries", max_move_config.max_exec_tries, 5);
    node->get_parameter_or<int>("plan_number_target", max_move_config.plan_number_target, 12);
    node->get_parameter_or<int>("plan_number_limit", max_move_config.plan_number_limit, 32);
    node->get_parameter_or<std::string>("smoothing_type", max_move_config.smoothing_type, "time_optimal");

    manymove_planner::msg::MovementConfig mid_move_config = max_move_config;
    mid_move_config.velocity_scaling_factor = max_move_config.velocity_scaling_factor / 2.0;
    mid_move_config.acceleration_scaling_factor = max_move_config.acceleration_scaling_factor / 2.0;
    mid_move_config.max_cartesian_speed = 0.2;

    manymove_planner::msg::MovementConfig slow_move_config = max_move_config;
    slow_move_config.velocity_scaling_factor = max_move_config.velocity_scaling_factor / 4.0;
    slow_move_config.acceleration_scaling_factor = max_move_config.acceleration_scaling_factor / 4.0;
    slow_move_config.max_cartesian_speed = 0.05;

    // moveit_msgs::msg::CollisionObject found_object;
    // bool test = findCollisionObject("grasp", found_object);

    // if (test)
    // {
    //     RCLCPP_INFO(node->get_logger(), "Grasp object found!");
    // }
    // else
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Grasp object NOT found!!!!");
    // }

    // Building different target/pose examples

    // pose
    geometry_msgs::msg::Pose pose_target;

    pose_target.orientation.x = 1.0;
    pose_target.orientation.y = 0.0;
    pose_target.orientation.z = 0.0;
    pose_target.orientation.w = 0.0;
    pose_target.position.x = 0.2;
    pose_target.position.y = 0.0;
    pose_target.position.z = 0.2;

    // named
    std::string named_target_ = "home";

    // joint
    std::vector<double> ready_joint_values = {0.0, 0.0, 1.57, 0.0, 0.0, 0.0};
    std::vector<double> rest_joint_values = {0.0, -0.785, 0.785, 0.0, 1.57, 0.0};
    std::vector<double> scan_sx_joint_values = {-0.175, -0.419, 1.378, 0.349, 1.535, -0.977};
    std::vector<double> scan_dx_joint_values = {0.733, -0.297, 1.378, -0.576, 1.692, 1.291};
    std::vector<double> shutdown_joint_values = {0.0, 0.175, 0.175, 0.0, 0.0, 0.0};

    // cartesian
    // std::vector<geometry_msgs::msg::Pose> cartesian_waypoints_;

    // Create a goal
    MoveManipulator::Goal goal;

    // Set goal parameters
    goal.movement_type = "joint";
    goal.joint_values = rest_joint_values;
    goal.config = mid_move_config;

    int current_exec_tries = 0;
    int max_overall_exec_tries = 16;

    while (current_exec_tries < max_overall_exec_tries)
    {
        if (sendAndExecuteGoal(node, client, goal))
        {
            break;
        }
        else
        {
            rclcpp::sleep_for(std::chrono::seconds(5));
            ++current_exec_tries;
        }
    }
    // same movement type, just update key fields
    goal.joint_values = scan_sx_joint_values;
    goal.config = max_move_config;

    while (current_exec_tries < max_overall_exec_tries)
    {
        if (sendAndExecuteGoal(node, client, goal))
        {
            break;
        }
        else
        {
            rclcpp::sleep_for(std::chrono::seconds(5));
            ++current_exec_tries;
        }
    }

    // same movement type and config, just update key fields
    goal.joint_values = scan_dx_joint_values;

    while (current_exec_tries < max_overall_exec_tries)
    {
        if (sendAndExecuteGoal(node, client, goal))
        {
            break;
        }
        else
        {
            rclcpp::sleep_for(std::chrono::seconds(5));
            ++current_exec_tries;
        }
    }

    // let's try pose movement
    goal.movement_type = "pose";
    goal.pose_target = pose_target;
    goal.config = slow_move_config;

    while (current_exec_tries < max_overall_exec_tries)
    {
        if (sendAndExecuteGoal(node, client, goal))
        {
            break;
        }
        else
        {
            rclcpp::sleep_for(std::chrono::seconds(5));
            ++current_exec_tries;
        }
    }

    // let's try pose movement
    goal.movement_type = "cartesian";
    goal.pose_target = pose_target;
    goal.pose_target.position.z += 0.05;
    goal.config = slow_move_config;

    while (current_exec_tries < max_overall_exec_tries)
    {
        if (sendAndExecuteGoal(node, client, goal))
        {
            break;
        }
        else
        {
            rclcpp::sleep_for(std::chrono::seconds(5));
            ++current_exec_tries;
        }
    }

    // back to rest position
    goal.movement_type = "joint";
    goal.joint_values = rest_joint_values;
    goal.config = mid_move_config;

    while (current_exec_tries < max_overall_exec_tries)
    {
        if (sendAndExecuteGoal(node, client, goal))
        {
            break;
        }
        else
        {
            rclcpp::sleep_for(std::chrono::seconds(5));
            ++current_exec_tries;
        }
    }

    rclcpp::shutdown();
    return 0;
}
