#include <rclcpp/rclcpp.hpp>
#include <thread> 
#include "manymove_planner/manymove_planner.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    // Create the Node
    auto node = rclcpp::Node::make_shared("manymove_planner_node", "", node_options);

    // Set up executor and start spinning
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]()
                        { executor.spin(); });

    // Get parameters
    std::string planning_group;
    node->get_parameter_or<std::string>("planning_group", planning_group, "lite6");
    std::string base_frame;
    node->get_parameter_or<std::string>("base_link", base_frame, "link_base");
    std::string tcp_frame;
    node->get_parameter_or<std::string>("tcp_frame", tcp_frame, "link_tcp");
    std::string traj_controller;
    node->get_parameter_or<std::string>("traj_controller", traj_controller, "lite6_traj_controller");

    manymove_planner::msg::MovementConfig  max_move_config;
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

    ManyMovePlanner planner(node, planning_group, base_frame, tcp_frame, traj_controller);

    rclcpp::sleep_for(std::chrono::seconds(1));

    // moveit_msgs::msg::CollisionObject found_object;
    // bool test = planner.findCollisionObject("grasp", found_object);

    // if (test)
    // {
    //     RCLCPP_INFO(node->get_logger(), "Grasp object found!");
    // }
    // else
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Grasp object NOT found!!!!");
    // }

    rclcpp::sleep_for(std::chrono::seconds(1));

    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.x = 1.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.0;
    target_pose.position.x = 0.2;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.2;

    // bool success = planner.moveToPoseTarget(target_pose, slow_move_config);
    // if (success)
    // {
    //     RCLCPP_INFO(node->get_logger(), "Move to pose succeeded");
    // }
    // else
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Move to pose failed");
    // }

    // // Joint targets
    // std::vector<double> rest_joint_values = {0.0, -0.785, 0.785, 0.0, 1.57, 0.0};
    // std::vector<double> scan_sx_joint_values = {-0.175, -0.419, 1.378, 0.349, 1.535, -0.977};
    // std::vector<double> scan_dx_joint_values = {0.733, -0.297, 1.378, -0.576, 1.692, 1.291};

    // bool result = false;
    // int counter = 0;

    // // Move to joint target
    // do
    // {
    //     result = planner.moveToJointTarget(rest_joint_values, mid_move_config);
    //     counter++;
    // } while ((!result) && (counter < 16));

    // // Move to joint target
    // counter = 0;
    // do
    // {
    //     result = planner.moveToJointTarget(scan_sx_joint_values, max_move_config);
    //     counter++;
    // } while ((!result) && (counter < 16));

    // // Move to joint target
    // counter = 0;
    // do
    // {
    //     result = planner.moveToJointTarget(scan_dx_joint_values, max_move_config);
    //     counter++;
    // } while ((!result) && (counter < 16));

    // // Move to joint target
    // counter = 0;
    // do
    // {
    //     result = planner.moveToJointTarget(rest_joint_values, mid_move_config);
    //     counter++;
    // } while ((!result) && (counter < 16));

    // Shutdown and join the spinner thread
    rclcpp::shutdown();
    spinner.join();

    return 0;
}
