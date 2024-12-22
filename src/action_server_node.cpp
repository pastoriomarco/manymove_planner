#include <rclcpp/rclcpp.hpp>
#include "manymove_planner/move_group_planner.hpp"
#include "manymove_planner/moveit_cpp_planner.hpp"
#include "action_server.cpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = rclcpp::Node::make_shared("action_server_node", "", node_options);

    // Retrieve parameters
    std::string planner_type;
    node->get_parameter_or<std::string>("planner_type", planner_type, "moveitcpp");

    std::string planning_group;
    node->get_parameter_or<std::string>("planning_group", planning_group, "lite6");
    std::string base_frame;
    node->get_parameter_or<std::string>("base_link", base_frame, "link_base");
    std::string tcp_frame;
    node->get_parameter_or<std::string>("tcp_frame", tcp_frame, "link_tcp");
    std::string traj_controller;
    node->get_parameter_or<std::string>("traj_controller", traj_controller, "lite6_traj_controller");

    // Instantiate the appropriate planner based on the planner_type parameter
    std::shared_ptr<PlannerInterface> planner;

    if (planner_type == "moveitcpp")
    {
        planner = std::make_shared<MoveItCppPlanner>(node, planning_group, base_frame, tcp_frame, traj_controller);
        RCLCPP_INFO(node->get_logger(), "===================================================");
        RCLCPP_INFO(node->get_logger(), "Using MoveItCppPlanner.");
        RCLCPP_INFO(node->get_logger(), "===================================================");
    }
    else if (planner_type == "movegroup")
    {
        planner = std::make_shared<MoveGroupPlanner>(node, planning_group, base_frame, tcp_frame, traj_controller);
        RCLCPP_INFO(node->get_logger(), "===================================================");
        RCLCPP_INFO(node->get_logger(), "Using MoveGroupPlanner.");
        RCLCPP_INFO(node->get_logger(), "===================================================");
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Unknown planner_type: %s. Valid options are 'moveitcpp' and 'movegroup'.", planner_type.c_str());
        return 1;
    }

    auto server = std::make_shared<MoveManipulatorActionServer>(node, planner);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
