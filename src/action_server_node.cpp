#include <rclcpp/rclcpp.hpp>
#include "manymove_planner/manymove_planner.hpp"
#include "action_server.cpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = rclcpp::Node::make_shared("action_server_node", "", node_options);

    std::string planning_group;
    node->get_parameter_or<std::string>("planning_group", planning_group, "lite6");
    std::string base_frame;
    node->get_parameter_or<std::string>("base_link", base_frame, "link_base");
    std::string tcp_frame;
    node->get_parameter_or<std::string>("tcp_frame", tcp_frame, "link_tcp");
    std::string traj_controller;
    node->get_parameter_or<std::string>("traj_controller", traj_controller, "lite6_traj_controller");

    auto planner = std::make_shared<ManyMovePlanner>(node, planning_group, base_frame, tcp_frame, traj_controller);
    auto server = std::make_shared<MoveManipulatorActionServer>(node, planner);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
