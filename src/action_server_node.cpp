#include <rclcpp/rclcpp.hpp>
#include "manymove_planner/manymove_planner.hpp"
#include "action_server.cpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto node = rclcpp::Node::make_shared("action_server_node", "", node_options);

    std::string robot_type = "lite6";
    std::string base_frame = "link_base";
    std::string tcp_frame = "link_tcp";

    auto planner = std::make_shared<ManyMovePlanner>(node, robot_type, base_frame, tcp_frame);
    auto server = std::make_shared<MoveManipulatorActionServer>(node, planner);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
