#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("dummy_service_node");
    
    auto service = node->create_service<std_srvs::srv::Trigger>(
        "bt_commands_service",  // match client name
        [](const std_srvs::srv::Trigger::Request::SharedPtr request,
           std_srvs::srv::Trigger::Response::SharedPtr response) {
            response->success = true;
            response->message = "Command executed successfully from dummy service";
            RCLCPP_INFO(rclcpp::get_logger("dummy_service"), 
                       "Service request received, responding with success");
        });

    RCLCPP_INFO(node->get_logger(), "Dummy service node is ready and waiting for requests...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}