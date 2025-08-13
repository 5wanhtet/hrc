#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <string>

// Generated service header (from srv/RunBehaviorTree.srv)
#include "hrc_task_manager/srv/run_behavior_tree.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create a simple node for the client
  auto node = rclcpp::Node::make_shared("bt_client_node");

  // Create a client for the service exposed by the server
  auto client = node->create_client<hrc_task_manager::srv::RunBehaviorTree>(
      "/hrc_task_manager/run_bt");

  // Parse CLI args: bt_client [tree_id] [xml_path_optional]
  // Default tree_id if none provided
  std::string tree_id = "PickAndPlace";
  std::string xml_path;  // empty means "use server map"

  // NOTE: argv is an array; use argv[1], argv[2] when argc permits
  if (argc >= 2) { tree_id = argv[1]; }
  if (argc >= 3) { xml_path = argv[2]; }

  // Wait until the service becomes available
  while (!client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting...");
  }

  // Fill the request
  auto request = std::make_shared<hrc_task_manager::srv::RunBehaviorTree::Request>();
  request->tree_id = tree_id;
  request->xml_path = xml_path;     // leave empty to use server map
  request->params_json = "";        // reserved for future use

  // Send request asynchronously
  auto future = client->async_send_request(request);

  // Wait up to 60 seconds for the server to finish executing the BT
  auto ret = rclcpp::spin_until_future_complete(node, future, 60s);
  if (ret != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Service call failed or timed out");
    rclcpp::shutdown();
    return 1;
  }

  // Read the response
  auto response = future.get();
  RCLCPP_INFO(node->get_logger(), "Server response: success=%s, message=%s",
              response->success ? "true" : "false",
              response->message.c_str());

  // Shutdown
  rclcpp::shutdown();
  return response->success ? 0 : 2;
}
