#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/xml_parsing.h>
#include <unordered_map>
#include <mutex>
#include <string>

// Generated from srv/RunBehaviorTree.srv
#include "hrc_task_manager/srv/run_behavior_tree.hpp"

// Include your BT node classes so the factory can register them
#include "hrc_task_manager/action/Ask_For_Help.hpp"
#include "hrc_task_manager/condition/Check_Tool.hpp"
#include "hrc_task_manager/action/Move_To.hpp"
#include "hrc_task_manager/action/Pick_Object.hpp"
#include "hrc_task_manager/condition/Verify_Position.hpp"
#include "hrc_task_manager/action/Place_Object.hpp"
#include "hrc_task_manager/action/Detach_Tool.hpp"
#include "hrc_task_manager/action/Attach_Tool.hpp"
#include "hrc_task_manager/action/Unscrewing.hpp"
#include "hrc_task_manager/action/Detect_Object.hpp"
#include "hrc_task_manager/action/Align_Tool_With_Target.hpp"
#include "hrc_task_manager/action/Change_Tool.hpp"
#include "hrc_task_manager/service_utils.hpp"


// Helper to register all your custom BT nodes once per factory
void RegisterCustomNodes(BT::BehaviorTreeFactory& factory)
{
  // Register each custom node with the ID you use in XML
  factory.registerNodeType<hrc_task_manager::Ask_For_Help>("Ask_For_Help");
  factory.registerNodeType<hrc_task_manager::Check_Tool>("Check_Tool");
  factory.registerNodeType<hrc_task_manager::Move_To>("Move_To");
  factory.registerNodeType<hrc_task_manager::Pick_Object>("Pick_Object");
  factory.registerNodeType<hrc_task_manager::Verify_Position>("Verify_Position");
  factory.registerNodeType<hrc_task_manager::Place_Object>("Place_Object");
  factory.registerNodeType<hrc_task_manager::Detach_Tool>("Detach_Tool");
  factory.registerNodeType<hrc_task_manager::Attach_Tool>("Attach_Tool");
  factory.registerNodeType<hrc_task_manager::Unscrewing>("Unscrewing");
  factory.registerNodeType<hrc_task_manager::Detect_Object>("Detect_Object");
  factory.registerNodeType<hrc_task_manager::Align_Tool_With_Target>("Align_Tool_With_Target");
  factory.registerNodeType<hrc_task_manager::Change_Tool>("Change_Tool");
}

class BTServerNode : public rclcpp::Node
{
public:
  BTServerNode()
  : rclcpp::Node("bt_server_node")
  {
    // Map logical tree IDs to XML file paths.
    // You can add more entries as you create more trees.
    tree_map_ = {
      {"Pick_And_Place", "/home/robotara/hrc/src/hrc_task_manager/config/Pick_And_Place.xml"},
      {"Process","/home/robotara/hrc/src/hrc_task_manager/config/process.xml"},
      {"Tool_Change","/home/robotara/hrc/src/hrc_task_manager/config/Tool_Change.xml"},
      {"Unbolting","/home/robotara/hrc/src/hrc_task_manager/config/Unbolting.xml"}
    };

    // Advertise the service. Clients will call this to run a BT.
    service_ = this->create_service<hrc_task_manager::srv::RunBehaviorTree>(
      "/hrc_task_manager/run_bt",
      std::bind(&BTServerNode::handle_request, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "BT Server ready on /hrc_task_manager/run_bt");
  }

private:
  // Service callback: runs when a client sends a request
  void handle_request(
    const std::shared_ptr<hrc_task_manager::srv::RunBehaviorTree::Request> req,
    std::shared_ptr<hrc_task_manager::srv::RunBehaviorTree::Response> res)
  {
    // Serialize executions so two clients can't run trees at the same time
    std::lock_guard<std::mutex> lk(mutex_);

    const std::string tree_id = req->tree_id;        // logical tree name (e.g., "PickAndPlace")
    const std::string xml_override = req->xml_path;  // optional: direct file path override

    // Resolve which XML file to load
    std::string xml_path;
    if (!xml_override.empty())
    {
      // Client provided an explicit XML path: use it directly
      xml_path = xml_override;
    }
    else
    {
      // Look up by tree_id in the map
      auto it = tree_map_.find(tree_id);
      if (it == tree_map_.end())
      {
        res->success = false;
        res->message = "Unknown tree_id: " + tree_id;
        RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
        return;
      }
      xml_path = it->second;
    }

    try
    {
      // Create a factory and register your custom nodes
      BT::BehaviorTreeFactory factory;
      RegisterCustomNodes(factory);

      // Load the behavior tree(s) from the XML file
      factory.registerBehaviorTreeFromFile(xml_path);

      // Create the tree by ID (assumes your XML has <BehaviorTree ID="PickAndPlace">)
      BT::Tree tree = factory.createTree(tree_id);

      // Optional: print the structure to console
      BT::printTreeRecursively(tree.rootNode());

      // Execute the tree synchronously until completion
      RCLCPP_INFO(get_logger(), "Executing BT id='%s' from '%s'",
                  tree_id.c_str(), xml_path.c_str());
      BT::NodeStatus status = tree.tickWhileRunning();

      // Convert final status to response
      const bool ok = (status == BT::NodeStatus::SUCCESS);
      res->success = ok;
      res->message = ok ? "Behavior Tree completed successfully"
                        : std::string("Behavior Tree finished with status: ") +
                          (status == BT::NodeStatus::FAILURE ? "FAILURE" :
                           status == BT::NodeStatus::RUNNING ? "RUNNING" : "UNKNOWN");

      RCLCPP_INFO(get_logger(), "Result: %s", res->message.c_str());
    }
    catch (const std::exception& e)
    {
      // Any exception in loading/execution -> report as failure
      res->success = false;
      res->message = std::string("Exception while running tree: ") + e.what();
      RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
    }
  }

  // Members
  std::mutex mutex_;  // ensures one execution at a time
  std::unordered_map<std::string, std::string> tree_map_;  // tree_id -> xml file path
  rclcpp::Service<hrc_task_manager::srv::RunBehaviorTree>::SharedPtr service_;  // the ROS2 service
};

int main(int argc, char** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_client_node");

  // Initialize ServiceUtils with the node 
  hrc_task_manager::ServiceUtils::initialize(node);

  // Spin the server node (blocks until Ctrl+C)
  rclcpp::spin(std::make_shared<BTServerNode>());

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}
