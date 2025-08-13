#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/xml_parsing.h>
#include <unordered_map>
#include <mutex>
#include <string>

// Generated from srv/RunBehaviorTree.srv
#include "hrc_task_manager/srv/run_behavior_tree.hpp"

// Include your BT node classes so the factory can register them
#include "hrc_task_manager/action/AskForHelp.hpp"
#include "hrc_task_manager/condition/CheckTool.hpp"
#include "hrc_task_manager/action/MoveTo.hpp"
#include "hrc_task_manager/action/PickObject.hpp"
#include "hrc_task_manager/condition/VerifyPosition.hpp"
#include "hrc_task_manager/action/PlaceObject.hpp"
#include "hrc_task_manager/action/DetachTool.hpp"
#include "hrc_task_manager/action/AttachTool.hpp"
#include "hrc_task_manager/action/Unbolt.hpp"
#include "hrc_task_manager/action/DetectObject.hpp"
#include "hrc_task_manager/action/AlignToolWithTarget.hpp"
#include "hrc_task_manager/action/ChangeTool.hpp"
#include "hrc_task_manager/service_utils.hpp"


// Helper to register all your custom BT nodes once per factory
void RegisterCustomNodes(BT::BehaviorTreeFactory& factory)
{
  // Register each custom node with the ID you use in XML
  factory.registerNodeType<hrc_task_manager::AskForHelp>("AskForHelp");
  factory.registerNodeType<hrc_task_manager::CheckTool>("Check_Tool");
  factory.registerNodeType<hrc_task_manager::MoveTo>("Move_To");
  factory.registerNodeType<hrc_task_manager::PickObject>("Pick_Object");
  factory.registerNodeType<hrc_task_manager::VerifyPosition>("Verify_Position");
  factory.registerNodeType<hrc_task_manager::PlaceObject>("Place_Object");
  factory.registerNodeType<hrc_task_manager::DetachTool>("Detach_Tool");
  factory.registerNodeType<hrc_task_manager::AttachTool>("Attach_Tool");
  factory.registerNodeType<hrc_task_manager::Unbolt>("Unbolt");
  factory.registerNodeType<hrc_task_manager::DetectObject>("DetectObject");
  factory.registerNodeType<hrc_task_manager::AlignToolWithTarget>("AlignToolWithTarget");
  factory.registerNodeType<hrc_task_manager::ChangeTool>("Change_Tool");
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
      {"PickAndPlace", "/home/robotara/hrc/src/hrc_task_manager/config/PickAndPlace.xml"},
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
