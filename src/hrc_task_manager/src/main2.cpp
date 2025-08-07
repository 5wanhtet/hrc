#include <behaviortree_cpp/xml_parsing.h>
#include <behaviortree_cpp/bt_factory.h>
#include <iostream>
#include <thread>
#include <chrono>

// Include only your non-vision node headers here
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

void RegisterCustomNodes(BT::BehaviorTreeFactory& factory)
{
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

int main(int argc, char** argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("bt_client_node");
    
    // Initialize ServiceUtils with the node 
    hrc_task_manager::ServiceUtils::initialize(node);

    BT::BehaviorTreeFactory factory;

    RegisterCustomNodes(factory);

    // static const char* xml_text = R"(
    //     <root BTCPP_format="4">
    //       <BehaviorTree ID="Untitled">
    //         <Sequence>
    //             <AskForHelp/>
    //         </Sequence>
    //       </BehaviorTree>
    //     </root>
    //         )";

    const std::string xml_filepath = "/home/robotara/hrc_ws/src/hrc_task_manager/config/PickAndPlace.xml";

    factory.registerBehaviorTreeFromFile(xml_filepath);
    //auto tree = factory.createTreeFromText(xml_text);

    auto tree = factory.createTree("PickAndPlace");

    // helper function to print the tree
    BT::printTreeRecursively(tree.rootNode());

    // wait for dummy  node to be available
    std::cout << "Checking for dummy service..." << std::endl;
    if (!hrc_task_manager::ServiceUtils::wait_for_service(std::chrono::seconds(10))) {
        std::cout << "Failed to connect, dummy_node is not running!" << std::endl;
        return 1;
    }
    
    //execute tree when dummy node is available
    std::cout << "\n=== Starting Behavior Tree Execution ===" << std::endl;
    
    auto status = tree.tickWhileRunning();
    
    std::cout << "\n=== Execution Complete ===" << std::endl;
    std::cout << "Final status: ";
    switch(status) {
        case BT::NodeStatus::SUCCESS:
            std::cout << "SUCCESS" << std::endl;
            break;
        case BT::NodeStatus::FAILURE:
            std::cout << "FAILURE" << std::endl;
            break;
        case BT::NodeStatus::RUNNING:
            std::cout << "RUNNING (unexpected)" << std::endl;
            break;
        default:
            std::cout << "UNKNOWN" << std::endl;
            break;
    }
    
    //cleanup
    hrc_task_manager::ServiceUtils::shutdown();
    rclcpp::shutdown();

    return 0;
}
