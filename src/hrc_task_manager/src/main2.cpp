#include <behaviortree_cpp/xml_parsing.h>
#include <behaviortree_cpp/bt_factory.h>
#include <iostream>
#include <thread>
#include <chrono>

// Include only your non-vision node headers here
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

void RegisterCustomNodes(BT::BehaviorTreeFactory& factory)
{
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
    //             <Ask_For_Help/>
    //         </Sequence>
    //       </BehaviorTree>
    //     </root>
    //         )";

    const std::string xml_filepath = "/home/robotara/hrc/src/hrc_task_manager/config/PickAndPlace.xml";

    factory.registerBehaviorTreeFromFile(xml_filepath);
    //auto tree = factory.createTreeFromText(xml_text);

    auto tree = factory.createTree("Pick_And_Place");

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
