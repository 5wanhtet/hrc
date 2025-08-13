#include <hrc_task_manager/action/Ask_For_Help.hpp>
#include <hrc_task_manager/service_utils.hpp>
#include <iostream>

namespace hrc_task_manager
{
    Ask_For_Help::Ask_For_Help(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus Ask_For_Help::tick()
    {
        std::cout << "[AskForhelp] Robot asks for help\n";
        ServiceUtils::delay_ms(500);  
        
        bool success = ServiceUtils::call_service("Ask_For_help");
        if (success) {
            std::cout << "[AskForhelp] ✓ Robot successfully asked for help\n" << std::endl;
            return BT::NodeStatus::SUCCESS;
        } else {
            std::cout << "[AskForhelp] ✗ Robot failed to ask for help\n" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    }

    BT::PortsList Ask_For_Help::providedPorts()
    {
        return {};
    }
} // namespace hrc_task_manager


