#include <hrc_task_manager/action/AskForHelp.hpp>
#include <hrc_task_manager/service_utils.hpp>
#include <iostream>

namespace hrc_task_manager
{
    AskForHelp::AskForHelp(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus AskForHelp::tick()
    {
        std::cout << "[AskForHelp] Robot asks for help\n";
        ServiceUtils::delay_ms(500);  
        
        bool success = ServiceUtils::call_service("AskForHelp");
        if (success) {
            std::cout << "[AskForHelp] ✓ Robot successfully asked for help\n" << std::endl;
            return BT::NodeStatus::SUCCESS;
        } else {
            std::cout << "[AskForHelp] ✗ Robot failed to ask for help\n" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    }

    BT::PortsList AskForHelp::providedPorts()
    {
        return {};
    }
} // namespace hrc_task_manager


