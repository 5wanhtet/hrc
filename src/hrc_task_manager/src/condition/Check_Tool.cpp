#include "hrc_task_manager/condition/Check_Tool.hpp"
#include <hrc_task_manager/service_utils.hpp>
#include <iostream>

namespace hrc_task_manager
{

    Check_Tool::Check_Tool(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
    {
        // constructor body if needed
    }

    BT::NodeStatus Check_Tool::tick()
    {
        std::cout << "[CheckTool] Checking if the current tool is correct..." << std::endl;
        ServiceUtils::delay_ms(500); 
        // Your actual check logic here 
        
        bool success = ServiceUtils::call_service("Check_Tool");
        if (success) {
            std::cout << "[CheckTool] ✓ Tool is correct\n" << std::endl;
            return BT::NodeStatus::SUCCESS;
        } else {
            std::cout << "[CheckTool] ✗ Tool is incorrect, need to change to before movin on\n" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    }

    BT::PortsList Check_Tool::providedPorts()
    {
        return {};  // No ports defined currently; define ports here if needed.
    }

}  // namespace hrc_task_manager
