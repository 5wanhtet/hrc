#include "hrc_task_manager/action/Change_Tool.hpp"
#include <hrc_task_manager/service_utils.hpp>
#include <iostream>

namespace hrc_task_manager
{

  Change_Tool::Change_Tool(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  BT::NodeStatus Change_Tool::tick()
  {
      auto res = getInput<std::string>("tool_location");
        std::string location = res ? res.value() : "(undefined)";
        std::cout << "[ChangeTool] Changing tool...: " << location << " ... ";
        ServiceUtils::delay_ms(500);  
        
        bool success = ServiceUtils::call_service("Change_Tool");
        if (success) {
            std::cout << "[ChangeTool] ✓ Robot successfully changed tool\n" << std::endl;
            return BT::NodeStatus::SUCCESS;
        } else {
            std::cout << "[ChangeTool] ✗ Robot failed to change tool\n" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
  }

  BT::PortsList Change_Tool::providedPorts()
  {
    return { BT::InputPort<std::string>("tool_location") };
  }

} // namespace hrc_task_manager
