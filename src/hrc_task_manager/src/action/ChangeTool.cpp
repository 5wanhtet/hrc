#include "hrc_task_manager/action/ChangeTool.hpp"
#include <hrc_task_manager/service_utils.hpp>
#include <iostream>

namespace hrc_task_manager
{

  ChangeTool::ChangeTool(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  BT::NodeStatus ChangeTool::tick()
  {
      auto res = getInput<std::string>("tool_location");
        std::string location = res ? res.value() : "(undefined)";
        std::cout << "[ChangeTool] Changing tool...: " << location << " ... ";
        ServiceUtils::delay_ms(500);  
        
        bool success = ServiceUtils::call_service("ChangeTool");
        if (success) {
            std::cout << "[ChangeTool] ✓ Robot successfully changed tool\n" << std::endl;
            return BT::NodeStatus::SUCCESS;
        } else {
            std::cout << "[ChangeTool] ✗ Robot failed to change tool\n" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
  }

  BT::PortsList ChangeTool::providedPorts()
  {
    return { BT::InputPort<std::string>("tool_location") };
  }

} // namespace hrc_task_manager
