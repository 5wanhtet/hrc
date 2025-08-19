#include "hrc_task_manager/action/Attach_Tool.hpp"
#include <hrc_task_manager/service_utils.hpp>
#include <iostream>

namespace hrc_task_manager
{

  Attach_Tool::Attach_Tool(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  BT::NodeStatus Attach_Tool::tick()
  {
    std::cout << "[AttachTool] Attaching the new tool..." << std::endl;
    ServiceUtils::delay_ms(500);  
        
      bool success = ServiceUtils::call_service("Attach_Tool");
        if (success) {
            std::cout << "[AttachTool] ✓ Robot has successfully attached tool\n" << std::endl;
            return BT::NodeStatus::SUCCESS;
        } else {
            std::cout << "[AttachTool] ✗ Robot failed to attach tool\n" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
  }

  BT::PortsList Attach_Tool::providedPorts()
  {
      return {};
  }

}  // namespace hrc_task_manager

