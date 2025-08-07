#include "hrc_task_manager/action/AttachTool.hpp"
#include <hrc_task_manager/service_utils.hpp>
#include <iostream>

namespace hrc_task_manager
{

  AttachTool::AttachTool(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  BT::NodeStatus AttachTool::tick()
  {
    std::cout << "[AttachTool] Attaching the new tool..." << std::endl;
    ServiceUtils::delay_ms(500);  
        
      bool success = ServiceUtils::call_service("AttachTool");
        if (success) {
            std::cout << "[AttachTool] ✓ Robot has successfully attached tool\n" << std::endl;
            return BT::NodeStatus::SUCCESS;
        } else {
            std::cout << "[AttachTool] ✗ Robot failed to attach tool\n" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
  }

  BT::PortsList AttachTool::providedPorts()
  {
      return {};
  }

}  // namespace hrc_task_manager

