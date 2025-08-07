#include "hrc_task_manager/action/DetachTool.hpp"
#include <hrc_task_manager/service_utils.hpp>
#include <iostream>

namespace hrc_task_manager
{

  DetachTool::DetachTool(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  BT::NodeStatus DetachTool::tick()
  {
    std::cout << "[DetachTool] Detaching the old tool..." << std::endl;
     ServiceUtils::delay_ms(500);  
        
      bool success = ServiceUtils::call_service("DetachTool");
        if (success) 
         {
            std::cout << "[DetachTool] ✓ Robot successfully detected tool\n" << std::endl;
            return BT::NodeStatus::SUCCESS;
         } 
        else 
         {
            std::cout << "[DetachTool] ✗ Robot failed to detect tool\n" << std::endl;
            return BT::NodeStatus::FAILURE;
         }
  }

  BT::PortsList DetachTool::providedPorts()
  {
      return {};
  }

}  // namespace hrc_task_manager
