#include "hrc_task_manager/action/AlignToolWithTarget.hpp"
#include <hrc_task_manager/service_utils.hpp>
#include <iostream>

namespace hrc_task_manager
{

  AlignToolWithTarget::AlignToolWithTarget(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  BT::NodeStatus AlignToolWithTarget::tick()
  {
    std::cout << "[AlignToolWithTarget] Performing micro adjustment...\n." << std::endl;
    ServiceUtils::delay_ms(500);  
        
      bool success = ServiceUtils::call_service("AlignToolWithTarget");
        if (success) 
         {
            std::cout << "[AlignToolWithTarget] ✓ Robot successfully aligned tool with target\n" << std::endl;
            return BT::NodeStatus::SUCCESS;
         } 
        else 
         {
            std::cout << "[AlignToolWithTarget] ✗ Robot failed to align tool with target\n" << std::endl;
            return BT::NodeStatus::FAILURE;
         }
  }

  BT::PortsList AlignToolWithTarget::providedPorts()
  {
      return {};
  }

}  // namespace hrc_task_manager
