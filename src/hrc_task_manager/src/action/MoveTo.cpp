#include "hrc_task_manager/action/MoveTo.hpp"
#include <hrc_task_manager/service_utils.hpp>
#include <iostream>

namespace hrc_task_manager
{

  MoveTo::MoveTo(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  BT::NodeStatus MoveTo::tick()
  {
    std::cout << "[MoveTo] Moving to the target location..." << std::endl;
    ServiceUtils::delay_ms(500);  
        
      bool success = ServiceUtils::call_service("MoveTo");
        if (success) 
        {
            std::cout << "[MoveTo] ✓ Robot has successfully moved to target location\n" << std::endl;
            return BT::NodeStatus::SUCCESS;
        } 
        else 
        {
            std::cout << "[MoveTo] ✗ Robot has failed to move to target location\n" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
  }

  BT::PortsList MoveTo::providedPorts()
  {
      return {};
  }

}  // namespace hrc_task_manager
