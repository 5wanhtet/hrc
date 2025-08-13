#include "hrc_task_manager/action/Move_To.hpp"
#include <hrc_task_manager/service_utils.hpp>
#include <iostream>

namespace hrc_task_manager
{

  Move_To::Move_To(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  BT::NodeStatus Move_To::tick()
  {
    std::cout << "[MoveTo] Moving to the target location..." << std::endl;
    ServiceUtils::delay_ms(500);  
        
      bool success = ServiceUtils::call_service("Move_To");
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

  BT::PortsList Move_To::providedPorts()
  {
      return {};
  }

}  // namespace hrc_task_manager
