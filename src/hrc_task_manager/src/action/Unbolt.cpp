#include "hrc_task_manager/action/Unbolt.hpp"
#include <hrc_task_manager/service_utils.hpp>
#include <iostream>

namespace hrc_task_manager
{

  Unbolt::Unbolt(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  BT::NodeStatus Unbolt::tick()
  {
      std::cout << "[Unbolt] unbolting... unbolting..." << std::endl;
      ServiceUtils::delay_ms(500);  
        
        bool success = ServiceUtils::call_service("Unbolt");
        if (success) {
            std::cout << "[Unbolt] ✓ Robot has successfully unbolt the screw\n" << std::endl;
            return BT::NodeStatus::SUCCESS;
        } else {
            std::cout << "[Unbolt] ✗ Robot failed to unbolt screw\n" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
  }

  BT::PortsList Unbolt::providedPorts()
  {
      return {};
  }

}  // namespace hrc_task_manager
