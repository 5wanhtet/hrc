#include "hrc_task_manager/action/Unscrewing.hpp"
#include <hrc_task_manager/service_utils.hpp>
#include <iostream>

namespace hrc_task_manager
{

  Unscrewing::Unscrewing(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  BT::NodeStatus Unscrewing::tick()
  {
      std::cout << "[Unscrewing] unbolting... unbolting..." << std::endl;
      ServiceUtils::delay_ms(500);  
        
        bool success = ServiceUtils::call_service("Unscrewing");
        if (success) {
            std::cout << "[Unscrewing] ✓ Robot has successfully unbolt the screw\n" << std::endl;
            return BT::NodeStatus::SUCCESS;
        } else {
            std::cout << "[Unscrewing] ✗ Robot failed to unbolt screw\n" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
  }

  BT::PortsList Unscrewing::providedPorts()
  {
      return {};
  }

}  // namespace hrc_task_manager
