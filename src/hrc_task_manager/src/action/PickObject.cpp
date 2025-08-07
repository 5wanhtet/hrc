#include "hrc_task_manager/action/PickObject.hpp"
#include <hrc_task_manager/service_utils.hpp>
#include <iostream>

namespace hrc_task_manager
{

  PickObject::PickObject(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  BT::NodeStatus PickObject::tick()
  {
    std::cout << "[PickObject] EPicking up..." << std::endl;
    ServiceUtils::delay_ms(500);

      bool success = ServiceUtils::call_service("PickObject");
        if (success) {
            std::cout << "[PickObject] ✓ Robot successfully picked object\n" << std::endl;
            return BT::NodeStatus::SUCCESS;
        } 
        else 
        {
            std::cout << "[PickObject] ✗ Robot failed to pick object\n" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
  }

  BT::PortsList PickObject::providedPorts()
  {
      return {};
  }

}  // namespace hrc_task_manager
