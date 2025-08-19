#include "hrc_task_manager/action/Place_Object.hpp"
#include <hrc_task_manager/service_utils.hpp>
#include <iostream>

namespace hrc_task_manager
{

  Place_Object::Place_Object(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  BT::NodeStatus Place_Object::tick()
  {
      std::cout << "[PlaceObject] Placing object at target location..." << std::endl;
      ServiceUtils::delay_ms(500);  
        
        bool success = ServiceUtils::call_service("Place_Object");
        if (success) {
            std::cout << "[PlaceObject] ✓ Robot successfully placed object down\n" << std::endl;
            return BT::NodeStatus::SUCCESS;
        } else {
            std::cout << "[PlaceObject] ✗ Robot failed to place object\n" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
  }

  BT::PortsList Place_Object::providedPorts()
  {
      return {};
  }

}  // namespace hrc_task_manager
