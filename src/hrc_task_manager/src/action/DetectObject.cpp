#include "hrc_task_manager/action/DetectObject.hpp"
#include <hrc_task_manager/service_utils.hpp>
#include <iostream>

namespace hrc_task_manager
{

DetectObject::DetectObject(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{}

BT::NodeStatus DetectObject::tick()
{
  std::cout << "[DetectObject] Detecting object using vision system." << std::endl; 
  ServiceUtils::delay_ms(500);  

    bool success = ServiceUtils::call_service("DetectObject");
      if (success) {
            std::cout << "[DetectObject] ✓ Robot has successfully deteced object\n" << std::endl;
            return BT::NodeStatus::SUCCESS;
        } 
      else 
        {
            std::cout << "[DetectObject] ✗ Robot has failed to detect object\n" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
}

BT::PortsList DetectObject::providedPorts()
{
    return {};
}

} // namespace hrc_task_manager
