#include "hrc_task_manager/condition/VerifyPosition.hpp"
#include <hrc_task_manager/service_utils.hpp>
#include <iostream>

namespace hrc_task_manager
{

    VerifyPosition::VerifyPosition(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
    {
        // Constructor body (if needed)
    }

    BT::NodeStatus VerifyPosition::tick()
    {
        std::cout << "[VerifyPosition] Verifying current position..." << std::endl;
        // TODO: Add your actual verification logic here

        // For now, assume success
        bool success = ServiceUtils::call_service("VerifyPosition");
        if (success) {
            std::cout << "[VerifyPosition] ✓ ...\n" << std::endl;
            return BT::NodeStatus::SUCCESS;
        } else {
            std::cout << "[VerifyPosition] ✗ ...\n" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    }

    BT::PortsList VerifyPosition::providedPorts()
    {
        return {};  // Return any input/output ports if needed
    }

}  // namespace hrc_task_manager
