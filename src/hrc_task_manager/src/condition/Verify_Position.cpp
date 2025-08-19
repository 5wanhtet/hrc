#include "hrc_task_manager/condition/Verify_Position.hpp"
#include <hrc_task_manager/service_utils.hpp>
#include <iostream>

namespace hrc_task_manager
{

    Verify_Position::Verify_Position(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
    {
        // Constructor body (if needed)
    }

    BT::NodeStatus Verify_Position::tick()
    {
        std::cout << "[VerifyPosition] Verifying current position..." << std::endl;
        // TODO: Add your actual verification logic here

        // For now, assume success
        bool success = ServiceUtils::call_service("Verify_Position");
        if (success) {
            std::cout << "[VerifyPosition] ✓ ...\n" << std::endl;
            return BT::NodeStatus::SUCCESS;
        } else {
            std::cout << "[VerifyPosition] ✗ ...\n" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    }

    BT::PortsList Verify_Position::providedPorts()
    {
        return {};  // Return any input/output ports if needed
    }

}  // namespace hrc_task_manager
