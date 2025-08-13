#ifndef HRC_TASK_MANAGER__CONDITION__VERIFY_POSITION_HPP
#define HRC_TASK_MANAGER__CONDITION__VERIFY_POSITION_HPP

#include "behaviortree_cpp/bt_factory.h"
#include <string>

namespace hrc_task_manager
{
    // Condition node: Checks if the system/robot is at the expected position.
    class Verify_Position : public BT::ConditionNode
    {
    public:
        Verify_Position(const std::string& name, const BT::NodeConfig& config);
        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts();
    };

} // namespace hrc_task_manager

#endif // HRC_TASK_MANAGER__CONDITION__VERIFY_POSITION_HPP
