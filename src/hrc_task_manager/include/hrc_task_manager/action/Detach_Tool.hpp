#ifndef HRC_TASK_MANAGER__ACTION__DETACH_TOOL_HPP
#define HRC_TASK_MANAGER__ACTION__DETACH_TOOL_HPP

#include "behaviortree_cpp/bt_factory.h"
#include <string>

namespace hrc_task_manager
{
    // Action node: Detaches the old tool in the Pick & Place task.
    class Detach_Tool : public BT::SyncActionNode
    {
    public:
        Detach_Tool(const std::string& name, const BT::NodeConfig& config);
        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts();
    };

} // namespace hrc_task_manager

#endif // HRC_TASK_MANAGER__ACTION__DETACH_TOOL_HPP
