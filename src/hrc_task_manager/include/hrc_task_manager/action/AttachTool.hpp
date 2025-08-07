#ifndef HRC_TASK_MANAGER__ACTION__ATTACH_TOOL_HPP
#define HRC_TASK_MANAGER__ACTION__ATTACH_TOOL_HPP

#include "behaviortree_cpp/bt_factory.h"
#include <string>

namespace hrc_task_manager
{
    // Action node: Attach a new tool in Pick & Place task
    class AttachTool : public BT::SyncActionNode
    {
    public:
        AttachTool(const std::string& name, const BT::NodeConfig& config);
        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts();
    };

} // namespace hrc_task_manager

#endif // HRC_TASK_MANAGER__ACTION__ATTACH_TOOL_HPP
