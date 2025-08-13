#ifndef HRC_TASK_MANAGER__ACTION__CHANGE_TOOL_HPP_
#define HRC_TASK_MANAGER__ACTION__CHANGE_TOOL_HPP_

#include <behaviortree_cpp/action_node.h>
#include <string>

namespace hrc_task_manager
{
    // Action node for changing tools: drops the current tool and picks up a new one.
    class Change_Tool : public BT::SyncActionNode
    {
    public:
        Change_Tool(const std::string& name, const BT::NodeConfig& config);
        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts();
    };
}  // namespace hrc_task_manager

#endif  // HRC_TASK_MANAGER__ACTION__CHANGE_TOOL_HPP_
