#ifndef HRC_TASK_MANAGER__ACTION__ALIGN_TOOL_WITH_TARGET_HPP_
#define HRC_TASK_MANAGER__ACTION__ALIGN_TOOL_WITH_TARGET_HPP_

#include <behaviortree_cpp/action_node.h>
#include <string>
namespace hrc_task_manager {
    // Action node: Performs micro adjustment/alignment of the tool with the target.
    class Align_Tool_With_Target : public BT::SyncActionNode
    {
    public:
        Align_Tool_With_Target(const std::string& name, const BT::NodeConfig& config);
        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts();
    };
}// namespace hrc_task_manager
#endif  // HRC_TASK_MANAGER__ACTION__ALIGN_TOOL_WITH_TARGET_HPP_
