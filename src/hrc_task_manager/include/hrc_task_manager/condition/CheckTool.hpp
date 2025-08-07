#ifndef HRC_TASK_MANAGER__CONDITION__CHECK_TOOL_HPP_
#define HRC_TASK_MANAGER__CONDITION__CHECK_TOOL_HPP_

#include <behaviortree_cpp/condition_node.h>
#include <string>

namespace hrc_task_manager {

    // Inherit from ConditionNode for condition logic.
    class CheckTool : public BT::ConditionNode
    {
    public:
        CheckTool(const std::string& name, const BT::NodeConfig& config);
        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts();
    };
}
#endif  // HRC_TASK_MANAGER__CONDITION__CHECK_TOOL_HPP_
