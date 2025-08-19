#ifndef HRC_TASK_MANAGER__CONDITION__CHECK_TOOL_HPP_
#define HRC_TASK_MANAGER__CONDITION__CHECK_TOOL_HPP_

#include <behaviortree_cpp/condition_node.h>
#include <string>

namespace hrc_task_manager {

    // Inherit from ConditionNode for condition logic.
    class Check_Tool : public BT::ConditionNode
    {
    public:
        Check_Tool(const std::string& name, const BT::NodeConfig& config);
        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts();
    };
}
#endif  // HRC_TASK_MANAGER__CONDITION__CHECK_TOOL_HPP_
