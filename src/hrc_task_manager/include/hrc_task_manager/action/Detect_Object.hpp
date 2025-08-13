#ifndef HRC_TASK_MANAGER__ACTION__DETECT_OBJECT_HPP_
#define HRC_TASK_MANAGER__ACTION__DETECT_OBJECT_HPP_

#include <behaviortree_cpp/action_node.h>
#include <string>

namespace hrc_task_manager {

    // Action node: Commands the vision system to identify the target object
    class Detect_Object : public BT::SyncActionNode
    {
    public:
        Detect_Object(const std::string& name, const BT::NodeConfig& config);
        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts();
    };
}
#endif  // HRC_TASK_MANAGER__ACTION__DETECT_OBJECT_HPP_
