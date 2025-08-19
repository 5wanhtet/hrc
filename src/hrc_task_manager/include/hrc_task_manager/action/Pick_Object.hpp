#ifndef HRC_TASK_MANAGER__ACTION__PICK_OBJECT_HPP_
#define HRC_TASK_MANAGER__ACTION__PICK_OBJECT_HPP_

#include <behaviortree_cpp/action_node.h>
#include <string>

namespace hrc_task_manager {

    // Action node: Handles the logic for picking up an object
    // (e.g., using gripper, suction, or pneumatic methods).
    class Pick_Object : public BT::SyncActionNode
    {
    public:
        Pick_Object(const std::string& name, const BT::NodeConfig& config);
        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts();
    };
}
#endif  // HRC_TASK_MANAGER__ACTION__PICK_OBJECT_HPP_
