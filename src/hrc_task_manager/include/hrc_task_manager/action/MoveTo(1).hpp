#ifndef HRC_TASK_MANAGER__ACTION__MOVE_TO_HPP_
#define HRC_TASK_MANAGER__ACTION__MOVE_TO_HPP_

#include <behaviortree_cpp/action_node.h>
#include <string>

namespace hrc_task_manager {

    // Action node: Performs generic motion to a target location.
    // The position to move to can be passed through ports (parameters).
    class MoveTo : public BT::SyncActionNode
    {
    public:
        MoveTo(const std::string& name, const BT::NodeConfig& config);
        BT::NodeStatus tick() override;

        // Declare ports (e.g., position name or coordinates passed as input)
        static BT::PortsList providedPorts();
    };
}
#endif  // HRC_TASK_MANAGER__ACTION__MOVE_TO_HPP_
