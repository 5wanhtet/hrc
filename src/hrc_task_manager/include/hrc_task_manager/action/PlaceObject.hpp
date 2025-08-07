#ifndef HRC_TASK_MANAGER__ACTION__PLACE_OBJECT_HPP_
#define HRC_TASK_MANAGER__ACTION__PLACE_OBJECT_HPP_

#include <behaviortree_cpp/action_node.h>
#include <string>

namespace hrc_task_manager {

    // Action node: Releases the held object, completing the place operation in Pick & Place task.
    class PlaceObject : public BT::SyncActionNode
    {
    public:
        PlaceObject(const std::string& name, const BT::NodeConfig& config);
        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts();
    };
}
#endif  // HRC_TASK_MANAGER__ACTION__PLACE_OBJECT_HPP_
