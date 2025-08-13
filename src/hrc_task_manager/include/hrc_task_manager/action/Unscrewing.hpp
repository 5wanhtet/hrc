#ifndef HRC_TASK_MANAGER__ACTION__UNSCREWING_HPP_
#define HRC_TASK_MANAGER__ACTION__UNSCREWING_HPP_

#include <behaviortree_cpp/action_node.h>
#include <string>

namespace hrc_task_manager {

    // Action node: Implements the rotation unbolt logic for the robot.
    // This node commands the robot to perform unbolting during the Unbolting task.
    class Unscrewing : public BT::SyncActionNode
    {
    public:
        Unscrewing(const std::string& name, const BT::NodeConfig& config);
        
        // The tick function is called when executing this node. 
        // It should perform the unbolt operation (or simulate it)
        // and return SUCCESS, FAILURE, or RUNNING as appropriate.
        BT::NodeStatus tick() override;

        // Declare ports here if you want input/output parameters (optional)
        static BT::PortsList providedPorts();
    };
}
#endif  // HRC_TASK_MANAGER__ACTION__UNBOLT_HPP_
