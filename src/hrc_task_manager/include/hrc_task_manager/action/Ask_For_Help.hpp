#ifndef HRC_TASK_MANAGER__ASK_FOR_HELP_HPP
#define HRC_TASK_MANAGER__ASK_FOR_HELP_HPP

#include "behaviortree_cpp/bt_factory.h"

namespace hrc_task_manager
{
    class Ask_For_Help : public BT::SyncActionNode
    {
        public:
            Ask_For_Help(const std::string& name, const BT::NodeConfig& config);
            BT::NodeStatus tick();
            static BT::PortsList providedPorts();
    };

} // namespace hrc_task_manager

#endif // HRC_TASK_MANAGER__ASK_FOR_HELP_HPP