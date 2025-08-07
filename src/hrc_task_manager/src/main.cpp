#include "behaviortree_cpp/bt_factory.h"
#include "hrc_task_manager/action/AskForHelp.hpp"

int main(int argc, char** argv)
{
    // ----- Behavior Tree Setup -----
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<hrc_task_manager::AskForHelp>("AskForHelp");

    static const char* xml_text = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="Untitled">
    <Sequence>
        <AskForHelp/>
    </Sequence>
  </BehaviorTree>
</root>
    )";

    // auto tree = factory.createTreeFromText(xml_text);
    auto tree = factory.createTreeFromFile(
        "/home/robotara/swan/Behaviour_Tree/0.o_For_Santosh/bt_ws/src/hrc_task_manager/config/test.xml");

    tree.tickWhileRunning();

    return 0;
}
