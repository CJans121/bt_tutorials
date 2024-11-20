#include "bt_tutorials_basic/ports_with_generic_types.hpp"
#include <behaviortree_cpp/bt_factory.h>

using namespace ports_with_generic_types;
int main()
{
    BT::BehaviorTreeFactory factory;

    // Register nodes
    factory.registerNodeType<CalculateGoal>("CalculateGoal");
    factory.registerNodeType<PrintTarget>("PrintTarget");

    // Create and run the tree
    auto tree = factory.createTreeFromFile(
        "/home/crasun/ws_ros2/src/bt_tutorials/bt_tutorials_basic/trees/ports_with_generic_types.xml");
    tree.tickWhileRunning();

    return 0;
}
