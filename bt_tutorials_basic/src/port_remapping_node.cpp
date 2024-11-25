#include "bt_tutorials_basic/blackboard_and_ports.hpp"
#include "bt_tutorials_basic/reactive_behaviors.hpp"
#include <behaviortree_cpp/bt_factory.h>

using namespace blackboard_and_ports;
using namespace reactive_behaviors;

int main()
{

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerNodeType<MoveBaseAction>("MoveBase");

    factory.registerBehaviorTreeFromFile(
        "/home/crasun/ws_ros2/src/bt_tutorials/bt_tutorials_basic/trees/port_remapping.xml");
    auto tree = factory.createTree("MainTree");

    tree.tickWhileRunning();

    // visualize info about the blackboard current state
    std::cout << "\n------ First BB ------\n";
    // Debug message prints types of the blackboard keys in this subtree, which is useful
    tree.subtrees[0]->blackboard->debugMessage();
    std::cout << "\n------ Second BB ------\n";
    // Debug message prints remapping info
    tree.subtrees[1]->blackboard->debugMessage();

    return 0;
}
