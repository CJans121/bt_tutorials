#include "behaviortree_cpp/bt_factory.h"
#include "bt_tutorials_basic/blackboard_and_ports.hpp"

using namespace blackboard_and_ports;

int main()
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");

    auto tree = factory.createTreeFromFile(
        "/home/crasun/ws_ros2/src/bt_tutorials/bt_tutorials_basic/trees/blackboard_and_ports.xml");
    tree.tickWhileRunning();
    return 0;
}

/*  Expected output:
  Robot says: hello
  Robot says: The answer is 42
*/
