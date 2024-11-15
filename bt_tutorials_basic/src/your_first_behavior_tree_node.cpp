#include "behaviortree_cpp/bt_factory.h"
#include "bt_tutorials_basic/your_first_behavior_tree.hpp"

using namespace your_first_behavior_tree;

int main()
{
    // We use the BehaviorTreeFactory to register our custom nodes
    BT::BehaviorTreeFactory factory;

    // The recommended way to create a Node is through inheritance.
    factory.registerNodeType<ApproachObject>("ApproachObject");

    // Registering a SimpleActionNode using a simple function.
    // You can use C++11 lambdas or std::bind
    factory.registerSimpleCondition("CheckBattery", [&](BT::TreeNode &) { return CheckBattery(); });

    // Registering a SimpleActionNode using a function pointer.
    factory.registerSimpleCondition("CheckBatteryFP", CheckBatteryFP);

    // You can also create SimpleActionNodes using methods of a class
    GripperInterface gripper;
    factory.registerSimpleAction("OpenGripper", [&](BT::TreeNode &) { return gripper.open(); });
    factory.registerSimpleAction("CloseGripper", [&](BT::TreeNode &) { return gripper.close(); });

    // Trees are created at deployment-time (i.e. at run-time, but only
    // once at the beginning).

    // IMPORTANT: when the object "tree" goes out of scope, all the
    // TreeNodes are destroyed
    auto tree =
        factory.createTreeFromFile("/home/crasun/ws_ros2/src/bt_tutorials/bt_tutorials_basic/trees/your_first_behavior_tree.xml");

    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.
    tree.tickWhileRunning();

    return 0;
}

/* Expected output:
*
  [ Battery: OK ]
  [ Battery via FP: OK ]
  GripperInterface::open
  ApproachObject: approach_object
  GripperInterface::close
*/
