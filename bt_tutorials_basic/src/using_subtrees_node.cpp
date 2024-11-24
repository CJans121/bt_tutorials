#include "bt_tutorials_basic/using_subtrees.hpp"

using namespace using_subtrees;
int main()
{
    BT::BehaviorTreeFactory factory;

    CrossDoor cross_door;
    cross_door.registerNodes(factory);

    factory.registerBehaviorTreeFromFile(
        "/home/crasun/ws_ros2/src/bt_tutorials/bt_tutorials_basic/trees/using_subtrees.xml");

    auto tree = factory.createTree("MainTree");

    std::cout << "\nPrinting tree:";
    BT::printTreeRecursively(tree.rootNode());

    // Setting the RetryUntilSuccessful num_attempts in the tree XML to 3 or more ensures that the picking lock strategy
    // succeeds in opening the door. Otherwise, the fallback strategy of smashing the door is used.

    std::cout << "\nTicking tree:";
    tree.tickWhileRunning();

    return 0;
}
