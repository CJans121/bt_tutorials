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

    std::cout << "\nTicking tree:";
    tree.tickWhileRunning();

    return 0;
}
