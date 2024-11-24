#ifndef USING_SUBTREES
#define USING_SUBTREES

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <iostream>

namespace using_subtrees
{

class CrossDoor
{
  public:
    /**
     * @brief Helper method to register nodes
     */
    void registerNodes(BT::BehaviorTreeFactory &factory);

    /**
     * @brief  SUCCESS if _door_open != true
     */
    BT::NodeStatus isDoorClosed();

    /**
     * @brief  SUCCESS if _door_open == true
     */
    BT::NodeStatus passThroughDoor();

    /**
     * @brief  After 3 attempts, will open a locked door
     */
    BT::NodeStatus pickLock();

    /**
     * @brief  FAILURE if door locked
     */
    BT::NodeStatus openDoor();

    /**
     * @brief  Will always open door
     */
    BT::NodeStatus smashDoor();

  private:
    bool _door_open = false;
    bool _door_locked = true;
    int _pick_attempts = 0;
};
} // namespace using_subtrees

#endif
