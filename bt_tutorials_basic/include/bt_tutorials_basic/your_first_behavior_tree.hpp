#ifndef YOUR_FIRST_BEHAVIOR_TREE
#define YOUR_FIRST_BEHAVIOR_TREE

#include <behaviortree_cpp/action_node.h>
#include <iostream>
#include <string>

namespace your_first_behavior_tree
{

/**
 * @brief To demonstrate usage of a simple function to create a TreeNode
 *
 * @return
 */
BT::NodeStatus CheckBattery();

/**
 * @brief To demonstrate dependency injection to create a TreeNode using a
 * function pointer.
 *
 * @param self BT::TreeNode. Not used.
 *
 * @return
 */
BT::NodeStatus CheckBatteryFP(BT::TreeNode &self);

/**
 * @brief To demonstrate create of a TreeNode by inheritance. Recommended approach.
 */
class ApproachObject : public BT::SyncActionNode
{
  public:
    explicit ApproachObject(const std::string &name);

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
};

/**
 * @brief To demonstrate wrapping into an ActionNode the methods open() and close()
 */
class GripperInterface
{

  public:
    GripperInterface();

    BT::NodeStatus open();

    BT::NodeStatus close();

  private:
    bool _open; // shared information
};
} // namespace your_first_behavior_tree
#endif
