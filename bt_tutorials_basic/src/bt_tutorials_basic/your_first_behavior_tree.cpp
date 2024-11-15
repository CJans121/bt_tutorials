#include "bt_tutorials_basic/your_first_behavior_tree.hpp"
namespace your_first_behavior_tree
{
BT::NodeStatus CheckBattery()
{
    std::cout << "[ Battery: OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CheckBatteryFP(BT::TreeNode &self)
{
    std::cout << "[ Battery via FP: OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

ApproachObject::ApproachObject(const std::string &name) : BT::SyncActionNode(name, {}) {}

BT::NodeStatus ApproachObject::tick()
{
    std::cout << "ApproachObject: " << this->name() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

GripperInterface::GripperInterface() : _open(true) {}

BT::NodeStatus GripperInterface::open()
{
    _open = true;
    std::cout << "GripperInterface::open" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GripperInterface::close()
{
    std::cout << "GripperInterface::close" << std::endl;
    _open = false;
    return BT::NodeStatus::SUCCESS;
}
} // namespace your_first_behavior_tree
