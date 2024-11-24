#include "bt_tutorials_basic/using_subtrees.hpp"
namespace using_subtrees
{
void CrossDoor::registerNodes(BT::BehaviorTreeFactory &factory)
{
    factory.registerSimpleCondition("IsDoorClosed", std::bind(&CrossDoor::isDoorClosed, this));
    factory.registerSimpleAction("PassThroughDoor", std::bind(&CrossDoor::passThroughDoor, this));
    factory.registerSimpleAction("OpenDoor", std::bind(&CrossDoor::openDoor, this));
    factory.registerSimpleAction("PickLock", std::bind(&CrossDoor::pickLock, this));
    factory.registerSimpleCondition("SmashDoor", std::bind(&CrossDoor::smashDoor, this));
}

BT::NodeStatus CrossDoor::isDoorClosed()
{
    if (!_door_open)
    {
        std::cout << "\nDoor closed? Yes";
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        std::cout << "\nDoor closed? No";
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus CrossDoor::passThroughDoor()
{
    if (_door_open)
    {
        std::cout << "\nPassed through door.\n";
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        std::cout << "\nFailed to pass through door. It is not open.";
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus CrossDoor::pickLock()
{
    _pick_attempts += 1;

    std::cout << "\nAttempted picking door lock " << _pick_attempts << " time(s). ";

    if (_pick_attempts >= 3)
    {
        _door_open = true;
        std::cout << "Successfully picked door lock." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus CrossDoor::openDoor()
{

    if (!_door_locked)
    {
        std::cout << "\nSuccessfully opened door";
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        std::cout << "\nFailed to open door. It is locked.";
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus CrossDoor::smashDoor()
{

    std::cout << "\nSmashed door";
    _door_open = true;
    return BT::NodeStatus::SUCCESS;
}
} // namespace using_subtrees
