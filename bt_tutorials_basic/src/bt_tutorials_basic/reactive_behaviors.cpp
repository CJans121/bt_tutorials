#include "bt_tutorials_basic/reactive_behaviors.hpp"

namespace reactive_behaviors
{

MoveBaseAction::MoveBaseAction(const std::string &name, const BT::NodeConfig &config) : StatefulActionNode(name, config)
{
}

BT::PortsList MoveBaseAction::providedPorts() { return {BT::InputPort<Pose2D>("goal")}; }

BT::NodeStatus MoveBaseAction::onStart()
{
    if (!getInput<Pose2D>("goal", _goal))
    {
        throw BT::RuntimeError("missing required input [goal]");
    }

    printf("[MoveBase: SEND REQUEST ]. goal: x=%f y=%f theta=%f\n", _goal.x, _goal.y, _goal.theta);

    // Delay to simulate some task
    _completion_time = std::chrono::system_clock::now() + std::chrono::milliseconds(220);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveBaseAction::onRunning()
{

    // Simulate quickly checking if some reply has been received. Do not block inside this function for too long.
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Check for task completion
    if (std::chrono::system_clock::now() >= _completion_time)
    {
        std::cout << "[ MoveBase: FINISHED ]" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING; // i.e. keep returning RUNNING until the task has finished
}

void MoveBaseAction::onHalted() { printf("[ MoveBASE: ABORTED ]"); }
} // namespace reactive_behaviors

// Specialize BT::convertFromString for Pose2D
template <> reactive_behaviors::Pose2D BT::convertFromString(StringView str)
{
    // Split the input string by semicolon
    auto parts = splitString(str, ';');

    // Ensure the string has exactly three parts
    if (parts.size() != 3)
    {
        throw BT::RuntimeError("invalid input for Pose2D: '" + std::string(str) + "'");
    }

    reactive_behaviors::Pose2D pose;
    pose.x = convertFromString<double>(parts[0]);
    pose.y = convertFromString<double>(parts[1]);
    pose.theta = convertFromString<double>(parts[2]);

    return pose;
}
