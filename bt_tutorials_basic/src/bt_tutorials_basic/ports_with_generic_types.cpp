#include "bt_tutorials_basic/ports_with_generic_types.hpp"

namespace ports_with_generic_types
{

CalculateGoal::CalculateGoal(const std::string &name, const BT::NodeConfig &config) : SyncActionNode(name, config) {}

BT::PortsList CalculateGoal::providedPorts() { return {BT::OutputPort<Position2D>("goal")}; }

BT::NodeStatus CalculateGoal::tick()
{
    Position2D mygoal = {1.1, 2.3};
    setOutput<Position2D>("goal", mygoal);
    return BT::NodeStatus::SUCCESS;
}

PrintTarget::PrintTarget(const std::string &name, const BT::NodeConfig &config) : SyncActionNode(name, config) {}

BT::PortsList PrintTarget::providedPorts()
{
    // Optionally, a port may have a human-readable description
    const char *description = "Simply print the goal on console...";
    return {BT::InputPort<Position2D>("target", description)};
}

BT::NodeStatus PrintTarget::tick()
{
    auto res = getInput<Position2D>("target");

    if (!res)
    {
        throw BT::RuntimeError("error reading port [target]:", res.error());
    }

    Position2D target = res.value();
    printf("Target positions: [%.1f, %.1f ]\n", target.x, target.y);
    return BT::NodeStatus::SUCCESS;
}

} // namespace ports_with_generic_types

template <> ports_with_generic_types::Position2D BT::convertFromString(BT::StringView str)
{

    // We expect real numbers separated by semicolons
    auto parts = splitString(str, ';');

    if (parts.size() != 2)
    {
        throw BT::RuntimeError("invalid input");
    }
    else
    {
        printf("Converting string: \"%s\"\n", str.data());
        ports_with_generic_types::Position2D output;
        output.x = convertFromString<double>(parts[0]);
        output.y = convertFromString<double>(parts[1]);
        return output;
    }
}
