#ifndef PORTS_WITH_GENERIC_TYPES
#define PORTS_WITH_GENERIC_TYPES

#include <behaviortree_cpp/action_node.h>
#include <iostream>
#include <string>

namespace ports_with_generic_types
{

/**
 * @brief Custom type to parse
 */
struct Position2D
{
    double x;
    double y;
};

/**
 * @brief To write into a port
 */
class CalculateGoal : public BT::SyncActionNode
{
  public:
    CalculateGoal(const std::string &name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

/**
 * @brief To read from a port
 */
class PrintTarget : public BT::SyncActionNode
{
  public:
    PrintTarget(const std::string &name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

} // namespace ports_with_generic_types

namespace BT
{

/**
 * @brief Template specialization to convert a string to Position2D
 */
template <> ports_with_generic_types::Position2D convertFromString(BT::StringView str);

} // namespace BT
#endif
