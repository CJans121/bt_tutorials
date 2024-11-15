#ifndef BLACKBOARD_AND_PORTS
#define BLACKBOARD_AND_PORTS

#include <behaviortree_cpp/action_node.h>
#include <iostream>
#include <string>

namespace blackboard_and_ports
{
/**
 * @brief To demonstrate input port usage
 */
class SaySomething : public BT::SyncActionNode
{
  public:
    /**
     * @brief If your Node has ports, you must use this constructor signature
     *
     * @param name
     * @param config
     */
    explicit SaySomething(const std::string &name, const BT::NodeConfig &config);

    /**
     * @brief It is mandatory to define this static method
     *
     * @return List of provided ports
     */
    static BT::PortsList providedPorts();

    /**
     * @brief Mandatory tick override
     *
     * @return Node status
     */
    BT::NodeStatus tick() override;
};

/**
 * @brief To demonstrate output port usage
 */
class ThinkWhatToSay : public BT::SyncActionNode
{
  public:
    explicit ThinkWhatToSay(const std::string &name, const BT::NodeConfig &config);

    static BT::PortsList providedPorts();

    /**
     * @brief Writes a value into the port "text"
     *
     * @return Node status
     */
    BT::NodeStatus tick() override;
};
} // namespace blackboard_and_ports

#endif
