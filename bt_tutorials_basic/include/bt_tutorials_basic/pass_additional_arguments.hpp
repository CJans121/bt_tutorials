#ifndef PASS_ADDITIONAL_ARGUMENTS
#define PASS_ADDITIONAL_ARGUMENTS

#include <behaviortree_cpp/action_node.h>

namespace pass_additional_arguments
{

/**
 * @brief To demonstrate adding arguments to the constructor
 */
class Action_A : public BT::SyncActionNode
{
  public:
    /**
     * @brief Constructor with additional parameters
     */
    Action_A(const std::string &name, const BT::NodeConfig &config, int arg_int, std::string arg_str);

    /**
     * @brief Mandatory method but unused in this case
     */
    static inline BT::PortsList providedPorts() { return {}; }

    /**
     * @brief We'll print the arguments using the tick function
     */
    BT::NodeStatus tick() override;

  private:
    int _arg1;
    std::string _arg2;
    std::string _node_name;
};

/**
 * @brief To demonstrate using an initialize method to pass different argument values to individual instances of a Node
 * type
 */
class Action_B : public BT::SyncActionNode
{

  public:
    /**
     * @brief Regular constructor
     */
    inline Action_B(const std::string &name, const BT::NodeConfig &config)
        : BT::SyncActionNode(name, config), _node_name(name)
    {
    }

    /**
     * @brief We want this method to be called ONCE and BEFORE the first tick()
     */
    void initialize(int arg_int, const std::string &arg_str);

    /**
     * @brief Mandatory method but unused in this case
     */
    static inline BT::PortsList providedPorts() { return {}; }

    /**
     * @brief We'll print the arguments using the tick function
     */
    BT::NodeStatus tick() override;

  private:
    int _arg1;
    std::string _arg2;
    std::string _node_name;
};

} // namespace pass_additional_arguments

#endif
