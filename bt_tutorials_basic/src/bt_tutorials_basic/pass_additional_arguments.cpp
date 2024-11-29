#include "bt_tutorials_basic/pass_additional_arguments.hpp"

namespace pass_additional_arguments
{

Action_A::Action_A(const std::string &name, const BT::NodeConfig &config, int arg_int, std::string arg_str)
    : BT::SyncActionNode(name, config), _arg1(arg_int), _arg2(arg_str), _node_name(name)
{
}

BT::NodeStatus Action_A::tick()
{

    std::cout << "_arg1 for " << _node_name << ": " << _arg1 << std::endl;
    std::cout << "_arg2 for " << _node_name << ": " << _arg2 << std::endl;

    return BT::NodeStatus::SUCCESS;
}

void Action_B::initialize(int arg_int, const std::string &arg_str)
{
    _arg1 = arg_int;
    _arg2 = arg_str;
}

BT::NodeStatus Action_B::tick()
{

    std::cout << "_arg1 for " << _node_name << ": " << _arg1 << std::endl;
    std::cout << "_arg2 for " << _node_name << ": " << _arg2 << std::endl;
    return BT::NodeStatus::SUCCESS;
}

} // namespace pass_additional_arguments
