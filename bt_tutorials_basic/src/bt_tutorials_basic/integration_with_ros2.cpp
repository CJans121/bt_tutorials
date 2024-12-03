#include "bt_tutorials_basic/integration_with_ros2.hpp"

namespace integration_with_ros2
{

FibonacciAction::FibonacciAction(const std::string &name, const BT::NodeConfig &conf, const BT::RosNodeParams &params)
    : BT::RosActionNode<Fibonacci>(name, conf, params)
{
    auto shared_node = params.nh.lock(); // Convert weak_ptr to shared_ptr
    if (!shared_node)
    {
        throw std::runtime_error("FibonacciAction: Failed to lock node from params.nh");
    }
    shared_node_ = shared_node;
}
BT::PortsList FibonacciAction::providedPorts()
{
    return RosActionNode::providedBasicPorts({BT::InputPort<unsigned>("order")});
}

bool FibonacciAction::setGoal(RosActionNode::Goal &goal)
{

    // get "order" from the Input port
    getInput("order", goal.order);
    // return true if we were able to set the goal correctly
    return true;
}

BT::NodeStatus FibonacciAction::onResultReceived(const WrappedResult &wr)
{
    std::stringstream ss;
    ss << "Result received: ";

    for (auto number : wr.result->sequence)
    {
        ss << number << " ";
    }

    RCLCPP_INFO(shared_node_->get_logger(), ss.str().c_str());
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus FibonacciAction::onFailure(BT::ActionNodeErrorCode error)
{
    RCLCPP_ERROR(shared_node_->get_logger(), "Error: %d", error);
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus FibonacciAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
    std::stringstream ss;
    ss << "Next number in sequence received: ";

    for (auto number : feedback->partial_sequence)
    {
        ss << number << " ";
    }
    RCLCPP_INFO(shared_node_->get_logger(), ss.str().c_str());
    return BT::NodeStatus::RUNNING;
}

} // namespace integration_with_ros2
