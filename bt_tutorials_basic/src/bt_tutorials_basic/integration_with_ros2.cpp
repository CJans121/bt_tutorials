#include "bt_tutorials_basic/integration_with_ros2.hpp"

namespace integration_with_ros2
{

FibonacciActionNode::FibonacciActionNode(const std::string &name, const BT::NodeConfig &conf,
                                         const BT::RosNodeParams &params)
    : BT::RosActionNode<Fibonacci>(name, conf, params)
{
    auto shared_node = params.nh.lock(); // Convert weak_ptr to shared_ptr
    if (!shared_node)
    {
        throw std::runtime_error("FibonacciActionNode: Failed to lock node from params.nh");
    }
    shared_node_ = shared_node;
}
BT::PortsList FibonacciActionNode::providedPorts()
{
    return RosActionNode::providedBasicPorts({BT::InputPort<unsigned>("order")});
}

bool FibonacciActionNode::setGoal(RosActionNode::Goal &goal)
{

    // get "order" from the Input port
    getInput("order", goal.order);
    // return true if we were able to set the goal correctly
    return true;
}

BT::NodeStatus FibonacciActionNode::onResultReceived(const WrappedResult &wr)
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

BT::NodeStatus FibonacciActionNode::onFailure(BT::ActionNodeErrorCode error)
{
    RCLCPP_ERROR(shared_node_->get_logger(), "Error: %d", error);
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus FibonacciActionNode::onFeedback(const std::shared_ptr<const Feedback> feedback)
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

AddTwoIntsNode::AddTwoIntsNode(const std::string &name, const BT::NodeConfig &conf, const BT::RosNodeParams &params)
    : RosServiceNode<AddTwoInts>(name, conf, params)
{
    auto shared_node = params.nh.lock(); // Convert weak_ptr to shared_ptr
    if (!shared_node)
    {
        throw std::runtime_error("AddTwoIntsNode: Failed to lock node from params.nh");
    }
    shared_node_ = shared_node;
}

BT::PortsList AddTwoIntsNode::providedPorts()
{
    return RosServiceNode::providedBasicPorts({BT::InputPort<unsigned>("A"), BT::InputPort<unsigned>("B")});
}

bool AddTwoIntsNode::setRequest(Request::SharedPtr &request)
{
    // get numbers to add from the input port
    getInput("A", request->a);
    getInput("B", request->b);
    // return true if we were able to set the goal correctly
    return true;
}

BT::NodeStatus AddTwoIntsNode::onResponseReceived(const Response::SharedPtr &response)
{
    RCLCPP_INFO(shared_node_->get_logger(), "Sum: %ld", response->sum);
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus AddTwoIntsNode::onFailure(BT::ServiceNodeErrorCode error)
{
    RCLCPP_ERROR(shared_node_->get_logger(), "Error: %d", error);
    return BT::NodeStatus::FAILURE;
}

} // namespace integration_with_ros2
