#include "bt_tutorials_ros2/bt_tutorials_ros2_service_client.hpp"

namespace bt_tutorials_ros2_service_client
{

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

} // namespace bt_tutorials_ros2_service_client
