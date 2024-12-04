#ifndef INTEGRATION_WITH_ROS2
#define INTEGRATION_WITH_ROS2
#include <action_tutorials_interfaces/action/fibonacci.hpp>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/bt_service_node.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

namespace integration_with_ros2
{
using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
class FibonacciActionNode : public BT::RosActionNode<Fibonacci>
{
  public:
    using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

    FibonacciActionNode(const std::string &name, const BT::NodeConfig &conf, const BT::RosNodeParams &params);

    /**
     * @brief Merge the ports of this Derived class with the ports of the base class using
     * RosActionNode::providedBasicPorts(). See implementation of this function
     */
    static BT::PortsList providedPorts();

    /**
     * @brief Called when the TreeNode is ticked and should send the request to the action server
     */
    bool setGoal(RosActionNode::Goal &goal) override;

    /**
     * @brief Callback executed when a reply is received. Based on the reply, you may decide to return SUCCESS or
     * FAILURE
     */
    BT::NodeStatus onResultReceived(const WrappedResult &wr) override;

    /**
     * @brief Callback invoked when there is communication error between the action client and the server. Sets the
     * status of the TreeNode to SUCCESS or FAILURE depending on the return value.
     */
    BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

    /**
     * @brief Callback for the feedback. Usually, this should return RUNNING, but you may decide based on the value of
     * the feedback whether to abort the action, or consider the TreeNode completed. In that case, return SUCCESS or
     * FAILURE. The cancel request is automatically sent to the server.
     */
    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);

  private:
    std::shared_ptr<rclcpp::Node> shared_node_;
};

using AddTwoInts = example_interfaces::srv::AddTwoInts;
class AddTwoIntsNode : public BT::RosServiceNode<AddTwoInts>
{
  public:
    AddTwoIntsNode(const std::string &name, const BT::NodeConfig &conf, const BT::RosNodeParams &params);

    /**
     * @brief Merge the ports of this Derived class with the ports of the base class using
     * RosServiceNode::providedBasicPorts(). See implementation of this function.
     */
    static BT::PortsList providedPorts();

    /**
     * @brief Called when the TreeNode is ticked and should send the request to the service server
     * @return
     */
    bool setRequest(Request::SharedPtr &request) override;

    /**
     * @brief Callback invoked when a response is received from the server. Must return SUCCESS or FAILURE.
     */
    BT::NodeStatus onResponseReceived(const Response::SharedPtr &response) override;

    /**
     * @brief Callack invoked when there is communication error between the service client and server. Sets the status
     * of the TreeNode to either SUCCESS or FAILURE depending on the return value. Note that if not overriden, returns
     * FAILURE by default.
     */
    BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

  private:
    std::shared_ptr<rclcpp::Node> shared_node_;
};

} // namespace integration_with_ros2
#endif
