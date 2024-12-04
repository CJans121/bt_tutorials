#ifndef BT_TUTORIALS_ROS2_ACTION_CLIENT_HPP
#define BT_TUTORIALS_ROS2_ACTION_CLIENT_HPP

#include <action_tutorials_interfaces/action/fibonacci.hpp>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <string>

namespace bt_tutorials_ros2_action_client
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

} // namespace bt_tutorials_ros2_action_client
#endif
