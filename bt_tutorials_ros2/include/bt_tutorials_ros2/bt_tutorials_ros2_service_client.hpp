#ifndef BT_TUTORIALS_ROS2_SERVICE_CLIENT_HPP
#define BT_TUTORIALS_ROS2_SERVICE_CLIENT_HPP

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_ros2/bt_service_node.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <string>

namespace bt_tutorials_ros2_service_client
{
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

} // namespace bt_tutorials_ros2_service_client
#endif
