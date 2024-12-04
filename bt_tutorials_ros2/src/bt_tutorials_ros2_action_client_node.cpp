#include "bt_tutorials_ros2/bt_tutorials_ros2_action_client.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

using namespace bt_tutorials_ros2_action_client;

static const char *xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence name="root">
            <Script code=" fib_order:=10 " />
            <FibonacciActionNode order="{fib_order}" />
        </Sequence>
    </BehaviorTree>
</root>
)";

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("bt_tutorials_ros2_action_client_node");

    // Params for Fibonacci Action Node
    BT::RosNodeParams fib_params;
    fib_params.nh = std::weak_ptr<rclcpp::Node>(node);
    fib_params.default_port_value = "/fibonacci"; // Action server name

    // Register the node
    BT::BehaviorTreeFactory factory;
    try
    {
        factory.registerNodeType<FibonacciActionNode>("FibonacciActionNode", fib_params);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Error during node registration: %s", e.what());
    }

    // Create tree from XML and tick it
    auto tree = factory.createTreeFromText(xml_text);
    tree.tickWhileRunning();

    rclcpp::shutdown();
    return 0;
}
