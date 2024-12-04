#include "bt_tutorials_basic/integration_with_ros2.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

using namespace integration_with_ros2;

static const char *xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence name="root">
            <Script code=" fib_order:=10 " />
            <FibonacciActionNode order="{fib_order}" />
            <Script code=" A:=10; B:=20 " />
            <AddTwoIntsNode A="{A}" B="{B}" />
        </Sequence>
    </BehaviorTree>
</root>
)";

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("bt_example_node");

    // Create BehaviorTreeFactory
    BT::BehaviorTreeFactory factory;

    // Params for Fibonacci Action Node
    BT::RosNodeParams fib_params;
    fib_params.nh = std::weak_ptr<rclcpp::Node>(node);
    fib_params.default_port_value = "/fibonacci"; // Action server name

    // Params for AddTwoInts Service Node
    BT::RosNodeParams add_ints_params;
    add_ints_params.nh = std::weak_ptr<rclcpp::Node>(node);
    add_ints_params.default_port_value = "/add_two_ints"; // Service server name

    try
    {
        // Register nodes with their specific parameters
        factory.registerNodeType<FibonacciActionNode>("FibonacciActionNode", fib_params);
        factory.registerNodeType<AddTwoIntsNode>("AddTwoIntsNode", add_ints_params);
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
