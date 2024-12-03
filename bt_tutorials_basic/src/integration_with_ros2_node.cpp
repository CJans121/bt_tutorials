#include "bt_tutorials_basic/integration_with_ros2.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

using namespace integration_with_ros2;

static const char *xml_text = R"(
 <root BTCPP_format="4" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root">
            <Script code=" fib_order:=10 " />
            <FibonacciAction order="{fib_order}" />
        </Sequence>
     </BehaviorTree>
 </root>
)";

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("fibonacci_Action_client");

    // Register the action node
    BT::BehaviorTreeFactory factory;
    BT::RosNodeParams params;
    params.nh = std::weak_ptr<rclcpp::Node>(node); // takes a weak pointer
    params.default_port_value = "/fibonacci";      // action server name
    factory.registerNodeType<FibonacciAction>("FibonacciAction", params);

    // Create tree from xml and tick it
    auto tree = factory.createTreeFromText(xml_text);
    tree.tickWhileRunning();

    return 0;
}
