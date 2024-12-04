#include "bt_tutorials_ros2/bt_tutorials_ros2_service_client.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

using namespace bt_tutorials_ros2_service_client;

static const char *xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence name="root">
            <Script code=" A:=10; B:=20 " />
            <AddTwoIntsNode A="{A}" B="{B}" />
        </Sequence>
    </BehaviorTree>
</root>
)";

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("bt_tutorials_ros2_service_client_node");

    // Params for AddTwoInts Service Node
    BT::RosNodeParams add_ints_params;
    add_ints_params.nh = std::weak_ptr<rclcpp::Node>(node);
    add_ints_params.default_port_value = "/add_two_ints"; // Service server name

    // Register the action node
    BT::BehaviorTreeFactory factory;
    try
    {
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
