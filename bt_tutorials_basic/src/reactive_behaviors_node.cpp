#include "bt_tutorials_basic/blackboard_and_ports.hpp"
#include "bt_tutorials_basic/reactive_behaviors.hpp"
#include "bt_tutorials_basic/your_first_behavior_tree.hpp"
#include <behaviortree_cpp/bt_factory.h>

using namespace your_first_behavior_tree;
using namespace blackboard_and_ports;
using namespace reactive_behaviors;

static const char *SEQUENCE_TREE = R"(
    <root BTCPP_format="4">
        <BehaviorTree>
            <Sequence>
		<!-- Check if battery is OK. Prints OK and returns success as implemented in the CheckBattery function -->
                <BatteryOK />
		<!-- read the input message from a static string and print it -->
                <SaySomething message="mission started..." />
		<!-- execute the MoveBase onStart function with Pose2D {1,2,3} and then constantly call onRunning -->
                <MoveBase goal="1;2;3" />
		<!-- read the input message from a static string and print it -->
                <SaySomething message="mission completed!" />
		<!-- because this is not a reactive sequence, the enclosed sequence is not restarted here and BatteryOK is not checked again. -->
            </Sequence>
        </BehaviorTree>
    </root>
)";

/**
 * @brief to manually tick a BT::tree using reactive or regular sequence, to show to the difference between the two.
 */
void tick_tree(BT::Tree &tree)
{

    // Using our own loop instead of tree.tickWhileRunning()
    std::cout << "--- ticking\n";
    auto status = tree.tickOnce();
    std::cout << "--- status: " << toStr(status) << "\n\n";

    while (status == BT::NodeStatus::RUNNING)
    {
        // Sleep to avoid busy waiting. Must use the tree sleep function. Here using a large sleep to prevent flooding
        // console with messages, but generally use short sleep
        tree.sleep(std::chrono::milliseconds(100));

        std::cout << "--- ticking\n";
        status = tree.tickOnce();
        std::cout << "--- status: " << toStr(status) << "\n\n";
    }
}

int main()
{

    BT::BehaviorTreeFactory factory;
    factory.registerSimpleCondition("BatteryOK", std::bind(CheckBattery));
    factory.registerNodeType<MoveBaseAction>("MoveBase");
    factory.registerNodeType<SaySomething>("SaySomething");

    auto sequence_tree = factory.createTreeFromText(SEQUENCE_TREE);
    auto reactive_sequence_tree = factory.createTreeFromFile(
        "/home/crasun/ws_ros2/src/bt_tutorials/bt_tutorials_basic/trees/reactive_behaviors.xml");

    std::cout << "**Using a sequence tree: \n";
    tick_tree(sequence_tree);
    std::cout << "**Using a reactive sequence tree: \n";
    tick_tree(reactive_sequence_tree);

    return 0;
}
