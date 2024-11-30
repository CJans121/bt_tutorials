#include "bt_tutorials_basic/blackboard_and_ports.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <filesystem>

using namespace blackboard_and_ports;

int main()
{

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<SaySomething>("SaySomething");

    // Registering enums
    enum Color
    {
        RED = 1,
        BLUE = 2,
        GREEN = 3
    };
    factory.registerScriptingEnums<Color>(); // Note BT uses magic_enums, a limitation of which is the default range is
                                             // [-128, 128]

    // Registering an enum manually
    factory.registerScriptingEnum("THE_ANSWER", 42);

    // Path to xml
    std::filesystem::path source_directory = std::filesystem::path(__FILE__).parent_path();
    std::filesystem::path xml_path = source_directory / "../trees/scripting_example.xml";

    // Create the tree
    auto tree = factory.createTreeFromFile(xml_path.string());

    // Tick the tree
    tree.tickWhileRunning();

    return 0;
}
