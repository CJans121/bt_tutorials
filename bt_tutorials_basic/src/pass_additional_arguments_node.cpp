#include "bt_tutorials_basic/pass_additional_arguments.hpp"
#include <behaviortree_cpp/bt_factory.h>
using namespace pass_additional_arguments;

int main()
{
    BT::BehaviorTreeFactory factory;

    // Approach 1: Register Action_A with predefined arguments for all instances using the constructor
    factory.registerNodeType<Action_A>("Action_A", 42, "hello world");

    // Alternatively, specify template parameters explicitly (optional)
    // factory.registerNodeType<Action_A, int, std::string>("Action_A", 42, "hello world");

    // Approach 2: Register Action_B without arguments. Instances are initialized later using an initialize function
    factory.registerNodeType<Action_B>("Action_B");

    // Define the path to the XML tree file
    std::filesystem::path source_directory = std::filesystem::path(__FILE__).parent_path();
    std::filesystem::path xml_path = source_directory / "../trees/pass_additional_arguments.xml";

    // Create the tree structure from the XML file
    auto tree = factory.createTreeFromFile(xml_path.string());

    // Visitor function to traverse all nodes in the tree and initialize Action_B nodes with unique values.
    // The goal is to initialize different instances of Action_B with different values.
    // It checks if a node is of type Action_B using dynamic_cast and calls its initialize method to set custom
    // arguments. Nodes of other types are skipped without any changes.
    auto visitor = [&tree](BT::TreeNode *node) {
        if (auto action_B_node = dynamic_cast<Action_B *>(node))
        {
            // Initialize different instances of Action_B with different values
            if (action_B_node->name() == "Action_B_1")
            {
                // For the first instance (Action_B_1), pass 69 and "first_value"
                action_B_node->initialize(69, "first_value");
            }
            else if (action_B_node->name() == "Action_B_2")
            {
                // For the second instance (Action_B_2), pass 42 and "second_value"
                action_B_node->initialize(42, "second_value");
            }
        }
    };

    // Apply the visitor to all nodes in the tree to perform custom initialization
    tree.applyVisitor(visitor);

    tree.tickWhileRunning();

    return 0; // Successfully initialized and constructed the Behavior Tree
}
