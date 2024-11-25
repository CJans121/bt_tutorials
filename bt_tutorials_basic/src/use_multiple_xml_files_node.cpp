#include "bt_tutorials_basic/blackboard_and_ports.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <filesystem>

using namespace blackboard_and_ports;

int main()
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<SaySomething>("SaySomething");

    std::cout << "***** Loading multiple files manually(recommended) *****\n";

    // We'll look into the following relative directory and register all XMLs from there
    std::filesystem::path source_directory = std::filesystem::path(__FILE__).parent_path();
    std::filesystem::path search_directory = source_directory / "../trees/use_multiple_xml_files";

    for (const auto &entry : std::filesystem::directory_iterator(search_directory))
    {
        if (entry.path().extension() == ".xml")
        {
            factory.registerBehaviorTreeFromFile(entry.path().string());
        }
    }

    // We'll create the main tree here. Subtrees will be added automatically
    std::cout << "----- Maintree tick -----\n";
    auto main_tree = factory.createTree("MainTree");
    main_tree.tickWhileRunning();

    // Now we'll create the main tree from the main-tree xml that "includes" the link to the subtree xmls
    std::cout << "***** Loading multiple files with include *****\n";
    factory.registerBehaviorTreeFromFile(source_directory.string() + "/../trees/use_multiple_xml_files.xml");
    std::cout << "----- MaintreeInclude tick -----\n";
    auto main_tree_include = factory.createTree("MainTreeInclude");
    main_tree_include.tickWhileRunning();

    return 0;
}
