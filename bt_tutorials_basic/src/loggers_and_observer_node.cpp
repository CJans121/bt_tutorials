#include "behaviortree_cpp/loggers/bt_observer.h"
#include <behaviortree_cpp/bt_factory.h>
#include <filesystem>

int main()
{

    BT::BehaviorTreeFactory factory;

    std::filesystem::path source_directory = std::filesystem::path(__FILE__).parent_path();
    std::filesystem::path xml_path = source_directory / "../trees/loggers_and_observer.xml";
    factory.registerBehaviorTreeFromFile(xml_path.string());

    auto tree = factory.createTree("MainTree");

    // Print the tree
    std::cout << "Printing tree:" << std::endl;
    BT::printTreeRecursively(tree.rootNode());

    BT::TreeObserver observer(tree); // saves statistics about the tree like no. of times a certain node returns SUCCESS
                                     // or FAILURE. Particularly useful during unit testing

    // Print unique ID and corresponding human readbale path for the nodes
    std::map<uint16_t, std::string> ordered_UID_to_path;
    for (const auto &[name, uid] : observer.pathToUID())
    {
        ordered_UID_to_path[uid] = name;
    }

    std::cout << "Printing UID and path:" << std::endl;
    for (const auto &[uid, name] : ordered_UID_to_path)
    {
        std::cout << uid << " -> " << name << std::endl;
    }

    // Tick the tree
    tree.tickWhileRunning();

    // Using the full path or UID, we can access a specific statistic
    const auto &last_action_stats = observer.getStatistics("last_action");
    assert(last_action_stats.transitions_count >
           0); // The transition count tracks the number of times a node changes state, e.g., from IDLE to RUNNING, then
               // RUNNING to SUCCESS or FAILURE, and so on.

    std::cout << "------------------" << std::endl;
    std::cout << "Printing statistics:" << std::endl;

    // Print the statistics
    for (const auto &[uid, name] : ordered_UID_to_path)
    {
        const auto &stats = observer.getStatistics(uid);

        std::cout << "[" << name << "] \tT/S/F: " << stats.transitions_count << "/" << stats.success_count << "/"
                  << stats.failure_count << std::endl;
    }

    return 0;
}
