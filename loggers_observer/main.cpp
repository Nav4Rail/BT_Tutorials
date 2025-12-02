#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_observer.h"
#include <cassert>
#include <filesystem>
#include <iostream>
#include <map>

int main()
{
    BT::BehaviorTreeFactory factory;

    const auto tree_file = std::filesystem::path(__FILE__).parent_path() / "my_tree.xml";
    if (!std::filesystem::exists(tree_file))
    {
        std::cerr << "Fichier XML introuvable: " << tree_file << std::endl;
        return 1;
    }

    factory.registerBehaviorTreeFromFile(tree_file.string());
    auto tree = factory.createTree("MainTree");

    BT::printTreeRecursively(tree.rootNode());

    BT::TreeObserver observer(tree);

    std::map<uint16_t, std::string> ordered_UID_to_path;
    for (const auto& [path, uid] : observer.pathToUID())
    {
        ordered_UID_to_path[uid] = path;
    }

    for (const auto& [uid, path] : ordered_UID_to_path)
    {
        std::cout << uid << " -> " << path << std::endl;
    }

    tree.tickWhileRunning();

    const auto& last_stats = observer.getStatistics("last_action");
    assert(last_stats.transitions_count > 0);

    std::cout << "----------------" << std::endl;
    for (const auto& [uid, path] : ordered_UID_to_path)
    {
        const auto& stats = observer.getStatistics(uid);
        std::cout << "[" << path << "] \tT/S/F:  "
                  << stats.transitions_count << "/"
                  << stats.success_count << "/"
                  << stats.failure_count << std::endl;
    }

    return 0;
}
