#include "behaviortree_cpp/bt_factory.h"
#include "dummy_nodes.h"
#include <filesystem>
#include <iostream>

int main()
{
    BT::BehaviorTreeFactory factory;
    CrossDoor cross_door;
    cross_door.registerNodes(factory);

    const auto tree_file = std::filesystem::path(__FILE__).parent_path() / "my_tree.xml";
    if (!std::filesystem::exists(tree_file))
    {
        std::cerr << "Fichier XML introuvable: " << tree_file << std::endl;
        return 1;
    }

    factory.registerBehaviorTreeFromFile(tree_file.string());
    auto tree = factory.createTree("MainTree");

    tree.tickWhileRunning();
    return 0;
}
