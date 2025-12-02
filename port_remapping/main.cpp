#include "behaviortree_cpp/bt_factory.h"
#include "dummy_nodes.h"
#include <filesystem>
#include <iostream>

int main()
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerNodeType<MoveBaseAction>("MoveBase");

    const auto tree_file = std::filesystem::path(__FILE__).parent_path() / "my_tree.xml";
    if (!std::filesystem::exists(tree_file))
    {
        std::cerr << "Fichier XML introuvable: " << tree_file << std::endl;
        return 1;
    }

    factory.registerBehaviorTreeFromFile(tree_file.string());
    auto tree = factory.createTree("MainTree");

    tree.tickWhileRunning();

    std::cout << "\n------ First BB ------" << std::endl;
    tree.subtrees[0]->blackboard->debugMessage();
    std::cout << "\n------ Second BB------" << std::endl;
    tree.subtrees[1]->blackboard->debugMessage();

    return 0;
}
