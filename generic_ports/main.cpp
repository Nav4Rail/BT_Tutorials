#include "behaviortree_cpp/bt_factory.h"
#include "dummy_nodes.h"
#include <filesystem>
#include <iostream>

int main()
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<CalculateGoal>("CalculateGoal");
    factory.registerNodeType<PrintTarget>("PrintTarget");

    const auto tree_file = std::filesystem::path(__FILE__).parent_path() / "my_tree.xml";
    if (!std::filesystem::exists(tree_file))
    {
        std::cerr << "Fichier XML introuvable: " << tree_file << std::endl;
        return 1;
    }

    auto tree = factory.createTreeFromFile(tree_file.string());
    tree.tickWhileRunning();
    return 0;
}
