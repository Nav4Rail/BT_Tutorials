#include "behaviortree_cpp/bt_factory.h"
#include "dummy_nodes.h"
#include <filesystem>
#include <iostream>

int main()
{
    BT::BehaviorTreeFactory factory;

    // Enregistrer ActionA avec des arguments fixes
    factory.registerNodeType<ActionA>("ActionA", 42, std::string("hello world"));

    // Enregistrer ActionB (initialisee ensuite)
    factory.registerNodeType<ActionB>("ActionB");

    const auto tree_file = std::filesystem::path(__FILE__).parent_path() / "my_tree.xml";
    if (!std::filesystem::exists(tree_file))
    {
        std::cerr << "Fichier XML introuvable: " << tree_file << std::endl;
        return 1;
    }

    auto tree = factory.createTreeFromFile(tree_file.string());

    // Initialiser chaque instance d'ActionB avant la premiere tick
    auto visitor = [](BT::TreeNode* node)
    {
        if (auto b = dynamic_cast<ActionB*>(node))
        {
            b->initialize(69, "interesting_value");
        }
    };
    tree.applyVisitor(visitor);

    tree.tickWhileRunning();
    return 0;
}
