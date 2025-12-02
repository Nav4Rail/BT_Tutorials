#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "dummy_nodes.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>

int main()
{
    BT::BehaviorTreeFactory factory;

    CrossDoor cross_door;
    cross_door.registerNodes(factory);

    // Générer les modèles pour Groot2
    std::string xml_models = BT::writeTreeNodesModelXML(factory);
    {
        std::ofstream models_file("models.xml");
        if (models_file)
        {
            models_file << xml_models;
        }
    }

    const auto tree_file = std::filesystem::path(__FILE__).parent_path() / "my_tree.xml";
    if (!std::filesystem::exists(tree_file))
    {
        std::cerr << "Fichier XML introuvable: " << tree_file << std::endl;
        return 1;
    }

    factory.registerBehaviorTreeFromFile(tree_file.string());
    auto tree = factory.createTree("MainTree");

    // Publieur Groot2 (visualisation temps réel)
    BT::Groot2Publisher publisher(tree);

    // Exécuter quelques cycles
    for (int i = 0; i < 2; ++i)
    {
        std::cout << "Start cycle " << i + 1 << std::endl;
        cross_door.reset();
        tree.tickWhileRunning();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return 0;
}
