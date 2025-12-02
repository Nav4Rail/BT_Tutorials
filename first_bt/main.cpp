#include "behaviortree_cpp/bt_factory.h"
#include "dummy_nodes.h"
#include <filesystem>
#include <iostream>

using namespace DummyNodes;

int main()
{
    BT::BehaviorTreeFactory factory;

    // Enregistrement de notre nœud dérivé:contentReference[oaicite:6]{index=6}.
    factory.registerNodeType<ApproachObject>("ApproachObject");

    // Enregistrement d’un nœud simple (condition) pour CheckBattery:contentReference[oaicite:7]{index=7}.
    factory.registerSimpleCondition("CheckBattery",
        [&](BT::TreeNode&) { return CheckBattery(); });

    // Enregistrement d’actions à partir des méthodes de GripperInterface:contentReference[oaicite:8]{index=8}.
    GripperInterface gripper;
    factory.registerSimpleAction("OpenGripper",
        [&](BT::TreeNode&) { return gripper.open(); });
    factory.registerSimpleAction("CloseGripper",
        [&](BT::TreeNode&) { return gripper.close(); });

    // Création de l’arbre depuis le fichier XML.
    const auto tree_file = std::filesystem::path(__FILE__).parent_path() / "my_tree.xml";
    if (!std::filesystem::exists(tree_file))
    {
        std::cerr << "Fichier d'arbre introuvable: " << tree_file << std::endl;
        return 1;
    }
    auto tree = factory.createTreeFromFile(tree_file.string());

    // Exécution de l’arbre : tickWhileRunning() parcourt la séquence jusqu’à terminer:contentReference[oaicite:9]{index=9}.
    tree.tickWhileRunning();

    return 0;
}
