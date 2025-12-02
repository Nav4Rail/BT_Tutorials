#include "behaviortree_cpp/bt_factory.h"
#include "dummy_nodes.h"
#include <chrono>
#include <filesystem>
#include <iostream>

int main()
{
    BT::BehaviorTreeFactory factory;
    factory.registerSimpleCondition("BatteryOK", [](BT::TreeNode&) { return CheckBattery(); });
    factory.registerNodeType<MoveBaseAction>("MoveBase");
    factory.registerNodeType<SaySomething>("SaySomething");

    const auto tree_file = std::filesystem::path(__FILE__).parent_path() / "my_tree.xml";
    if (!std::filesystem::exists(tree_file))
    {
        std::cerr << "Fichier XML introuvable: " << tree_file << std::endl;
        return 1;
    }

    auto tree = factory.createTreeFromFile(tree_file.string());

    std::cout << "--- ticking\n";
    auto status = tree.tickOnce();
    std::cout << "--- status: " << BT::toStr(status) << "\n\n";

    while (status == BT::NodeStatus::RUNNING)
    {
        tree.sleep(std::chrono::milliseconds(100));

        std::cout << "--- ticking\n";
        status = tree.tickOnce();
        std::cout << "--- status: " << BT::toStr(status) << "\n\n";
    }

    return 0;
}
