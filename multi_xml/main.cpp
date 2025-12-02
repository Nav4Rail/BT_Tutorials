#include "behaviortree_cpp/bt_factory.h"
#include "dummy_nodes.h"
#include <filesystem>
#include <iostream>

int main()
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<DummyNodes::SaySomething>("SaySomething");

    const std::string search_directory = ".";
    namespace fs = std::filesystem;

    for (const auto& entry : fs::directory_iterator(search_directory))
    {
        if (entry.path().extension() == ".xml")
        {
            factory.registerBehaviorTreeFromFile(entry.path().string());
        }
    }

    std::cout << "Registered BehaviorTrees:\n";
    for (const auto& name : factory.registeredBehaviorTrees())
    {
        std::cout << " - " << name << '\n';
    }

    std::cout << "----- MainTree tick ----" << std::endl;
    auto main_tree = factory.createTree("MainTree");
    main_tree.tickWhileRunning();

    std::cout << "----- SubA tick ----" << std::endl;
    auto subA_tree = factory.createTree("SubTreeA");
    subA_tree.tickWhileRunning();

    return 0;
}
