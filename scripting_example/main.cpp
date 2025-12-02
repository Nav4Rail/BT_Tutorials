#include "behaviortree_cpp/bt_factory.h"
#include "dummy_nodes.h"
#include <iostream>

enum Color
{
    RED = 1,
    BLUE = 2,
    GREEN = 3
};

int main()
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<DummyNodes::SaySomething>("SaySomething");

    factory.registerScriptingEnums<Color>();
    factory.registerScriptingEnum("THE_ANSWER", 42);

    const std::string xml_path = "./my_tree.xml";
    auto tree = factory.createTreeFromFile(xml_path);
    tree.tickWhileRunning();

    return 0;
}
