#pragma once
#include "behaviortree_cpp/bt_factory.h"
#include <cstdio>
#include <string>

namespace DummyNodes
{
class SaySomething : public BT::SyncActionNode
{
public:
    SaySomething(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("message") };
    }

    BT::NodeStatus tick() override
    {
        auto msg = getInput<std::string>("message");
        if (!msg)
        {
            throw BT::RuntimeError("missing required input [message]: ", msg.error());
        }
        std::printf("Robot says: %s\n", msg->c_str());
        return BT::NodeStatus::SUCCESS;
    }
};
} // namespace DummyNodes
