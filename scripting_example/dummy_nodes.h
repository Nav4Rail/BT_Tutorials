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
        if (auto msg = getInput<std::string>("message"))
        {
            std::printf("Robot says: %s\n", msg->c_str());
            return BT::NodeStatus::SUCCESS;
        }
        if (auto msg_d = getInput<double>("message"))
        {
            std::printf("Robot says: %f\n", *msg_d);
            return BT::NodeStatus::SUCCESS;
        }
        if (auto msg_i = getInput<int>("message"))
        {
            std::printf("Robot says: %f\n", static_cast<double>(*msg_i));
            return BT::NodeStatus::SUCCESS;
        }
        throw BT::RuntimeError("missing or invalid input [message]");
    }
};
} // namespace DummyNodes
