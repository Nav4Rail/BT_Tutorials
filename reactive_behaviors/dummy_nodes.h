#pragma once
#include "behaviortree_cpp/bt_factory.h"
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

// Types personnalisés
struct Pose2D
{
    double x;
    double y;
    double theta;
};

// Conversion string -> Pose2D
namespace BT
{
    template <> inline Pose2D convertFromString(StringView str)
    {
        auto parts = splitString(str, ';');
        if (parts.size() != 3)
        {
            throw RuntimeError("invalid input)");
        }
        Pose2D output;
        output.x = convertFromString<double>(parts[0]);
        output.y = convertFromString<double>(parts[1]);
        output.theta = convertFromString<double>(parts[2]);
        return output;
    }
} // namespace BT

// Condition simple: batterie OK
inline BT::NodeStatus CheckBattery()
{
    std::puts("[ Battery: OK ]");
    return BT::NodeStatus::SUCCESS;
}

// Action synchrone: SaySomething
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

// Action asynchrone: MoveBaseAction
namespace chr = std::chrono;

class MoveBaseAction : public BT::StatefulActionNode
{
public:
    MoveBaseAction(const std::string& name, const BT::NodeConfig& config)
        : StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<Pose2D>("goal") };
    }

    BT::NodeStatus onStart() override
    {
        if (!getInput<Pose2D>("goal", goal_))
        {
            throw BT::RuntimeError("missing required input [goal]");
        }
        std::printf("[ MoveBase: SEND REQUEST ]. goal: x=%f y=%f theta=%f\n",
                    goal_.x, goal_.y, goal_.theta);
        completion_time_ = chr::system_clock::now() + chr::milliseconds(220);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        std::this_thread::sleep_for(chr::milliseconds(10));
        if (chr::system_clock::now() >= completion_time_)
        {
            std::puts("[ MoveBase: FINISHED ]");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::puts("[ MoveBase: ABORTED ]");
    }

private:
    Pose2D goal_{};
    chr::system_clock::time_point completion_time_{};
};
