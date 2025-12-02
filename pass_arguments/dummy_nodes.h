#pragma once
#include "behaviortree_cpp/bt_factory.h"
#include <cstdio>
#include <string>

// Action_A: arguments passes au constructeur
class ActionA : public BT::SyncActionNode
{
public:
    ActionA(const std::string& name, const BT::NodeConfig& config,
            int arg_int, const std::string& arg_str)
        : BT::SyncActionNode(name, config), arg_int_(arg_int), arg_str_(arg_str) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        std::printf("ActionA args: %d, %s\n", arg_int_, arg_str_.c_str());
        return BT::NodeStatus::SUCCESS;
    }

private:
    int arg_int_;
    std::string arg_str_;
};

// Action_B: initialise via une méthode dedicatee
class ActionB : public BT::SyncActionNode
{
public:
    ActionB(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    void initialize(int arg_int, const std::string& arg_str)
    {
        arg_int_ = arg_int;
        arg_str_ = arg_str;
    }

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        std::printf("ActionB args: %d, %s\n", arg_int_, arg_str_.c_str());
        return BT::NodeStatus::SUCCESS;
    }

private:
    int arg_int_ = 0;
    std::string arg_str_;
};
