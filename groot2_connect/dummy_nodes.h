#pragma once
#include "behaviortree_cpp/bt_factory.h"
#include <cstdio>
#include <functional>
#include <string>

// Implémentation CrossDoor simplifiée
class CrossDoor
{
public:
    void registerNodes(BT::BehaviorTreeFactory& factory)
    {
        using namespace std::placeholders;
        factory.registerSimpleCondition("IsDoorClosed", std::bind(&CrossDoor::isDoorClosed, this, _1));
        factory.registerSimpleAction("PassThroughDoor", std::bind(&CrossDoor::passThroughDoor, this, _1));
        factory.registerSimpleAction("OpenDoor", std::bind(&CrossDoor::openDoor, this, _1));
        factory.registerSimpleAction("PickLock", std::bind(&CrossDoor::pickLock, this, _1));
        factory.registerSimpleAction("SmashDoor", std::bind(&CrossDoor::smashDoor, this, _1));
    }

    void reset()
    {
        _door_open = false;
        _door_locked = true;
        _pick_attempts = 0;
    }

    BT::NodeStatus isDoorClosed(BT::TreeNode&)
    {
        return _door_open ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus passThroughDoor(BT::TreeNode&)
    {
        if (_door_open)
        {
            std::puts("[ PassThroughDoor ]");
            return BT::NodeStatus::SUCCESS;
        }
        std::puts("[ PassThroughDoor ] Door is closed!");
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus openDoor(BT::TreeNode&)
    {
        if (_door_locked)
        {
            std::puts("[ OpenDoor ] Door is locked");
            return BT::NodeStatus::FAILURE;
        }
        _door_open = true;
        std::puts("[ OpenDoor ] Door opened");
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus pickLock(BT::TreeNode&)
    {
        _pick_attempts++;
        std::printf("[ PickLock ] attempt %d\n", _pick_attempts);
        if (_pick_attempts >= 3)
        {
            _door_locked = false;
            _door_open = true;
            std::puts("[ PickLock ] Unlocked and opened the door");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus smashDoor(BT::TreeNode&)
    {
        _door_locked = false;
        _door_open = true;
        std::puts("[ SmashDoor ] Door smashed open");
        return BT::NodeStatus::SUCCESS;
    }

private:
    bool _door_open{false};
    bool _door_locked{true};
    int _pick_attempts{0};
};
