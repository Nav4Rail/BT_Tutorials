#pragma once
#include "behaviortree_cpp/bt_factory.h"
#include <cstdio>
#include <functional>
#include <iostream>

class CrossDoor
{
public:
    void registerNodes(BT::BehaviorTreeFactory& factory)
    {
        factory.registerSimpleCondition(
            "IsDoorClosed", std::bind(&CrossDoor::isDoorClosed, this, std::placeholders::_1));

        factory.registerSimpleAction(
            "PassThroughDoor", std::bind(&CrossDoor::passThroughDoor, this, std::placeholders::_1));

        factory.registerSimpleAction(
            "OpenDoor", std::bind(&CrossDoor::openDoor, this, std::placeholders::_1));

        factory.registerSimpleAction(
            "PickLock", std::bind(&CrossDoor::pickLock, this, std::placeholders::_1));

        factory.registerSimpleAction(
            "SmashDoor", std::bind(&CrossDoor::smashDoor, this, std::placeholders::_1));
    }

    // SUCCESS if door is closed (i.e. not open)
    BT::NodeStatus isDoorClosed(BT::TreeNode&)
    {
        return _door_open ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }

    // SUCCESS if door is open
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

    // After 3 attempts, open the locked door
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

    // Try to open if not locked
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

    // Always open the door
    BT::NodeStatus smashDoor(BT::TreeNode&)
    {
        _door_locked = false;
        _door_open = true;
        std::puts("[ SmashDoor ] Door smashed open");
        return BT::NodeStatus::SUCCESS;
    }

private:
    bool _door_open   = false;
    bool _door_locked = true;
    int _pick_attempts = 0;
};
