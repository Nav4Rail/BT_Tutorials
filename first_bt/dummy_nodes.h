#pragma once
#include "behaviortree_cpp/bt_factory.h"
#include <iostream>

namespace DummyNodes {

/* Action personnalisée : approcher l’objet.
   Elle dérive de SyncActionNode selon l’exemple du tutoriel:contentReference[oaicite:2]{index=2}. */
class ApproachObject : public BT::SyncActionNode {
public:
    ApproachObject(const std::string& name)
        : BT::SyncActionNode(name, {}) {}
    BT::NodeStatus tick() override {
        std::cout << "ApproachObject: " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

/* Condition simple qui vérifie la batterie:contentReference[oaicite:3]{index=3}. */
inline BT::NodeStatus CheckBattery() {
    std::cout << "[ Battery: OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

/* Interface du “gripper” avec deux actions open et close:contentReference[oaicite:4]{index=4}. */
class GripperInterface {
public:
    GripperInterface() : _open(true) {}
    BT::NodeStatus open() {
        _open = true;
        std::cout << "GripperInterface::open" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    BT::NodeStatus close() {
        std::cout << "GripperInterface::close" << std::endl;
        _open = false;
        return BT::NodeStatus::SUCCESS;
    }
private:
    bool _open;
};

} // namespace DummyNodes
