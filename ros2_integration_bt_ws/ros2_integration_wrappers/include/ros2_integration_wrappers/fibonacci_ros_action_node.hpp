/*!
 * FibonacciRosActionNode - Action Node basé sur BehaviorTree.ROS2
 * qui appelle l'action ROS 2 Fibonacci via BT::RosActionNode.
 *
 * Cette version délègue toute la logique client (envoi de goal,
 * suivi, gestion des retours) à BehaviorTree.ROS2. Le nœud BT
 * se contente de :
 *  - construire le goal à partir des ports (setGoal)
 *  - interpréter le résultat (onResultReceived)
 */

#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "action_tutorials_interfaces/action/fibonacci.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/bt_action_node.hpp"

class FibonacciRosActionNode
  : public BT::RosActionNode<action_tutorials_interfaces::action::Fibonacci>
{
public:
  using ActionType = action_tutorials_interfaces::action::Fibonacci;
  using Goal = ActionType::Goal;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;
  using WrappedResult = GoalHandle::WrappedResult;

  FibonacciRosActionNode(const std::string & name,
                         const BT::NodeConfig & conf,
                         const BT::RosNodeParams & params)
  : BT::RosActionNode<ActionType>(name, conf, params)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("order", 10, "Length of the Fibonacci sequence")
    };
  }

  bool setGoal(Goal & goal) override
  {
    // Récupère la valeur du port d'entrée "order" et la copie dans le goal.
    int order = 0;
    if (!getInput<int>("order", order)) {
      RCLCPP_ERROR(rclcpp::get_logger("FibonacciRosActionNode"),
                   "FibonacciRosActionNode: missing required input port [order]");
      return false;
    }
    goal.order = order;
    return true;
  }

  BT::NodeStatus onResultReceived(const WrappedResult & result) override
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(rclcpp::get_logger("FibonacciRosActionNode"),
                  "FibonacciRosActionNode: action succeeded, sequence length=%zu",
                  result.result->sequence.size());
      for (const auto & value : result.result->sequence) {
        RCLCPP_INFO(rclcpp::get_logger("FibonacciRosActionNode"), "  %d", value);
      }
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_ERROR(rclcpp::get_logger("FibonacciRosActionNode"),
                 "FibonacciRosActionNode: action failed or was aborted (code=%d)",
                 static_cast<int>(result.code));
    return BT::NodeStatus::FAILURE;
  }
};


