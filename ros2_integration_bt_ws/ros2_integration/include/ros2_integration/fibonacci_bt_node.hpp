/*!
 * FibonacciActionNode - Action Node BehaviorTree.CPP qui appelle l'action ROS 2 Fibonacci
 * via un rclcpp_action::Client.
 *
 * Approche : on envoie le goal au premier tick (onStart), puis on surveille la
 * disponibilité du résultat à chaque tick (onRunning). Le résultat est reçu
 * dans un callback asynchrone et stocké dans last_result_.
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "action_tutorials_interfaces/action/fibonacci.hpp"

#include "behaviortree_cpp/action_node.h"

class FibonacciActionNode : public BT::StatefulActionNode
{
public:
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;
  using WrappedResult = GoalHandleFibonacci::WrappedResult;

  FibonacciActionNode(const std::string & name,
                      const BT::NodeConfiguration & config,
                      const rclcpp::Node::SharedPtr & node)
  : BT::StatefulActionNode(name, config),
    node_(node)
  {
    client_ = rclcpp_action::create_client<Fibonacci>(node_, "fibonacci");
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<int>("order", 10, "Length of the Fibonacci sequence") };
  }

  BT::NodeStatus onStart() override
  {
    int order = 0;
    if (!getInput<int>("order", order)) {
      RCLCPP_ERROR(node_->get_logger(), "FibonacciActionNode: missing required input port [order]");
      return BT::NodeStatus::FAILURE;
    }

    if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(node_->get_logger(), "FibonacciActionNode: action server [/fibonacci] not available");
      return BT::NodeStatus::FAILURE;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_result_.reset();
    }

    Fibonacci::Goal goal;
    goal.order = order;

    typename rclcpp_action::Client<Fibonacci>::SendGoalOptions options;
    options.result_callback =
      [this](const WrappedResult & result)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        last_result_ = std::make_shared<WrappedResult>(result);
      };

    client_->async_send_goal(goal, options);

    RCLCPP_INFO(node_->get_logger(), "FibonacciActionNode: goal sent with order=%d", order);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!last_result_) {
      // Pas encore de résultat : l'action est toujours en cours
      return BT::NodeStatus::RUNNING;
    }

    if (last_result_->code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(node_->get_logger(), "FibonacciActionNode: action succeeded");
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_ERROR(node_->get_logger(), "FibonacciActionNode: action failed or was aborted");
    return BT::NodeStatus::FAILURE;
  }

  void onHalted() override
  {
    RCLCPP_WARN(node_->get_logger(), "FibonacciActionNode: halted, cancelling goals");
    client_->async_cancel_all_goals();

    std::lock_guard<std::mutex> lock(mutex_);
    last_result_.reset();
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Fibonacci>::SharedPtr client_;

  std::mutex mutex_;
  std::shared_ptr<WrappedResult> last_result_;
};


