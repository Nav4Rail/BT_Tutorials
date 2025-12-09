#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "behaviortree_cpp/bt_factory.h"

#include "action_server_bt/action/run_tree.hpp"

class BtActionServerNode : public rclcpp::Node
{
public:
  using RunTree = action_server_bt::action::RunTree;
  using GoalHandleRunTree = rclcpp_action::ServerGoalHandle<RunTree>;

  BtActionServerNode();

private:
  rclcpp_action::Server<RunTree>::SharedPtr action_server_;
  BT::BehaviorTreeFactory factory_;

  std::mutex tree_mutex_;
  std::shared_ptr<BT::Tree> active_tree_;
  std::thread bt_thread_;
  std::atomic_bool stop_requested_{false};

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const RunTree::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleRunTree> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleRunTree> goal_handle);

  void execute_tree(const std::shared_ptr<GoalHandleRunTree> goal_handle);

  void register_simple_nodes();
};


