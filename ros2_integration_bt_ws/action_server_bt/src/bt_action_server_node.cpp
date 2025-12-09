#include "action_server_bt/bt_action_server_node.hpp"

#include <chrono>
#include <fstream>

using namespace std::chrono_literals;

BtActionServerNode::BtActionServerNode()
: rclcpp::Node("bt_action_server_node")
{
  register_simple_nodes();

  action_server_ = rclcpp_action::create_server<RunTree>(
    shared_from_this(),
    "run_tree",
    std::bind(&BtActionServerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&BtActionServerNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&BtActionServerNode::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "BT Action Server [/run_tree] ready");
}

rclcpp_action::GoalResponse BtActionServerNode::handle_goal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const RunTree::Goal> goal)
{
  if (goal->tree_id.empty()) {
    RCLCPP_WARN(get_logger(), "Received goal with empty tree_id, using 'simple_mission'");
  } else {
    RCLCPP_INFO(get_logger(), "Received goal for tree_id='%s'", goal->tree_id.c_str());
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse BtActionServerNode::handle_cancel(
  const std::shared_ptr<GoalHandleRunTree> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  stop_requested_.store(true);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void BtActionServerNode::handle_accepted(
  const std::shared_ptr<GoalHandleRunTree> goal_handle)
{
  // Exécuter l'arbre dans un thread séparé pour ne pas bloquer l'exécuteur.
  std::thread(
    [this, goal_handle]()
    {
      execute_tree(goal_handle);
    }).detach();
}

void BtActionServerNode::execute_tree(
  const std::shared_ptr<GoalHandleRunTree> goal_handle)
{
  const auto goal = goal_handle->get_goal();

  const std::string package_share =
    ament_index_cpp::get_package_share_directory("action_server_bt");

  const std::string tree_id = goal->tree_id.empty() ? "simple_mission" : goal->tree_id;
  const std::string tree_file = package_share + "/trees/" + tree_id + ".xml";

  {
    std::ifstream fin(tree_file);
    if (!fin.good()) {
      RCLCPP_ERROR(
        get_logger(), "BT XML file not found: %s. Aborting goal.",
        tree_file.c_str());
      auto result = std::make_shared<RunTree::Result>();
      result->success = false;
      result->message = "Tree XML file not found: " + tree_file;
      goal_handle->abort(result);
      return;
    }
  }

  RCLCPP_INFO(get_logger(), "Loading BehaviorTree from: %s", tree_file.c_str());

  BT::Tree tree = factory_.createTreeFromFile(tree_file);

  {
    std::lock_guard<std::mutex> lock(tree_mutex_);
    active_tree_ = std::make_shared<BT::Tree>(std::move(tree));
  }

  stop_requested_.store(false);

  auto feedback = std::make_shared<RunTree::Feedback>();

  const auto start_time = now();
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  while (rclcpp::ok() && !stop_requested_.load() && status == BT::NodeStatus::RUNNING) {
    {
      std::lock_guard<std::mutex> lock(tree_mutex_);
      if (active_tree_) {
        status = active_tree_->tickRoot();
        feedback->current_node = active_tree_->rootNode()->name();
      } else {
        status = BT::NodeStatus::FAILURE;
      }
    }

    goal_handle->publish_feedback(feedback);
    RCLCPP_DEBUG(
      get_logger(), "BT running for %.2f seconds",
      (now() - start_time).seconds());

    std::this_thread::sleep_for(50ms);
  }

  auto result = std::make_shared<RunTree::Result>();

  if (stop_requested_.load()) {
    RCLCPP_WARN(get_logger(), "BT execution cancelled by client");
    result->success = false;
    result->message = "Cancelled by client";
    goal_handle->canceled(result);
  } else if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(get_logger(), "BT finished with SUCCESS");
    result->success = true;
    result->message = "Tree finished with SUCCESS";
    goal_handle->succeed(result);
  } else {
    RCLCPP_ERROR(get_logger(), "BT finished with FAILURE");
    result->success = false;
    result->message = "Tree finished with FAILURE";
    goal_handle->abort(result);
  }

  {
    std::lock_guard<std::mutex> lock(tree_mutex_);
    active_tree_.reset();
  }
}

void BtActionServerNode::register_simple_nodes()
{
  // Simple action qui affiche un message.
  factory_.registerSimpleAction(
    "SaySomething",
    [](BT::TreeNode & self) -> BT::NodeStatus
    {
      auto msg = self.getInput<std::string>("message").value_or("Hello from BT");
      std::cout << "[BT] SaySomething: " << msg << std::endl;
      return BT::NodeStatus::SUCCESS;
    });

  // Simple action qui attend un certain temps (en millisecondes).
  factory_.registerSimpleAction(
    "WaitMs",
    [](BT::TreeNode & self) -> BT::NodeStatus
    {
      int ms = self.getInput<int>("milliseconds").value_or(1000);
      if (ms < 0) {
        ms = 0;
      }
      std::cout << "[BT] WaitMs: sleeping for " << ms << " ms" << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(ms));
      return BT::NodeStatus::SUCCESS;
    });
}


