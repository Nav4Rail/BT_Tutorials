#include "action_server_bt/bt_action_server_node.hpp"

#include <chrono>
#include <fstream>
#include <iostream>

using namespace std::chrono_literals;

namespace
{

class SaySomethingNode : public BT::SyncActionNode
{
public:
  SaySomethingNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("message", "Hello from BT", "Message to print") };
  }

  BT::NodeStatus tick() override
  {
    std::string msg;
    (void)getInput("message", msg);
    std::cout << "[BT] SaySomething: " << msg << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

class WaitMsNode : public BT::SyncActionNode
{
public:
  WaitMsNode(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<int>("milliseconds", 1000, "Delay in milliseconds") };
  }

  BT::NodeStatus tick() override
  {
    int ms = 1000;
    (void)getInput("milliseconds", ms);
    if (ms < 0) {
      ms = 0;
    }
    std::cout << "[BT] WaitMs: sleeping for " << ms << " ms" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace

BtActionServerNode::BtActionServerNode()
: rclcpp::Node("bt_action_server_node")
{
  register_simple_nodes();
}

void BtActionServerNode::init_action_server()
{
  // IMPORTANT : ne pas appeler shared_from_this() dans le constructeur.
  // Cette méthode doit être appelée APRÈS la création du shared_ptr<BtActionServerNode>.
  auto self = shared_from_this();

  action_server_ = rclcpp_action::create_server<RunTree>(
    self,
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
  stop_requested_.store(false);

  auto feedback = std::make_shared<RunTree::Feedback>();

  // Exécute l'arbre en boucle interne BehaviorTree.CPP.
  // Pour garder l'exemple simple, on ne préempte pas l'exécution en cours :
  // une annulation sera prise en compte à la fin du tickWhileRunning.
  BT::NodeStatus status = tree.tickWhileRunning(10ms);

  // Feedback final : nom du nœud racine (exemple minimal).
  feedback->current_node = tree.rootNode()->name();
  goal_handle->publish_feedback(feedback);

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
}

void BtActionServerNode::register_simple_nodes()
{
  factory_.registerNodeType<SaySomethingNode>("SaySomething");
  factory_.registerNodeType<WaitMsNode>("WaitMs");
}

