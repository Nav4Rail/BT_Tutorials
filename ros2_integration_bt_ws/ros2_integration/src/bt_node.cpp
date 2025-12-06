#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "behaviortree_cpp/bt_factory.h"

#include "ros2_integration/fibonacci_bt_node.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("bt_fibonacci_manual_node");

  // Exécuteur ROS 2 dans un thread séparé pour traiter les callbacks d'action
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spin_thread([&exec]() { exec.spin(); });

  BT::BehaviorTreeFactory factory;

  // Enregistre le nœud d'action Fibonacci en fournissant le node ROS 2
  factory.registerBuilder<FibonacciActionNode>(
    "FibonacciActionNode",
    [node](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<FibonacciActionNode>(name, config, node);
    });

  // Localise le fichier XML installé dans share/ros2_integration/trees
  const std::string package_share =
    ament_index_cpp::get_package_share_directory("ros2_integration");
  const std::string tree_file = package_share + "/trees/fibonacci_manual.xml";

  RCLCPP_INFO(node->get_logger(), "Loading BehaviorTree from: %s", tree_file.c_str());

  BT::Tree tree = factory.createTreeFromFile(tree_file);

  // Tick l'arbre tant qu'il renvoie RUNNING (bloquant côté BT, mais les callbacks ROS
  // tournent dans le thread de l'exécuteur).
  const BT::NodeStatus status = tree.tickWhileRunning(10ms);

  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Behavior tree finished with SUCCESS");
  } else if (status == BT::NodeStatus::FAILURE) {
    RCLCPP_ERROR(node->get_logger(), "Behavior tree finished with FAILURE");
  }

  exec.cancel();
  spin_thread.join();

  rclcpp::shutdown();
  return 0;
}
