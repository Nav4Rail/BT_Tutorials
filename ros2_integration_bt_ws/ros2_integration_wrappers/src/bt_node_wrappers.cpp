#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/bt_action_node.hpp"

#include "ros2_integration_wrappers/fibonacci_ros_action_node.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("bt_fibonacci_ros2_wrapper_node");

  // Exécuteur ROS 2 dans un thread séparé pour traiter les callbacks.
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spin_thread([&exec]() { exec.spin(); });

  BT::BehaviorTreeFactory factory;

  // Paramètres ROS pour BehaviorTree.ROS2
  BT::RosNodeParams params;
  params.nh = node;
  // Nom de l'action à appeler si aucun port n'est spécifié dans le XML.
  // Correspond à l'action "fibonacci" du tutoriel ROS 2 Humble.
  params.default_port_value = "fibonacci";
  params.server_timeout = 5s;

  factory.registerBuilder<FibonacciRosActionNode>(
    "FibonacciRosActionNode",
    [params](const std::string & name, const BT::NodeConfig & config)
    {
      return std::make_unique<FibonacciRosActionNode>(name, config, params);
    });

  // Localise le fichier XML installé dans share/ros2_integration_wrappers/trees
  const std::string package_share =
    ament_index_cpp::get_package_share_directory("ros2_integration_wrappers");
  const std::string tree_file = package_share + "/trees/fibonacci_ros2.xml";

  RCLCPP_INFO(node->get_logger(), "Loading BehaviorTree from: %s", tree_file.c_str());

  BT::Tree tree = factory.createTreeFromFile(tree_file);

  const BT::NodeStatus status = tree.tickWhileRunning(10ms);

  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Behavior tree (wrappers) finished with SUCCESS");
  } else if (status == BT::NodeStatus::FAILURE) {
    RCLCPP_ERROR(node->get_logger(), "Behavior tree (wrappers) finished with FAILURE");
  }

  exec.cancel();
  spin_thread.join();

  rclcpp::shutdown();
  return 0;
}


