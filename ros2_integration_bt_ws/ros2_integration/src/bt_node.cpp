#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "behaviortree_cpp/bt_factory.h"

#include "ros2_integration/fibonacci_bt_node.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("bt_fibonacci_manual_node");

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

  rclcpp::Rate rate(10.0);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    const BT::NodeStatus status = tree.tickRoot();
    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "Behavior tree finished with SUCCESS");
      break;
    }
    if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_ERROR(node->get_logger(), "Behavior tree finished with FAILURE");
      break;
    }

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}


