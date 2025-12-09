#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "behaviortree_cpp/bt_factory.h"

#include "turtlebot_bt_sim/turtlebot_bt_nodes.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("turtlebot_bt_node");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spin_thread([&exec]() { exec.spin(); });

  auto handle = std::make_shared<turtlebot_bt_sim::CmdVelHandle>();
  handle->node = node;
  handle->cmd_vel_pub =
    node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  BT::BehaviorTreeFactory factory;
  turtlebot_bt_sim::register_turtlebot_nodes(factory, handle);

  const std::string package_share =
    ament_index_cpp::get_package_share_directory("turtlebot_bt_sim");
  const std::string tree_file = package_share + "/trees/turtlebot_mission.xml";

  RCLCPP_INFO(node->get_logger(), "Loading BehaviorTree from: %s", tree_file.c_str());

  BT::Tree tree = factory.createTreeFromFile(tree_file);

  const BT::NodeStatus status = tree.tickWhileRunning(10ms);

  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "TurtleBot BT mission finished with SUCCESS");
  } else if (status == BT::NodeStatus::FAILURE) {
    RCLCPP_ERROR(node->get_logger(), "TurtleBot BT mission finished with FAILURE");
  }

  exec.cancel();
  spin_thread.join();
  rclcpp::shutdown();

  return 0;
}


