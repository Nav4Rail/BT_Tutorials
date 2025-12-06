#include "behaviortree_cpp/bt_factory.h"
#include "dummy_nodes.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fibonacci_action_client");
  RosNodeParams params;
  params.nh = node;
  params.default_port_value = "fibonacci";
  factory.registerNodeType<FibonacciAction>("Fibonacci", params);
}