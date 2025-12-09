#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include "action_server_bt/bt_action_server_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<BtActionServerNode>();
  node->init_action_server();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}


