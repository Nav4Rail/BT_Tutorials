#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "behaviortree_cpp/bt_factory.h"

namespace turtlebot_bt_sim
{

struct CmdVelHandle
{
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
};

void register_turtlebot_nodes(BT::BehaviorTreeFactory & factory,
                              const std::shared_ptr<CmdVelHandle> & handle);

}  // namespace turtlebot_bt_sim


