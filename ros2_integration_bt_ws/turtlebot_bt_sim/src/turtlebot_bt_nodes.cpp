#include "turtlebot_bt_sim/turtlebot_bt_nodes.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace turtlebot_bt_sim
{

void register_turtlebot_nodes(BT::BehaviorTreeFactory & factory,
                              const std::shared_ptr<CmdVelHandle> & handle)
{
  factory.registerSimpleAction(
    "DriveForward",
    [handle](BT::TreeNode & self) -> BT::NodeStatus
    {
      const double duration =
        self.getInput<double>("duration").value_or(2.0);
      const double speed =
        self.getInput<double>("speed").value_or(0.2);

      if (!handle || !handle->node || !handle->cmd_vel_pub) {
        RCLCPP_ERROR(rclcpp::get_logger("turtlebot_bt_sim"),
                     "DriveForward: invalid cmd_vel handle");
        return BT::NodeStatus::FAILURE;
      }

      auto twist = geometry_msgs::msg::Twist();
      twist.linear.x = speed;

      rclcpp::Rate rate(10.0);
      const auto start = handle->node->now();

      RCLCPP_INFO(handle->node->get_logger(),
                  "DriveForward: speed=%.2f m/s, duration=%.2f s",
                  speed, duration);

      while (rclcpp::ok() &&
             (handle->node->now() - start).seconds() < duration) {
        handle->cmd_vel_pub->publish(twist);
        rate.sleep();
      }

      twist.linear.x = 0.0;
      handle->cmd_vel_pub->publish(twist);

      return BT::NodeStatus::SUCCESS;
    });

  factory.registerSimpleAction(
    "Rotate",
    [handle](BT::TreeNode & self) -> BT::NodeStatus
    {
      const double duration =
        self.getInput<double>("duration").value_or(3.0);
      const double angular_speed =
        self.getInput<double>("angular_speed").value_or(0.6);

      if (!handle || !handle->node || !handle->cmd_vel_pub) {
        RCLCPP_ERROR(rclcpp::get_logger("turtlebot_bt_sim"),
                     "Rotate: invalid cmd_vel handle");
        return BT::NodeStatus::FAILURE;
      }

      auto twist = geometry_msgs::msg::Twist();
      twist.angular.z = angular_speed;

      rclcpp::Rate rate(10.0);
      const auto start = handle->node->now();

      RCLCPP_INFO(handle->node->get_logger(),
                  "Rotate: angular_speed=%.2f rad/s, duration=%.2f s",
                  angular_speed, duration);

      while (rclcpp::ok() &&
             (handle->node->now() - start).seconds() < duration) {
        handle->cmd_vel_pub->publish(twist);
        rate.sleep();
      }

      twist.angular.z = 0.0;
      handle->cmd_vel_pub->publish(twist);

      return BT::NodeStatus::SUCCESS;
    });

  factory.registerSimpleAction(
    "StopRobot",
    [handle](BT::TreeNode &) -> BT::NodeStatus
    {
      if (!handle || !handle->cmd_vel_pub) {
        RCLCPP_ERROR(rclcpp::get_logger("turtlebot_bt_sim"),
                     "StopRobot: invalid cmd_vel handle");
        return BT::NodeStatus::FAILURE;
      }

      auto twist = geometry_msgs::msg::Twist();
      handle->cmd_vel_pub->publish(twist);

      RCLCPP_INFO(handle->node->get_logger(), "StopRobot: cmd_vel set to zero");

      return BT::NodeStatus::SUCCESS;
    });
}

}  // namespace turtlebot_bt_sim


