#include "turtlebot_bt_sim/turtlebot_bt_nodes.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace turtlebot_bt_sim
{

namespace
{

class IdleNode : public BT::SyncActionNode
{
public:
  IdleNode(const std::string & name,
           const BT::NodeConfiguration & config,
           std::shared_ptr<CmdVelHandle> handle)
  : BT::SyncActionNode(name, config),
    handle_(std::move(handle))
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("duration", 2.0, "Duration of idle motion (seconds)")
    };
  }

  BT::NodeStatus tick() override
  {
    double duration = 2.0;
    (void)getInput("duration", duration);

    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;

    rclcpp::WallRate rate(10.0);
    const auto start = std::chrono::steady_clock::now();
    const auto max_duration = std::chrono::duration<double>(duration);

    RCLCPP_INFO(handle_->node->get_logger(),
                "Idle: duration=%.2f s",
                duration);

    while (rclcpp::ok() &&
           (std::chrono::steady_clock::now() - start) < max_duration) {
      handle_->cmd_vel_pub->publish(twist);
      rate.sleep();
    }

    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    handle_->cmd_vel_pub->publish(twist);

    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<CmdVelHandle> handle_;
};

class DriveForwardNode : public BT::SyncActionNode
{
public:
  DriveForwardNode(const std::string & name,
                   const BT::NodeConfiguration & config,
                   std::shared_ptr<CmdVelHandle> handle)
  : BT::SyncActionNode(name, config),
    handle_(std::move(handle))
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("duration", 2.0, "Duration of forward motion (seconds)"),
      BT::InputPort<double>("speed", 0.2, "Linear speed (m/s)")
    };
  }

  BT::NodeStatus tick() override
  {
    if (!handle_ || !handle_->node || !handle_->cmd_vel_pub) {
      RCLCPP_ERROR(rclcpp::get_logger("turtlebot_bt_sim"),
                   "DriveForward: invalid cmd_vel handle");
      return BT::NodeStatus::FAILURE;
    }

    double duration = 2.0;
    double speed = 0.2;

    (void)getInput("duration", duration);
    (void)getInput("speed", speed);

    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = speed;

    rclcpp::WallRate rate(10.0);
    const auto start = std::chrono::steady_clock::now();
    const auto max_duration = std::chrono::duration<double>(duration);

    RCLCPP_INFO(handle_->node->get_logger(),
                "DriveForward: speed=%.2f m/s, duration=%.2f s",
                speed, duration);

    while (rclcpp::ok() &&
           (std::chrono::steady_clock::now() - start) < max_duration) {
      handle_->cmd_vel_pub->publish(twist);
      rate.sleep();
    }

    twist.linear.x = 0.0;
    handle_->cmd_vel_pub->publish(twist);

    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<CmdVelHandle> handle_;
};

class RotateNode : public BT::SyncActionNode
{
public:
  RotateNode(const std::string & name,
             const BT::NodeConfiguration & config,
             std::shared_ptr<CmdVelHandle> handle)
  : BT::SyncActionNode(name, config),
    handle_(std::move(handle))
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("duration", 3.0, "Rotation duration (seconds)"),
      BT::InputPort<double>("angular_speed", 0.6, "Angular speed (rad/s)")
    };
  }

  BT::NodeStatus tick() override
  {
    if (!handle_ || !handle_->node || !handle_->cmd_vel_pub) {
      RCLCPP_ERROR(rclcpp::get_logger("turtlebot_bt_sim"),
                   "Rotate: invalid cmd_vel handle");
      return BT::NodeStatus::FAILURE;
    }

    double duration = 3.0;
    double angular_speed = 0.6;

    (void)getInput("duration", duration);
    (void)getInput("angular_speed", angular_speed);

    auto twist = geometry_msgs::msg::Twist();
    twist.angular.z = angular_speed;

    rclcpp::WallRate rate(10.0);
    const auto start = std::chrono::steady_clock::now();
    const auto max_duration = std::chrono::duration<double>(duration);

    RCLCPP_INFO(handle_->node->get_logger(),
                "Rotate: angular_speed=%.2f rad/s, duration=%.2f s",
                angular_speed, duration);

    while (rclcpp::ok() &&
           (std::chrono::steady_clock::now() - start) < max_duration) {
      handle_->cmd_vel_pub->publish(twist);
      rate.sleep();
    }

    twist.angular.z = 0.0;
    handle_->cmd_vel_pub->publish(twist);

    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<CmdVelHandle> handle_;
};

class StopRobotNode : public BT::SyncActionNode
{
public:
  StopRobotNode(const std::string & name,
                const BT::NodeConfiguration & config,
                std::shared_ptr<CmdVelHandle> handle)
  : BT::SyncActionNode(name, config),
    handle_(std::move(handle))
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    if (!handle_ || !handle_->cmd_vel_pub) {
      RCLCPP_ERROR(rclcpp::get_logger("turtlebot_bt_sim"),
                   "StopRobot: invalid cmd_vel handle");
      return BT::NodeStatus::FAILURE;
    }

    auto twist = geometry_msgs::msg::Twist();
    handle_->cmd_vel_pub->publish(twist);

    RCLCPP_INFO(handle_->node->get_logger(), "StopRobot: cmd_vel set to zero");

    return BT::NodeStatus::SUCCESS;
  }

private:
  std::shared_ptr<CmdVelHandle> handle_;
};

}  // namespace

void register_turtlebot_nodes(BT::BehaviorTreeFactory & factory,
                              const std::shared_ptr<CmdVelHandle> & handle)
{
  factory.registerBuilder<IdleNode>(
    "Idle",
    [handle](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<IdleNode>(name, config, handle);
    });

  factory.registerBuilder<DriveForwardNode>(
    "DriveForward",
    [handle](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<DriveForwardNode>(name, config, handle);
    });

  factory.registerBuilder<RotateNode>(
    "Rotate",
    [handle](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<RotateNode>(name, config, handle);
    });

  factory.registerBuilder<StopRobotNode>(
    "StopRobot",
    [handle](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<StopRobotNode>(name, config, handle);
    });
}

}  // namespace turtlebot_bt_sim
