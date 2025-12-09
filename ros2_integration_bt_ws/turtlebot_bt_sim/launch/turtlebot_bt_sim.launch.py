import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_model_arg = DeclareLaunchArgument(
        "model",
        default_value="burger",
        description="TurtleBot3 model [burger, waffle, waffle_pi]",
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    # Use the standard TurtleBot3 world launch (starts Gazebo, robot, and
    # robot_state_publisher). This is the same launch you used originally.
    turtlebot3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")

    world_launch = os.path.join(
        turtlebot3_gazebo_share,
        "launch",
        "turtlebot3_world.launch.py",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch),
        launch_arguments={
            "model": LaunchConfiguration("model"),
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Behavior Tree node (BT waits internally for /cmd_vel subscriber)
    bt_node = Node(
        package="turtlebot_bt_sim",
        executable="turtlebot_bt_node",
        name="turtlebot_bt_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Delay BT start a bit so that Gazebo + robot spawn can complete.
    # The BT main still blocks until /cmd_vel has a subscriber, so there
    # is no movement before the robot controller is ready.
    delayed_bt = TimerAction(period=5.0, actions=[bt_node])

    return LaunchDescription(
        [
            declare_model_arg,
            declare_use_sim_time_arg,
            gazebo,
            delayed_bt,
        ]
    )
