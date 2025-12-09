import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    model = LaunchConfiguration("model")
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

    turtlebot3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")

    world_launch = os.path.join(
        turtlebot3_gazebo_share,
        "launch",
        "turtlebot3_world.launch.py",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch),
        launch_arguments={
            "model": model,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    bt_node = Node(
        package="turtlebot_bt_sim",
        executable="turtlebot_bt_node",
        name="turtlebot_bt_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            declare_model_arg,
            declare_use_sim_time_arg,
            gazebo,
            bt_node,
        ]
    )




