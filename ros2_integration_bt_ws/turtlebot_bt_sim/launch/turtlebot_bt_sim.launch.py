import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    LogInfo,
)
from launch.event_handlers import OnProcessExit
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

    # Paths to required packages/files
    turtlebot3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    turtlebot3_description_share = get_package_share_directory(
        "turtlebot3_description"
    )

    world_path = os.path.join(
        turtlebot3_gazebo_share,
        "worlds",
        "turtlebot3_world.world",
    )

    # Gazebo server + client with the TurtleBot3 world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world_path,
        }.items(),
    )

    # Robot state publisher (URDF loaded from TurtleBot3 description)
    burger_urdf = os.path.join(
        turtlebot3_description_share,
        "urdf",
        "turtlebot3_burger.urdf",
    )

    with open(burger_urdf, "r") as urdf_file:
        robot_description = urdf_file.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            }
        ],
    )

    # Spawn the TurtleBot3 robot into Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_turtlebot3",
        output="screen",
        arguments=[
            "-entity",
            "burger",
            "-file",
            os.path.join(
                turtlebot3_gazebo_share,
                "models",
                "turtlebot3_burger",
                "model.sdf",
            ),
            "-x",
            "-2.0",
            "-y",
            "-0.5",
            "-z",
            "0.01",
        ],
    )

    # Behavior Tree node (BT waits internally for /cmd_vel subscriber)
    bt_node = Node(
        package="turtlebot_bt_sim",
        executable="turtlebot_bt_node",
        name="turtlebot_bt_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Start BT only after spawn_entity has completed successfully
    def on_spawn_exit(event, context):
        if event.returncode == 0:
            return [
                LogInfo(
                    msg="SpawnEntity finished successfully. Starting BT node."
                ),
                bt_node,
            ]
        return [
            LogInfo(
                msg=(
                    "SpawnEntity exited with return code "
                    f"{event.returncode}. Not starting BT node."
                )
            )
        ]

    spawn_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=on_spawn_exit,
        )
    )

    return LaunchDescription(
        [
            declare_model_arg,
            declare_use_sim_time_arg,
            gazebo,
            robot_state_publisher,
            spawn_entity,
            spawn_exit_handler,
        ]
    )
