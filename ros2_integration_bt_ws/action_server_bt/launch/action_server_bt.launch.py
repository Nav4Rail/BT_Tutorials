from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bt_action_server_node = Node(
        package="action_server_bt",
        executable="bt_action_server",
        name="bt_action_server",
        output="screen",
    )

    return LaunchDescription([bt_action_server_node])


