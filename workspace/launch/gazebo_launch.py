from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="qr_code_reader",
                executable="launch",
                name="qr_code_reader",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="memory_node",
                executable="listener",
                name="memory_node",
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
