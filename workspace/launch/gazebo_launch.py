from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="qr_code_reader", executable="launch", name="qr_code_reader"),
            Node(package="memory_node", executable="listener", name="memory_node"),
        ]
    )
