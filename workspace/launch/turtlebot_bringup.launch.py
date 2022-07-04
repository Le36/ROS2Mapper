import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    turtlebot3_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("turtlebot3_bringup"), "launch"
                ),
                "/robot.launch.py",
            ]
        )
    )
    return LaunchDescription(
        [
            turtlebot3_bringup,
            Node(package="ros2mapper_camera_node", executable="launch", name="ros2mapper_camera_node"),
            Node(package="ros2mapper_qr_code_reader", executable="launch", name"ros2mapper_qr_code_reader"),
        ]
    )
