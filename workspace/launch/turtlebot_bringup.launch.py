import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    slam = IncludeLaunchDescription(
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
            slam,
            Node(package="camera_node", executable="launch", name="camera_node"),
        ]
    )
