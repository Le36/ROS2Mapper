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
                    get_package_share_directory("turtlebot3_cartographer"), "launch"
                ),
                "/cartographer.launch.py",
            ]
        )
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("nav2_bringup"), "launch"),
                "/navigation_launch.py",
            ]
        )
    )

    return LaunchDescription(
        [
            slam,
            nav2,
            Node(package="ros2mapper_memory_node", executable="listener", name="ros2mapper_memory_node"),
            Node(package="ros2mapper_explore_node", executable="launch", name="ros2mapper_explore_node"),
        ]
    )
