#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

TURTLEBOT3_MODEL = os.environ["TURTLEBOT3_MODEL"]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    file_dir = os.path.dirname(__file__)
    world_file_name = "test_world_1/test_world_1.world"
    world = os.path.join(file_dir, "..", "worlds", world_file_name)

    home_dir = os.path.expanduser("~")
    turtlebot3_launch_file_dir = os.path.join(
        home_dir,
        "turtlebot3_ws/src/turtlebot3/",
        "turtlebot3_simulations/turtlebot3_gazebo/launch/",
    )

    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
                ),
                launch_arguments={"world": world}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [turtlebot3_launch_file_dir, "/robot_state_publisher.launch.py"]
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
        ]
    )
