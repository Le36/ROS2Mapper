#!/bin/bash
trap "exit" INT TERM ERR
trap "kill 0" EXIT

SETUP_FILE=$(find $HOME -iwholename "*/m-explore-ros2/install/setup.bash")
if [ -f "$SETUP_FILE" ]; then
    source "$SETUP_FILE"
    ros2 launch turtlebot3_navigation2 navigation2.launch.py &
    ros2 launch slam_toolbox online_async_launch.py &
    ros2 launch explore_lite explore.launch.py &
    wait
else
    echo "Couldnt find ~/**/m-explore-ros2/install/setup.bash"
fi
