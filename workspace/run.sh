#!/bin/bash
trap "exit" INT TERM ERR
trap "kill 0" EXIT

source install/setup.bash

if [[ -z $IP || -z $BOT ]]; then
    # Remote
    ros2 launch launch/test_world_2.launch.py >/dev/null 2>&1 &
elif [ -z $IP ]; then
    # Remote -> Raspi
    ssh ubuntu@$(IP) 'cd ~/ros2-mapper/workspace && BOT=1 ./run.sh' &

    # TODO: Run the remote part of the software
else
    # Raspi
    ros2 launch turtlebot3_bringup robot.launch.py
fi

if [ -z $BOT ]; then
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True >/dev/null 2>&1 &
    ros2 launch slam_toolbox online_async_launch.py >/dev/null 2>&1 &
    ros2 launch explore_lite explore.launch.py >/dev/null 2>&1 &
    ros2 run v4l2_camera v4l2_camera_node >/dev/null 2>&1 &
    ros2 launch launch/launch.py &
fi

wait
