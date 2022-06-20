#!/bin/bash
trap "exit" INT TERM ERR
trap "kill 0" EXIT

source install/setup.bash

if [[ -z $IP && -z $BOT ]]; then
  # Gazebo
  ros2 launch launch/test_world_${1:-2}.launch.py use_sim_time:=True >/dev/null 2>&1 &
  ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True >/dev/null 2>&1 &
  sleep 0.5
  ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True >/dev/null 2>&1 &
  ros2 launch launch/gazebo_launch.py use_sim_time:=True >/dev/null 2>&1 &
  ros2 run io_node control use_sim_time:=True
  exit 0
elif [ -z "$BOT" ]; then
  # Remote -> Raspi
  ssh ubuntu@"$IP" 'cd ~/ros2-mapper/workspace && BOT=1 ./run.sh' >/dev/null 2>&1 &
  ros2 launch/remote_launch.py >/dev/null 2>&1 &
  ros2 run io_node control
  exit 0
else
  # Raspi
  ros2 launch turtlebot3_bringup robot.launch.py >/dev/null 2>&1 &
  sleep 20
  ros2 run camera_node launch >/dev/null 2>&1 
  wait
fi
