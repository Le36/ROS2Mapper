#!/bin/bash
trap "exit" INT TERM ERR
trap "kill 0" EXIT

ros2 run v4l2_camera v4l2_camera_node &
ros2 run rqt_image_view rqt_image_view &

wait
