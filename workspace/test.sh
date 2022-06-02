#!/bin/bash

source install/setup.bash
ros2 launch launch/launch.py >/dev/null &
PID=$!

sleep 1
pytest src/tests
RET=$?

pkill -TERM -P $PID
exit $RET
