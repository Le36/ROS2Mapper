#!/bin/bash

source install/setup.bash
ros2 launch launch/launch.py >/dev/null &
PID=$!

sleep 5
pytest src/tests -s

RET=$?

pkill -TERM -P $PID

pytest src/memory_node/test/memory_node/ -s
exit $RET
