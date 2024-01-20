#!/bin/bash

## ROS2 Packages
if [ -x "$(command -v ./ros2/src/livox_ros_driver2/build.sh)" ]; then
    ./ros2/src/livox_ros_driver2/build.sh humble
else
    echo "[livox_ros_driver2/build.sh] not found"
    exit 1
fi

cd ros2
colcon build --packages-select autorccar_interfaces
if [ $? -ne 0 ]; then
    echo "[autorccar_interfaces] build error!"
    exit 1
fi
source install/setup.bash

## remain packages
colcon build
source install/setup.bash
