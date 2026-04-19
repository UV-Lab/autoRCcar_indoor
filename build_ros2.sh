#!/bin/bash

## ROS2 Packages
if [ -x "$(command -v ./ros2/src/livox_ros_driver2/build.sh)" ]; then
    ./ros2/src/livox_ros_driver2/build.sh humble
else
    echo "[livox_ros_driver2/build.sh] not found"
    exit 1
fi

cd ros2
#  colcon build
colcon build --packages-select rob_common
colcon build --packages-select autorccar_costmap autorccar_gcs autorccar_hardware_control autorccar_interfaces autorccar_planning_control
colcon build --packages-select lio_sam  --event-handlers console_direct+
source install/setup.bash
