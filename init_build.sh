#!/bin/bash

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

# CPU 모델 확인
CPU_MODEL=$(cat /proc/cpuinfo | grep "model name" | head -1 | awk -F'i[3579]-' '{split($2, a, " "); sub(/[^0-9]+$/, "", a[1]); print a[1]}')
CPU_VER=${CPU_MODEL//[!0-9]/}
CPU_NOW=$(cat /proc/cpuinfo | awk -F: '/model name/ {print $2; exit}')
# CPU 모델 비교
if [[ "$CPU_VER" -lt "5000" ]]; then
    ## intel 5세대보다 작으면
    echo "[$CPU_NOW] >> parallel-workers 1"
    colcon build --parallel-workers 1
else
    colcon build
fi
source install/setup.bash

