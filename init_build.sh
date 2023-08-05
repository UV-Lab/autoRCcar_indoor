#!/bin/bash

# CPU 모델 확인
CPU_MODEL=$(cat /proc/cpuinfo | grep "model name" | head -1 | awk -F'i[3579]-' '{split($2, a, " "); sub(/[^0-9]+$/, "", a[1]); print a[1]}')
CPU_VER=${CPU_MODEL//[!0-9]/}
CPU_NOW=$(cat /proc/cpuinfo | awk -F: '/model name/ {print $2; exit}')

## Livox-SDK2
echo "Would you want to build include Livox-SDK2?"
echo "You need to build Livox-SDK2 once for the first time."
echo "If your answer is no, only the ROS2 packages is build."
read -p "Answer yes or no : " answer


if [[ "$answer" == "YES" || "$answer" == "Yes" || "$answer" == "yes" ]]; then

    echo "Important) You need to enter your sudo password during build."
    sleep 3
    
    cd Livox-SDK2
    if [ -d "build" ]; then
        rm -rf build
    fi
    mkdir build && cd build
    if [[ "$CPU_VER" -lt "5000" ]]; then
        cmake .. && make -j 2
    else
        cmake .. && make -j
    fi
    sudo make install
    cd ../..

elif [[ "$answer" == "NO" || "$answer" == "No" || "$answer" == "no" ]]; then
    echo ""

else
    echo "entered incorrectly"
    exit 1
fi


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


# CPU 모델 비교
if [[ "$CPU_VER" -lt "5000" ]]; then
    ## intel 5세대보다 작으면
    echo "[$CPU_NOW] >> parallel-workers 1"
    colcon build --parallel-workers 1
else
    colcon build
fi
source install/setup.bash

