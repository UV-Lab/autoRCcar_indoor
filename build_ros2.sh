#!/bin/bash

apt update
apt install ros-humble-xacro
source /opt/ros/humble/setup.bash
cd ros2

PARAMS_FILE="ros2/install/lio_sam/share/lio_sam/config/params.yaml"
sed -i -E 's|^([[:space:]]*rosLogFile:[[:space:]]*).*$|\1"/user_space/nav/data/run_nav.json"|' "$PARAMS_FILE"
echo "Updated rosLogFile in $PARAMS_FILE"


#colcon build --packages-select rob_common  --event-handlers console_direct+
#bash src/livox_ros_driver2/build.sh humble
# colcon build --packages-select autorccar_costmap autorccar_gcs autorccar_hardware_control autorccar_interfaces autorccar_planning_control
#colcon build --packages-select lio_sam  --event-handlers console_direct+
source install/setup.bash
cd ..
