#!/bin/bash

apt update
apt install ros-humble-xacro
source /opt/ros/humble/setup.bash

PARAMS_FILE="ros2/install/lio_sam/share/lio_sam/config/params.yaml"
sed -i -E 's|^([[:space:]]*rosLogFile:[[:space:]]*).*$|\1"/user_space/nav/data/run_nav.json"|' "$PARAMS_FILE"
echo "Updated rosLogFile in $PARAMS_FILE"

