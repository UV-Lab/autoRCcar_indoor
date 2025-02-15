# autoRCcar_indoor

## Bootstrapping

### Install and Build
Run `init_setup.sh` to install dependencies and build ROS2 packages. Or you can do it manually by refering to the script.

### Build
Run `build_ros2.sh` for the first build. It correctly builds the Livox package.


## Launch
```bash
ros2 launch ocam_publisher ocam.launch.py

ros2 launch livox_ros_driver2 msg_MID360_launch.py

ros2 launch lio_sam run.launch.py

ros2 launch autorccar_planning_control planning_control.launch.py

ros2 launch autorccar_hardware_control hardware_control.launch.py 

ros2 launch autorccar_costmap costmap.launch.py

ros2 launch autorccar_gcs autorccar_gcs.launch.py
```
