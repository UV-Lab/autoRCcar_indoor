# autoRCcar_indoor

## Requirements
### oCam Publisher
```bash
sudo apt-get install libv4l-dev libudev-dev
```

## Launch
```bash
ros2 launch ocam ocam.launch.py

ros2 launch livox_ros_driver2 msg_MID360_launch.py

ros2 launch lio_sam run.launch.py

ros2 launch autorccar_planning_control planning_control.launch.py
- ros2 launch autorccar_planning_control simulator.launch.py

ros2 run autorccar_gcs autorccar_gcs
```
