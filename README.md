# autoRCcar_indoor

## Requirements

### LIO-SAM
```bash
DISTRO=humble #ros-version
sudo apt-get install ros-$DISTRO-perception-pcl ros-$DISTRO-pcl-msgs ros-$DISTRO-vision-opencv ros-$DISTRO-xacro
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt-get install libgtsam-dev libgtsam-unstable-dev
```

### oCam Publisher
```bash
sudo apt-get install libv4l-dev libudev-dev
```

## Launch
```bash
ros2 launch ocam ocam.launch.py

ros2 launch livox_ros_driver2 msg_MID360_launch.py

ros2 launch lio_sam run.launch.py
```
