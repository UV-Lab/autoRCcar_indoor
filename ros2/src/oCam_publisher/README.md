# ROS2 wrapper of Withrobot oCam-1CGN-U-T
This wrapper is based on Withrobot C++ API for oCam-1CGN-U
http://withrobot.com/camera/ocam-1cgn-u/

# Dependencies
- libv4l       (video for linux Two)
- libudev       (udev, the device manager for the Linux kernel)
- libopencv-dev (development files for OpenCV)

```
sudo apt-get install libv4l-dev libudev-dev libopencv-dev
```

# Run
```
ros2 run ocam ocam_node
ros2 launch ocam ocam.launch.py
ros2 launch ocam ocam_rviz.launch.py
```

