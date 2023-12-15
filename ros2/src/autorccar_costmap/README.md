# autoRccar_costmap

## Requirements
```bash
apt-get install libeigen3-dev libyaml-cpp-dev
```

## Config 
Set config.yaml before launch   
- width: map width [m]  
- height: map height [m]  
- resolution: resolution [m/cell]  

## Launch
### Launch autorccar_costmap 
```bash
ros2 launch autorccar_costmap costmap.launch.py
```
### Publish occupancy grid map 
Topic name : ```occupancy_grid```   
```bash
ros2 topic pub /costmap/publish std_msgs/msg/Bool "{data: True}" -1
```
### Save occupancy grid map as *.pgm
```bash
ros2 topic pub /costmap/save std_msgs/msg/Bool "{data: True}" -1
```
### Visualize occupancy grid map with RViz
```bash
ros2 launch autorccar_costmap rviz2.launch.py  
```

