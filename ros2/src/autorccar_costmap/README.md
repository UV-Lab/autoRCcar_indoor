# autoRCcar_costmap

## Requirements
```bash
apt-get install libeigen3-dev libyaml-cpp-dev
```
## Topics
Subscription
- lidar: ```livox/lidar```, <livox_ros_driver2::msg::CustomMsg> 
- pose: ```/nav_topic```, <autorccar_interfaces::msg::NavState>

Publisher
- global costmap: ```occupancy_grid```, <nav_msgs::msg::OccupancyGrid>
- local costmap: ```occupancy_grid/local```, <nav_msgs::msg::OccupancyGrid>

## Config 
Set config.yaml before launch.  
``` 
global: 
    width: map width [m]  
    height: map height [m]  
    resolution: resolution [m/cell]  
    updateEveryNthLidar: update costmap every Nth lidar input  

local:
    width: map width [m]  
    height: map height [m]  
```  

*lidar frequency >= global costmap update frequency = global & local costmap publish frequency  
*local costmap is part of global costmap.

## Launch
### Launch autorccar_costmap 
```bash
ros2 launch autorccar_costmap costmap.launch.py
```
### Save occupancy grid map as *.pgm
```bash
ros2 topic pub /costmap/save std_msgs/msg/Bool "{data: True}" -1
```
### Visualize occupancy grid map with RViz
```bash
ros2 launch autorccar_costmap rviz2.launch.py  
```

