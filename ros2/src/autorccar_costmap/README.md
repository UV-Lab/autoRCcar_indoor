# autoRCcar_costmap

## Requirements
```bash
apt install libeigen3-dev libyaml-cpp-dev
apt install ros-${ROS_DISTRO}-vision-msgs
```
## Topics
Subscription
- lidar: ```livox/lidar```, <livox_ros_driver2::msg::CustomMsg> 
- pose: ```/nav_topic```, <autorccar_interfaces::msg::NavState>

Publisher
- global costmap: ```occupancy_grid```, <nav_msgs::msg::OccupancyGrid>
- local costmap: ```occupancy_grid/local```, <nav_msgs::msg::OccupancyGrid>
- bounding boxes: ```bounding_boxes```, <autorccar_interfaces::msg::BoundingBoxes>  

## Config 
Set config.yaml before launch.  
``` 
global: 
    publish: publish ros2 topic
    width: map width [m]  
    height: map height [m]  
    resolution: resolution [m/cell]  
    update_every_Nth_lidar: update costmap every Nth lidar input  

local:
    publish: publish ros2 topic        
    width: map width [m]  
    height: map height [m]  

object_detection:
    publish: publish ros2 topic
    visualize: show marker on rviz
    dbscan:           
        eps: maximum distance between two points to be considered as in the same cluster  
        min_samples: minimum number of points to form a cluster  
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

