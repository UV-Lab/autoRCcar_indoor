#!/bin/bash

gnome-terminal --tab -- bash -c "ros2 launch livox_ros_driver2 msg_MID360_launch.py; exec bash"

gnome-terminal --tab -- bash -c "ros2 launch lio_sam run.launch.py; exec bash"

gnome-terminal --tab -- bash -c "ros2 launch autorccar_planning_control planning_control.launch.py; exec bash"

gnome-terminal --tab -- bash -c "ros2 launch autorccar_hardware_control hardware_control.launch.py; exec bash"

gnome-terminal --tab -- bash -c "ros2 launch autorccar_costmap costmap.launch.py; exec bash"
