"""Simulator launcher for Auto RC Car."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    planning_control_config_file_path = os.path.join(
        get_package_share_directory("autorccar_planning_control"),
        "launch",
        "planning_control.yaml",
    )
    planning_control_node = Node(
        package="autorccar_planning_control",
        executable="simulator_node",
        name="simulator",
        parameters=[planning_control_config_file_path],
        output="screen",
    )
    ld = LaunchDescription()
    ld.add_action(planning_control_node)

    return ld
