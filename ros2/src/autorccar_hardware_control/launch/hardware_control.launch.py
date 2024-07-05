"""Hardware Control launcher for Auto RC Car."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    hardware_control_config_file_path = os.path.join(
        get_package_share_directory("autorccar_hardware_control"),
        "launch",
        "hardware_control.yaml",
    )
    hardware_control_node = Node(
        package="autorccar_hardware_control",
        executable="hardware_control_node",
        name="hardware_control",
        parameters=[hardware_control_config_file_path],
        output="screen",
    )
    ld = LaunchDescription()
    ld.add_action(hardware_control_node)

    return ld
