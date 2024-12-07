"""GCS launcher for Auto RC Car."""

import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    planning_control_node = Node(
        package="autorccar_gcs",
        executable="autorccar_gcs",
        name="autorccar_gcs",
        parameters=[],
        output="screen",
    )
    ld = LaunchDescription()
    ld.add_action(planning_control_node)

    return ld
