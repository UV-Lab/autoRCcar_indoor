import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    share_dir = get_package_share_directory('autorccar_costmap')
    config_file = os.path.join(share_dir, 'config', 'config.yaml')

    return LaunchDescription([
        Node(
            package='autorccar_costmap',
            executable='costmap',
            arguments=[config_file],
            output='screen'
        )
    ])
