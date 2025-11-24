"""Launch file for the devkit driver node."""

import os

import ament_index_python.packages
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for devkit driver."""
    config_file = LaunchConfiguration('config_file')
    startup_file = LaunchConfiguration('startup_file')

    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory(
            'devkit_launch'),
        'config')

    config_file_launch_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(config_directory, 'devkit.yaml')
    )

    startup_file_launch_arg = DeclareLaunchArgument(
        'startup_file',
        default_value=os.path.join(config_directory, 'devkit.liz'),
        description='Path to the Lizard startup file (devkit.liz)'
    )

    return LaunchDescription([
        config_file_launch_arg,
        startup_file_launch_arg,
        Node(
            package='devkit_driver',
            executable='devkit_driver_node',
            name='controller',
            parameters=[
                config_file,
                {
                    'startup_file': startup_file,
                }
            ],
            respawn=True,
            respawn_delay=5,
        )
    ])
