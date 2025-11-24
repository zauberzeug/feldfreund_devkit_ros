"""Launch file for the complete camera system (AXIS and USB cameras with Foxglove Bridge)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the complete camera system."""
    pkg_dir = get_package_share_directory('devkit_launch')

    # Include the AXIS cameras launch file
    axis_cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'axis_cameras.launch.py')
        )
    )

    # Include the USB camera launch file
    usb_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'usb_camera.launch.py')
        )
    )

    # Foxglove Bridge Node
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',  # Allow external connections
            'tls': False,
            'compression': 'jpeg',  # Use JPEG compression for images
            'jpeg_quality': 75,
            'max_qos_depth': 10,
        }],
    )

    return LaunchDescription([
        axis_cameras_launch,
        usb_camera_launch,
        foxglove_bridge,
    ])
