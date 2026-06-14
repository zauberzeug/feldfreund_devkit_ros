"""Launch file for the USB camera."""

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
# pylint: disable=wrong-import-position

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_utils import config_path


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for USB camera."""
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[config_path('camera.yaml')],
            remappings=[
                ('image_raw', '/devkit/camera/image_raw'),
                ('camera_info', '/devkit/camera/camera_info'),
            ]
        ),
    ])
