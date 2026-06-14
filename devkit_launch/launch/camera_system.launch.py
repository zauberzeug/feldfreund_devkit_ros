"""Launch file for the complete camera system (AXIS and USB cameras)."""

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
# pylint: disable=wrong-import-position

from launch import LaunchDescription
from launch_utils import include_launch


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for the complete camera system."""
    return LaunchDescription([
        include_launch('axis_cameras.launch.py'),
        include_launch('usb_camera.launch.py'),
    ])
