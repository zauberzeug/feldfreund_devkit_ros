"""Main launch file for the devkit project."""

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
# pylint: disable=wrong-import-position,duplicate-code

from launch import LaunchDescription
from launch_utils import include_launch


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for the complete devkit system."""
    return LaunchDescription([
        include_launch('gnss.launch.py'),
        include_launch('devkit_driver.launch.py'),
        include_launch('camera_system.launch.py'),
        include_launch('ui.launch.py'),
    ])
