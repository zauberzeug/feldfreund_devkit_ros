"""Launch file for the devkit project without cameras (driver, GNSS, UI, Foxglove Bridge)."""

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
# pylint: disable=wrong-import-position,duplicate-code

from launch import LaunchDescription
from launch_utils import foxglove_bridge, include_launch


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for the devkit system without cameras."""
    return LaunchDescription([
        foxglove_bridge(),
        include_launch('gnss.launch.py'),
        include_launch('devkit_driver.launch.py'),
        include_launch('ui.launch.py', package='devkit_ui'),
    ])
