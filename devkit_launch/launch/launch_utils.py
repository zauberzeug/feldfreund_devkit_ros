"""Shared helpers for the devkit_launch launch files.

``devkit_launch`` is an ``ament_cmake`` package, so these helpers are not importable
as a regular Python module.
The launch files that use them prepend their own directory (where this file is
installed alongside them) to ``sys.path`` before importing, e.g.::

    import os
    import sys
    sys.path.insert(0, os.path.dirname(__file__))
    from launch_utils import include_launch  # noqa: E402
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

PACKAGE = 'devkit_launch'

# Default network ports
FOXGLOVE_BRIDGE_PORT = 8765


def config_path(file_name: str, package: str = PACKAGE) -> str:
    """Return the absolute path to a file in a package's ``config`` directory."""
    return os.path.join(get_package_share_directory(package), 'config', file_name)


def include_launch(launch_file: str, package: str = PACKAGE) -> IncludeLaunchDescription:
    """Include another launch file from a package's ``launch`` directory."""
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path))


def foxglove_bridge(port: int = FOXGLOVE_BRIDGE_PORT) -> Node:
    """Create the Foxglove Bridge node for remote visualization."""
    return Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': port,
            'address': '0.0.0.0',  # NOTE: allow connections from the operator's machine on the robot network
            'tls': False,
            'compression': 'jpeg',  # compress images for remote viewing
            'jpeg_quality': 75,
            'max_qos_depth': 10,
        }],
    )
