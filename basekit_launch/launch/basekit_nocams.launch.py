"""Main launch file for the basekit project."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Generate launch description for the complete basekit system."""
    pkg_dir = get_package_share_directory('basekit_launch')
    ui_pkg_dir = get_package_share_directory('basekit_ui')

    # Include the GNSS launch file
    gnss_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'gnss.launch.py')
        )
    )

    # Include the basekit driver launch file
    basekit_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'basekit_driver.launch.py')
        )
    )

    # Include the UI launch file
    ui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ui_pkg_dir, 'launch', 'ui.launch.py')
        )
    )

    return LaunchDescription([
        gnss_launch,
        basekit_driver_launch,
        ui_launch
    ])
