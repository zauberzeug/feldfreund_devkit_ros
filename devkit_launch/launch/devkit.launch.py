"""Main launch file for the devkit project."""
# pylint: disable=duplicate-code

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Generate launch description for the complete devkit system."""
    pkg_dir = get_package_share_directory('devkit_launch')
    ui_pkg_dir = get_package_share_directory('devkit_ui')

    # Include the GNSS launch file
    gnss_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'gnss.launch.py')
        )
    )

    # Include the devkit driver launch file
    devkit_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'devkit_driver.launch.py')
        )
    )

    # Include the camera system launch file
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'camera_system.launch.py')
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
        devkit_driver_launch,
        camera_launch,
        ui_launch
    ])
