"""Launch file for the devkit driver node."""

import os

import ament_index_python.packages
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for devkit driver."""
    config_file = LaunchConfiguration('config_file')
    simulation = LaunchConfiguration('simulation')

    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory(
            'devkit_launch'),
        'config')

    config_file_launch_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(config_directory, 'devkit.yaml')
    )

    simulation_launch_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Run in simulation mode (true/false)'
    )

    # Set environment variable for simulation mode
    set_simulation_env = SetEnvironmentVariable(
        'FELDFREUND_SIMULATION',
        simulation
    )

    # DevKit driver node
    devkit_driver_node = Node(
        package='devkit_driver',
        executable='devkit_driver_node',
        name='controller',
        parameters=[
            config_file,
            {'use_sim_time': simulation}  # Enable ROS2 simulation time
        ],
        respawn=True,
        respawn_delay=5,
    )

    return LaunchDescription([
        config_file_launch_arg,
        simulation_launch_arg,
        set_simulation_env,
        devkit_driver_node,
    ])
