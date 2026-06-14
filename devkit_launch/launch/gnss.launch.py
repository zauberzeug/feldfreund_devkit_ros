"""Launch file for the Septentrio GNSS receiver."""

import glob
import os

import launch
import launch.logging
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

SERIAL_BAUDRATE = 921600

# Static transforms for the receiver's antenna/sensor frames.
TF_FRAMES = [
    ('base_link', 'imu'),
    ('imu', 'gnss'),
    ('imu', 'vsm'),
    ('imu', 'aux1'),
]


def generate_launch_description() -> launch.LaunchDescription:
    """Generate launch description for the Septentrio GNSS receiver."""
    arg_file_name = DeclareLaunchArgument(
        'file_name',
        default_value=TextSubstitution(text='gnss.yaml')
    )
    arg_file_path = DeclareLaunchArgument(
        'path_to_config',
        default_value=[
            get_package_share_directory('devkit_launch'),
            '/config/',
            LaunchConfiguration('file_name')
        ]
    )

    serial_port = find_septentrio_port()
    config = {
        # NOTE: 'serial://' + path without the leading slash, otherwise the URI ends up with three slashes
        'device': 'serial://' + serial_port.lstrip('/'),
        'serial.baudrate': SERIAL_BAUDRATE,
        'serial.hw_flow_control': 'off'
    }

    composable_node = ComposableNode(
        name='septentrio_gnss_driver',
        package='septentrio_gnss_driver',
        plugin='rosaic_node::ROSaicNode',
        parameters=[
            LaunchConfiguration('path_to_config'),
            config
        ]
    )
    container = ComposableNodeContainer(
        name='septentrio_gnss_driver_container',
        namespace='septentrio_gnss_driver',
        package='rclcpp_components',
        executable='component_container_isolated',
        emulate_tty=True,
        sigterm_timeout='20',
        composable_node_descriptions=[composable_node],
        output='screen'
    )

    tf_publishers = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=f'0 0 0 0 0 0 {source} {target}'.split(' '),
            name=f'tf_{source}_{target}'
        )
        for source, target in TF_FRAMES
    ]

    return launch.LaunchDescription([
        arg_file_name,
        arg_file_path,
        container,
        *tf_publishers
    ])


def find_septentrio_port() -> str:
    """Locate the Septentrio receiver's serial device, preferring stable by-id symlinks."""
    logger = launch.logging.get_logger('gnss.launch')

    # Prefer the stable, name-based symlinks so we don't grab an unrelated serial device.
    for by_id in sorted(glob.glob('/dev/serial/by-id/*')):
        if 'septentrio' in os.path.basename(by_id).lower():
            resolved = os.path.realpath(by_id)
            logger.info(f'Found Septentrio receiver at {by_id} -> {resolved}')
            return resolved

    # Fall back to the first generic serial device, but make the guess explicit in the log.
    for pattern in ('/dev/ttyACM*', '/dev/ttyUSB*', '/dev/ttyS*'):
        ports = sorted(glob.glob(pattern))
        if ports:
            logger.warning(f'No Septentrio by-id symlink found; falling back to {ports[0]}')
            return ports[0]

    logger.error('No serial device found for the GNSS receiver; defaulting to /dev/ttyACM0')
    return '/dev/ttyACM0'
