"""Launch file for the AXIS cameras.

The DevKit uses a single multi-channel AXIS encoder: one hostname serves several
video channels, so one ``axis_camera`` node is launched per channel.
Camera credentials are read from ``config/secrets.yaml`` (see ``secrets.yaml.template``);
the launch fails fast if they are missing instead of falling back to insecure defaults.
"""

import os
import sys

import yaml

sys.path.insert(0, os.path.dirname(__file__))
# pylint: disable=wrong-import-position

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_utils import config_path

# Single multi-channel AXIS encoder: one hostname, one node per video channel.
AXIS_HOSTNAME = '192.168.42.3'
AXIS_HTTP_PORT = 80
NUM_CHANNELS = 5


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for the AXIS cameras."""
    username, password = _load_credentials()
    nodes = [_axis_camera_node(channel, username, password) for channel in range(1, NUM_CHANNELS + 1)]
    return LaunchDescription(nodes)


def _load_credentials() -> tuple[str, str]:
    """Load the AXIS camera credentials from ``secrets.yaml`` or fail with a clear error."""
    secrets_file = config_path('secrets.yaml')
    try:
        with open(secrets_file, encoding='utf-8') as f:
            secrets = yaml.safe_load(f)
        return secrets['axis_cameras']['username'], secrets['axis_cameras']['password']
    except (FileNotFoundError, KeyError, TypeError) as error:
        raise RuntimeError(
            f'Could not read AXIS camera credentials from {secrets_file}: {error}. '
            'Copy config/secrets.yaml.template to config/secrets.yaml and fill in the credentials.'
        ) from error


def _axis_camera_node(channel: int, username: str, password: str) -> Node:
    """Create an ``axis_camera`` node for a single video channel of the encoder."""
    namespace = f'axis_camera{channel}'
    return Node(
        package='axis_camera',
        executable='axis_camera_node',
        namespace=namespace,
        name='camera',
        parameters=[
            config_path('axis_camera.yaml'),
            {
                'hostname': AXIS_HOSTNAME,
                'http_port': AXIS_HTTP_PORT,
                'username': username,
                'password': password,
                'use_encrypted_password': False,
                'frame_id': f'{namespace}/camera',
                'camera_info_url': '',
                'stream': 'mjpeg',
                'camera': channel,
                'height': 480,
                'width': 640,
                'fps': 20,
                'tf_prefix': f'camera{channel}',
                'ir': False,
                'wiper': False,
                'defog': False,
                'ptz': False,
                'ptz_teleop': False,
            }
        ],
    )
