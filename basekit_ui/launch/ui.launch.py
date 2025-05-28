from launch import LaunchDescription  # type: ignore
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='basekit_ui',
            executable='ui_node.py',
            name='ui_node',
            output='screen',
            prefix=['python3']
        )
    ])
