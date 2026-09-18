"""Launch file for the ros2_control stack (robot_state_publisher, controller_manager, controllers).

Experimental: not part of the default devkit.launch.py/devkit_nocams.launch.py entrypoint yet, runs
as its own compose service instead (see docker/docker-compose.yml's `devkit-control`) so the
existing hardware stack is unaffected while this is evaluated - same pattern already used for the
navigation/twist_mux prototype.

Also starts devkit_implement_automation's WeedingScrewAutomation, the task-level node that turns
weeding_screw/punch and weeding_screw/clear_view into sequences of ordinary
weeding_screw_position_controller commands (see that node's module docstring).

Structure follows ros2_control_demos (checked against both its jazzy and rolling branches):
robot_description is only given to robot_state_publisher, not to ros2_control_node (it picks it up
from the /robot_description topic); each controller's `type` is declared in its own config file
section and loaded per-controller via `spawner --param-file`, not pre-declared on ros2_control_node.

The five axis-range/velocity launch arguments default to generic placeholders, not any real
robot's actual values - a robot-specific launch invocation overrides them, e.g.:
    ros2 launch devkit_launch devkit_control.launch.py y_min:=-0.1 y_max:=0.1 z_min:=-0.14 z_max:=0.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the ros2_control stack."""
    controllers_yaml = os.path.join(
        get_package_share_directory('devkit_launch'), 'config', 'weeding_screw_controllers.yaml')

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([get_package_share_directory('devkit_description'), 'urdf', 'devkit.urdf.xacro']),
        ' y_min:=', LaunchConfiguration('y_min'),
        ' y_max:=', LaunchConfiguration('y_max'),
        ' z_min:=', LaunchConfiguration('z_min'),
        ' z_max:=', LaunchConfiguration('z_max'),
        ' max_velocity:=', LaunchConfiguration('max_velocity'),
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    return LaunchDescription([
        DeclareLaunchArgument('y_min', default_value='-0.12',
                              description='Weeding screw Y-axis lower limit (m) - a generic '
                                          "placeholder; override with the robot's real config"),
        DeclareLaunchArgument('y_max', default_value='0.12',
                              description='Weeding screw Y-axis upper limit (m)'),
        DeclareLaunchArgument('z_min', default_value='-0.2',
                              description='Weeding screw Z-axis lower limit (m) - a generic '
                                          "placeholder; override with the robot's real config"),
        DeclareLaunchArgument('z_max', default_value='0.0',
                              description='Weeding screw Z-axis upper limit (m)'),
        DeclareLaunchArgument('max_velocity', default_value='0.2',
                              description='Weeding screw axis velocity limit (m/s), URDF-only - '
                                          'the actual motion is still bounded by the axis config '
                                          'used by devkit_driver (max_speed/reference_speed)'),
        DeclareLaunchArgument('drill_depth', default_value='0.14',
                              description='Weeding screw punch depth (m) - the depth WeedingScrewAutomation '
                                          "drives the Z-axis to; overridable at runtime via the "
                                          'weeding_screw/set_drill_depth topic'),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            parameters=[{'update_rate': 5}],
            # 5 Hz matches the existing 0.2 s state-publish cadence of WeedingScrewHandler
            # (devkit_driver) - the ros2_control loop can't usefully run faster than the bridged
            # Python driver updates anyway.
            output='both',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description],
            output='both',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            name='joint_state_broadcaster_spawner',
            arguments=['joint_state_broadcaster', '--param-file', controllers_yaml],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            name='weeding_screw_position_controller_spawner',
            arguments=['weeding_screw_position_controller', '--param-file', controllers_yaml],
        ),
        Node(
            package='devkit_implement_automation',
            executable='weeding_screw_automation_node',
            name='weeding_screw_automation',
            parameters=[{
                # LaunchConfiguration substitutions resolve to strings, so each is cast explicitly -
                # the node declares these parameters with float defaults and would otherwise reject
                # a plain string at declare_parameter() time.
                'y_min': ParameterValue(LaunchConfiguration('y_min'), value_type=float),
                'y_max': ParameterValue(LaunchConfiguration('y_max'), value_type=float),
                'drill_depth': ParameterValue(LaunchConfiguration('drill_depth'), value_type=float),
            }],
        ),
    ])
