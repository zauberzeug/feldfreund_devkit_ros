import asyncio
import math
from collections.abc import Coroutine
from typing import Any

import rosys
from rclpy.node import Node
from rosys import background_tasks
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float64

from ..qos import SAFETY_QOS
from ..weeding_screw import Axis, WeedingScrew
from .base import Handler

Y_JOINT_NAME = 'weeding_screw_y_joint'
Z_JOINT_NAME = 'weeding_screw_z_joint'
"""Must match the joint names declared in devkit_description's weeding_screw.ros2_control.xacro."""


class WeedingScrewHandler(Handler):
    """Bridge the weeding screw's Y-/Z-axis hardware (rosys/Lizard) to ROS.

    Deliberately narrow: home (raw velocity, seeks an endstop - no ros2_control command interface
    can express that), stop (the actual motor halt), and axis status/position. Everything that is
    just a sequence of ordinary position moves - punch, clear_view - lives in
    devkit_implement_automation's WeedingScrewAutomation instead, built purely on the standard
    weeding_screw_position_controller/joint_states interface below, with no rosys/Lizard dependency
    of its own. See that node's module docstring for the full reasoning.

    The joint_commands_topic/joint_states_topic subscription/publisher are the Python-side
    counterpart of the joint_state_topic_hardware_interface plugin declared in devkit_description's
    URDF (devkit_launch/launch/devkit_control.launch.py): they are how any ros2_control controller -
    including WeedingScrewAutomation's - actually reaches this axis hardware. Both that path and
    home/stop above end up calling the same guarded _run_command()/Axis.move_to().
    """

    def __init__(self, node: Node, weeding_screw: WeedingScrew):
        super().__init__(node)
        self._weeding_screw = weeding_screw
        self._active_tasks: dict[str, asyncio.Task] = {}
        """Currently running move task per axis (keyed by joint name), so stop() can cancel them
        and a repeated command to the *same* axis gets rejected while one is already in flight.

        Keyed per axis - not a single shared task - because Y and Z are independent motors that
        legitimately move at the same time: a ros2_control command topic update always carries
        both joints' targets together (the axis not being actively driven is just held at its
        current position), so a single shared guard would let the first joint in the message
        permanently starve the second one every cycle. Homing occupies both keys at once (with the
        same task) since it moves both axes together and must not race with an independent move of
        either.

        Without this, stop() only disables the motors for one cycle: the still-running move_to()
        loop sees ctrl_enable go false, re-enables the motor itself (it wants to reach its target)
        and continues - stop briefly pauses the axis instead of aborting the move.
        """

        node.create_subscription(Empty, 'weeding_screw/home', self._handle_home, 10)
        node.create_subscription(Empty, 'weeding_screw/stop', self._handle_stop, 10)

        self._y_position_pub = node.create_publisher(Float64, 'weeding_screw/y_position', 10)
        self._z_position_pub = node.create_publisher(Float64, 'weeding_screw/z_position', 10)
        self._is_referenced_pub = node.create_publisher(Bool, 'weeding_screw/is_referenced', SAFETY_QOS)
        self._alarm_pub = node.create_publisher(Bool, 'weeding_screw/alarm', SAFETY_QOS)

        # ros2_control bridge: same two topic names as the URDF's joint_commands_topic/
        # joint_states_topic parameters (devkit_description/urdf/weeding_screw.ros2_control.xacro).
        node.create_subscription(JointState, 'weeding_screw/joint_commands', self._handle_joint_command, 10)
        self._joint_state_pub = node.create_publisher(JointState, 'weeding_screw/joint_states', 10)

        rosys.on_repeat(self._publish_state, 0.2)

    async def _run_and_log(self, coro: Coroutine[Any, Any, Any], label: str) -> None:
        await coro
        self.log.info(f'{label}: done')

    def _is_busy(self, keys: tuple[str, ...]) -> bool:
        return any(key in self._active_tasks and not self._active_tasks[key].done() for key in keys)

    def _run_command(self, coro: Coroutine[Any, Any, Any], label: str, name: str, keys: tuple[str, ...]) -> None:
        task = background_tasks.create(self._run_and_log(coro, label), name=name)
        for key in keys:
            self._active_tasks[key] = task

    def _handle_home(self, _: Empty) -> None:
        keys = (Y_JOINT_NAME, Z_JOINT_NAME)
        if self._is_busy(keys):
            self.log.warning('Homing weeding screw: another weeding screw command is still running, ignoring')
            return
        self._run_command(self._weeding_screw.try_home(), 'Homing weeding screw', 'weeding_screw: home', keys=keys)

    def _handle_stop(self, _: Empty) -> None:
        for task in {task for task in self._active_tasks.values() if not task.done()}:
            task.cancel()
        background_tasks.create(self._run_and_log(self._weeding_screw.stop(), 'Stopping weeding screw'),
                                name='weeding_screw: stop')

    def _axis_by_joint_name(self, joint_name: str) -> Axis | None:
        if joint_name == Y_JOINT_NAME:
            return self._weeding_screw.y_axis
        if joint_name == Z_JOINT_NAME:
            return self._weeding_screw.z_axis
        return None

    def _handle_joint_command(self, msg: JointState) -> None:
        """Move each commanded axis directly to a position commanded via ros2_control (e.g. the
        weeding_screw_position_controller, driven in turn by WeedingScrewAutomation).

        Y and Z move independently: a message commanding both joints at once (the normal case -
        see the class docstring) runs both moves concurrently. A repeated command to the *same*
        axis while it is still moving is ignored (logged as busy) until that one finishes.
        """
        for joint_name, position in zip(msg.name, msg.position, strict=True):
            if math.isnan(position):
                # ForwardCommandController writes NaN every cycle until a real command has been
                # sent at least once - not an actual command, ignore silently instead of letting
                # it hit move_to()'s range/reference checks and spam a traceback five times a second.
                continue
            axis = self._axis_by_joint_name(joint_name)
            if axis is None:
                continue
            label = f'Moving {joint_name} to {position:.3f}'
            if self._is_busy((joint_name,)):
                self.log.warning(f'{label}: another weeding screw command is still running, ignoring')
                continue
            self._run_command(axis.move_to(position), label, f'weeding_screw: move {joint_name}', keys=(joint_name,))

    def _publish_state(self) -> None:
        if not self.active:
            return
        y_axis = self._weeding_screw.y_axis
        z_axis = self._weeding_screw.z_axis
        self._y_position_pub.publish(Float64(data=y_axis.position))
        self._z_position_pub.publish(Float64(data=z_axis.position))
        self._is_referenced_pub.publish(Bool(data=y_axis.is_referenced and z_axis.is_referenced))
        self._alarm_pub.publish(Bool(data=y_axis.alarm or z_axis.alarm))
        self._joint_state_pub.publish(JointState(
            name=[Y_JOINT_NAME, Z_JOINT_NAME],
            position=[y_axis.position, z_axis.position],
        ))
