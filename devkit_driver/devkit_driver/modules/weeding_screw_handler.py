import asyncio
from collections.abc import Coroutine
from typing import Any

import rosys
from rclpy.node import Node
from rosys import background_tasks
from std_msgs.msg import Bool, Empty, Float64

from ..qos import SAFETY_QOS
from ..weeding_screw import WeedingScrew
from .base import Handler


class WeedingScrewHandler(Handler):
    """Handle the weeding screw implement: home/clear-view/stop/punch commands and axis status.

    Manual control only, matching feldfreund_devkit_ros' current scope: no camera-based weed
    targeting or FieldNavigation-driven autonomous weeding here (see the concept doc).
    """

    def __init__(self, node: Node, weeding_screw: WeedingScrew):
        super().__init__(node)
        self._weeding_screw = weeding_screw
        self._active_task: asyncio.Task | None = None
        """The currently running home/clear_view/punch task, if any - so stop() can cancel it.

        Without this, stop() only disables the motors for one cycle: the still-running move_to()
        loop sees ctrl_enable go false, re-enables the motor itself (it wants to reach its target)
        and continues - stop briefly pauses the axis instead of aborting the move.
        """

        node.create_subscription(Empty, 'weeding_screw/home', self._handle_home, 10)
        node.create_subscription(Empty, 'weeding_screw/clear_view', self._handle_clear_view, 10)
        node.create_subscription(Empty, 'weeding_screw/stop', self._handle_stop, 10)
        node.create_subscription(Float64, 'weeding_screw/punch', self._handle_punch, 10)
        node.create_subscription(Float64, 'weeding_screw/set_drill_depth', self._handle_set_drill_depth, 10)

        self._y_position_pub = node.create_publisher(Float64, 'weeding_screw/y_position', 10)
        self._z_position_pub = node.create_publisher(Float64, 'weeding_screw/z_position', 10)
        self._is_referenced_pub = node.create_publisher(Bool, 'weeding_screw/is_referenced', SAFETY_QOS)
        self._alarm_pub = node.create_publisher(Bool, 'weeding_screw/alarm', SAFETY_QOS)
        self._drill_depth_pub = node.create_publisher(Float64, 'weeding_screw/drill_depth', 10)
        rosys.on_repeat(self._publish_state, 0.2)

    async def _run_and_log(self, coro: Coroutine[Any, Any, Any], label: str) -> None:
        await coro
        self.log.info(f'{label}: done')

    def _run_command(self, coro: Coroutine[Any, Any, Any], label: str, name: str) -> None:
        if self._active_task is not None and not self._active_task.done():
            self.log.warning(f'{label}: another weeding screw command is still running, ignoring')
            return
        self._active_task = background_tasks.create(self._run_and_log(coro, label), name=name)

    def _handle_home(self, _: Empty) -> None:
        self._run_command(self._weeding_screw.try_home(), 'Homing weeding screw', 'weeding_screw: home')

    def _handle_clear_view(self, _: Empty) -> None:
        self._run_command(self._weeding_screw.clear_view(), 'Clearing view', 'weeding_screw: clear_view')

    def _handle_stop(self, _: Empty) -> None:
        if self._active_task is not None and not self._active_task.done():
            self._active_task.cancel()
        background_tasks.create(self._run_and_log(self._weeding_screw.stop(), 'Stopping weeding screw'),
                                name='weeding_screw: stop')

    def _handle_punch(self, msg: Float64) -> None:
        self._run_command(self._weeding_screw.punch(msg.data), f'Punching at y={msg.data:.3f}', 'weeding_screw: punch')

    def _handle_set_drill_depth(self, msg: Float64) -> None:
        self._weeding_screw.drill_depth = msg.data
        self.log.info(f'Drill depth set to {msg.data:.3f} m')

    def _publish_state(self) -> None:
        if not self.active:
            return
        y_axis = self._weeding_screw.y_axis
        z_axis = self._weeding_screw.z_axis
        self._y_position_pub.publish(Float64(data=y_axis.position))
        self._z_position_pub.publish(Float64(data=z_axis.position))
        self._is_referenced_pub.publish(Bool(data=y_axis.is_referenced and z_axis.is_referenced))
        self._alarm_pub.publish(Bool(data=y_axis.alarm or z_axis.alarm))
        self._drill_depth_pub.publish(Float64(data=self._weeding_screw.drill_depth))
