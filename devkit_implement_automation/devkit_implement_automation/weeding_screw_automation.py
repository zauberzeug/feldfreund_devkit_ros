"""Task-level automation for the weeding screw: home/punch/clear-view, built purely on standard
ROS/ros2_control interfaces.

Unlike devkit_driver's WeedingScrewHandler (which talks rosys/Lizard directly to the CANopen axes),
this node has no rosys or hardware-specific dependency at all: it only knows the standard
weeding_screw_position_controller command topic and the standard /joint_states feedback topic, both
provided by devkit_description's ros2_control setup (see devkit_launch/launch/devkit_control.launch.py).
That is deliberate: "punch" and "clear_view" are just a sequence of ordinary position commands, no
different in kind from a task-level node driving any other ros2_control-controlled robot (the same
relationship joint_trajectory_controller has to MoveIt, only much simpler here).

weeding_screw_position_controller is kept active at all times (never (de)activated between
sequences) - see _tick()'s idle-heartbeat branch for why: a ros2_control controller that goes
inactive still leaves whatever it last wrote latched in the hardware interface's raw command state,
so intermittently deactivating it does not actually clear anything, it just stops *new* commands
from overwriting that stale value. Continuously re-publishing "hold at the current position"
whenever no sequence is in flight keeps that raw state fresh instead.

Homing stays out of scope here on purpose: seeking an endstop at a raw velocity is not something a
position command interface can express, so it remains devkit_driver's WeedingScrewHandler's job
(the one thing that still has to talk to the axis hardware directly).
"""

import rclpy
from devkit_driver.qos import SAFETY_QOS
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray

Y_JOINT_NAME = 'weeding_screw_y_joint'
Z_JOINT_NAME = 'weeding_screw_z_joint'
"""Must match the joint names declared in devkit_description's weeding_screw.ros2_control.xacro."""

# Order of devkit_launch/config/weeding_screw_controllers.yaml's weeding_screw_position_controller
# `joints` list - the commands topic is a plain Float64MultiArray (no joint names), so this order
# has to be hardcoded on both ends.
_COMMAND_ORDER = (Y_JOINT_NAME, Z_JOINT_NAME)

_CONTROLLER_NAME = 'weeding_screw_position_controller'

Z_REFERENCE = 0.0
"""The Z-axis' parked/referenced position - always 0 by definition of being referenced."""


class WeedingScrewAutomation(Node):
    """Sequences weeding_screw_position_controller commands into punch/clear_view; home/stop stay
    devkit_driver's job (see module docstring)."""

    def __init__(self) -> None:
        super().__init__('weeding_screw_automation')

        self.declare_parameter('y_min', -0.12)
        self.declare_parameter('y_max', 0.12)
        self.declare_parameter('drill_depth', 0.14)
        self.declare_parameter('position_tolerance', 0.002)
        self.declare_parameter('step_timeout', 20.0)
        self._y_min = self.get_parameter('y_min').value
        self._y_max = self.get_parameter('y_max').value
        self._drill_depth = self.get_parameter('drill_depth').value
        self._position_tolerance = self.get_parameter('position_tolerance').value
        self._step_timeout = self.get_parameter('step_timeout').value

        self._current_position: dict[str, float] = {}
        self._is_referenced = False
        self._alarm = False

        self._label: str | None = None
        """Human-readable description of the running sequence, for logging - also doubles as the
        busy flag: a sequence is active whenever this is not None."""
        self._sequence: list[tuple[str, float]] = []
        """Remaining (joint_name, target_position) steps, not counting the one in flight."""
        self._pending_step: tuple[str, float] | None = None
        self._step_deadline: Time | None = None

        self._commands_pub = self.create_publisher(Float64MultiArray, f'{_CONTROLLER_NAME}/commands', 10)
        self._drill_depth_pub = self.create_publisher(Float64, 'weeding_screw/drill_depth', 10)

        self.create_subscription(JointState, 'joint_states', self._on_joint_states, 10)
        self.create_subscription(Bool, 'weeding_screw/is_referenced', self._on_is_referenced, SAFETY_QOS)
        self.create_subscription(Bool, 'weeding_screw/alarm', self._on_alarm, SAFETY_QOS)
        self.create_subscription(Float64, 'weeding_screw/punch', self._handle_punch, 10)
        self.create_subscription(Empty, 'weeding_screw/clear_view', self._handle_clear_view, 10)
        self.create_subscription(Empty, 'weeding_screw/stop', self._handle_stop, 10)
        self.create_subscription(Float64, 'weeding_screw/set_drill_depth', self._handle_set_drill_depth, 10)

        self.create_timer(0.1, self._tick)
        self._drill_depth_pub.publish(Float64(data=self._drill_depth))

    def _on_joint_states(self, msg: JointState) -> None:
        for joint_name, position in zip(msg.name, msg.position, strict=True):
            if joint_name in _COMMAND_ORDER:
                self._current_position[joint_name] = position

    def _on_is_referenced(self, msg: Bool) -> None:
        was_referenced = self._is_referenced
        self._is_referenced = msg.data
        if was_referenced and not msg.data and self._label is not None:
            # Losing the reference mid-sequence means any in-flight step's target is no longer
            # trustworthy - abort right away instead of waiting for the next _tick() to notice.
            self.get_logger().error(f'{self._label}: lost reference mid-sequence, aborting')
            self._abort_sequence()

    def _on_alarm(self, msg: Bool) -> None:
        self._alarm = msg.data
        if msg.data and self._label is not None:
            self.get_logger().error(f'{self._label}: alarm triggered mid-sequence, aborting')
            self._abort_sequence()

    def _handle_punch(self, msg: Float64) -> None:
        y = msg.data
        if not self._y_min <= y <= self._y_max:
            self.get_logger().error(f'punch: y={y:.3f} is out of range [{self._y_min}, {self._y_max}]')
            return
        if self._alarm:
            self.get_logger().error('punch: weeding screw is in alarm, aborting')
            return
        if not self._is_referenced:
            self.get_logger().error('punch: weeding screw is not referenced, home it first')
            return
        self._start_sequence(f'Punching at y={y:.3f}',
                             [(Y_JOINT_NAME, y), (Z_JOINT_NAME, -self._drill_depth), (Z_JOINT_NAME, Z_REFERENCE)])

    def _handle_clear_view(self, _: Empty) -> None:
        current_y = self._current_position.get(Y_JOINT_NAME)
        if current_y is None:
            self.get_logger().error('clear_view: no /joint_states received yet, aborting')
            return
        edge = (self._y_min + 0.002) if current_y <= 0 else (self._y_max - 0.002)
        self._start_sequence('Clearing view', [(Z_JOINT_NAME, Z_REFERENCE), (Y_JOINT_NAME, edge)])

    def _handle_stop(self, _: Empty) -> None:
        """Clear our own sequence bookkeeping; devkit_driver's WeedingScrewHandler is the one that
        actually halts the motors on the same topic."""
        if self._label is not None:
            self.get_logger().info(f'{self._label}: aborted by stop')
        self._abort_sequence()

    def _handle_set_drill_depth(self, msg: Float64) -> None:
        self._drill_depth = msg.data
        self._drill_depth_pub.publish(Float64(data=self._drill_depth))
        self.get_logger().info(f'Drill depth set to {msg.data:.3f} m')

    def _start_sequence(self, label: str, steps: list[tuple[str, float]]) -> None:
        if self._label is not None:
            self.get_logger().warning(f'{label}: another weeding screw command is still running, ignoring')
            return
        self._label = label
        self._sequence = steps
        self._advance()

    def _advance(self) -> None:
        if not self._sequence:
            self.get_logger().info(f'{self._label}: done')
            self._label = None
            self._pending_step = None
            return
        self._pending_step = self._sequence.pop(0)
        self._step_deadline = self.get_clock().now() + Duration(seconds=self._step_timeout)
        self._publish_step_target()

    def _publish_step_target(self) -> None:
        assert self._pending_step is not None
        joint_name, target = self._pending_step
        targets = {**self._current_position, joint_name: target}
        if any(targets.get(name) is None for name in _COMMAND_ORDER):
            self.get_logger().error(f'{self._label}: no /joint_states received yet, aborting')
            self._abort_sequence()
            return
        self._commands_pub.publish(Float64MultiArray(data=[targets[name] for name in _COMMAND_ORDER]))

    def _tick(self) -> None:
        if self._pending_step is None:
            # Idle - keep re-publishing the current position as a heartbeat so
            # weeding_screw_position_controller (which stays active throughout, see module
            # docstring) never lets a stale target sit latched in the hardware interface.
            self._publish_hold_command()
            return
        if not self._is_referenced or self._alarm:
            # Defense in depth alongside the reactive abort in _on_is_referenced()/_on_alarm() -
            # catches it even if a callback ordering/QoS edge case ever let one slip through.
            self.get_logger().error(f'{self._label}: reference lost or alarm active, aborting')
            self._abort_sequence()
            return
        joint_name, target = self._pending_step
        current = self._current_position.get(joint_name)
        if current is not None and abs(current - target) <= self._position_tolerance:
            self._advance()
            return
        if self.get_clock().now() >= self._step_deadline:
            self.get_logger().error(f'{self._label}: {joint_name} did not reach {target:.3f} m in time, aborting')
            self._abort_sequence()

    def _abort_sequence(self) -> None:
        """Clear the in-flight sequence and immediately publish a hold-at-current-position
        command, instead of waiting for the next idle _tick() to do it - weeding_screw_position_controller
        stays active throughout (see module docstring), so this is the only thing needed."""
        self._sequence = []
        self._pending_step = None
        self._step_deadline = None
        self._label = None
        self._publish_hold_command()

    def _publish_hold_command(self) -> None:
        """Publish each joint's current actual position as its target - called both reactively (on
        abort) and every idle _tick(). The repeated idle call is what actually matters: it keeps
        weeding_screw_position_controller's commanded value (and in turn the hardware interface's
        latched raw command) continuously fresh, instead of leaving whatever the last real sequence
        step commanded sitting there indefinitely - a ros2_control controller with nothing new to
        write does not clear what it last wrote, it just stops overwriting it (see module
        docstring). Skips silently if /joint_states hasn't been received yet for a joint; nothing
        trustworthy to hold at in that case.
        """
        if any(name not in self._current_position for name in _COMMAND_ORDER):
            return
        self._commands_pub.publish(Float64MultiArray(
            data=[self._current_position[name] for name in _COMMAND_ORDER]))


def main() -> None:
    rclpy.init()
    node = WeedingScrewAutomation()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
