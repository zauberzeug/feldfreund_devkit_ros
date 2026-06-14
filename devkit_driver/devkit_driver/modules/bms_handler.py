from rclpy.node import Node
from rclpy.time import Time
from rosys.hardware import Bms
from rosys.hardware.bms_state import BmsState
from sensor_msgs.msg import BatteryState

from .base import Handler


class BMSHandler(Handler):
    """Publish the battery management system state as a ROS BatteryState."""

    def __init__(self, node: Node, bms: Bms):
        super().__init__(node)
        self._publisher = node.create_publisher(BatteryState, 'battery_state', 10)
        self._bms = bms
        self._bms.STATE_UPDATED.subscribe(self._handle_bms_update)

    def _handle_bms_update(self, state: BmsState) -> None:
        """Handle BMS update event."""
        message = self._state_to_ros_message(state)
        if message is None:
            self.log.warning('BMS state has None values, skipping publish')
            return
        self._publisher.publish(message)

    def _state_to_ros_message(self, state: BmsState) -> BatteryState | None:
        """Convert a BMS state to a ROS BatteryState, or None if any field is missing."""
        voltage, current = state.voltage, state.current
        percentage, temperature = state.percentage, state.temperature
        if voltage is None or current is None or percentage is None or temperature is None:
            return None
        message = BatteryState()
        # Stamp with the time the reading was taken (RoSys time, seconds) rather than now.
        message.header.stamp = Time(nanoseconds=int(state.last_update * 1e9)).to_msg()
        message.voltage = float(voltage)
        message.current = float(current)
        message.percentage = float(percentage) / 100.0
        message.temperature = float(temperature)
        return message
