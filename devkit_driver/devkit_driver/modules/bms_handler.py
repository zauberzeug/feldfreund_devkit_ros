""" Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
    Modified by Zauberzeug GmbH
"""

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from devkit_driver.communication.communication import Communication
from devkit_driver.data.data_bms import DataBMS


class BMSHandler:
    """Handle the odometry."""

    def __init__(
            self,
            node: Node,
            comm: Communication):

        self._logger = node.get_logger()
        self._comm = comm
        self._clock = node.get_clock()
        self.current_pose = PoseStamped()
        self._node = node
        self._publisher = node.create_publisher(BatteryState, 'battery_state', 10)

        comm.register_bms_observer(self)
        self._send_request_timer = node.create_timer(1, self.send_request)

    def update(self, data: dict) -> None:
        """Read the data from a list of words."""
        try:
            data_bms = DataBMS([int(w, 16) for w in data])
            data_bms.check()
        except AssertionError:
            self._logger.error('Cannot read data!')
            return
        msg = data_bms.get_ros_message()
        msg.header.stamp = self._clock.now().to_msg()
        self._publisher.publish(msg)

    def send_request(self):
        """Send twist to serial."""
        self._comm.send('bms.send(0xdd, 0xa5, 0x03, 0x00, 0xff, 0xfd, 0x77)')
