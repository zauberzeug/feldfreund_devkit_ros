""" Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
"""

from geometry_msgs.msg import Twist
from rclpy.node import Node

from basekit_driver.communication.communication import Communication


class TwistHandler:
    """Handle the odometry."""

    def __init__(
            self,
            node: Node, comm: Communication):

        self._node = node
        self._comm = comm
        self._logger = node.get_logger()

        self._twist = Twist()

        self.cmd_subscription = node.create_subscription(
            Twist, 'cmd_vel', self.cmd_callback, 10
        )

        self.send_twist_timer = node.create_timer(0.05, self.send_twist)

    def send(self) -> str:
        """Send message to serial port."""
        return f'wheels.speed({self._twist.linear.x:3f}, {self._twist.angular.z:.3f})'

    def update(self, data: Twist) -> None:
        """Read the data from a list of words."""
        # self._logger.info('Data: ' + str(data))
        self._twist = data

    def cmd_callback(self, cmd_msg: Twist):
        """Implement callback for cmd_vel message."""
        self.update(cmd_msg)

    def send_twist(self):
        """Send twist message to serial device."""
        self._comm.send(self.send())
