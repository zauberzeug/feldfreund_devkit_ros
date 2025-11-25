""" Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
    Modified by Zauberzeug GmbH
"""


from rclpy.node import Node
from std_msgs.msg import Bool

from devkit_driver.communication.communication import Communication


class EStopHandler:
    """Handle the estop."""

    def __init__(self, node: Node, comm: Communication):
        self._node = node
        self._logger = node.get_logger()
        self._comm = comm
        self._software_estop: bool = False
        self.subscription = node.create_subscription(Bool, 'emergency_stop', self.callback, 10)
        # Add publishers for hardware estops
        self.estop1_publisher = node.create_publisher(Bool, 'estop1_state', 10)
        self.estop2_publisher = node.create_publisher(Bool, 'estop2_state', 10)
        comm.register_core_observer(self)

    def update(self, data: dict):
        """Update estop position from core data."""
        if 'estop1_active' in data:
            msg = Bool()
            msg.data = bool(data['estop1_active'])
            self.estop1_publisher.publish(msg)
        if 'estop2_active' in data:
            msg = Bool()
            msg.data = bool(data['estop2_active'])
            self.estop2_publisher.publish(msg)

    def send(self):
        """Send estop command."""
        return f"en3.level({'false' if self._software_estop else 'true'})"

    def callback(self, msg: Bool):
        """Implement a callback for the estop."""
        self._software_estop = msg.data
        self._comm.send(self.send())
