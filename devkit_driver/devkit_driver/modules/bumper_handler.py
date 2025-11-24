""" Copyright (c) 2025 Zauberzeug GmbH
"""

from rclpy.node import Node
from std_msgs.msg import Bool

from devkit_driver.communication.communication import Communication


class BumperHandler:
    """Handle the bumper states from core data."""

    def __init__(self, node: Node, comm: Communication):
        self._node = node
        self._logger = node.get_logger()
        self._comm = comm
        self._pub_front_top = node.create_publisher(Bool, 'bumper_front_top_state', 10)
        self._pub_front_bottom = node.create_publisher(Bool, 'bumper_front_bottom_state', 10)
        self._pub_back = node.create_publisher(Bool, 'bumper_back_state', 10)
        comm.register_core_observer(self)

    def update(self, data: dict):
        """Update bumper states from core data and publish."""
        if 'bumper_front_top_active' in data:
            msg = Bool()
            msg.data = bool(data['bumper_front_top_active'])
            self._pub_front_top.publish(msg)
        if 'bumper_front_bottom_active' in data:
            msg = Bool()
            msg.data = bool(data['bumper_front_bottom_active'])
            self._pub_front_bottom.publish(msg)
        if 'bumper_back_active' in data:
            msg = Bool()
            msg.data = bool(data['bumper_back_active'])
            self._pub_back.publish(msg)
