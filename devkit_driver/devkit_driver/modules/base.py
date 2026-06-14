"""Base class for the devkit driver handlers."""

from rclpy.node import Node


class Handler:
    """Common base for handlers that bridge RoSys hardware modules to ROS2 topics.

    Owns the references shared by every handler: the ROS node and its logger.
    """

    def __init__(self, node: Node) -> None:
        self.node = node
        self.log = node.get_logger()
