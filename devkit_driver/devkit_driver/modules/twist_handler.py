from geometry_msgs.msg import Twist
from rclpy.node import Node
from rosys.hardware import Wheels


class TwistHandler:
    """Handle the odometry."""

    def __init__(self, node: Node, wheels: Wheels):
        self.log = node.get_logger()
        self.wheels = wheels
        self.cmd_subscription = node.create_subscription(Twist, 'cmd_vel', self.handle_twist, 10)

    def handle_twist(self, cmd_msg: Twist):
        """Implement callback for cmd_vel message."""
        self.wheels.drive(cmd_msg.linear.x, cmd_msg.angular.z)
