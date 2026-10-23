import rosys
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rosys import background_tasks
from rosys.hardware import Wheels

from .base import Handler

TIMEOUT_CHECK_INTERVAL = 0.05


class TwistHandler(Handler):
    """Relate ROS cmd_vel messages to RoSys wheel commands and stop the wheels when the commands stop arriving."""

    def __init__(self, node: Node, wheels: Wheels):
        super().__init__(node)
        self._wheels = wheels
        node.declare_parameter('cmd_vel_timeout', 0.5)
        self._timeout: float = node.get_parameter('cmd_vel_timeout').value
        self._moving_command_time: float | None = None
        self._cmd_subscription = node.create_subscription(Twist, 'cmd_vel', self.handle_twist, 10)
        rosys.on_repeat(self._stop_on_timeout, TIMEOUT_CHECK_INTERVAL)

    def handle_twist(self, cmd_msg: Twist) -> None:
        """Implement callback for cmd_vel message."""
        background_tasks.create(self._drive(cmd_msg.linear.x, cmd_msg.angular.z), name='twist: drive')

    async def _drive(self, linear: float, angular: float) -> None:
        self._moving_command_time = rosys.time() if linear or angular else None
        await self._wheels.drive(linear, angular)

    async def _stop_on_timeout(self) -> None:
        if self._moving_command_time is None or rosys.time() - self._moving_command_time <= self._timeout:
            return
        self.log.warning(f'No cmd_vel for {self._timeout} s, stopping the wheels')
        self._moving_command_time = None
        await self._wheels.stop()
