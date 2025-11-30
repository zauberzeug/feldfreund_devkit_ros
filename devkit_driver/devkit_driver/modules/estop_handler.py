import rosys
from rclpy.node import Node
from rosys import background_tasks
from rosys.hardware import EStop
from std_msgs.msg import Bool


class EStopHandler:
    """Handle the estop."""

    def __init__(self, node: Node, estop: EStop):
        self.log = node.get_logger()
        self._estop = estop

        self.subscription = node.create_subscription(Bool, 'estop/soft', self.soft_estop_callback, 10)
        self.estop_front_publisher = node.create_publisher(Bool, 'estop/front', 10)
        self.estop_back_publisher = node.create_publisher(Bool, 'estop/back', 10)

        self._estop.ESTOP_TRIGGERED.subscribe(self._handle_estop_triggered)
        self._estop.ESTOP_RELEASED.subscribe(self._handle_estop_released)
        rosys.on_startup(self._check_on_startup)

    def _check_on_startup(self) -> None:
        self._handle_estop_triggered()

    # TODO
    def _handle_estop_triggered(self) -> None:
        if 1 in self._estop.pressed_estops:
            self.estop_front_publisher.publish(Bool(data=True))
        if 2 in self._estop.pressed_estops:
            self.estop_back_publisher.publish(Bool(data=True))

    def _handle_estop_released(self) -> None:
        if 1 not in self._estop.pressed_estops:
            self.estop_front_publisher.publish(Bool(data=False))
        if 2 not in self._estop.pressed_estops:
            self.estop_back_publisher.publish(Bool(data=False))

    def soft_estop_callback(self, msg: Bool):
        """Implement a callback for the estop."""
        background_tasks.create(self._estop.set_soft_estop(msg.data))
