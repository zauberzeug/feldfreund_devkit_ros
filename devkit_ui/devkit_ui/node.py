# pylint: disable=duplicate-code
from __future__ import annotations

from geometry_msgs.msg import Twist
from gps_msgs.msg import GPSFix
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, Duration, LivelinessPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Empty

from .dashboard import Dashboard

SAFETY_QOS = QoSProfile(depth=1,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        liveliness=LivelinessPolicy.AUTOMATIC,
                        liveliness_lease_duration=Duration(seconds=1))


class NiceGuiNode(Node):
    """ROS2 node bridging the dashboard to the devkit driver topics.

    Publishers/commands are invoked from the NiceGUI (main) thread; the subscription
    callbacks run on the ROS executor thread and only assign plain attributes that the
    dashboard's refreshable cards read on the UI thread, so no extra synchronization
    is required.
    """

    def __init__(self) -> None:
        super().__init__('nicegui')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.esp_enable_publisher = self.create_publisher(Empty, 'esp/enable', 1)
        self.esp_disable_publisher = self.create_publisher(Empty, 'esp/disable', 1)
        self.esp_reset_publisher = self.create_publisher(Empty, 'esp/reset', 1)
        self.esp_restart_publisher = self.create_publisher(Empty, 'esp/restart', 1)
        self.esp_configure_publisher = self.create_publisher(Empty, 'esp/configure', 1)
        self.estop_publisher = self.create_publisher(Bool, 'estop/soft', SAFETY_QOS)

        self.create_subscription(GPSFix, 'gpsfix', self.store_gps, 1)
        self.create_subscription(BatteryState, 'battery_state', self.store_battery, 1)
        self.create_subscription(Bool, 'bumper/front_top', self.update_bumper_front_top, SAFETY_QOS)
        self.create_subscription(Bool, 'bumper/front_bottom', self.update_bumper_front_bottom, SAFETY_QOS)
        self.create_subscription(Bool, 'bumper/back', self.update_bumper_back, SAFETY_QOS)
        self.create_subscription(Bool, 'estop/front', self.update_estop_front, SAFETY_QOS)
        self.create_subscription(Bool, 'estop/back', self.update_estop_back, SAFETY_QOS)

        # State polled by the dashboard cards (written from the ROS subscription callbacks).
        self.bumper_front_top_active = False
        self.bumper_front_bottom_active = False
        self.bumper_back_active = False
        self.soft_estop_active = False
        self.estop_front_active = False
        self.estop_back_active = False
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.latest_gps: GPSFix | None = None
        self.latest_battery: BatteryState | None = None

        self._dashboard = Dashboard(self)

    # --- commands (invoked from the UI thread) ---

    def toggle_estop(self) -> None:
        """Toggle the software emergency stop state and publish it."""
        self.soft_estop_active = not self.soft_estop_active
        self.estop_publisher.publish(Bool(data=self.soft_estop_active))

    def send_speed(self, x: float, y: float) -> None:
        """Publish a velocity command and reflect it on the dashboard sliders."""
        msg = Twist()
        msg.linear.x = x
        msg.angular.z = -y
        self.linear_velocity = x
        self.angular_velocity = y
        self.cmd_vel_publisher.publish(msg)

    # --- subscription callbacks (invoked from the ROS thread) ---

    def store_gps(self, msg: GPSFix) -> None:
        """Store the latest GPS message."""
        self.latest_gps = msg

    def store_battery(self, msg: BatteryState) -> None:
        """Store the latest battery state."""
        self.latest_battery = msg

    def update_bumper_front_top(self, msg: Bool) -> None:
        self.bumper_front_top_active = msg.data

    def update_bumper_front_bottom(self, msg: Bool) -> None:
        self.bumper_front_bottom_active = msg.data

    def update_bumper_back(self, msg: Bool) -> None:
        self.bumper_back_active = msg.data

    def update_estop_front(self, msg: Bool) -> None:
        self.estop_front_active = msg.data

    def update_estop_back(self, msg: Bool) -> None:
        self.estop_back_active = msg.data
