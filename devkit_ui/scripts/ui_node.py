import threading
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from gps_msgs.msg import GPSFix
from nicegui import app, ui, ui_run
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Empty, String


class NiceGuiNode(Node):

    def __init__(self) -> None:
        super().__init__('nicegui')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.configure_publisher = self.create_publisher(Empty, 'configure', 1)
        self.esp_control_publisher = self.create_publisher(String, 'esp_control', 1)

        # Create reliable QoS profile for emergency stop
        estop_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               depth=1)
        self.estop_publisher = self.create_publisher(Bool, 'emergency_stop', estop_qos)
        self.subscription = self.create_subscription(GPSFix, 'gpsfix', self.store_gps, 1)
        self.battery_subscription = self.create_subscription(BatteryState, 'battery_state', self.store_battery, 1)
        self.bumper_front_top_subscription = self.create_subscription(Bool,
                                                                      'bumper_front_top_state',
                                                                      self.update_bumper_front_top,
                                                                      1)
        self.bumper_front_bottom_subscription = self.create_subscription(Bool,
                                                                         'bumper_front_bottom_state',
                                                                         self.update_bumper_front_bottom,
                                                                         1)
        self.bumper_back_subscription = self.create_subscription(Bool, 'bumper_back_state', self.update_bumper_back, 1)
        self.estop1_subscription = self.create_subscription(Bool, 'estop1_state', self.update_estop1, 1)
        self.estop2_subscription = self.create_subscription(Bool, 'estop2_state', self.update_estop2, 1)

        self.bumper_front_top_active = False
        self.bumper_front_bottom_active = False
        self.bumper_back_active = False
        self.soft_estop_active = False
        self.estop1_active = False
        self.estop2_active = False
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.battery_percentage = 0.0
        self.battery_voltage = 0.0
        self.latest_gps = None
        self.latest_battery = None
        self.latest_gps = None

        @ui.page('/')
        def page():
            with ui.row().classes('items-stretch w-[48rem] gap-3'):  # Add gap between items
                with ui.card().classes('flex-1 text-center items-center'):  # Use flex-1 for equal width distribution
                    ui.label('Control').classes('text-2xl')
                    ui.joystick(color='blue', size=50,
                                on_move=lambda e: self.send_speed(float(e.y), float(e.x)),
                                on_end=lambda _: self.send_speed(0.0, 0.0))
                    ui.label('Publish steering commands by dragging your mouse around in the blue field').classes('mt-6')
                    ui.button('EMERGENCY STOP', color='blue', on_click=self.toggle_estop) \
                        .classes('mt-4 text-white font-bold w-40 min-h-[3rem]')
                with ui.card().classes('flex-1 text-center items-center'):
                    ui.label('Data').classes('text-2xl')
                    ui.label('linear velocity').classes('text-xs mb-[-1.8em]')
                    slider_props = 'readonly selection-color=transparent'
                    ui.slider(min=-1, max=1, step=0.05, value=0) \
                        .props(slider_props) \
                        .bind_value(self, 'linear_velocity')
                    ui.label('angular velocity').classes('text-xs mb-[-1.8em]')
                    ui.slider(min=-1, max=1, step=0.05, value=0) \
                        .props(slider_props) \
                        .bind_value(self, 'angular_velocity')
                    ui.label('Battery').classes('text-xs mb-[-1.4em]')
                    ui.label().bind_text_from(self, 'latest_battery',
                                              lambda msg: f'{msg.percentage * 100:.1f}% ({msg.voltage:.1f}V)')
                    ui.button('Send Lizard Config', color='green', on_click=self.send_config).classes('mt-4')
                with ui.card().classes('flex-1 text-center items-center'):
                    ui.label('Safety').classes('text-2xl')
                    ui.label('Bumpers').classes('text-xs mb-[-1.4em]')
                    ui.label('Front Top: ---') \
                        .classes('text-sm') \
                        .bind_text_from(self, 'bumper_front_top_active',
                                        lambda active: 'ACTIVE' if active else 'inactive')
                    ui.label('Front Bottom: ---') \
                        .classes('text-sm') \
                        .bind_text_from(self, 'bumper_front_bottom_active',
                                        lambda active: 'ACTIVE' if active else 'inactive')
                    ui.label('Back: ---') \
                        .classes('text-sm') \
                        .bind_text_from(self, 'bumper_back_active',
                                        lambda active: 'ACTIVE' if active else 'inactive')
                    ui.label('E-Stops').classes('text-xs mb-[-1.4em] mt-4')
                    ui.label('E-Stop 1 (vorne): ---') \
                        .classes('text-sm') \
                        .bind_text_from(self, 'estop1_active', lambda active: 'ACTIVE' if active else 'inactive')
                    ui.label('E-Stop 2 (hinten): ---') \
                        .classes('text-sm') \
                        .bind_text_from(self, 'estop2_active', lambda active: 'ACTIVE' if active else 'inactive')
            with ui.card().classes('w-[48rem] items-center mt-3'):
                ui.label('ESP Control').classes('text-2xl')
                with ui.row().classes('gap-4'):
                    ui.button('Enable', color='green', on_click=lambda: self.send_esp_control('enable')).classes('w-24')
                    ui.button('Disable', color='red', on_click=lambda: self.send_esp_control('disable')).classes('w-24')
                    ui.button('Reset', color='orange', on_click=lambda: self.send_esp_control('reset')).classes('w-24')
                    ui.button('Soft Reset', color='blue',
                              on_click=lambda: self.send_esp_control('soft_reset')).classes('w-24')
            with ui.card().classes('w-[48rem] items-center mt-3'):
                ui.label('GPS Map').classes('text-2xl')
                leaflet = ui.leaflet(center=(51.98278, 7.43440), zoom=16).classes('w-full h-96')
                marker = leaflet.marker(latlng=leaflet.center)

                def update_gps_ui() -> None:
                    """Update the UI with the latest GPS data every 2 seconds."""
                    if self.latest_gps is not None:
                        leaflet.set_center((self.latest_gps.latitude, self.latest_gps.longitude))
                        marker.move(self.latest_gps.latitude, self.latest_gps.longitude)
                ui.timer(2.0, update_gps_ui)

    def toggle_estop(self) -> None:
        """Toggle the emergency stop state."""
        self.soft_estop_active = not self.soft_estop_active
        msg = Bool()
        msg.data = self.soft_estop_active
        self.estop_publisher.publish(msg)

        # Update button appearance while maintaining size
        # if self.soft_estop_active:
        #     self.estop_button.props('color=red')
        #     self.estop_button.text = 'STOPPED'
        #     self.estop_button.classes('w-40 min-h-[3rem]')  # Maintain width and height
        # else:
        #     self.estop_button.props('color=blue')
        #     self.estop_button.text = 'EMERGENCY STOP'
        #     self.estop_button.classes('w-40 min-h-[3rem]')  # Maintain width and height

    def send_esp_control(self, command: str) -> None:
        """Send ESP control command."""
        msg = String()
        msg.data = command
        self.esp_control_publisher.publish(msg)

    def send_speed(self, x: float, y: float) -> None:
        msg = Twist()
        msg.linear.x = x
        msg.angular.z = -y
        self.linear.value = x
        self.angular.value = y
        self.cmd_vel_publisher.publish(msg)

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

    def update_estop1(self, msg: Bool) -> None:
        self.estop1_active = msg.data

    def update_estop2(self, msg: Bool) -> None:
        self.estop2_active = msg.data

    def send_config(self) -> None:
        """Send trigger to configuration handler to apply lizard config."""
        msg = Empty()
        self.configure_publisher.publish(msg)


def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass


def ros_main() -> None:
    rclpy.init()
    node = NiceGuiNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


app.on_startup(lambda: threading.Thread(target=ros_main).start())
ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖', port=80)
