from __future__ import annotations

from typing import TYPE_CHECKING

from nicegui import ui
from std_msgs.msg import Empty, Float64

if TYPE_CHECKING:
    from .node import NiceGuiNode

DEFAULT_MAP_CENTER = (51.98278, 7.43440)  # Zauberzeug HQ, Münster
MAP_ZOOM = 16
GPS_UPDATE_INTERVAL = 2.0  # seconds
UI_REFRESH_INTERVAL = 0.5  # seconds


class Dashboard:
    """Builds the NiceGUI control dashboard for a NiceGuiNode.

    The data-driven cards are ``@ui.refreshable_method`` builders that read the node
    state directly; the e-stop card refreshes on click. The GPS card builds the map
    once and moves its marker via its own timer.
    """

    def __init__(self, node: NiceGuiNode) -> None:
        self._node = node

        @ui.page('/')
        def page() -> None:
            self.content()

    def content(self) -> None:
        with ui.row().classes('items-stretch w-[48rem] gap-3'):
            self._control_card()
            self._data_card()
            self._safety_card()
        self._esp_card()
        self._weeding_screw_card()
        self._gps_card()
        # NiceGUI has no auto-refresh, so poll the data-driven parts on a timer. The
        # joystick lives in the (non-refreshed) control card so dragging is never interrupted.
        ui.timer(UI_REFRESH_INTERVAL, self._estop_button.refresh)
        ui.timer(UI_REFRESH_INTERVAL, self._data_card.refresh)
        ui.timer(UI_REFRESH_INTERVAL, self._safety_card.refresh)
        ui.timer(UI_REFRESH_INTERVAL, self._weeding_screw_status.refresh)

    def _control_card(self) -> None:
        node = self._node
        with ui.card().classes('flex-1 text-center items-center'):
            ui.label('Control').classes('text-2xl')
            ui.joystick(color='blue', size=50,
                        on_move=lambda e: node.send_speed(float(e.y), float(e.x)),
                        on_end=lambda _: node.send_speed(0.0, 0.0))
            ui.label('Publish steering commands by dragging your mouse around in the blue field').classes('mt-6')
            self._estop_button()

    @ui.refreshable_method
    def _estop_button(self) -> None:
        label, color = ('STOPPED', 'red') if self._node.soft_estop_active else ('EMERGENCY STOP', 'blue')
        ui.button(label, color=color, on_click=self._toggle_estop).classes('w-40 min-h-[3rem]')

    def _toggle_estop(self) -> None:
        self._node.toggle_estop()
        self._estop_button.refresh()

    @ui.refreshable_method
    def _data_card(self) -> None:
        node = self._node
        with ui.card().classes('flex-1 text-center items-center'):
            ui.label('Data').classes('text-2xl')
            slider_props = 'readonly selection-color=transparent'
            ui.label('linear velocity').classes('text-xs mb-[-1.8em]')
            ui.slider(min=-1, max=1, step=0.05, value=node.linear_velocity).props(slider_props)
            ui.label('angular velocity').classes('text-xs mb-[-1.8em]')
            ui.slider(min=-1, max=1, step=0.05, value=node.angular_velocity).props(slider_props)
            ui.label('Battery').classes('text-xs mb-[-1.4em]')
            battery = node.latest_battery
            text = f'{battery.percentage * 100:.1f}% ({battery.voltage:.1f}V)' if battery is not None else 'N/A'
            ui.label(text)

    @ui.refreshable_method
    def _safety_card(self) -> None:
        node = self._node
        with ui.card().classes('flex-1 text-center items-center'):
            ui.label('Safety').classes('text-2xl')
            ui.label('Bumpers').classes('text-xs mb-[-1.4em]')
            self._status_label('Front Top', node.bumper_front_top_active)
            self._status_label('Front Bottom', node.bumper_front_bottom_active)
            self._status_label('Back', node.bumper_back_active)
            ui.label('E-Stops').classes('text-xs mb-[-1.4em] mt-4')
            self._status_label('Front', node.estop_front_active)
            self._status_label('Back', node.estop_back_active)

    @staticmethod
    def _status_label(title: str, active: bool) -> None:
        """Add a label reflecting an active/inactive boolean state."""
        ui.label(f'{title}: ' + ('ACTIVE' if active else 'inactive')).classes('text-sm')

    def _esp_card(self) -> None:
        node = self._node
        with ui.card().classes('w-[48rem] items-center mt-3'):
            ui.label('ESP Control').classes('text-2xl')
            with ui.row().classes('gap-4'):
                ui.button('Enable', color='green',
                          on_click=lambda: node.esp_enable_publisher.publish(Empty())).classes('w-24')
                ui.button('Disable', color='red',
                          on_click=lambda: node.esp_disable_publisher.publish(Empty())).classes('w-24')
                ui.button('Reset', color='orange',
                          on_click=lambda: node.esp_reset_publisher.publish(Empty())).classes('w-24')
                ui.button('Restart', color='blue',
                          on_click=lambda: node.esp_restart_publisher.publish(Empty())).classes('w-24')
                ui.button('Configure', color='purple',
                          on_click=lambda: node.esp_configure_publisher.publish(Empty())).classes('w-24')

    def _weeding_screw_card(self) -> None:
        node = self._node
        with ui.card().classes('w-[48rem] items-center mt-3'):
            ui.label('Weeding Screw').classes('text-2xl')
            with ui.row().classes('gap-4'):
                ui.button('Home', color='blue',
                          on_click=lambda: node.weeding_screw_home_publisher.publish(Empty())).classes('w-28')
                ui.button('Clear View', color='blue',
                          on_click=lambda: node.weeding_screw_clear_view_publisher.publish(Empty())).classes('w-28')
                ui.button('Stop', color='red',
                          on_click=lambda: node.weeding_screw_stop_publisher.publish(Empty())).classes('w-28')
            with ui.row().classes('items-center gap-2 mt-3'):
                y_input = ui.number('Y-offset', value=0.0, step=0.005, format='%.3f') \
                    .props('dense outlined suffix=m').classes('w-32')
                ui.button('Punch', color='purple',
                          on_click=lambda: node.weeding_screw_punch_publisher.publish(Float64(data=y_input.value))) \
                    .classes('w-24')
            with ui.row().classes('items-center gap-2 mt-2'):
                depth_input = ui.number('Drill depth', value=0.14, step=0.01, format='%.2f') \
                    .props('dense outlined suffix=m').classes('w-32')
                ui.button('Set Depth', color='teal',
                          on_click=lambda: node.weeding_screw_set_drill_depth_publisher.publish(
                              Float64(data=depth_input.value))) \
                    .classes('w-28')
            self._weeding_screw_status()

    @ui.refreshable_method
    def _weeding_screw_status(self) -> None:
        node = self._node
        with ui.row().classes('gap-6 mt-2'):
            self._status_label('Referenced', node.weeding_screw_is_referenced)
            self._status_label('Alarm', node.weeding_screw_alarm)
            ui.label(f'Y: {node.weeding_screw_y_position:.3f} m').classes('text-sm')
            ui.label(f'Z: {node.weeding_screw_z_position:.3f} m').classes('text-sm')
            ui.label(f'Depth: {node.weeding_screw_drill_depth:.3f} m').classes('text-sm')

    def _gps_card(self) -> None:
        with ui.card().classes('w-[48rem] items-center mt-3'):
            ui.label('GPS Map').classes('text-2xl')
            leaflet = ui.leaflet(center=DEFAULT_MAP_CENTER, zoom=MAP_ZOOM).classes('w-full h-96')
            marker = leaflet.marker(latlng=leaflet.center)

            def update_gps_ui() -> None:
                """Move the marker to the latest GPS position (without rebuilding the map)."""
                gps = self._node.latest_gps
                if gps is not None:
                    leaflet.set_center((gps.latitude, gps.longitude))
                    marker.move(gps.latitude, gps.longitude)
            ui.timer(GPS_UPDATE_INTERVAL, update_gps_ui)
