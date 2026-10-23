"""Robot status endpoint polled by the fleet dashboard."""

from typing import Any

import feldfreund_devkit
import rosys
from feldfreund_devkit import Feldfreund, FeldfreundHardware, System
from nicegui import app


def register_status_route(system: System) -> None:
    """Register ``GET /api/status`` and keep the Lizard versions it reports up to date.

    :param system: the devkit system to report on
    """
    feldfreund = system.feldfreund
    if isinstance(feldfreund, FeldfreundHardware):
        lizard_firmware = feldfreund.robot_brain.lizard_firmware

        async def read_lizard_versions() -> None:
            await lizard_firmware.read_core_version()
            await lizard_firmware.read_p0_version()
        # NOTE: flashing restarts the ESP, so reading on connect keeps the versions current
        feldfreund.robot_brain.ESP_CONNECTED.subscribe(read_lizard_versions)

    @app.get('/api/status')
    async def status() -> dict[str, Any]:
        return collect_status(system)


def collect_status(system: System) -> dict[str, Any]:
    """Collect the robot status in the format the fleet dashboard reads.

    :param system: the devkit system to report on
    :return: the status as a JSON-serializable dictionary
    """
    feldfreund = system.feldfreund
    battery = feldfreund.bms.state
    return {
        'robot_id': system.robot_id,
        'brain_id': system.config.robot_brain.name,
        'status': _work_status(feldfreund),
        'battery': round(battery.percentage) if battery.percentage is not None else None,
        'battery_charging': bool(battery.is_charging),
        'position': _position(feldfreund),
        'maintenance_mode': None,
        'versions': _versions(feldfreund),
    }


def _work_status(feldfreund: Feldfreund) -> str:
    if feldfreund.estop.active:
        return 'emergency stop'
    if feldfreund.bumper is not None and feldfreund.bumper.active_bumpers:
        return 'bumper active'
    if feldfreund.wheels.linear_target_speed or feldfreund.wheels.angular_target_speed:
        return 'working'
    return 'idle'


def _position(feldfreund: Feldfreund) -> dict[str, float] | None:
    if feldfreund.gnss is None or feldfreund.gnss.last_measurement is None:
        return None
    latitude, longitude = feldfreund.gnss.last_measurement.point.degree_tuple
    return {'lat': latitude, 'lon': longitude}


def _versions(feldfreund: Feldfreund) -> dict[str, str | None]:
    lizard_firmware = feldfreund.robot_brain.lizard_firmware if isinstance(feldfreund, FeldfreundHardware) else None
    return {
        'feldfreund_devkit': feldfreund_devkit.__version__,
        'rosys': rosys.__version__,
        'lizard_core': lizard_firmware.core_version if lizard_firmware is not None else None,
        'lizard_p0': lizard_firmware.p0_version if lizard_firmware is not None else None,
    }
