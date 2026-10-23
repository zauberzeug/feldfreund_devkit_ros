import dataclasses
from pathlib import Path

import feldfreund_devkit
import pytest
import rosys
from devkit_driver.status import DRIVER_VERSION, collect_status
from feldfreund_devkit import System
from feldfreund_devkit.config import FeldfreundConfiguration, GnssConfiguration, Secrets, config_from_file
from rosys.geometry import GeoReference
from rosys.hardware import BumperSimulation
from rosys.testing import forward

CONFIG_FILE = Path(__file__).parents[1] / 'devkit_launch' / 'config' / 'feldfreund.py'


@pytest.fixture
def secrets(monkeypatch: pytest.MonkeyPatch) -> Secrets:
    monkeypatch.setenv('TELTONIKA_PASSWORD', 'unused')
    monkeypatch.setenv('MJPEG_CAMERA_PASSWORD', 'unused')
    return Secrets()


@pytest.fixture
def config(secrets: Secrets) -> FeldfreundConfiguration:
    return config_from_file(CONFIG_FILE, secrets=secrets)


@pytest.fixture
async def system(rosys_integration: None,
                 config: FeldfreundConfiguration, secrets: Secrets) -> System:
    return await _simulated_system(config, secrets)


async def _simulated_system(config: FeldfreundConfiguration, secrets: Secrets) -> System:
    rosys.enter_simulation()
    system = System(config, secrets=secrets)
    await forward(2)
    return system


async def test_idle_robot_without_gnss(system: System) -> None:
    assert collect_status(system) == {
        'robot_id': 'feldfreund',
        'brain_id': 'rb57',
        'status': 'idle',
        'battery': 50,
        'battery_charging': False,
        'position': None,
        'maintenance_mode': None,
        'versions': {
            'feldfreund': f'ros {DRIVER_VERSION}',
            'feldfreund_devkit': feldfreund_devkit.__version__,
            'rosys': rosys.__version__,
            'lizard_core': 'simulation',
            'lizard_p0': 'simulation',
        },
    }


async def test_soft_estop_reports_emergency_stop(system: System) -> None:
    await system.feldfreund.estop.set_soft_estop(True)
    assert collect_status(system)['status'] == 'emergency stop'


async def test_active_bumper_reports_bumper_active(system: System) -> None:
    bumper = system.feldfreund.bumper
    assert isinstance(bumper, BumperSimulation)
    bumper.set_active('front_top', True)
    assert collect_status(system)['status'] == 'bumper active'


async def test_driving_reports_working(system: System) -> None:
    await system.feldfreund.wheels.drive(0.5, 0.0)
    assert collect_status(system)['status'] == 'working'
    await system.feldfreund.wheels.stop()
    assert collect_status(system)['status'] == 'idle'


async def test_gnss_reports_position(rosys_integration: None,
                                     config: FeldfreundConfiguration,
                                     secrets: Secrets) -> None:
    system = await _simulated_system(dataclasses.replace(config, gnss=GnssConfiguration()), secrets)
    assert GeoReference.current is not None
    latitude, longitude = GeoReference.current.origin.degree_tuple
    assert collect_status(system)['position'] == {'lat': pytest.approx(latitude, abs=1e-5),
                                                  'lon': pytest.approx(longitude, abs=1e-5)}
