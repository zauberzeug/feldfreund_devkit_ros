from collections.abc import Generator
from pathlib import Path

import pytest
import rclpy
import rosys
from feldfreund_devkit import System
from feldfreund_devkit.config import Secrets, config_from_file
from rclpy.node import Node
from rosys.testing import forward

CONFIG_DIRECTORY = Path(__file__).parents[1] / 'devkit_launch' / 'config'


@pytest.fixture
async def system(rosys_integration: None, monkeypatch: pytest.MonkeyPatch) -> System:
    monkeypatch.setenv('TELTONIKA_PASSWORD', 'unused')
    monkeypatch.setenv('MJPEG_CAMERA_PASSWORD', 'unused')
    secrets = Secrets()
    rosys.enter_simulation()
    system = System(config_from_file(CONFIG_DIRECTORY / 'feldfreund.py', secrets=secrets), secrets=secrets)
    await forward(2)
    return system


@pytest.fixture
def node() -> Generator[Node, None, None]:
    rclpy.init()
    node = Node('controller', cli_args=['--ros-args', '--params-file', str(CONFIG_DIRECTORY / 'devkit.yaml')])
    yield node
    node.destroy_node()
    rclpy.shutdown()
