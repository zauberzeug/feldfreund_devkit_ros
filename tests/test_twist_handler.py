import pytest
from devkit_driver.modules import TwistHandler
from feldfreund_devkit import System
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from rosys.geometry import Velocity
from rosys.testing import forward


@pytest.fixture
def handler(system: System, node: Node) -> TwistHandler:
    return TwistHandler(node, system.feldfreund.wheels)


@pytest.fixture
def commanded(system: System) -> list[float]:
    linear_speeds: list[float] = []

    def record(velocity: Velocity) -> None:
        linear_speeds.append(velocity.linear)
    system.feldfreund.wheels.VELOCITY_COMMANDED.subscribe(record)
    return linear_speeds


def forward_command(linear: float) -> Twist:
    return Twist(linear=Vector3(x=linear))


async def test_keeps_driving_while_commands_arrive(handler: TwistHandler, commanded: list[float]) -> None:
    for _ in range(20):
        handler.handle_twist(forward_command(0.5))
        await forward(0.2)
    assert commanded == [0.5] * 20


async def test_stops_once_when_commands_stop_arriving(
        system: System, handler: TwistHandler, commanded: list[float]) -> None:
    handler.handle_twist(forward_command(0.5))
    await forward(0.3)
    assert system.feldfreund.wheels.linear_target_speed == 0.5
    await forward(3)
    assert system.feldfreund.wheels.linear_target_speed == 0.0
    assert commanded == [0.5, 0.0]


async def test_resumes_driving_with_the_next_command_after_a_timeout(
        system: System, handler: TwistHandler, commanded: list[float]) -> None:
    handler.handle_twist(forward_command(0.5))
    await forward(1)
    handler.handle_twist(forward_command(0.3))
    await forward(0.2)
    assert system.feldfreund.wheels.linear_target_speed == 0.3
    assert commanded == [0.5, 0.0, 0.3]


@pytest.mark.usefixtures('handler')
async def test_does_not_stop_at_standstill(commanded: list[float]) -> None:
    await forward(3)
    assert commanded == []


async def test_does_not_stop_again_after_a_zero_command(handler: TwistHandler, commanded: list[float]) -> None:
    handler.handle_twist(forward_command(0.0))
    await forward(3)
    assert commanded == [0.0]
