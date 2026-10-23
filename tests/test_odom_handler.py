import pytest
from devkit_driver.modules import OdomHandler
from feldfreund_devkit import System
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosys.geometry import Velocity
from rosys.testing import forward


@pytest.fixture
def published(system: System, node: Node, monkeypatch: pytest.MonkeyPatch) -> list[tuple[float, float, float, float]]:
    handler = OdomHandler(node, system.odometer, system.feldfreund.wheels)
    messages: list[tuple[float, float, float, float]] = []

    def record(message: Odometry) -> None:
        messages.append((message.pose.pose.position.x, message.pose.pose.position.y,
                         message.twist.twist.linear.x, message.twist.twist.angular.z))
    monkeypatch.setattr(handler._publisher, 'publish', record)
    return messages


async def test_publishes_with_every_wheel_measurement_at_standstill(
        system: System, published: list[tuple[float, float, float, float]]) -> None:
    measurements: list[list[Velocity]] = []
    system.feldfreund.wheels.VELOCITY_MEASURED.subscribe(measurements.append, expect_args=True)
    await forward(2)
    assert measurements
    assert len(published) == len(measurements)
    assert all(message == (0.0, 0.0, 0.0, 0.0) for message in published)


async def test_publishes_the_integrated_pose_and_the_measured_velocity_while_driving(
        system: System, published: list[tuple[float, float, float, float]]) -> None:
    await system.feldfreund.wheels.drive(0.5, 0.1)
    await forward(2)
    x, y, linear, angular = published[-1]
    assert x == pytest.approx(system.odometer.pose.x)
    assert y == pytest.approx(system.odometer.pose.y)
    assert x > 0.9
    assert linear == pytest.approx(0.5)
    assert angular == pytest.approx(0.1)
