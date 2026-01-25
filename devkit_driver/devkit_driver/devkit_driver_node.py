#!/usr/bin/env python3

import threading
from pathlib import Path

import rclpy
from feldfreund_devkit import FeldfreundHardware, System, api
from feldfreund_devkit.config import config_from_file
from nicegui import app, ui, ui_run
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from devkit_driver.modules import BMSHandler, BumperHandler, EStopHandler, OdomHandler, RobotBrainHandler, TwistHandler


class DevkitDriver(Node):
    """Devkit node handler."""

    def __init__(self, system: System):
        super().__init__('devkit_driver_node')
        self.system = system
        assert isinstance(self.system.feldfreund, FeldfreundHardware)
        self._robot_brain_handler = RobotBrainHandler(self, self.system.feldfreund.robot_brain)
        self._odom_handler = OdomHandler(self, self.system.odometer)
        self._bms_handler = BMSHandler(self, self.system.feldfreund.bms)
        if self.system.feldfreund.bumper is not None:
            self._bumper_handler = BumperHandler(self, self.system.feldfreund.bumper, self.system.feldfreund.estop)
        self._twist_handler = TwistHandler(self, self.system.feldfreund.wheels)
        self._estop_handler = EStopHandler(self, self.system.feldfreund.estop)


def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass


def on_startup() -> None:
    config = config_from_file('/workspace/src/devkit_launch/config/feldfreund.py')
    system = System(config)
    api.Online()
    threading.Thread(target=ros_main, args=(system,)).start()


def ros_main(system: System) -> None:
    rclpy.init()
    devkit_driver = DevkitDriver(system)
    try:
        rclpy.spin(devkit_driver)
    except ExternalShutdownException:
        pass


app.on_startup(on_startup)
ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖')
