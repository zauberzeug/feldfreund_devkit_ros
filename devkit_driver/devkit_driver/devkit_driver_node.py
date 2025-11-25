#!/usr/bin/env python3

""" Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
    Modified by Zauberzeug GmbH
"""

import rclpy

# from feldfreund_devkit import System
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from devkit_driver.communication.serial_communication import SerialCommunication
from devkit_driver.modules.bms_handler import BMSHandler
from devkit_driver.modules.bumper_handler import BumperHandler
from devkit_driver.modules.configuration_handler import ConfigurationHandler
from devkit_driver.modules.esp_handler import ESPHandler
from devkit_driver.modules.estop_handler import EStopHandler

# from devkit_driver.modules.odom_handler import OdomHandler
from devkit_driver.modules.twist_handler import TwistHandler


class DevkitDriver(Node):
    """Devkit node handler."""

    def __init__(self):
        super().__init__('devkit_driver_node')

        # Declare parameters
        self.declare_parameter('startup_file', '')

        # self._system = System(robot_id='f23')

        self._serial_communication = SerialCommunication(self)
        # self._odom_handler = OdomHandler(self, self._serial_communication)
        self._bms_handler = BMSHandler(self, self._serial_communication)
        self._bumper_handler = BumperHandler(self, self._serial_communication)
        self._twist_handler = TwistHandler(self, self._serial_communication)
        self._estop_handler = EStopHandler(self, self._serial_communication)
        self._configuration_handler = ConfigurationHandler(
            self, self._serial_communication)
        self._esp_handler = ESPHandler(self, self._serial_communication)

        # Temporary fix to clear any buffered data before starting to read
        if self._serial_communication.serial is not None:
            self._serial_communication.serial.reset_input_buffer()

        self.read_timer = self.create_timer(0.05, self.read_data)

    def read_data(self):
        """Read data from the serial communication."""
        self._serial_communication.read()


def main(args=None):
    """Implmenets main function call."""
    rclpy.init(args=args)

    try:
        devkit_driver = DevkitDriver()

        executor = SingleThreadedExecutor()
        executor.add_node(devkit_driver)

        try:
            executor.spin()
        finally:
            executor.shutdown()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
