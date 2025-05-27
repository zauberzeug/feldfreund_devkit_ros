""" Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
    Modified by Zauberzeug GmbH
"""

import os

from rclpy.node import Node
from std_msgs.msg import Empty

from basekit_driver.communication.communication import Communication


class ConfigurationHandler:
    """Handle lizard configuration file."""

    def __init__(self, node: Node, comm: Communication):
        self._node = node
        self._logger = node.get_logger()
        self._comm = comm
        self._software_estop: bool = False
        self._subscription = node.create_subscription(
            Empty, 'configure', self.handle_configure, 10
        )
        self._startup_file = node.get_parameter('startup_file').value

    def handle_configure(self, _msg: Empty):
        """Push startup.liz to microcontroller if available."""
        if not self._startup_file:
            self._logger.warn('No startup file configured. Skipping configuration.')
            return

        if not os.path.exists(self._startup_file):
            self._logger.error('Startup file ' + self._startup_file + ' not found!')
            return

        try:
            with open(self._startup_file, encoding='utf-8') as f:
                self._logger.info('Applying configuration from ' + self._startup_file)
                self._comm.send('!-')
                for line in f.read().splitlines():
                    self._comm.send(f'!+{line}')
                self._comm.send('!.')
                self._comm.send('core.restart()')
                self._logger.info('Configuration applied successfully')
        except Exception as e:
            self._logger.error('Failed to apply configuration: ' + str(e))
