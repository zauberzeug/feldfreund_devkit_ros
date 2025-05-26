#!/usr/bin/env python3
""" Copyright (c) 2025 Zauberzeug GmbH
"""

import os
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ..communication.serial_communication import SerialCommunication


class ESPHandler:
    """Handler for ESP operations using espresso.py."""

    def __init__(self, node: Node, serial_comm: SerialCommunication):
        """Initialize ESP handler."""
        self._logger = node.get_logger()
        self._logger.info('Initializing ESP handler')
        self._espresso_path = '/root/.lizard/espresso.py'
        self._serial_comm = serial_comm

        # Get flash parameters from configuration
        node.declare_parameter('flash_parameters', '')
        self._flash_parameters = node.get_parameter('flash_parameters').value
        self._logger.info(f'Using flash parameters: {self._flash_parameters}')

        # Create subscriber for ESP control
        self._control_sub = node.create_subscription(
            String,
            'esp_control',
            self._handle_control_message,
            10
        )

        # enable esp
        # self.enable()

    def _execute_command(self, command: str) -> bool:
        """Execute espresso command and return success status."""
        # Construct command with parameters if they exist
        cmd_parts = ['sudo', self._espresso_path]
        if self._flash_parameters:
            cmd_parts.append(self._flash_parameters)
        cmd_parts.append(command)

        full_command = ' '.join(cmd_parts)
        self._logger.info(f'Executing command: {full_command}')
        result = os.system(full_command)
        success = result == 0
        if success:
            self._logger.info(f'Command {command} executed successfully')
        else:
            self._logger.error(f'Command {command} failed with code {result}')
        return success

    def _handle_control_message(self, msg: String) -> None:
        """Handle incoming ESP control messages."""
        self._logger.info(f'Received ESP control command: {msg.data}')

        if msg.data == 'enable':
            self._logger.info('Enabling ESP')
            print('Enabling ESP', flush=True)
            self.enable()
        elif msg.data == 'disable':
            self._logger.info('Disabling ESP')
            print('Disabling ESP', flush=True)
            self.disable()
        elif msg.data == 'reset':
            self._logger.info('Resetting ESP')
            print('Resetting ESP', flush=True)
            self.reset()
        elif msg.data == 'soft_reset':
            self._logger.info('Soft resetting ESP')
            print('Soft resetting ESP', flush=True)
            self.soft_reset()
        else:
            print('Unknown command received: {msg.data}', flush=True)
            self._logger.warning(f'Unknown command received: {msg.data}')

    def enable(self) -> bool:
        """Enable the ESP."""
        return self._execute_command('enable')

    def disable(self) -> bool:
        """Disable the ESP."""
        return self._execute_command('disable')

    def reset(self) -> bool:
        """Perform a hard reset of the ESP."""
        return self._execute_command('reset')

    def soft_reset(self) -> bool:
        """Perform a soft reset of the ESP by sending a restart command."""
        self._logger.info('Sending soft reset command to ESP')
        self._serial_comm.send('core.restart()')
        return True
