# pylint: disable=duplicate-code
import asyncio

import rosys
from feldfreund_devkit.hardware import SafetyMixin
from rosys.automation import uninterruptible
from rosys.helpers import remove_indentation

from .axis import Axis
from .configuration import YCanOpenConfiguration
from .errors import CanOpenHardwareException


class YAxisCanOpenHardware(Axis, rosys.hardware.ModuleHardware, SafetyMixin):
    """Controls the horizontal axis using a CANopen motor."""

    def __init__(self, config: YCanOpenConfiguration,
                 robot_brain: rosys.hardware.RobotBrain, *,
                 can: rosys.hardware.CanHardware,
                 expander: rosys.hardware.ExpanderHardware | None) -> None:
        self.config = config
        self.expander = expander
        self._ctrl_enable = False
        self._initialized = False
        self._operational = False
        lizard_code = remove_indentation(f'''
            {config.name}_motor = {expander.name + "." if config.motor_on_expander and expander else ""}CanOpenMotor({can.name}, {config.can_address})
            {config.name}_end_l = {expander.name + "." if config.end_stops_on_expander and expander else ""}Input({config.end_left_pin})
            {config.name}_end_l.inverted = {str(config.end_stops_inverted).lower()}
            {config.name}_end_r = {expander.name + "." if config.end_stops_on_expander and expander else ""}Input({config.end_right_pin})
            {config.name}_end_r.inverted = {str(config.end_stops_inverted).lower()}
            {config.name} = {expander.name + "." if config.motor_on_expander and expander else ""}MotorAxis({config.name}_motor, {config.name + "_end_l" if config.reversed_direction else config.name + "_end_r"}, {config.name + "_end_r" if config.reversed_direction else config.name + "_end_l"})
        ''')
        core_message_fields = [
            f'{config.name}_end_l.active',
            f'{config.name}_end_r.active',
            f'{config.name}_motor.actual_position',
            f'{config.name}_motor.status_target_reached',
            f'{config.name}_motor.status_fault',
            f'{config.name}_motor.ctrl_enable',
            f'{config.name}_motor.initialized',
            f'{config.name}_motor.is_operational',
        ]
        super().__init__(max_speed=config.max_speed,
                         reference_speed=config.reference_speed,
                         min_position=config.min_position,
                         max_position=config.max_position,
                         axis_offset=config.axis_offset,
                         steps_per_m=config.steps_per_m,
                         reversed_direction=config.reversed_direction,
                         robot_brain=robot_brain,
                         lizard_code=lizard_code,
                         core_message_fields=core_message_fields)

    @property
    def enable_code(self) -> str:
        return f'{self.config.name}.enable();'

    @property
    def disable_code(self) -> str:
        return f'{self.config.name}.disable();'

    @property
    def ctrl_enable(self) -> bool:
        return self._ctrl_enable

    @property
    def initialized(self) -> bool:
        return self._initialized

    @property
    def operational(self) -> bool:
        return self._operational

    async def stop(self) -> None:
        self._stop_requested = True
        if not self.robot_brain.is_ready:
            self.log.warning('robot brain not ready')
            return
        await self.robot_brain.send(f'{self.config.name}_motor.set_ctrl_enable(false);')

    @uninterruptible
    async def move_to(self, position: float, speed: int | None = None) -> None:
        self._stop_requested = False
        if speed is None:
            speed = self.max_speed
        try:
            await super().move_to(position, speed)
        except RuntimeError as error:
            self._raise_exception(f'could not move to {position} because of {error}')
        steps = self.compute_steps(position)
        assert self.robot_brain.is_ready, 'robot brain is not ready'
        assert self.initialized, 'motor is not initialized'
        assert self.operational, 'motor is not operational'
        while self.steps != steps:
            await rosys.sleep(0.2)
            if self._stop_requested:
                self.log.info('%s move stopped', self.config.name)
                raise asyncio.CancelledError()
            if self.alarm:
                self._raise_exception('alarm state detected')
            if not self.ctrl_enable:
                self.log.warning('%s is not enabled, sending enable command', self.config.name)
                await self.enable_motor()
                continue
            await self.robot_brain.send(f'{self.config.name}.position({steps},{speed}, 0);')
        await self.robot_brain.send(f'{self.config.name}_motor.set_ctrl_halt(true);')

    async def enable_motor(self) -> None:
        await self.robot_brain.send(f'{self.config.name}_motor.set_ctrl_enable(true);')

    async def disable_motor(self) -> None:
        await self.robot_brain.send(f'{self.config.name}_motor.set_ctrl_enable(false);')

    async def reset_fault(self) -> None:
        await self.robot_brain.send(f'{self.config.name}_motor.reset_fault()')
        await rosys.sleep(1.0)

    async def recover(self) -> None:
        await self.reset_fault()
        await rosys.run.retry(self.try_reference, max_attempts=2, max_timeout=60.0)

    @uninterruptible
    async def try_reference(self) -> bool:
        if not await super().try_reference():
            return False
        try:
            assert self.robot_brain.is_ready, 'robot brain is not ready'
            await self.enable_motor()
            await rosys.sleep(1)
            self.check_motor()
            await self.robot_brain.send(f'{self.config.name}_motor.position_offset = 0;')
            await rosys.sleep(1)
            await self.robot_brain.send(f'{self.config.name}_motor.enter_pv_mode();')
            await rosys.sleep(1)

            # if in end l stop, move out
            if self.end_l:
                velocity = -self.reference_speed * (-1 if self.reversed_direction else 1)
                await self.robot_brain.send(f'{self.config.name}.speed({velocity}, 0);')
                while self.end_l:
                    await rosys.sleep(0.2)
                await self.robot_brain.send(f'{self.config.name}_motor.set_ctrl_halt(true);')
            await rosys.sleep(0.5)

            # move to end r stop if not already there
            if not self.end_r:
                velocity = -self.reference_speed * (-1 if self.reversed_direction else 1)
                await self.robot_brain.send(f'{self.config.name}.speed({velocity}, 0);')
                while not self.end_r:
                    await rosys.sleep(0.2)
            await rosys.sleep(0.5)

            # move out of end r stop
            velocity = self.reference_speed * (-1 if self.reversed_direction else 1)
            await self.robot_brain.send(f'{self.config.name}.speed({velocity}, 0);')
            while self.end_r:
                await rosys.sleep(0.2)
            await rosys.sleep(0.5)

            # move slowly to end r stop
            slow_velocity = -25 * (-1 if self.reversed_direction else 1)
            await self.robot_brain.send(f'{self.config.name}.speed({slow_velocity}, 0);')
            while not self.end_r:
                await rosys.sleep(0.2)
            await rosys.sleep(0.5)

            # move slowly out of end r stop
            slow_velocity = 25 * (-1 if self.reversed_direction else 1)
            await self.robot_brain.send(f'{self.config.name}.speed({slow_velocity}, 0);')
            while self.end_r:
                await rosys.sleep(0.2)
            await rosys.sleep(0.5)

            await self.robot_brain.send(f'{self.config.name}_motor.enter_pv_mode(0);')
            await self.robot_brain.send(f'{self.config.name}_motor.position_offset = {self.steps};')
            await rosys.sleep(0.2)
            self.is_referenced = True
            await self.robot_brain.send(f'{self.config.name}_motor.enter_pp_mode(0);')
            return True
        except Exception as error:
            self.log.error('could not reference %s because of %s', self.config.name, error)
            return False
        finally:
            await self.stop()

    def check_motor(self) -> None:
        if self.alarm:
            self._raise_exception('alarm state detected')
        if not self.initialized:
            self._raise_exception('is not initialized')
        if not self.operational:
            self._raise_exception('is not operational')
        if not self.ctrl_enable:
            self._raise_exception('is not enabled')

    def _raise_exception(self, message: str) -> None:
        self.is_referenced = False
        self.log.error('%s %s', self.config.name, message)
        raise CanOpenHardwareException(f'{self.config.name}: {message}')

    def handle_core_output(self, time: float, words: list[str]) -> None:
        self.end_l = words.pop(0) == 'true'
        self.end_r = words.pop(0) == 'true'
        if self.end_l or self.end_r:
            self.is_referenced = False
        self.steps = int(words.pop(0))
        self.idle = words.pop(0) == 'true'
        self.alarm = words.pop(0) == 'true'
        if self.alarm:
            self.is_referenced = False
        self._ctrl_enable = words.pop(0) == 'true'
        self._initialized = words.pop(0) == 'true'
        self._operational = words.pop(0) == 'true'
