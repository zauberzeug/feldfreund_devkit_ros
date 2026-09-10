# pylint: disable=duplicate-code
import asyncio

import rosys
from feldfreund_devkit.hardware import SafetyMixin, TracksHardware
from rosys.automation import uninterruptible
from rosys.helpers import remove_indentation

from .axis import Axis
from .configuration import ZCanOpenConfiguration
from .errors import CanOpenHardwareException


class ZAxisCanOpenHardware(Axis, rosys.hardware.ModuleHardware, SafetyMixin):
    """Controls the vertical axis using a CANopen motor, with a Lizard-side drive interlock."""

    DRIVE_RELEASE_TOLERANCE = 0.01
    """The screw counts as parked (drive released) within this distance (m) of its reference point."""

    def __init__(self, config: ZCanOpenConfiguration,
                 robot_brain: rosys.hardware.RobotBrain, *,
                 can: rosys.hardware.CanHardware,
                 expander: rosys.hardware.ExpanderHardware | None,
                 wheels: TracksHardware) -> None:
        self.config = config
        self.expander = expander
        self._ctrl_enable = False
        self._initialized = False
        self._operational = False

        lizard_code = remove_indentation(f'''
            {config.name}_motor = {expander.name + "." if config.motor_on_expander and expander else ""}CanOpenMotor({can.name}, {config.can_address})
            {config.name}_end_t = {expander.name + "." if config.end_stops_on_expander and expander else ""}Input({config.end_top_pin})
            {config.name}_end_t.inverted = {str(config.end_stops_inverted).lower()}
            {config.name}_end_b = {expander.name + "." if config.end_stops_on_expander and expander else ""}Input({config.end_bottom_pin})
            {config.name}_end_b.inverted = {str(config.end_stops_inverted).lower()}
            {config.name} = {expander.name + "." if config.motor_on_expander and expander else ""}MotorAxis({config.name}_motor, {config.name + "_end_t" if config.reversed_direction else config.name + "_end_b"}, {config.name + "_end_b" if config.reversed_direction else config.name + "_end_t"})
        ''')
        flag = f'{config.name}_drive_release'
        lizard_code += remove_indentation(f'''
            bool {flag} = false
            when {flag} then {wheels.name}.locked = false; end
            when {flag} == false then {wheels.name}.locked = true; end
        ''')
        core_message_fields = [
            f'{config.name}_end_t.active',
            f'{config.name}_end_b.active',
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
        self.REFERENCE_CHANGED.subscribe(self._refresh_drive_release, unsubscribe_on_delete=False)
        self.REFERENCE_CHANGED.subscribe(self._notify_reference_lost, unsubscribe_on_delete=False)

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
        try:
            steps = self.compute_steps(position)
            assert self.robot_brain.is_ready, 'robot brain is not ready'
            assert self.initialized, 'motor is not initialized'
            assert self.operational, 'motor is not operational'
            await self.robot_brain.send(f'{self.config.name}_drive_release = false;')  # lock for the move
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
        finally:
            # the loop leaves the streamed steps at the target, so this reads where the screw is;
            # an error or cancel judges wherever it came to a halt
            await self._refresh_drive_release()

    def _is_parked(self, position: float) -> bool:
        """Whether the screw is retracted close enough to its reference for the robot to drive."""
        return abs(position - self.compute_position(0)) < self.DRIVE_RELEASE_TOLERANCE

    async def _refresh_drive_release(self) -> None:
        """Lock or release the wheels according to where the screw actually is.

        The brain's only input on whether the screw is parked, subscribed to the reference edge and
        called after every move. The value is derived right before it goes out, so concurrent callers
        reach the brain in the order they read the state. Callers must make sure the streamed steps
        are current: right after writing ``position_offset`` they are not.

        Skipped while the brain is unreachable, which is safe in both directions: the brain declares
        the flag as false on every restart and cannot drive while it is down.
        """
        if not self.robot_brain.is_ready:
            self.log.warning('robot brain not ready')
            return
        released = self.is_referenced and self._is_parked(self.position)
        await self.robot_brain.send(f'{self.config.name}_drive_release = {str(released).lower()};')

    def _notify_reference_lost(self, is_referenced: bool) -> None:
        if is_referenced:
            return
        self.log.warning('%s lost its reference; driving stays locked until it is referenced again', self.config.name)

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
            await self.robot_brain.send(f'{self.config.name}_drive_release = false;')  # lock while homing
            await self.enable_motor()
            await rosys.sleep(1)
            self.check_motor()
            await self.robot_brain.send(f'{self.config.name}_motor.position_offset = 0;')
            await rosys.sleep(1)
            await self.robot_brain.send(f'{self.config.name}_motor.enter_pv_mode();')
            await rosys.sleep(1)

            # if in end b stop, move out
            if self.end_b:
                velocity = self.reference_speed * (-1 if self.reversed_direction else 1)
                await self.robot_brain.send(f'{self.config.name}.speed({velocity}, 0);')
                while self.end_b:
                    await rosys.sleep(0.2)
                await self.robot_brain.send(f'{self.config.name}_motor.set_ctrl_halt(true);')
            await rosys.sleep(0.5)

            # move to end t stop if not already there
            if not self.end_t:
                velocity = self.reference_speed * (-1 if self.reversed_direction else 1)
                await self.robot_brain.send(f'{self.config.name}.speed({velocity}, 0);')
                while not self.end_t:
                    await rosys.sleep(0.2)
            await rosys.sleep(0.5)

            # move out of end t stop
            velocity = -self.reference_speed * (-1 if self.reversed_direction else 1)
            await self.robot_brain.send(f'{self.config.name}.speed({velocity}, 0);')
            while self.end_t:
                await rosys.sleep(0.2)
            await rosys.sleep(0.5)

            # move slowly to end t stop
            slow_velocity = 25 * (-1 if self.reversed_direction else 1)
            await self.robot_brain.send(f'{self.config.name}.speed({slow_velocity}, 0);')
            while not self.end_t:
                await rosys.sleep(0.2)
            await rosys.sleep(0.5)

            # move slowly out of end t stop
            slow_velocity = -25 * (-1 if self.reversed_direction else 1)
            await self.robot_brain.send(f'{self.config.name}.speed({slow_velocity}, 0);')
            while self.end_t:
                await rosys.sleep(0.2)
            await rosys.sleep(0.5)

            await self.robot_brain.send(f'{self.config.name}_motor.position_offset = {self.steps};')
            # frames already in flight still carry the raw count, which at this axis' resolution
            # sits far outside the release band, so wait for the re-based one
            deadline = rosys.time() + 2.0
            while not self._is_parked(self.position):
                if rosys.time() >= deadline:
                    raise TimeoutError(f'position {self.position} did not settle at the reference')
                await rosys.sleep(0.1)
            self.is_referenced = True
            return True
        except Exception as error:
            self.log.error('could not reference %s because of %s', self.config.name, error)
            return False
        finally:
            await self.stop()
            # the steps are re-based on the reference point now, so this reads whether the screw
            # ended up parked; a failed or cancelled reference has no reference and stays locked
            await self._refresh_drive_release()

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
        self.end_t = words.pop(0) == 'true'
        self.end_b = words.pop(0) == 'true'
        if self.end_t or self.end_b:
            self.is_referenced = False
        self.steps = int(words.pop(0))
        self.idle = words.pop(0) == 'true'
        self.alarm = words.pop(0) == 'true'
        if self.alarm:
            self.is_referenced = False
        self._ctrl_enable = words.pop(0) == 'true'
        self._initialized = words.pop(0) == 'true'
        self._operational = words.pop(0) == 'true'
