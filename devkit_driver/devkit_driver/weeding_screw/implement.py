from functools import partial
from typing import Any

import rosys
from feldfreund_devkit import FeldfreundHardware, FeldfreundSimulation, Implement, ImplementException
from feldfreund_devkit.hardware import TracksHardware
from rosys.geometry import Point

from .axis import AxisSimulation
from .configuration import WeedingScrewConfiguration
from .y_axis import YAxisCanOpenHardware
from .z_axis import ZAxisCanOpenHardware


class WeedingScrew(Implement):
    """Manual axis control for f18's weeding screw (Y-/Z-axis punch tool).

    Ported from the feldfreund app's WeedingScrew/Axis classes (used there as a template, not
    imported) with camera-based weed targeting and FieldNavigation-driven autonomous work
    dropped; only home / clear-view / stop / punch-at-Y-offset are exposed here.
    """

    DRILL_DEPTH = 0.14

    def __init__(self, config: WeedingScrewConfiguration,
                 feldfreund: FeldfreundHardware | FeldfreundSimulation) -> None:
        super().__init__(config)
        self._config: WeedingScrewConfiguration = config
        self.y_axis = self._setup_y_axis(feldfreund)
        self.z_axis = self._setup_z_axis(feldfreund)
        self.drill_depth: float = self.DRILL_DEPTH

    @property
    def modules(self) -> list[rosys.hardware.Module]:
        return [self.y_axis, self.z_axis]

    async def try_home(self) -> bool:
        if not await rosys.run.retry(self.z_axis.try_reference, max_attempts=2, max_timeout=60.0):
            return False
        await rosys.sleep(0.5)
        return await rosys.run.retry(self.y_axis.try_reference, max_attempts=2, max_timeout=60.0)

    async def clear_view(self) -> None:
        await rosys.run.retry(self.z_axis.return_to_reference, max_attempts=2, max_timeout=20.0)
        y = (self.y_axis.min_position + 0.002) if self.y_axis.position <= 0 \
            else (self.y_axis.max_position - 0.002)
        await rosys.run.retry(partial(self.y_axis.move_to, y, speed=self.y_axis.max_speed),
                              max_attempts=2, max_timeout=20.0)

    async def punch(self, y: float) -> None:
        """Punch at the given Y-offset (m, robot-local, before the implement's mounting offset)."""
        y = round(y + self.offset.y, 5)
        if not self.y_axis.min_position <= y <= self.y_axis.max_position:
            raise ImplementException(f'y position {y} out of range')
        if not await self.is_ready():
            raise ImplementException('weeding screw is not ready (alarm or referencing failed)')
        await rosys.run.retry(partial(self.y_axis.move_to, y), max_attempts=2, max_timeout=20.0)
        await rosys.run.retry(partial(self.z_axis.move_to, -self.drill_depth), max_attempts=2, max_timeout=20.0)
        await rosys.run.retry(self.z_axis.return_to_reference, max_attempts=2, max_timeout=20.0)

    async def stop(self) -> None:
        await self.z_axis.stop()
        await self.y_axis.stop()

    async def is_ready(self) -> bool:
        if self.z_axis.alarm:
            self.log.error('Z-Axis is in alarm, aborting')
            return False
        if self.y_axis.alarm:
            self.log.error('Y-Axis is in alarm, aborting')
            return False
        if not self.z_axis.is_referenced and \
                not await rosys.run.retry(self.z_axis.try_reference, max_attempts=2, max_timeout=60.0):
            self.log.error('referencing Z-Axis failed, aborting')
            return False
        if not self.y_axis.is_referenced and \
                not await rosys.run.retry(self.y_axis.try_reference, max_attempts=2, max_timeout=60.0):
            self.log.error('referencing Y-Axis failed, aborting')
            return False
        return True

    def can_reach(self, local_point: Point) -> bool:
        axis_position = local_point.y + self.offset.y
        return self.y_axis.min_position <= axis_position <= self.y_axis.max_position

    def backup_to_dict(self) -> dict[str, Any]:
        return {'drill_depth': self.drill_depth}

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        self.drill_depth = data.get('drill_depth', self.drill_depth)

    def _setup_y_axis(self, feldfreund: FeldfreundHardware | FeldfreundSimulation
                      ) -> YAxisCanOpenHardware | AxisSimulation:
        if rosys.is_simulation():
            return AxisSimulation(min_position=self._config.y_axis.min_position,
                                  max_position=self._config.y_axis.max_position,
                                  axis_offset=self._config.y_axis.axis_offset)
        assert isinstance(feldfreund, FeldfreundHardware)
        return YAxisCanOpenHardware(self._config.y_axis, feldfreund.robot_brain,
                                    can=feldfreund.can, expander=feldfreund.expander)

    def _setup_z_axis(self, feldfreund: FeldfreundHardware | FeldfreundSimulation
                      ) -> ZAxisCanOpenHardware | AxisSimulation:
        if rosys.is_simulation():
            return AxisSimulation(min_position=self._config.z_axis.min_position,
                                  max_position=self._config.z_axis.max_position,
                                  axis_offset=self._config.z_axis.axis_offset)
        assert isinstance(feldfreund, FeldfreundHardware)
        wheels = feldfreund.wheels
        if not isinstance(wheels, TracksHardware):
            raise ValueError(f'weeding screw drive interlock requires tracked wheels, got {type(wheels).__name__}')
        return ZAxisCanOpenHardware(self._config.z_axis, feldfreund.robot_brain,
                                    can=feldfreund.can, expander=feldfreund.expander, wheels=wheels)
