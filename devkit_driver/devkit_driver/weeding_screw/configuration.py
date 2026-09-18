from dataclasses import dataclass

from feldfreund_devkit.config import ImplementConfiguration


@dataclass(slots=True, kw_only=True)
class YCanOpenConfiguration:
    """Hardware configuration for the horizontal (Y) axis, driven by a CANopen motor."""
    name: str
    can_address: int
    max_position: float
    min_position: float
    axis_offset: float
    reversed_direction: bool
    end_left_pin: int
    end_right_pin: int
    end_stops_on_expander: bool
    motor_on_expander: bool
    end_stops_inverted: bool
    max_speed: int
    reference_speed: int
    steps_per_m: float


@dataclass(slots=True, kw_only=True)
class ZCanOpenConfiguration:
    """Hardware configuration for the vertical (Z) axis, driven by a CANopen motor."""
    name: str
    can_address: int
    max_position: float
    min_position: float
    axis_offset: float
    reversed_direction: bool
    end_top_pin: int
    end_bottom_pin: int
    end_stops_on_expander: bool
    motor_on_expander: bool
    end_stops_inverted: bool
    max_speed: int
    reference_speed: int
    steps_per_m: float


@dataclass(kw_only=True)
class WeedingScrewConfiguration(ImplementConfiguration):
    """Configuration for the weeding screw implement.

    Only the CANopen axis variant is supported here (f18's actual hardware); the stepper and
    D1-servo variants that exist in the feldfreund app repo have no ROS-side use case yet.

    Defaults:
        lizard_name: 'weeding_screw'
        display_name: 'Weeding Screw'
        work_radius: 0.025
    """
    lizard_name: str = 'weeding_screw'
    display_name: str = 'Weeding Screw'
    work_radius: float = 0.025
    y_axis: YCanOpenConfiguration
    z_axis: ZCanOpenConfiguration
