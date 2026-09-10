"""Regression test for the weeding screw axis.

Guards against a real bug found during the first f18 hardware test: stop() only disabled the
motor for one poll cycle, and the still-running move_to() loop noticed the motor was disabled and
simply re-enabled it itself - "stop" paused the axis for a moment instead of aborting the move.
The first fix attempt (cancelling the caller's asyncio task) didn't help either: every move_to()
call goes through rosys.run.retry(..., max_timeout=...), which runs the move in its own detached
task internally, so cancelling the caller never reaches it. Only a flag the axis checks itself
(set by stop(), read inside move_to()'s own poll loop) actually works, regardless of which task
called it.
"""
import rosys
from devkit_driver.weeding_screw import AxisSimulation
from rosys.hardware import RobotSimulation
from rosys.testing import forward


async def test_stop_aborts_an_in_progress_move(rosys_integration: None) -> None:
    axis = AxisSimulation(min_position=-0.2, max_position=0.0)
    robot = RobotSimulation([axis])  # keep alive: on_repeat only holds a weak-ish handle
    assert await axis.try_reference()

    move_task = rosys.background_tasks.create(axis.move_to(-0.19, speed=50))
    await forward(seconds=0.5)
    position_before_stop = axis.position
    assert position_before_stop < -0.02, 'the move should be well underway by now'

    await axis.stop()
    await forward(seconds=1.0)

    assert move_task.done()
    assert move_task.cancelled()
    assert axis.position == position_before_stop, 'the axis must not keep moving after stop()'
    assert axis.position > -0.15, 'the axis should have stopped well short of its target'
    del robot
