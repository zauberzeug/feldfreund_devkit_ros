"""Entry point for the devkit UI node: starts NiceGUI and spins the ROS node."""

import threading
from pathlib import Path

import rclpy
from nicegui import app, ui, ui_run
from rclpy.executors import ExternalShutdownException

from .node import NiceGuiNode

UI_PORT = 80


class _State:
    """Module-level container for the spinning ROS thread (avoids a global statement)."""
    ros_thread: threading.Thread | None = None


_state = _State()


def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass


def on_startup() -> None:
    _state.ros_thread = threading.Thread(target=ros_main, name='ros_spin')
    _state.ros_thread.start()


def on_shutdown() -> None:
    """Stop the ROS thread cleanly when NiceGUI shuts down."""
    if rclpy.ok():
        rclpy.shutdown()  # makes rclpy.spin() in the ROS thread return
    if _state.ros_thread is not None:
        _state.ros_thread.join(timeout=5.0)


def ros_main() -> None:
    rclpy.init()
    node = NiceGuiNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


app.on_startup(on_startup)
app.on_shutdown(on_shutdown)
ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖', port=UI_PORT)
