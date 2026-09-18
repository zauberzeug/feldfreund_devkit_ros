"""Shared pytest fixtures for the driver test suite.

`make test` runs with PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 (see the Makefile: the ROS
launch_testing pytest plugins are incompatible with modern pytest, so plugin autoloading is
disabled and only pytest-asyncio is loaded explicitly). That also disables rosys's own pytest11
plugin registration, so its testing fixtures (rosys_integration, forward, ...) need to be
imported here explicitly instead of relying on autodiscovery.
"""
from rosys.testing.fixtures import enforce_spawn_process, rosys_integration

__all__ = ['enforce_spawn_process', 'rosys_integration']
