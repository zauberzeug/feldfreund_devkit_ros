""" Copyright (c) 2024 Leibniz-Institut für Agrartechnik und Bioökonomie e.V. (ATB)
"""


class Communication:
    """
    Implement communication interface.

    The implemenation scheme is the observer pattern.
    """

    def __init__(self):
        self._core_observers = []
        self._bms_observers = []

    def register_core_observer(self, observer) -> None:
        """Register core observer."""
        self._core_observers.append(observer)

    def notify_core_observers(self, words) -> None:
        """Notify core observer."""
        for observer in self._core_observers:
            observer.update(words)

    def register_bms_observer(self, observer) -> None:
        """Register bms observer."""
        self._bms_observers.append(observer)

    def notify_bms_observers(self, words) -> None:
        """Notify bms observer."""
        for observer in self._bms_observers:
            observer.update(words)

    def send(self, line: str) -> None:
        """Send data"""

    def read(self):
        """Read data"""
