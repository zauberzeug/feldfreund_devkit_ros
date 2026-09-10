class CanOpenHardwareException(Exception):
    """Raised when a CANopen axis reports a hardware fault (alarm, not initialized, not enabled)."""
