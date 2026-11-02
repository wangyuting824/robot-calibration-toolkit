"""Implement this interface using the ACTUAL robot manufacturer's SDK.

No generic movement commands, end-stop search, or inferred driver protocol is
provided. Raise clear errors until the manufacturer protocol is implemented.
"""
from core.io import InputError


class RobotAdapter:
    capabilities = {"read_state": False, "command": False, "watchdog": False, "stop": False}

    def __init__(self, config):
        raise InputError("Robot adapter requires the manufacturer's SDK, protocol and commissioning")

    def samples(self):
        raise InputError("Implement timestamped read_state according to manufacturer documentation")

    def command(self, target):
        raise InputError("Implement explicit position/velocity/effort commands according to manufacturer documentation")

    def stop(self):
        raise InputError("Implement the documented stop command")

    def close(self):
        pass
