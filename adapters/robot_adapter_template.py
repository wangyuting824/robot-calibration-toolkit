"""Implement this interface using the actual assembled robot's control protocol.

No generic movement commands, end-stop search, or inferred driver protocol is
provided. Raise clear errors until the project/component protocol is implemented.
"""
from core.io import InputError


class RobotAdapter:
    capabilities = {"read_state": False, "command": False, "watchdog": False, "stop": False}

    def __init__(self, config):
        raise InputError("Robot adapter requires the actual project/component control protocol and commissioning; ROS 2 FollowJointTrajectory motion is available through 控制关节.sh")

    def samples(self):
        raise InputError("Implement timestamped read_state according to the actual control interface documentation")

    def command(self, target):
        raise InputError("Implement explicit commands according to the actual project/component interface documentation")

    def stop(self):
        raise InputError("Implement the documented stop command")

    def close(self):
        pass
