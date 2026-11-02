"""Independent measurement device interface; implement using the device's SDK."""
from core.io import InputError


class MetrologyAdapter:
    def __init__(self, config):
        raise InputError("Configure an actual measurement device SDK and calibration reference")

    def samples(self):
        raise InputError("Provide timestamp, frame, units, pose/angle, quality and device source")
