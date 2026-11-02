"""Offline transport. Never presents recorded feedback as a physical response to new commands."""
from core.io import csv_rows


class ReplayAdapter:
    capabilities = {"read_state": True, "command": False}

    def __init__(self, config):
        self.rows = csv_rows(config["path"])

    def samples(self):
        yield from self.rows

    def close(self):
        pass
