
"""Optional lightweight diagnostics helpers. Rendering is intentionally non-invasive."""
from __future__ import annotations

class GridTelemetry:
    def __init__(self):
        self.frame = 0
        self.action = "boot"
        self.role = "solo"
        self.intercept_time = None
        self.last_error = None

    def update(self, action, role, intercept=None):
        self.frame += 1
        self.action = str(action)
        self.role = str(role)
        self.intercept_time = getattr(intercept, "time", None)

    def line(self):
        eta = "--" if self.intercept_time is None else f"{self.intercept_time:.2f}s"
        return f"GRID | {self.role} | {self.action} | ETA {eta}"
