"""Mission state machine (pure Python): WAITING (countdown) -> RUNNING -> DONE / TANK_EMPTY, with reset."""
from __future__ import annotations

import math

WAITING, RUNNING, DONE, TANK_EMPTY = "WAITING", "RUNNING", "DONE", "TANK_EMPTY"


class Mission:
    """Times are wall-clock seconds so the countdown is what the viewer sees, whatever the simulation speed."""

    def __init__(self, start_delay: float = 15.0):
        self.start_delay = start_delay
        self.state = WAITING
        self.t0 = None

    def reset(self, now: float):
        """Back to WAITING with a fresh countdown (the caller puts the robot back, refills the tank, relights the fires)."""
        self.state, self.t0 = WAITING, now

    def start(self):
        if self.state == WAITING:
            self.state = RUNNING

    def update(self, now: float, all_out: bool, tank_empty: bool) -> dict:
        if self.t0 is None:
            self.t0 = now
        if self.state == WAITING and now - self.t0 >= self.start_delay:
            self.state = RUNNING
        if self.state in (RUNNING, TANK_EMPTY, DONE):
            if all_out:
                self.state = DONE
            elif tank_empty and self.state == RUNNING:
                self.state = TANK_EMPTY
        countdown = max(0.0, self.start_delay - (now - self.t0)) if self.state == WAITING else 0.0
        return {"state": self.state, "countdown": round(countdown, 1), "message": self.message(countdown)}

    def message(self, countdown: float) -> str:
        return {WAITING: f"MISSION STARTS IN {math.ceil(countdown)} s",
                RUNNING: "",
                DONE: "MISSION DONE: all fires out",
                TANK_EMPTY: "TANK EMPTY: no water left"}[self.state]

    @property
    def robot_may_run(self) -> bool:
        return self.state != WAITING
