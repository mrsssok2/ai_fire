"""Water tank + pump: the level only drops while the pump sprays (pure Python)."""
from __future__ import annotations


class Tank:
    def __init__(self, capacity_ml=500.0, flow_ml_s=25.0, level_ml=None):
        self.capacity_ml, self.flow_ml_s = capacity_ml, flow_ml_s
        self.level_ml = capacity_ml if level_ml is None else level_ml
        self.empty_reported = False

    @property
    def percent(self) -> float:
        """Like the MCP3008 reading in fire.py (`wt.value * 100`)."""
        return 100.0 * self.level_ml / self.capacity_ml

    def step(self, dt, pump_on):
        """Returns (spraying, became_empty). Water is only used while the pump runs AND there is water."""
        spraying = bool(pump_on) and self.level_ml > 0.0
        became_empty = False
        if spraying:
            self.level_ml = max(0.0, self.level_ml - self.flow_ml_s * dt)
            if self.level_ml <= 0.0 and not self.empty_reported:
                became_empty = self.empty_reported = True
        return spraying, became_empty

    def refill(self):
        self.level_ml, self.empty_reported = self.capacity_ml, False
