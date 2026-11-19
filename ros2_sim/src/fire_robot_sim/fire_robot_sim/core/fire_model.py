"""Fire state: intensity, being put out by the water jet, smoke that fades last (pure Python)."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

SCALE_LEVELS = (1.0, 0.62, 0.32)       # flame billboard sizes (relative); fires shrink through these as they are sprayed


def scale_level(intensity: float):
    """Index into SCALE_LEVELS for a fire intensity in 0..1, or None when only embers are left."""
    if intensity >= 0.66:
        return 0
    if intensity >= 0.33:
        return 1
    if intensity >= 0.06:
        return 2
    return None


def jet_effectiveness(origin, jet_yaw, fire_xy, fire_radius, range_m, half_angle):
    """0..1: how well a water jet from `origin` along `jet_yaw` (a narrow horizontal cone) hits a fire disc."""
    dx, dy = fire_xy[0] - origin[0], fire_xy[1] - origin[1]
    d = math.hypot(dx, dy)
    if d > range_m + fire_radius or d < 1e-6:
        return 0.0
    off = abs((math.atan2(dy, dx) - jet_yaw + math.pi) % (2 * math.pi) - math.pi)
    if off > half_angle + math.asin(min(1.0, fire_radius / d)):
        return 0.0
    return max(0.3, 1.0 - d / (range_m + fire_radius))


@dataclass
class FireSim:
    id: str
    x: float
    y: float
    base_intensity: float = 1.0
    gas_ppm: float = 0.0
    radius: float = 0.14
    intensity: float = 1.0               # 0..1 of the base fire; 0 = out
    out: bool = False
    smoke: float = 1.0                   # 0..1, fades AFTER the flames are gone
    last_frame: int = -1
    shown: tuple | None = None           # (frame, level) of the visible flame variant
    hit_time: float = 0.0                # accumulated time under water (s)
    block_up: bool = True                # the solid footprint is in place (it is moved away once the fire is out)

    def step(self, dt, effectiveness, extinguish_time=4.0, regrow_per_s=0.004, smoke_fade_s=10.0):
        """Returns True on the step the fire goes out."""
        if self.out:
            self.smoke = max(0.0, self.smoke - dt / smoke_fade_s)
            return False
        if effectiveness > 0.0:
            self.hit_time += dt
            self.intensity -= dt * effectiveness / extinguish_time
        else:
            self.intensity = min(1.0, self.intensity + regrow_per_s * dt)
        if self.intensity <= 0.0:
            self.intensity, self.out = 0.0, True
            return True
        return False

    @property
    def effective_intensity(self) -> float:      # what the flame sensors see
        return self.base_intensity * self.intensity

    @property
    def effective_gas(self) -> float:
        return self.gas_ppm * self.intensity
