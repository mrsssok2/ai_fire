"""Analytic flame and gas sensor models (simulation parameters, not measurements of the real modules).

Flame sensor: a cone of half-angle `half_angle` around the sensor axis; a fire of radius r is seen if the near edge of the pile
(off-axis angle minus asin(r / distance)) is inside the cone, so a fire right next to the robot is not lost in a dead zone. For each fire the signal is
    sqrt(intensity) * gain(off-axis angle) * (range / distance)^2
with gain = cos(90 deg * angle / half_angle) inside the cone, 0 outside. Signal >= 1 trips the (digital) sensor,
(sqrt: a flame that is mostly sprayed out is still a flame to an IR diode; only a fire that is out is invisible)
so `range` is the distance at which an intensity-1 fire straight ahead just trips it.

Gas sensor: concentration = sum of  gas_i * exp(-distance / decay)  ppm; trips above a threshold.
(Step 2 replaces this with a spreading gas cloud from fire_manager; the node interface stays the same.)
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Fire:
    id: str
    x: float
    y: float
    intensity: float = 1.0
    gas_ppm: float = 0.0        # gas concentration at the fire itself
    radius: float = 0.0         # size of the burning pile: close up a fire is an extended source, not a point


@dataclass(frozen=True)
class FlameSensor:
    x: float                    # mount in the robot frame
    y: float
    yaw: float                  # rad, 0 = straight ahead
    half_angle: float           # rad
    range_m: float


def to_world(rx: float, ry: float, ryaw: float, x: float, y: float, yaw: float = 0.0):
    c, s = math.cos(ryaw), math.sin(ryaw)
    return rx + c * x - s * y, ry + s * x + c * y, ryaw + yaw


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def flame_signal(sensor: FlameSensor, robot_pose, fires) -> float:
    sx, sy, syaw = to_world(*robot_pose, sensor.x, sensor.y, sensor.yaw)
    total = 0.0
    for f in fires:
        dx, dy = f.x - sx, f.y - sy
        d = max(math.hypot(dx, dy), 0.05)
        off = abs(wrap(math.atan2(dy, dx) - syaw))
        off = max(0.0, off - math.asin(min(1.0, f.radius / d)))      # the near edge of the pile counts, not only its centre
        if off >= sensor.half_angle:
            continue
        gain = math.cos(0.5 * math.pi * off / sensor.half_angle)
        total += math.sqrt(max(f.intensity, 0.0)) * gain * (sensor.range_m / d) ** 2
    return total


def gas_ppm(robot_pose, mount_xy, fires, decay_m: float = 0.5) -> float:
    gx, gy, _ = to_world(*robot_pose, *mount_xy)
    return sum(f.gas_ppm * math.exp(-math.hypot(f.x - gx, f.y - gy) / decay_m) for f in fires if f.gas_ppm > 0)
