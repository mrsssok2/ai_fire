"""Flame-sensor + CNN fusion (pure Python).

  * the flame sensors TRIGGER: any analog level >= `trip` (that is what the digital module output does);
  * the CNN CONFIRMS: its fire confidence (best of the whole frame and the HSV-proposed boxes), held for `cnn_hold_s`
    because the fixed camera sees a fire only for a moment while the robot turns toward it;
  * confidence weighting: the two are mapped to 0..1 and mixed, score = w_cnn * p_cnn + w_flame * p_flame, with
    p_flame = s / (s + 1) (0.5 at the trip level, 0.75 at three times the trip level). A strong flame signal can make up
    for a so-so CNN reading and the other way round; with the default weights a flame at the trip level needs p_cnn >= 0.67;
  * alarm = flame sensors tripped AND score >= alarm_score;
  * aiming: the flame sensors give a bearing (signal-weighted mean of the sensor axes, robot frame, + = left), usable
    before the camera sees anything.

The weights were chosen up front (not tuned on the evaluation scenes).
"""
from __future__ import annotations

from dataclasses import dataclass, field

SENSOR_YAW_DEG = {"left": 40.0, "front": 0.0, "right": -40.0}


@dataclass
class FusionParams:
    trip: float = 1.0               # analog level at which a flame sensor's digital output trips
    w_flame: float = 0.4
    w_cnn: float = 0.6
    alarm_score: float = 0.6
    cnn_hold_s: float = 1.0
    cnn_alarm: float = 0.5          # CNN-alone alarm threshold (same as the detector node's `threshold`)
    min_bearing_signal: float = 0.2


def flame_bearing_deg(signals: dict, min_signal: float = 0.2):
    """Signal-weighted mean of the sensor axes (deg, robot frame, + = left); None if no sensor sees anything."""
    used = {n: s for n, s in signals.items() if s >= min_signal}
    if not used:
        return None
    return sum(SENSOR_YAW_DEG[n] * s for n, s in used.items()) / sum(used.values())


@dataclass
class Fusion:
    p: FusionParams = field(default_factory=FusionParams)
    _cnn: list = field(default_factory=list)       # (t, conf) readings within the hold window
    _last_cnn: float = 0.0

    def add_cnn(self, t: float, conf: float):
        self._cnn.append((t, conf))
        self._last_cnn = conf

    def update(self, t: float, signals: dict) -> dict:
        """signals: {'left': analog, 'front': analog, 'right': analog}. Returns the decision for time t."""
        p = self.p
        self._cnn = [(ti, c) for ti, c in self._cnn if t - ti <= p.cnn_hold_s]
        p_cnn = max((c for _, c in self._cnn), default=0.0)
        peak = max(signals.values(), default=0.0)
        tripped = peak >= p.trip
        p_flame = peak / (peak + 1.0) if peak > 0 else 0.0
        score = p.w_cnn * p_cnn + p.w_flame * p_flame
        return {"t": round(t, 3), "flame_alarm": tripped, "cnn_alarm": self._last_cnn >= p.cnn_alarm,
                "fused_alarm": tripped and score >= p.alarm_score,
                "p_flame": round(p_flame, 3), "p_cnn": round(p_cnn, 3), "score": round(score, 3),
                "bearing_deg": flame_bearing_deg(signals, p.min_bearing_signal) if tripped else None,
                "signals": {k: round(v, 3) for k, v in signals.items()}}
