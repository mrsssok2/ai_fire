"""Seeded evaluation scenes: fires and decoys placed around the robot (at the origin) for the fusion evaluation.

Fixed scenarios (the set of objects is fixed, the seed moves them and picks the fire sizes):
  fires_only   2 fires
  decoys_only  a hot lamp, a sunlit window, an orange cone, a red box
  mixed        2 fires + a hot lamp, a sunlit window, a red barrel
"""
from __future__ import annotations

import math
import random

SCENARIOS = {
    "fires_only": {"fires": 2, "decoys": []},
    "decoys_only": {"fires": 0, "decoys": ["lamp", "window", "orange_cone", "red_box"]},
    "mixed": {"fires": 2, "decoys": ["lamp", "window", "red_barrel"]},
}
FIRE_DIST = (0.6, 1.3)       # m from the robot; flame sensors trip at ~1 m for an intensity-1 fire, so the far ones are out of their range
DECOY_DIST = (0.6, 1.2)


def make_scene(name: str, seed: int) -> dict:
    """{'fires': [...], 'decoys': [...]} in the fires.yaml format; objects spread evenly around the robot with jitter."""
    spec = SCENARIOS[name]
    rng = random.Random(f"{name}-{seed}")
    kinds = ["fire"] * spec["fires"] + list(spec["decoys"])
    rng.shuffle(kinds)
    n = len(kinds)
    base = rng.uniform(-math.pi, math.pi)
    fires, decoys = [], []
    for i, kind in enumerate(kinds):
        ang = base + 2 * math.pi * i / n + rng.uniform(-0.15, 0.15) * 2 * math.pi / n
        if kind == "fire":
            d = rng.uniform(*FIRE_DIST)
            fires.append({"id": f"f{len(fires) + 1}", "x": round(d * math.cos(ang), 3), "y": round(d * math.sin(ang), 3),
                          "intensity": round(rng.uniform(0.9, 1.3), 2), "gas_ppm": 0.0})
        else:
            d = rng.uniform(*DECOY_DIST)
            decoys.append({"id": f"d{len(decoys) + 1}", "kind": kind, "x": round(d * math.cos(ang), 3), "y": round(d * math.sin(ang), 3)})
    return {"fires": fires, "decoys": decoys}
