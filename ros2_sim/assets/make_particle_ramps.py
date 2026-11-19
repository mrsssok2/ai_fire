#!/usr/bin/env python3
"""Colour-over-lifetime ramps for Gazebo particle emitters (<color_range_image>, left = birth, right = death)."""
import os

import numpy as np
from PIL import Image

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "particles")


def ramp(stops, width=256):
    """stops: [(t, (r, g, b, a)), ...] with colours 0..1, linearly interpolated."""
    t = np.linspace(0, 1, width)
    xs = [s[0] for s in stops]
    ch = [np.interp(t, xs, [s[1][c] for s in stops]) for c in range(4)]
    return Image.fromarray((np.stack(ch, axis=1)[None] * 255).astype(np.uint8), "RGBA")


os.makedirs(OUT, exist_ok=True)
ramp([(0, (0.30, 0.30, 0.30, 0.0)), (0.15, (0.30, 0.30, 0.30, 0.18)), (0.6, (0.45, 0.45, 0.45, 0.12)), (1, (0.65, 0.65, 0.65, 0.0))]).save(f"{OUT}/smoke.png")
ramp([(0, (1.0, 0.95, 0.5, 1.0)), (0.4, (1.0, 0.5, 0.1, 0.9)), (1, (0.6, 0.1, 0.0, 0.0))]).save(f"{OUT}/spark.png")
ramp([(0, (0.75, 0.88, 1.0, 0.85)), (0.7, (0.75, 0.88, 1.0, 0.55)), (1, (0.8, 0.9, 1.0, 0.0))]).save(f"{OUT}/water.png")
ramp([(0, (0.9, 0.95, 1.0, 0.6)), (1, (0.9, 0.95, 1.0, 0.0))]).save(f"{OUT}/steam.png")
