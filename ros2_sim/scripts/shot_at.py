#!/usr/bin/env python3
"""Put the docs camera at (x,y,z) looking at (lx,ly,lz) and save a PNG:  shot_at.py out.png x y z lx ly lz [frames]"""
import math
import os
import subprocess
import sys

out, v = sys.argv[1], [float(a) for a in sys.argv[2:8]]
(x, y, z), (lx, ly, lz) = v[:3], v[3:]
dx, dy, dz = lx - x, ly - y, lz - z
env = dict(os.environ, SHOT_FRAMES=sys.argv[8] if len(sys.argv) > 8 else "6")
subprocess.run([sys.executable, os.path.join(os.path.dirname(__file__), "shot.py"), out, str(x), str(y), str(z), "0",
                str(math.atan2(-dz, math.hypot(dx, dy))), str(math.atan2(dy, dx))], env=env, check=False)
