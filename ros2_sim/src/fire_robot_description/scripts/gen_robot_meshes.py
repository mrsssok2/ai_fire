#!/usr/bin/env python3
"""Generate meshes/tube.obj: the clear water tube swept along config/robot.yaml `tube.path` (Catmull-Rom smoothed).

    python3 scripts/gen_robot_meshes.py       # numpy + pyyaml only
"""
import os

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))


def catmull_rom(pts, per_segment=10):
    p = np.vstack([pts[0], pts, pts[-1]])
    out = []
    for i in range(1, len(p) - 2):
        for t in np.linspace(0, 1, per_segment, endpoint=False):
            out.append(0.5 * ((2 * p[i]) + (-p[i - 1] + p[i + 1]) * t + (2 * p[i - 1] - 5 * p[i] + 4 * p[i + 1] - p[i + 2]) * t * t
                              + (-p[i - 1] + 3 * p[i] - 3 * p[i + 1] + p[i + 2]) * t ** 3))
    out.append(p[-2])
    return np.array(out)


def sweep(path, radius, sides=12):
    tang = np.gradient(path, axis=0)
    tang /= np.linalg.norm(tang, axis=1, keepdims=True)
    ref = np.array([0.0, 0.0, 1.0])
    rings, normals = [], []
    for p, t in zip(path, tang):
        a = np.cross(t, ref if abs(t @ ref) < 0.9 else np.array([1.0, 0, 0]))
        a /= np.linalg.norm(a)
        b = np.cross(t, a)
        th = np.linspace(0, 2 * np.pi, sides, endpoint=False)
        n = np.outer(np.cos(th), a) + np.outer(np.sin(th), b)
        rings.append(p + radius * n)
        normals.append(n)
    V, N, F = np.vstack(rings), np.vstack(normals), []
    for i in range(len(path) - 1):
        for j in range(sides):
            a, b = i * sides + j, i * sides + (j + 1) % sides
            c, d = (i + 1) * sides + j, (i + 1) * sides + (j + 1) % sides
            F += [(a, c, b), (b, c, d)]
    return V, N, F


def main():
    cfg = yaml.safe_load(open(os.path.join(HERE, "..", "config", "robot.yaml")))
    top = cfg["chassis"]["thickness"]
    pts = np.array(cfg["tube"]["path"], float) + np.array([0, 0, top])
    V, N, F = sweep(catmull_rom(pts), cfg["tube"]["radius"])
    out = os.path.join(HERE, "..", "meshes", "tube.obj")
    with open(out, "w") as f:
        for v in V:
            f.write("v %.5f %.5f %.5f\n" % tuple(v))
        for n in N:
            f.write("vn %.4f %.4f %.4f\n" % tuple(n))
        for a, b, c in F:
            f.write(f"f {a+1}//{a+1} {b+1}//{b+1} {c+1}//{c+1}\n")
    print("wrote", os.path.normpath(out), len(V), "vertices")


if __name__ == "__main__":
    main()
