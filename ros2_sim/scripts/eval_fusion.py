#!/usr/bin/env python3
"""Flame sensors alone vs CNN alone vs fused, on fixed scenarios with decoys (fires only / decoys only / mixed).

For every scenario and seed a fresh headless simulation is started with a seeded scene (core/scenes.py): fires and decoys
(red box / barrel, orange cone, sunlit window, hot lamp) spread around the robot, flame sensors with 10 % noise. The robot
then turns on the spot through 360 deg (0.6 rad/s) while the fusion node publishes the three alarms side by side
(/fusion/state: same sensor + camera data, three decision rules). Ground truth (scene + odometry) decides what every alarm
was caused by:
  * flame-sensor alarm  -> the object contributing most to the (noise-free) sensor signal;
  * CNN alarm           -> the object closest to the camera axis inside the camera's field of view (none = background);
  * fused alarm         -> the object that tripped the flame sensors (the CNN only confirms).
Alarm samples are grouped per source into events (gap > 1 s starts a new one, events shorter than 0.2 s are ignored):
an event on a fire = detection of that fire; an event on a decoy or on nothing = false alarm. A fire without any event
during the full turn is missed. Time to detect = from the start of the turn to the first detection of that fire.

    docker compose run --rm -T sim /home/ubuntu/ros2_sim/scripts/in_container.sh scripts/eval_fusion.py [--seeds 6] [--speed 1.0]
Writes docs/results/fusion_eval.json and docs/results/fusion_eval.md (text after the line '<!-- observations -->' is kept).
"""
import argparse
import json
import math
import os
import statistics
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String

sys.path.insert(0, os.path.expanduser("~/ros2_sim/src/fire_robot_sim"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_harness import ROBOT_YAML, ROOT, Probe, Sim  # noqa: E402

from fire_robot_sim import worldgen  # noqa: E402
from fire_robot_sim.core.scenes import SCENARIOS, make_scene  # noqa: E402
from fire_robot_sim.core.sensors import Fire, FlameSensor, flame_signal, wrap  # noqa: E402
from fire_robot_sim.fire_sensor_sim import yaw_of  # noqa: E402

METHODS = ("flame", "cnn", "fused")
OMEGA = 0.6                    # rad/s turn rate
CAM_HALF_FOV = 0.525           # rad (horizontal_fov 1.05 in the URDF)
OBJ_RADIUS = {"fire": 0.17, "red_box": 0.1, "red_barrel": 0.1, "orange_cone": 0.1, "window": 0.48, "lamp": 0.1}   # apparent half-size, m
NOISE = 0.10
EVENT_GAP, MIN_EVENT = 1.0, 0.2


class Sweep(Probe):
    def __init__(self):
        super().__init__("eval_fusion")
        self.samples, self.pose, self.yaw_unwrapped, self.last_yaw, self.cnn_msgs, self.rec = [], None, 0.0, None, 0, False
        self.create_subscription(Odometry, "/fire_bot/odom", self.on_odom, 10)
        self.create_subscription(String, "/fusion/state", self.on_state, 50)
        self.create_subscription(Float32, "/fire/cnn_conf", lambda m: setattr(self, "cnn_msgs", self.cnn_msgs + 1), 10)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)

    def on_odom(self, m):
        p = m.pose.pose
        yaw = yaw_of(p.orientation)
        if self.last_yaw is not None:
            self.yaw_unwrapped += wrap(yaw - self.last_yaw)
        self.last_yaw = yaw
        self.pose = (p.position.x, p.position.y, yaw)

    def on_state(self, m):
        if self.rec and self.pose is not None:
            st = json.loads(m.data)
            st["pose"] = self.pose
            st["yaw_total"] = self.yaw_unwrapped
            self.samples.append(st)

    def turn(self):
        self.rec, yaw0, t0 = True, self.yaw_unwrapped, self.sim_time
        tw = Twist()
        tw.angular.z = OMEGA
        while self.yaw_unwrapped - yaw0 < 2 * math.pi + 0.05 and self.sim_time - t0 < 25:
            self.cmd.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cmd.publish(Twist())
        self.rec = False
        return t0


def sensors():
    cfg = yaml.safe_load(open(ROBOT_YAML))["flame_sensors"]
    return [FlameSensor(cfg[n]["x"], cfg[n]["y"], math.radians(cfg[n]["yaw_deg"]), math.radians(cfg["half_angle_deg"]), cfg["range_m"])
            for n in ("left", "front", "right")]


def flame_sources(scene):
    """Noise-free flame sources of a scene: id -> Fire (fires and IR decoys)."""
    src = {f["id"]: Fire(f["id"], f["x"], f["y"], f["intensity"], radius=0.14 * math.sqrt(f["intensity"])) for f in scene["fires"]}
    for d in scene["decoys"]:
        if worldgen.decoy_ir(d) > 0:
            src[d["id"]] = Fire(d["id"], d["x"], d["y"], worldgen.decoy_ir(d), radius=0.05)
    return src


def attribute(scene, sample, method, sens, src):
    """What caused this sample's alarm: an object id or None."""
    pose = tuple(sample["pose"])
    flame_src, best = None, 0.3
    for oid, f in src.items():
        s = max(flame_signal(se, pose, [f]) for se in sens)
        if s > best:
            flame_src, best = oid, s
    if method in ("flame", "fused"):
        return flame_src
    x, y, yaw = pose
    cam, near = None, 1e9
    for o in scene["fires"] + scene["decoys"]:           # nearest edge-to-axis angle among the objects (partly) in the camera's field of view
        d = max(math.hypot(o["x"] - x, o["y"] - y), 0.05)
        half = math.asin(min(1.0, OBJ_RADIUS.get(o.get("kind", "fire"), 0.1) * (math.sqrt(o["intensity"]) if "intensity" in o else 1.0) / d))
        b = abs(wrap(math.atan2(o["y"] - y, o["x"] - x) - yaw)) - half
        if b < CAM_HALF_FOV and b < near:
            cam, near = o["id"], b
    return cam


def events(samples, method, scene, sens, src):
    """[(source id or None, t_start, t_end, [samples])] for one alarm rule."""
    key = f"{method}_alarm"
    per = {}
    for s in samples:
        if s[key]:
            per.setdefault(attribute(scene, s, method, sens, src), []).append(s)
    out = []
    for oid, ss in per.items():
        cur = [ss[0]]
        for s in ss[1:]:
            if s["t"] - cur[-1]["t"] > EVENT_GAP:
                out.append((oid, cur))
                cur = []
            cur.append(s)
        out.append((oid, cur))
    return [(oid, c[0]["t"], c[-1]["t"], c) for oid, c in out if c[-1]["t"] - c[0]["t"] >= MIN_EVENT]


def analyse(scene, samples, t0):
    if samples:                     # exactly one full turn: the overshoot after the stop command would show the start view twice
        samples = [x for x in samples if x["yaw_total"] - samples[0]["yaw_total"] <= 2 * math.pi]
    sens, src = sensors(), flame_sources(scene)
    fire_ids = {f["id"] for f in scene["fires"]}
    res = {}
    for m in METHODS:
        ev = events(samples, m, scene, sens, src)
        first = {}
        for oid, ts, _, _ in ev:
            if oid in fire_ids:
                first[oid] = min(first.get(oid, 1e9), ts - t0)
        false = [(oid, ts) for oid, ts, _, _ in ev if oid not in fire_ids]
        decoy_hits = {oid for oid, _ in false if oid is not None}
        false_count = len(decoy_hits) + sum(oid is None for oid, _ in false)      # one per decoy that raised an alarm + each background event
        aim = []
        for oid, _, _, ss in ev:
            if oid in fire_ids:
                f = next(f for f in scene["fires"] if f["id"] == oid)
                for s in ss:
                    if s["bearing_deg"] is not None:
                        x, y, yaw = s["pose"]
                        aim.append(abs(s["bearing_deg"] - math.degrees(wrap(math.atan2(f["y"] - y, f["x"] - x) - yaw))))
        evlist = [{"source": str(oid), "t": round(ts - t0, 2), "dur": round(te - ts, 2), "max_p_cnn": max(x["p_cnn"] for x in ss),
                   "max_flame_signal": round(max(max(x["signals"].values()) for x in ss), 2)} for oid, ts, te, ss in ev]
        res[m] = {"events": evlist, "fires": len(fire_ids), "detected": len(first), "missed": len(fire_ids) - len(first),
                  "false_alarms": false_count, "decoys_alarming": len(decoy_hits), "decoys": len(scene["decoys"]),
                  "false_events_raw": len(false), "false_sources": sorted({str(o) for o, _ in false}),
                  "ttd": sorted(first.values()), "aim_err_deg": round(statistics.mean(aim), 1) if aim else None}
    return res


def run_one(name, seed, speed, outdir):
    scene = make_scene(name, seed)
    path = f"{outdir}/scene_{name}_{seed}.yaml"
    yaml.safe_dump(scene, open(path, "w"))
    sim = Sim("autonomous:=false", "start_delay:=0", f"speed:={speed}", f"fires:={path}", f"sensor_noise:={NOISE}", f"seed:={seed}",
              log=f"{outdir}/sim_{name}_{seed}.log")
    rclpy.init()
    try:
        p = Sweep()
        ready = p.spin_until(lambda: p.pose is not None and p.cnn_msgs >= 3, 120)
        if not ready:
            raise RuntimeError("simulation did not come up")
        p.spin_until(lambda: False, 2.0)
        t0 = p.turn()
        t_wall = time.time()
        samples = p.samples
        import gzip
        os.makedirs(f"{ROOT}/log", exist_ok=True)
        with gzip.open(f"{ROOT}/log/fusion_samples_{name}_{seed}.json.gz", "wt") as fh:
            json.dump({"t0": t0, "samples": samples}, fh)
        p.destroy_node()
    finally:
        rclpy.shutdown()
        sim.stop()
    return scene, samples, t0


def table(results):
    lines = []
    for name in SCENARIOS:
        runs = [r for r in results if r["scenario"] == name]
        if not runs:
            continue
        spec = SCENARIOS[name]
        lines.append(f"\n### {name.replace('_', ' ')}  ({spec['fires']} fire(s), decoys: {', '.join(spec['decoys']) or 'none'}; {len(runs)} runs)\n")
        lines.append("| method | fires | detected | **missed** | decoys that raised an alarm | **false alarms** (decoys + background events) | runs with a false alarm | mean time to detect | aim error |")
        lines.append("|---|---|---|---|---|---|---|---|---|")
        for m, label in (("flame", "flame sensors alone"), ("cnn", "CNN alone"), ("fused", "fused")):
            per = [r["methods"][m] for r in runs]
            fires, det = sum(x["fires"] for x in per), sum(x["detected"] for x in per)
            ttd = [t for x in per for t in x["ttd"]]
            aim = [x["aim_err_deg"] for x in per if x["aim_err_deg"] is not None and m != "cnn"]
            lines.append(f"| {label} | {fires} | {det} | {fires - det} | {sum(x['decoys_alarming'] for x in per)} / {sum(x['decoys'] for x in per)} | {sum(x['false_alarms'] for x in per)} | "
                         f"{sum(x['false_alarms'] > 0 for x in per)} / {len(per)} | {f'{statistics.mean(ttd):.1f} s' if ttd else '-'} | "
                         f"{f'{statistics.mean(aim):.0f} deg' if aim else '-'} |")
    return "\n".join(lines)


def by_kind(results):
    kinds = {}
    for r in results:
        for d in r["scene"]["decoys"]:
            k = kinds.setdefault(d["kind"], {"n": 0, **{m: 0 for m in METHODS}})
            k["n"] += 1
            for m in METHODS:
                k[m] += d["id"] in r["methods"][m]["false_sources"]
    lines = ["\n### False alarms by decoy kind (all scenarios; decoys that raised an alarm / decoys present)\n",
             "| decoy | flame sensors alone | CNN alone | fused |", "|---|---|---|---|"]
    for kind in ("red_box", "red_barrel", "orange_cone", "window", "lamp"):
        if kind in kinds:
            k = kinds[kind]
            lines.append(f"| {kind.replace('_', ' ')} | {k['flame']} / {k['n']} | {k['cnn']} / {k['n']} | {k['fused']} / {k['n']} |")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=6)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--scenarios", nargs="*", default=list(SCENARIOS))
    ap.add_argument("--report-only", action="store_true", help="rewrite the .md from the saved .json")
    a = ap.parse_args()
    outdir = "/tmp/fusion_eval"
    os.makedirs(outdir, exist_ok=True)
    results = json.load(open(f"{ROOT}/docs/results/fusion_eval.json")) if a.report_only else []
    for name in ([] if a.report_only else a.scenarios):
        for seed in range(1, a.seeds + 1):
            t = time.time()
            scene, samples, t0 = run_one(name, seed, a.speed, outdir)
            r = analyse(scene, samples, t0)
            results.append({"scenario": name, "seed": seed, "scene": scene, "methods": r, "n_samples": len(samples)})
            print(f"{name} seed {seed} ({time.time() - t:.0f} s wall, {len(samples)} samples):",
                  {m: (x["detected"], x["fires"], x["false_alarms"]) for m, x in r.items()}, flush=True)
    docs = f"{ROOT}/docs/results"
    os.makedirs(docs, exist_ok=True)
    if not a.report_only:
        json.dump(results, open(f"{docs}/fusion_eval.json", "w"), indent=1)
    keep = ""
    if os.path.exists(f"{docs}/fusion_eval.md"):
        txt = open(f"{docs}/fusion_eval.md").read()
        if "<!-- observations -->" in txt:
            keep = txt.split("<!-- observations -->", 1)[1]
    head = f"""# Fusion evaluation: flame sensors vs CNN vs fused

Generated by `scripts/eval_fusion.py`; raw per-run data in `fusion_eval.json`. {a.seeds} seeds per scenario, a fresh Gazebo
run each (real simulated camera + my CNN + simulated flame sensors with {NOISE:.0%} signal noise).

**Protocol.** The robot starts at the origin and turns once on the spot (360 deg at {OMEGA} rad/s, about {2 * math.pi / OMEGA:.0f} s) with fires and decoys
placed around it at 0.6-1.3 m (seeded). Three alarm rules run on the same sensor and camera data:
*flame sensors alone* (any sensor's digital output trips), *CNN alone* (confidence >= 0.5 on the latest frame, whole frame or an
HSV-proposed box), *fused* (flame sensors tripped AND weighted score 0.4 * p_flame + 0.6 * p_cnn >= 0.6, CNN reading held 1 s; weights
fixed beforehand, not tuned on these scenes; see `core/fusion.py`). Decoys: red box / barrel and orange cone (look like fire, no IR),
sunlit window (bright, IR 0.8 of an intensity-1 fire), hot lamp (bright orange bulb, IR 1.6). A flame-sensor range of ~1 m for an intensity-1 fire is my
assumption, so fires placed farther than that are out of the sensors' reach by construction.

**Counting.** Alarm samples are grouped by what caused them (ground truth) into events. An event on a fire is a detection; an event on a
decoy or on nothing is a **false alarm** (counted once per decoy that raised an alarm in a run, plus once per event on nothing); a fire with no event during the full turn is **missed**. Time to detect is from the start of the
turn to the first detection of that fire. Aim error: |flame-sensor bearing - true bearing| over the detection samples (the bearing is a weighted mean
of the three sensor axes, so it is coarse).

Seeds only change positions, fire sizes and noise, not the CNN, so the numbers are a small sample, not a benchmark.
{table(results)}
{by_kind(results)}

<!-- observations -->{keep}"""
    open(f"{docs}/fusion_eval.md", "w").write(head)
    print(table(results))


if __name__ == "__main__":
    main()
