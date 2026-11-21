#!/usr/bin/env python3
"""Original (blocking call / SMS, as in fire_sensor.ino) vs improved (call / SMS in the background) firmware on the default
3-fire mission (config/fires.yaml), several runs each.

Per run (headless, start_delay 0, flame sensors with 10 % seeded noise so runs differ the way real ones would; the same seeds
are used for both modes): sim time from mission start until all fires are out, water used (tank is 500 ml, pump 25 ml/s),
time the pump sprayed, number of calls, outcome (DONE / TANK_EMPTY / timeout).

    docker compose run --rm -T sim /home/ubuntu/ros2_sim/scripts/in_container.sh scripts/eval_alert_modes.py [--runs 5] [--speed 1.0]
Writes docs/results/alert_modes[TAG].json and .md (TAG from --tag; text after the line "<!-- observations -->" is kept).
"""
import argparse
import json
import os
import statistics
import sys
import time

import rclpy
from std_msgs.msg import Bool, Float32, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_harness import ROOT, Probe, Sim  # noqa: E402

NOISE = 0.0          # ideal sensors by default: the 3-fire chain layout is fragile (see docs); --noise to test it
SIM_TIMEOUT = 1200.0          # s of simulated time


class Mission(Probe):
    def __init__(self):
        super().__init__("eval_alert_modes")
        self.state, self.ml, self.spray_s, self.spraying, self.last_t = None, 500.0, 0.0, False, None
        self.t_start = self.t_end = None
        self.fire_out, self.calls, self.smss = {}, 0, 0
        self.create_subscription(String, "/mission/state", self.on_state, 10)
        self.create_subscription(Float32, "/water/ml", lambda m: setattr(self, "ml", m.data), 10)
        self.create_subscription(Bool, "/water/spraying", self.on_spray, 10)
        self.create_subscription(String, "/alerts", self.on_alert, 50)

    def on_state(self, m):
        self.state = json.loads(m.data)["state"]
        if self.state == "RUNNING" and self.t_start is None:
            self.t_start = self.sim_time
        if self.state in ("DONE", "TANK_EMPTY") and self.t_end is None:
            self.t_end = self.sim_time

    def on_spray(self, m):
        now = self.sim_time
        if self.spraying and self.last_t is not None:
            self.spray_s += now - self.last_t
        self.spraying, self.last_t = m.data, now

    def on_alert(self, m):
        e = json.loads(m.data)
        if e["type"] == "fire_out":
            self.fire_out[e["id"]] = round(e["t"] - (self.t_start or 0.0), 1)
        elif e["type"] == "call_start":
            self.calls += 1
        elif e["type"] == "sms":
            self.smss += 1


def run_one(firmware, seed, speed, outdir, noise, extinguish):
    sim = Sim("autonomous:=true", f"firmware:={firmware}", "start_delay:=0", "detection:=false", f"speed:={speed}", f"sensor_noise:={noise}", f"seed:={seed}",
              f"extinguish_time:={extinguish}", log=f"{outdir}/sim_{firmware}_{seed}.log")
    rclpy.init()
    try:
        p = Mission()
        if not p.spin_until(lambda: p.t_start is not None, 120):
            raise RuntimeError("mission did not start")
        wall0, sim0 = time.time(), p.sim_time
        last = [time.time()]

        def finished():
            if time.time() - last[0] > 30:
                last[0] = time.time()
                print(f"   ... {firmware} seed {seed}: sim {p.sim_time - p.t_start:.0f} s, wall {time.time() - wall0:.0f} s, water left {p.ml:.0f} ml, "
                      f"fires out {len(p.fire_out)}, state {p.state}", flush=True)
            return p.t_end is not None or p.sim_time - p.t_start > SIM_TIMEOUT
        done = p.spin_until(finished, 3600)
        p.spin_until(lambda: False, 1.0)
        outcome = p.state if p.t_end is not None else "TIMEOUT"
        r = {"firmware": firmware, "seed": seed, "outcome": outcome, "fires_out": len(p.fire_out), "fire_out_t": p.fire_out,
             "time_s": round((p.t_end or p.sim_time) - p.t_start, 1), "water_ml": round(500.0 - p.ml, 1),
             "spray_s": round(p.spray_s, 1), "calls": p.calls, "sms": p.smss,
             "rtf": round((p.sim_time - sim0) / max(1e-6, time.time() - wall0), 2)}
        p.destroy_node()
    finally:
        rclpy.shutdown()
        sim.stop()
    return r


def table(results):
    lines = ["| firmware | runs | all 3 fires out | time to put out all fires (simulated) | water used | pump on | pump duty | calls |",
             "|---|---|---|---|---|---|---|---|"]
    for fw, label in (("original", "original (blocking call, as in the .ino)"), ("improved", "improved (call / SMS in the background)")):
        rs = [r for r in results if r["firmware"] == fw]
        if not rs:
            continue
        ok = [r for r in rs if r["outcome"] == "DONE"]
        f = lambda key, rr: (f"{statistics.mean(r[key] for r in rr):.0f}" + (f" (sd {statistics.stdev(r[key] for r in rr):.0f})" if len(rr) > 1 else "")) if rr else "-"
        duty = statistics.mean(r["spray_s"] / r["time_s"] for r in rs) * 100
        lines.append(f"| {label} | {len(rs)} | {len(ok)} / {len(rs)} | {f('time_s', ok)} s | {f('water_ml', rs)} ml | {f('spray_s', rs)} s | {duty:.0f} % | {f('calls', rs)} |")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=5)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--noise", type=float, default=NOISE)
    ap.add_argument("--extinguish", type=float, default=4.0, help="s of full-strength spray that put a fire out (default 4)")
    ap.add_argument("--firmwares", nargs="*", default=["original", "improved"])
    ap.add_argument("--tag", default="", help="suffix of the result files, e.g. _2s")
    a = ap.parse_args()
    outdir = "/tmp/alert_eval"
    os.makedirs(outdir, exist_ok=True)
    results = []
    for seed in range(1, a.runs + 1):
        for fw in a.firmwares:
            t = time.time()
            r = run_one(fw, seed, a.speed, outdir, a.noise, a.extinguish)
            results.append(r)
            print(f"{fw} seed {seed} ({time.time() - t:.0f} s wall):", r, flush=True)
    docs = f"{ROOT}/docs/results"
    os.makedirs(docs, exist_ok=True)
    json.dump(results, open(f"{docs}/alert_modes{a.tag}.json", "w"), indent=1)
    keep = ""
    if os.path.exists(f"{docs}/alert_modes{a.tag}.md"):
        txt = open(f"{docs}/alert_modes{a.tag}.md").read()
        if "<!-- observations -->" in txt:
            keep = txt.split("<!-- observations -->", 1)[1]
    head = f"""# Blocking vs non-blocking alert (call / SMS)

Generated by `scripts/eval_alert_modes.py`; raw per-run data in `alert_modes{a.tag}.json`. The default 3-fire layout
(`config/fires.yaml`, {a.extinguish:g} s of full spray puts a fire out, 500 ml tank, 25 ml/s pump), {a.runs} run(s) per firmware, same seeds for both
(flame-sensor noise {a.noise:.0%}), headless. Times are simulated seconds from the start of the mission (the Arduino's 2 s `setup()` included).

*original* = `fire_sensor.ino` as ported (the call blocks 21 s after every spray burst). *improved* = same pins and timings, but
the call / SMS run as a background job (`program_improved`, launch `firmware:=improved`); not in the real `.ino`.

{table(results)}


<!-- observations -->{keep}"""
    open(f"{docs}/alert_modes{a.tag}.md", "w").write(head)
    print(table(results))


if __name__ == "__main__":
    main()
