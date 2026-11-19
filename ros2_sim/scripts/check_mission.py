#!/usr/bin/env python3
"""End-to-end check of start delay / Start / done / reset against a running sim (launch it with
`gui:=false monitor_window:=false start_delay:=60 speed:=1.0 extinguish_time:=2`).

    python3 scripts/check_mission.py
"""
import json
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, String


class Check(Node):
    def __init__(self):
        super().__init__("check_mission")
        self.state, self.pose, self.water, self.fires = None, None, 100.0, []
        self.create_subscription(String, "/mission/state", lambda m: setattr(self, "state", json.loads(m.data)), 10)
        self.create_subscription(Odometry, "/fire_bot/odom", lambda m: setattr(self, "pose", (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
        self.create_subscription(Float32, "/water_level", lambda m: setattr(self, "water", m.data), 10)
        self.create_subscription(String, "/fires/state", lambda m: setattr(self, "fires", json.loads(m.data)), 10)
        self.cmd = self.create_publisher(String, "/mission/command", 10)

    def wait(self, cond, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout and not cond():
            rclpy.spin_once(self, timeout_sec=0.1)
        return cond()

    def say(self, ok, text):
        print(("PASS " if ok else "FAIL ") + text, flush=True)
        if not ok:
            self.failed = True


def main():
    rclpy.init()
    c = Check()
    c.failed = False
    c.wait(lambda: c.state is not None and c.pose is not None, 60)
    c.say(c.state["state"] == "WAITING" and c.state["countdown"] > 0, f"starts WAITING with a countdown ({c.state})")
    p0 = c.pose
    c.wait(lambda: False, 8)
    c.say(math.hypot(c.pose[0] - p0[0], c.pose[1] - p0[1]) < 0.01, "robot stays still while waiting")
    c.cmd.publish(String(data="start"))
    c.say(c.wait(lambda: c.state["state"] == "RUNNING", 5), "Start command ends the countdown at once")
    c.say(c.wait(lambda: math.hypot(c.pose[0] - p0[0], c.pose[1] - p0[1]) > 0.05, 30), "robot moves after the start")
    t0 = time.time()
    ok = c.wait(lambda: c.state["state"] in ("DONE", "TANK_EMPTY"), 1500)
    c.say(ok and c.state["state"] == "DONE", f"mission ends in DONE after {time.time() - t0:.0f} s: {c.state}")
    c.say(all(f["out"] for f in c.fires), "all fires are out")
    c.wait(lambda: False, 20)
    c.say(c.state["state"] == "DONE" and c.pose is not None, "still running 20 s later (nothing closed or reset by itself)")
    used = 100.0 - c.water
    c.cmd.publish(String(data="reset"))
    c.say(c.wait(lambda: c.state["state"] == "WAITING", 5), "Reset goes back to WAITING")
    c.wait(lambda: False, 4)
    c.say(math.hypot(*c.pose) < 0.05, f"robot back at the start {c.pose}")
    c.say(c.water > 99.0, f"tank refilled ({used:.0f} % had been used)")
    c.say(all(not f["out"] and f["intensity"] > 0.95 for f in c.fires), "fires relit")
    c.cmd.publish(String(data="start"))
    c.wait(lambda: False, 3)
    c.say(c.state["state"] == "RUNNING", "can start again")
    c.say(c.wait(lambda: math.hypot(*c.pose) > 0.05, 30), "robot moves again after the reset")
    sys.exit(1 if c.failed else 0)


if __name__ == "__main__":
    main()
