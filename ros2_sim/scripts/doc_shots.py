#!/usr/bin/env python3
"""Take the documentation screenshots from a running sim (launch with shot_camera:=true monitor_window:=false).

    doc_shots.py static     # autonomous:=false: robot close-up, fire close-up, full tank
    doc_shots.py action     # autonomous:=true: tank before, robot spraying + detection window, fire out + tank after
"""
import json
import math
import os
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String

OUT = os.path.expanduser("~/ros2_sim/docs/img")
SHOT = os.path.expanduser("~/ros2_sim/scripts/shot.py")


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Watch(Node):
    def __init__(self):
        super().__init__("doc_shots")
        self.pose, self.spraying, self.fires, self.water, self.alerts, self.ann = None, False, [], 100.0, [], None
        self.create_subscription(Odometry, "/fire_bot/odom", self.on_odom, 10)
        self.create_subscription(Bool, "/water/spraying", lambda m: setattr(self, "spraying", m.data), 10)
        self.create_subscription(String, "/fires/state", lambda m: setattr(self, "fires", json.loads(m.data)), 10)
        self.create_subscription(Float32, "/water_level", lambda m: setattr(self, "water", m.data), 10)
        self.create_subscription(String, "/alerts", lambda m: self.alerts.append(json.loads(m.data)), 10)
        self.create_subscription(Image, "/fire/annotated", lambda m: setattr(self, "ann", m), 1)
        self.pump_pub = self.create_publisher(Bool, "/pump/state", 10)

    def on_odom(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def wait(self, cond, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout and not cond():
            rclpy.spin_once(self, timeout_sec=0.1)
        return cond()


def shot(name, pos, look, topic="/shot/image", frames=6):
    """Camera at pos looking at `look` (both xyz). frames: camera frames to wait for after moving (5 Hz)."""
    os.environ["SHOT_FRAMES"] = str(frames)
    dx, dy, dz = (look[i] - pos[i] for i in range(3))
    yaw, pitch = math.atan2(dy, dx), math.atan2(-dz, math.hypot(dx, dy))
    subprocess.run([sys.executable, SHOT, f"{OUT}/{name}.png", *map(str, (*pos, 0.0, pitch, yaw)), topic], check=False)


def rel(pose, dx, dy, z):
    x, y, yaw = pose
    return (x + math.cos(yaw) * dx - math.sin(yaw) * dy, y + math.sin(yaw) * dx + math.cos(yaw) * dy, z)


def save_annotated(w, name):
    import cv2
    import numpy as np
    m = w.ann
    if m is not None:
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        cv2.imwrite(f"{OUT}/{name}.png", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))


def main():
    rclpy.init()
    w = Watch()
    w.wait(lambda: w.pose is not None, 60)
    if sys.argv[1] == "static":
        shot("robot_closeup", (0.50, 0.42, 0.26), (0.0, -0.01, 0.06))
        shot("robot_tank_full", (-0.05, -0.46, 0.20), (-0.105, -0.055, 0.07))
        shot("fire_closeup", (0.28, -0.05, 0.30), (0.75, 0.45, 0.15))
        return
    w.wait(lambda: False, 6.0)           # the water body settles at the top (and we keep spinning for fresh poses)
    shot("tank_before", rel(w.pose, -0.05, -0.30, 0.17), rel(w.pose, -0.105, -0.055, 0.06))
    print("tank before", w.water, flush=True)
    w.wait(lambda: w.spraying and any(f["intensity"] < 0.9 and not f["out"] for f in w.fires), 400)
    w.wait(lambda: not w.spraying, 10)   # let the Arduino's own spray burst finish, then spray on purpose for the picture
    pose = w.pose
    t_end = time.time() + 4.0
    while time.time() < t_end:           # hold the pump pin on for ~4 s so the jet is there when the camera is ready
        w.pump_pub.publish(Bool(data=True))
        rclpy.spin_once(w, timeout_sec=0.1)
        if time.time() > t_end - 2.6 and not os.path.exists(f"{OUT}/spraying.png.tmp"):
            open(f"{OUT}/spraying.png.tmp", "w").close()
            shot("spraying", rel(pose, -0.30, 0.42, 0.26), rel(pose, 0.40, -0.02, 0.07), frames=2)
            save_annotated(w, "detection_window_spraying")
    os.remove(f"{OUT}/spraying.png.tmp")
    w.pump_pub.publish(Bool(data=False))
    print("spraying; fires", [(f["id"], f["intensity"]) for f in w.fires], flush=True)
    w.wait(lambda: any(f["id"] == "f1" and f["out"] for f in w.fires), 900)
    w.wait(lambda: False, 6.0)
    pose = w.pose
    shot("tank_after", rel(pose, -0.05, -0.30, 0.17), rel(pose, -0.105, -0.055, 0.06))
    shot("fire_out", (0.75 - 0.9, 0.45 - 0.9, 0.55), (0.75, 0.45, 0.1))
    save_annotated(w, "detection_window_fire_out")
    print("fire out; water", w.water, "alerts", [a["type"] for a in w.alerts], flush=True)


if __name__ == "__main__":
    main()
