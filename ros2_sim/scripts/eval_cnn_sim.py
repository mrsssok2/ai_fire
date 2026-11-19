#!/usr/bin/env python3
"""How confidently does the CNN detect the simulated fires from the robot camera?

Run against a sim launched with  autonomous:=false monitor_window:=false  (the fires burn, the robot stays where it is put):
    python3 scripts/eval_cnn_sim.py [out.json]

The robot is teleported to a few views (facing fire f1 from different distances, off to the side, looking away) and the
CNN confidence (/fire/cnn_conf, fire = 1 - sigmoid output) is sampled for a few seconds in each. Then all fires are put
out (/fires/extinguish_all) and the same close view is measured again (charred logs + fading smoke, no flames).
"""
import json
import math
import os
import sys
import time

import rclpy
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.expanduser("~/ros2_sim/src/fire_robot_sim"))
from fire_robot_sim.gzio import quat_from_rpy  # noqa: E402

FIRE = (0.75, 0.45)


class Probe(Node):
    def __init__(self):
        super().__init__("eval_cnn_sim")
        self.conf, self.boxes = [], []
        self.create_subscription(Float32, "/fire/cnn_conf", lambda m: self.conf.append(m.data), 10)
        self.create_subscription(String, "/fire/detections", lambda m: self.boxes.append(len(json.loads(m.data)["boxes"])), 10)
        self.gz = GzNode()

    def put_robot(self, x, y, yaw):
        req = Pose_V()
        m = req.pose.add()
        m.name = "fire_bot"
        m.position.x, m.position.y, m.position.z = x, y, 0.045
        m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = quat_from_rpy(0, 0, yaw)
        for _ in range(5):
            ok, rep = self.gz.request("/world/fire_world/set_pose_vector", req, Pose_V, Boolean, 1000)
            if ok and rep.data:
                return
        raise RuntimeError("could not move the robot")

    def measure(self, seconds=5.0, settle=2.0):
        t0 = time.time()
        while time.time() - t0 < settle:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.conf.clear(), self.boxes.clear()
        while time.time() - t0 < settle + seconds:
            rclpy.spin_once(self, timeout_sec=0.1)
        c = self.conf
        return {"n": len(c), "mean": round(sum(c) / max(1, len(c)), 3), "min": round(min(c, default=0), 3), "max": round(max(c, default=0), 3),
                "fraction_ge_0.5": round(sum(v >= 0.5 for v in c) / max(1, len(c)), 2),
                "mean_boxes": round(sum(self.boxes) / max(1, len(self.boxes)), 2)}


def main():
    rclpy.init()
    p = Probe()
    theta = math.atan2(FIRE[1], FIRE[0])
    views = {}
    cases = [("facing f1, 1.6 m", 1.6, 0.0), ("facing f1, 1.0 m", 1.0, 0.0), ("facing f1, 0.6 m", 0.6, 0.0),
             ("f1 at the edge of the view (25 deg off), 1.0 m", 1.0, math.radians(25)),
             ("looking away from every fire", 1.0, math.pi)]
    for name, d, off in cases:
        p.put_robot(FIRE[0] - d * math.cos(theta), FIRE[1] - d * math.sin(theta), theta + off)
        views[name] = p.measure()
        print(name, views[name], flush=True)
    cli = p.create_client(Trigger, "/fires/extinguish_all")
    cli.wait_for_service(10)
    rclpy.spin_until_future_complete(p, cli.call_async(Trigger.Request()), timeout_sec=5)
    p.put_robot(FIRE[0] - 0.6 * math.cos(theta), FIRE[1] - 0.6 * math.sin(theta), theta)
    views["all fires out, facing f1, 0.6 m (charred logs, fading smoke)"] = p.measure(seconds=4.0, settle=3.0)
    print(list(views.items())[-1], flush=True)
    if len(sys.argv) > 1:
        json.dump(views, open(sys.argv[1], "w"), indent=1)


if __name__ == "__main__":
    main()
