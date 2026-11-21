"""Fusion node: flame sensors trigger, the CNN confirms, confidence weighting, flame-sensor bearing for aiming (core/fusion.py).

In:   /sensors/flame_{left,front,right}/analog (Float32), /fire/detections (JSON from cnn_detector)
Out:  /fusion/state (JSON, 20 Hz: the three alarms flame / cnn / fused, p_flame, p_cnn, score, bearing_deg, signals)
      /fusion/alarm (Bool, the fused alarm), /fusion/bearing (Float32, degrees, robot frame, + = left; NaN = no flame)
The three alarms side by side are what scripts/eval_fusion.py compares. This node does not drive the robot.
"""
import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from fire_robot_sim.core.fusion import Fusion, FusionParams


class FusionNode(Node):
    def __init__(self):
        super().__init__("fusion")
        for name, default in (("w_flame", 0.4), ("w_cnn", 0.6), ("alarm_score", 0.6), ("cnn_hold_s", 1.0), ("trip", 1.0)):
            self.declare_parameter(name, default)
        g = lambda n: self.get_parameter(n).value
        self.fusion = Fusion(FusionParams(trip=g("trip"), w_flame=g("w_flame"), w_cnn=g("w_cnn"),
                                          alarm_score=g("alarm_score"), cnn_hold_s=g("cnn_hold_s")))
        self.signals = {"left": 0.0, "front": 0.0, "right": 0.0}
        self.state_pub = self.create_publisher(String, "/fusion/state", 10)
        self.alarm_pub = self.create_publisher(Bool, "/fusion/alarm", 10)
        self.bearing_pub = self.create_publisher(Float32, "/fusion/bearing", 10)
        for n in self.signals:
            self.create_subscription(Float32, f"/sensors/flame_{n}/analog", lambda m, n=n: self.signals.__setitem__(n, float(m.data)), 10)
        self.create_subscription(String, "/fire/detections", self.on_detections, 10)
        self.create_timer(0.05, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_detections(self, msg):
        d = json.loads(msg.data)
        self.fusion.add_cnn(self.now(), max([d["conf"]] + [b["conf"] for b in d["boxes"]]))

    def tick(self):
        st = self.fusion.update(self.now(), dict(self.signals))
        self.state_pub.publish(String(data=json.dumps(st)))
        self.alarm_pub.publish(Bool(data=st["fused_alarm"]))
        b = st["bearing_deg"]
        self.bearing_pub.publish(Float32(data=float("nan") if b is None else float(b)))


def main():
    rclpy.init()
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
