"""Live "Fire Detection" window (like the cv2.imshow window in hsv.py), with START MISSION / RESET buttons (also Enter/Space and R): the robot camera with a status banner and boxes
around fire regions from the CNN, plus a side panel with the three flame sensors, gas, water tank, pump, servo angle
and the last alert. Also published as the annotated image topic /fire/annotated."""
import json
import math
import os

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Float64, String

from fire_robot_sim.cnn_detector import image_to_rgb
from fire_robot_sim.core import overlay


def describe_alert(ev):
    kind = ev.get("type")
    if kind == "sms":
        return f'SMS to {ev["to"]}: "{ev["text"]}"'
    if kind == "call_start":
        return f'CALL to {ev["to"]}: dialing'
    if kind == "call_end":
        return f'CALL ended after {ev.get("duration_s", 0):.0f} s'
    if kind == "tank_empty":
        return "TANK EMPTY: no water left"
    if kind == "fire_out":
        return f'fire {ev["id"]} is OUT'
    return json.dumps(ev)


class FireMonitor(Node):
    def __init__(self):
        super().__init__("fire_monitor")
        self.declare_parameter("show", True)
        self.declare_parameter("threshold", 0.5)
        self.declare_parameter("window_x", 1000)
        self.declare_parameter("window_y", 60)
        self.s = {"flame": {}, "analog": {}, "gas": False, "ppm": 0.0, "water": 100.0, "ml": 0.0, "pump": False, "servo": 90.0,
                  "alert": "none yet"}
        self.conf, self.dets, self.mission = 0.0, [], None
        self.cmd_pub = self.create_publisher(String, "/mission/command", 10)
        self.cam_height = 480
        self.show = self.get_parameter("show").value and bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        self.pub = self.create_publisher(Image, "/fire/annotated", 1)
        for k in ("left", "front", "right"):
            self.create_subscription(Bool, f"/sensors/flame_{k}", lambda m, k=k: self.s["flame"].__setitem__(k, m.data), 10)
            self.create_subscription(Float32, f"/sensors/flame_{k}/analog", lambda m, k=k: self.s["analog"].__setitem__(k, m.data), 10)
        self.create_subscription(Bool, "/sensors/gas", lambda m: self.s.__setitem__("gas", m.data), 10)
        self.create_subscription(Float32, "/sensors/gas/ppm", lambda m: self.s.__setitem__("ppm", m.data), 10)
        self.create_subscription(Float32, "/water_level", lambda m: self.s.__setitem__("water", m.data), 10)
        self.create_subscription(Float32, "/water/ml", lambda m: self.s.__setitem__("ml", m.data), 10)
        self.create_subscription(Bool, "/pump/state", lambda m: self.s.__setitem__("pump", m.data), 10)
        self.create_subscription(Float64, "/nozzle/cmd", lambda m: self.s.__setitem__("servo", math.degrees(m.data) + 90), 10)
        self.create_subscription(String, "/alerts", self.on_alert, 10)
        self.create_subscription(Float32, "/fire/cnn_conf", lambda m: setattr(self, "conf", m.data), 10)
        self.create_subscription(String, "/fire/detections", lambda m: setattr(self, "dets", json.loads(m.data)["boxes"]), 10)
        self.create_subscription(String, "/mission/state", lambda m: setattr(self, "mission", json.loads(m.data)), 10)
        self.create_subscription(Image, "/camera/image_raw", self.on_image, 1)
        if self.show:
            cv2.namedWindow("Fire Detection", cv2.WINDOW_AUTOSIZE)
            cv2.moveWindow("Fire Detection", self.get_parameter("window_x").value, self.get_parameter("window_y").value)
            cv2.setMouseCallback("Fire Detection", self.on_mouse)

    def command(self, cmd, source=""):
        self.get_logger().info(f"window command: {cmd} ({source})")
        self.cmd_pub.publish(String(data=cmd))
        if cmd == "reset":
            self.s["alert"] = "mission reset"

    def on_mouse(self, event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            key = overlay.button_at(x, y, self.cam_height)
            if key:
                self.command(key, f"mouse at {x},{y}")

    def on_alert(self, msg):
        try:
            self.s["alert"] = describe_alert(json.loads(msg.data))
        except ValueError:
            pass

    def on_image(self, msg):
        frame = overlay.compose(image_to_rgb(msg), self.conf, self.dets, self.s, self.mission, self.get_parameter("threshold").value)
        self.cam_height = msg.height * 2
        out = Image()
        out.header, out.height, out.width, out.encoding = msg.header, frame.shape[0], frame.shape[1], "rgb8"
        out.step = frame.shape[1] * 3
        out.data = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB).tobytes()
        self.pub.publish(out)
        if self.show:
            cv2.imshow("Fire Detection", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (13, 10, 32, ord("s")):
                self.command("start", f"key {key}")
            elif key in (ord("r"), ord("R")):
                self.command("reset", f"key {key}")


def main():
    rclpy.init()
    node = FireMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
