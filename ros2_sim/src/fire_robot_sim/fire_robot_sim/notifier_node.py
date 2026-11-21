"""Phone alerts: SIM800 events -> Telegram bot (optional: launch with alerts:=telegram).

Reads /alerts (the simulated SIM800L's SMS / call events, tank_empty), /mission/state, and the latest /fire/annotated frame
and sensor values for a short status. Sends: gas SMS, fire call (with the annotated frame), tank empty, mission done.
The SIM800 emulator itself is untouched. Credentials come ONLY from the environment (TELEGRAM_BOT_TOKEN, TELEGRAM_CHAT_ID);
they are never logged or written anywhere. Without them the node logs one error and does nothing.
`dry_run:=true` prints the messages instead of sending them (no credentials needed).
"""
import json
import os
import queue
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String

from fire_robot_sim.core.notify import RateLimiter, TelegramClient, TelegramError, build_message, status_line


class Notifier(Node):
    def __init__(self):
        super().__init__("notifier")
        self.declare_parameter("dry_run", False)
        self.declare_parameter("min_interval_s", 60.0)     # per alert kind
        self.declare_parameter("max_per_10min", 10)        # all kinds together
        token, chat = os.environ.get("TELEGRAM_BOT_TOKEN", "").strip(), os.environ.get("TELEGRAM_CHAT_ID", "").strip()
        self.dry = self.get_parameter("dry_run").value
        self.client = TelegramClient(token, chat) if token and chat else None
        self.enabled = self.dry or self.client is not None
        if not self.enabled:
            self.get_logger().error("alerts:=telegram but TELEGRAM_BOT_TOKEN / TELEGRAM_CHAT_ID are not set in the environment: "
                                    "no phone alerts will be sent (see ros2_sim/README.md)")
        self.limiter = RateLimiter(self.get_parameter("min_interval_s").value, self.get_parameter("max_per_10min").value)
        self.jobs = queue.Queue(maxsize=5)
        self.frame, self.frame_t = None, 0.0
        self.cnn, self.gas, self.water, self.flames, self.mission = None, None, None, {"L": False, "F": False, "R": False}, None
        self.create_subscription(String, "/alerts", self.on_alert, 20)
        self.create_subscription(String, "/mission/state", self.on_mission, 10)
        self.create_subscription(Image, "/fire/annotated", self.on_frame, 1)
        self.create_subscription(Float32, "/fire/cnn_conf", lambda m: setattr(self, "cnn", float(m.data)), 10)
        self.create_subscription(Float32, "/sensors/gas/ppm", lambda m: setattr(self, "gas", float(m.data)), 10)
        self.create_subscription(Float32, "/water_level", lambda m: setattr(self, "water", float(m.data)), 10)
        for key, name in (("L", "left"), ("F", "front"), ("R", "right")):
            self.create_subscription(Bool, f"/sensors/flame_{name}", lambda m, k=key: self.flames.__setitem__(k, bool(m.data)), 10)
        threading.Thread(target=self.worker, daemon=True).start()
        if self.enabled:
            self.get_logger().info("phone alerts ON" + (" (dry run: messages are only printed)" if self.dry else " (Telegram)"))

    def on_frame(self, msg):
        if msg.encoding == "rgb8":
            self.frame, self.frame_t = (msg.height, msg.width, bytes(msg.data)), time.monotonic()

    def jpeg(self):
        if self.frame is None or time.monotonic() - self.frame_t > 10.0:
            return None
        h, w, data = self.frame
        ok, buf = cv2.imencode(".jpg", cv2.cvtColor(np.frombuffer(data, np.uint8).reshape(h, w, 3), cv2.COLOR_RGB2BGR))
        return buf.tobytes() if ok else None

    def on_alert(self, msg):
        try:
            ev = json.loads(msg.data)
        except ValueError:
            return
        kind = {"sms": "gas_sms", "call_start": "fire_call", "tank_empty": "tank_empty"}.get(ev.get("type"))
        if kind:
            self.alert(kind)

    def on_mission(self, msg):
        state = json.loads(msg.data)["state"]
        if state != self.mission:
            if state == "DONE":
                self.alert("mission_done")
            elif state == "WAITING":                      # reset: the next mission may alert again
                self.limiter.rearm("tank_empty", "mission_done")
            self.mission = state

    def alert(self, kind):
        if not self.enabled:
            return
        if not self.limiter.allow(kind, time.monotonic()):
            self.get_logger().info(f"alert '{kind}' suppressed (rate limit)")
            return
        status = status_line(self.cnn, [k for k, v in self.flames.items() if v], self.gas, self.water)
        text, wants_photo = build_message(kind, status)
        n = self.limiter.take_suppressed(kind)
        if n:
            text += f"\n(+{n} similar alert(s) suppressed)"
        try:
            self.jobs.put_nowait((text, self.jpeg() if wants_photo else None))
        except queue.Full:
            self.get_logger().warn("alert queue full, dropped")

    def worker(self):
        while True:
            text, photo = self.jobs.get()
            if self.dry:
                self.get_logger().info(f"[dry run] would send{' with photo' if photo else ''}: {text!r}")
                continue
            try:
                (self.client.send_photo(photo, text) if photo else self.client.send_message(text))
                self.get_logger().info(f"sent to Telegram: {text.splitlines()[0]}")
            except TelegramError as e:
                self.get_logger().error(f"Telegram: {e}")


def main():
    rclpy.init()
    node = Notifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
