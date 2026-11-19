"""Mission control: the robot stays idle until the countdown ends or you press Start; nothing shuts down afterwards
(MISSION DONE / TANK EMPTY are only messages); Reset puts the robot back, refills the tank and relights the fires.

  /mission/command  String: "start" | "reset"      (terminal Enter / r, window keys and buttons)
  /mission/state    String JSON {state, countdown, message}
  /mission/running  Bool (latched): False while WAITING; the Arduino program only runs while True
Also moves the Gazebo GUI camera to follow the robot (parameter `follow`).
"""
import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from fire_robot_sim.core.mission import DONE, Mission


class MissionNode(Node):
    def __init__(self):
        super().__init__("mission")
        self.declare_parameter("start_delay", 15.0)
        self.declare_parameter("follow", True)             # Gazebo GUI camera follows the robot
        self.declare_parameter("follow_offset", [-0.75, -0.95, 0.80])
        self.declare_parameter("visuals", True)
        self.mission = Mission(self.get_parameter("start_delay").value)
        self.all_out, self.tank_empty, self.fires_seen = False, False, False
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, "/mission/state", 10)
        self.running_pub = self.create_publisher(Bool, "/mission/running", latched)
        self.create_subscription(String, "/mission/command", self.on_command, 10)
        self.create_subscription(String, "/fires/state", self.on_fires, 10)
        self.create_subscription(String, "/alerts", self.on_alert, 10)
        self.refill = self.create_client(Trigger, "/water/refill")
        self.relight = self.create_client(Trigger, "/fires/relight")
        self.gz = None
        if self.get_parameter("visuals").value:
            try:
                from fire_robot_sim.gzio import GzIO
                self.gz = GzIO()
            except Exception as exc:                         # pragma: no cover
                self.get_logger().warn(f"no gz transport: {exc}")
        self.last_running = None
        self.create_timer(0.2, self.tick)
        if self.get_parameter("follow").value:
            threading.Thread(target=self.camera_follow, daemon=True).start()
        d = self.get_parameter("start_delay").value
        self.get_logger().info(f"mission starts in {d:.0f} s (Enter in the terminal, space/Enter or the START button in the window starts it now)")

    def on_fires(self, msg):
        fires = json.loads(msg.data)
        self.fires_seen = bool(fires)
        self.all_out = bool(fires) and all(f["out"] for f in fires)

    def on_alert(self, msg):
        try:
            if json.loads(msg.data).get("type") == "tank_empty":
                self.tank_empty = True
        except ValueError:
            pass

    def on_command(self, msg):
        cmd = msg.data.strip().lower()
        if cmd in ("start", ""):
            self.mission.start()
            self.get_logger().info("mission started by command")
        elif cmd in ("reset", "r"):
            self.do_reset()

    def do_reset(self):
        self.get_logger().warn("RESET: robot back to the start, tank refilled, fires relit")
        self.mission.reset(time.monotonic())
        self.tank_empty = False
        for cli in (self.refill, self.relight):
            if cli.service_is_ready():
                cli.call_async(Trigger.Request())
        if self.gz is not None:
            try:
                self.gz.set_poses([("fire_bot", 0.0, 0.0, 0.045, 0.0, 0.0, 0.0)])
            except Exception as exc:                         # pragma: no cover
                self.get_logger().warn(f"could not move the robot: {exc}")

    def tick(self):
        st = self.mission.update(time.monotonic(), self.all_out, self.tank_empty)
        self.state_pub.publish(String(data=json.dumps(st)))
        running = self.mission.robot_may_run
        if running != self.last_running:
            self.running_pub.publish(Bool(data=running))
            self.last_running = running
            if running:
                self.get_logger().info("MISSION RUNNING")

    def camera_follow(self):
        """Ask the Gazebo GUI camera to follow the robot, a short distance behind-left and above (retries until the GUI is up)."""
        try:
            from gz.msgs10.boolean_pb2 import Boolean
            from gz.msgs10.stringmsg_pb2 import StringMsg
            from gz.msgs10.vector3d_pb2 import Vector3d
            from gz.transport13 import Node as GzNode
        except Exception as exc:                             # pragma: no cover
            self.get_logger().warn(f"camera follow unavailable: {exc}")
            return
        node, off = GzNode(), self.get_parameter("follow_offset").value
        for _ in range(60):
            time.sleep(2.0)
            req = StringMsg()
            req.data = "fire_bot"
            ok, rep = node.request("/gui/follow", req, StringMsg, Boolean, 1000)
            if ok and rep.data:
                v = Vector3d()
                v.x, v.y, v.z = off
                node.request("/gui/follow/offset", v, Vector3d, Boolean, 1000)
                self.get_logger().info("Gazebo camera follows the robot")
                return


def main():
    rclpy.init()
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
