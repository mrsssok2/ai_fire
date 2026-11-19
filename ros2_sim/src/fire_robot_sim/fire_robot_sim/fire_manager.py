"""Fire manager: fire intensity, flicker, going out under the water jet, smoke that fades last.

Drives the world through gz transport (set_pose_vector: swap the flame variant models, light_config: flicker,
particle emitter messages: flame / smoke rates). Publishes /fires/state (JSON) for the sensors and the monitor and
/alerts events {"type": "fire_out"}.
"""
import json
import math
import random

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from fire_robot_sim import worldgen
from fire_robot_sim.core.fire_model import FireSim, jet_effectiveness, scale_level
from fire_robot_sim.core.geometry import nozzle_pose
from fire_robot_sim.fire_sensor_sim import yaw_of
from fire_robot_sim.gzio import GzIO


class FireManager(Node):
    def __init__(self):
        super().__init__("fire_manager")
        self.declare_parameter("robot_yaml", "")
        self.declare_parameter("fires_file", "")
        self.declare_parameter("extinguish_time", 4.0)     # s of full-strength spray to put a fire out
        self.declare_parameter("regrow_per_s", 0.004)      # intensity regained per second when not sprayed
        self.declare_parameter("jet_range", 1.1)           # m
        self.declare_parameter("jet_half_angle_deg", 6.0)
        self.declare_parameter("flicker_hz", 8.0)
        self.declare_parameter("visuals", True)
        cfg = yaml.safe_load(open(self.get_parameter("robot_yaml").value))
        self.nozzle_xy, self.nozzle_len = (cfg["nozzle"]["x"], cfg["nozzle"]["y"]), cfg["nozzle"]["length"]
        self.fires = [FireSim(f["id"], f["x"], f["y"], f["intensity"], f.get("gas_ppm", 0.0), radius=0.14 * math.sqrt(f["intensity"]))
                      for f in yaml.safe_load(open(self.get_parameter("fires_file").value))["fires"]]
        self.pose, self.nozzle_angle, self.spraying = None, 0.0, False
        self.rng = random.Random(7)
        self.gz = GzIO() if self.get_parameter("visuals").value else None
        self.state_pub = self.create_publisher(String, "/fires/state", 10)
        self.alert_pub = self.create_publisher(String, "/alerts", 10)
        self.create_subscription(Odometry, "/fire_bot/odom", self.on_odom, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(Bool, "/water/spraying", lambda m: setattr(self, "spraying", m.data), 10)
        self.create_service(Trigger, "/fires/extinguish_all", self.on_extinguish_all)    # test helper
        self.create_service(Trigger, "/fires/relight", self.on_relight)                  # mission reset
        self.last = self.now()
        self.create_timer(0.05, self.physics)
        self.create_timer(1.0 / self.get_parameter("flicker_hz").value, self.visuals)
        self.create_timer(0.2, self.publish_state)
        self.get_logger().info(f"{len(self.fires)} fires; {self.get_parameter('extinguish_time').value} s of spray puts one out")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_joints(self, msg):
        if "nozzle_servo_joint" in msg.name:
            self.nozzle_angle = msg.position[msg.name.index("nozzle_servo_joint")]

    def on_extinguish_all(self, _req, res):
        for f in self.fires:
            f.intensity, f.out = 0.0, True
        res.success, res.message = True, "all fires out"
        return res

    def on_relight(self, _req, res):
        for f in self.fires:
            f.intensity, f.out, f.smoke, f.hit_time = 1.0, False, 1.0, 0.0
        res.success, res.message = True, "all fires relit"
        return res

    def physics(self):
        now = self.now()
        dt, self.last = max(0.0, min(now - self.last, 0.5)), now
        origin, jet_yaw = None, 0.0
        if self.pose is not None and self.spraying:
            origin, _, jet_yaw = nozzle_pose(self.pose, self.nozzle_xy, self.nozzle_angle, self.nozzle_len)
        for f in self.fires:
            eff = 0.0
            if origin is not None and not f.out:
                eff = jet_effectiveness(origin, jet_yaw, (f.x, f.y), f.radius, self.get_parameter("jet_range").value,
                                        math.radians(self.get_parameter("jet_half_angle_deg").value))
            if f.step(dt, eff, self.get_parameter("extinguish_time").value, self.get_parameter("regrow_per_s").value):
                self.get_logger().warn(f"fire {f.id} is OUT (after {f.hit_time:.1f} s under water)")
                self.alert_pub.publish(String(data=json.dumps({"type": "fire_out", "id": f.id, "t": now, "spray_s": round(f.hit_time, 1)})))

    def visuals(self):
        if self.gz is None:
            return
        poses = []
        for f in self.fires:
            if f.block_up == f.out:                       # solid while burning, passable once the fire is out
                f.block_up = not f.out
                poses.append((worldgen.block_name(f.id), f.x, f.y, 0.0 if f.block_up else worldgen.HIDDEN_Z, 0, 0, 0))
            level = None if f.out else scale_level(f.intensity)
            want = None
            if level is not None:
                frame = self.rng.choice([k for k in range(worldgen.N_FRAMES) if k != f.last_frame])
                f.last_frame, want = frame, (frame, level)
            if f.shown is not None and f.shown != want:
                poses.append((worldgen.variant_name(f.id, *f.shown), f.x, f.y, worldgen.HIDDEN_Z, 0, 0, 0))
            if want is not None:
                poses.append((worldgen.variant_name(f.id, *want), f.x, f.y, 0.0, 0, 0, self.rng.uniform(0, math.pi)))
            f.shown = want
            try:
                burning = not f.out and f.intensity > 0.06
                self.gz.emitter(worldgen.emitter_topic(f.id, "flame"), emitting=burning, rate=30 * f.intensity)
                smoke = (0.5 + 0.5 * f.intensity) if not f.out else f.smoke
                self.gz.emitter(worldgen.emitter_topic(f.id, "smoke"), emitting=smoke > 0.03, rate=max(1.0, 16 * smoke))
            except Exception as exc:                     # pragma: no cover (world not up yet)
                self.get_logger().debug(f"gz call failed: {exc}")
        try:
            if poses:
                self.gz.set_poses(poses)
        except Exception as exc:                         # pragma: no cover
            self.get_logger().debug(f"gz call failed: {exc}")

    def publish_state(self):
        self.state_pub.publish(String(data=json.dumps([
            {"id": f.id, "x": f.x, "y": f.y, "intensity": round(f.intensity, 3), "effective": round(f.effective_intensity, 3),
             "gas_ppm": round(f.effective_gas, 1), "out": f.out, "smoke": round(f.smoke, 2)} for f in self.fires])))


def main():
    rclpy.init()
    node = FireManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
