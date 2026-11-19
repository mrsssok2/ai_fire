"""Water tank + pump + the water jet.

/pump/state (Bool, the Arduino pump pin) -> water leaves the tank only while the pump runs AND there is water:
  /water_level   Float32 percent (like the MCP3008 reading in fire.py: wt.value * 100)
  /water/ml      Float32
  /water/spraying Bool  (jet is on; the fire manager uses it)
  /water/level_cmd Float64 -> Gazebo joint that moves the water body inside the transparent bottle
  /alerts        {"type": "tank_empty"} once when the tank runs dry
  /water/refill  std_srvs/Trigger
The jet (translucent cone + droplets, two variants swapped ~10x/s) follows the nozzle, which sweeps with the servo, and is
shown only while spraying.
"""
import json
import math

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float64, String
from std_srvs.srv import Trigger

from fire_robot_sim import worldgen
from fire_robot_sim.core.geometry import nozzle_pose
from fire_robot_sim.core.water import Tank
from fire_robot_sim.fire_sensor_sim import yaw_of
from fire_robot_sim.gzio import GzIO


class WaterSystem(Node):
    def __init__(self):
        super().__init__("water_system")
        self.declare_parameter("robot_yaml", "")
        self.declare_parameter("capacity_ml", 0.0)       # 0 = value from robot.yaml
        self.declare_parameter("flow_ml_s", 0.0)
        self.declare_parameter("initial_percent", 100.0)
        self.declare_parameter("visuals", True)
        cfg = yaml.safe_load(open(self.get_parameter("robot_yaml").value))
        cap = self.get_parameter("capacity_ml").value or cfg["tank"]["capacity_ml"]
        flow = self.get_parameter("flow_ml_s").value or cfg["pump"]["flow_ml_s"]
        self.tank = Tank(cap, flow, cap * self.get_parameter("initial_percent").value / 100.0)
        self.water_height = cfg["tank"]["water_height"]
        self.nozzle_xy, self.nozzle_len = (cfg["nozzle"]["x"], cfg["nozzle"]["y"]), cfg["nozzle"]["length"]
        self.pump_on, self.spraying, self.pose, self.nozzle_angle = False, False, None, 0.0
        self.gz = GzIO() if self.get_parameter("visuals").value else None
        self.variant, self.shown = 0, False
        self.level_pub = self.create_publisher(Float32, "/water_level", 10)
        self.ml_pub = self.create_publisher(Float32, "/water/ml", 10)
        self.spray_pub = self.create_publisher(Bool, "/water/spraying", 10)
        self.cmd_pub = self.create_publisher(Float64, "/water/level_cmd", 10)
        self.alert_pub = self.create_publisher(String, "/alerts", 10)
        self.create_subscription(Bool, "/pump/state", lambda m: setattr(self, "pump_on", m.data), 10)
        self.create_subscription(Odometry, "/fire_bot/odom", self.on_odom, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_service(Trigger, "/water/refill", self.on_refill)
        self.last = self.now()
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f"tank {cap:.0f} ml, pump {flow:.0f} ml/s (about {cap / flow:.0f} s of spraying)")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_joints(self, msg):
        if "nozzle_servo_joint" in msg.name:
            self.nozzle_angle = msg.position[msg.name.index("nozzle_servo_joint")]

    def on_refill(self, _req, res):
        self.tank.refill()
        res.success, res.message = True, "tank refilled"
        return res

    def tick(self):
        now = self.now()
        dt, self.last = max(0.0, min(now - self.last, 0.5)), now
        self.spraying, empty = self.tank.step(dt, self.pump_on)
        if empty:
            self.get_logger().error("TANK EMPTY: no water left, the pump runs dry")
            self.alert_pub.publish(String(data=json.dumps({"type": "tank_empty", "t": now})))
        self.level_pub.publish(Float32(data=float(self.tank.percent)))
        self.ml_pub.publish(Float32(data=float(self.tank.level_ml)))
        self.spray_pub.publish(Bool(data=self.spraying))
        self.cmd_pub.publish(Float64(data=-(1.0 - self.tank.percent / 100.0) * self.water_height))
        if self.gz is not None:
            self.update_jet()

    def update_jet(self):
        """Show a jet variant at the nozzle tip while spraying (swapping variants makes the droplets dance)."""
        try:
            items = []
            if self.spraying and self.pose is not None:
                (_, _), (tx, ty), yaw = nozzle_pose(self.pose, self.nozzle_xy, self.nozzle_angle, self.nozzle_len)
                self.variant = (self.variant + 1) % len(worldgen.JET_VARIANTS)
                items.append((worldgen.JET_VARIANTS[self.variant], tx, ty, 0.0325 + 0.008 + 0.031, 0.0, 0.05, yaw))
                items.append((worldgen.JET_VARIANTS[1 - self.variant], 0.0, 0.0, worldgen.HIDDEN_Z, 0.0, 0.0, 0.0))
                self.shown = True
            elif self.shown:
                items = [(n, 0.0, 0.0, worldgen.HIDDEN_Z, 0.0, 0.0, 0.0) for n in worldgen.JET_VARIANTS]
                self.shown = False
            if items:
                self.gz.set_poses(items)
        except Exception as exc:                         # pragma: no cover
            self.get_logger().debug(f"gz call failed: {exc}")


def main():
    rclpy.init()
    node = WaterSystem()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
