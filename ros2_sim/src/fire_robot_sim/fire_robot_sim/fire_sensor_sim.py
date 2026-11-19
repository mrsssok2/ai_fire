"""Simulated flame sensors (left/front/right) and gas sensor from the robot's ground-truth pose + fires.yaml."""
import dataclasses
import json
import math

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from fire_robot_sim.core.sensors import Fire, FlameSensor, flame_signal, gas_ppm


def load_fires(path):
    with open(path) as f:
        return [Fire(**item, radius=0.14 * math.sqrt(item["intensity"])) for item in yaml.safe_load(f)["fires"]]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class FireSensorSim(Node):
    def __init__(self):
        super().__init__("fire_sensor_sim")
        self.declare_parameter("robot_yaml", "")
        self.declare_parameter("fires_file", "")
        self.declare_parameter("rate", 20.0)
        with open(self.get_parameter("robot_yaml").value) as f:
            cfg = yaml.safe_load(f)
        fs = cfg["flame_sensors"]
        self.sensors = {n: FlameSensor(fs[n]["x"], fs[n]["y"], math.radians(fs[n]["yaw_deg"]),
                                       math.radians(fs["half_angle_deg"]), fs["range_m"])
                        for n in ("left", "front", "right")}
        self.gas_mount = (cfg["gas_sensor"]["x"], cfg["gas_sensor"]["y"])
        self.gas_threshold = cfg["gas_sensor"]["threshold_ppm"]
        self.fires = load_fires(self.get_parameter("fires_file").value)
        self.pose = None
        self.pub = {n: (self.create_publisher(Bool, f"/sensors/flame_{n}", 10),
                        self.create_publisher(Float32, f"/sensors/flame_{n}/analog", 10)) for n in self.sensors}
        self.gas_pub = self.create_publisher(Bool, "/sensors/gas", 10)
        self.ppm_pub = self.create_publisher(Float32, "/sensors/gas/ppm", 10)
        self.create_subscription(Odometry, "/fire_bot/odom", self.on_odom, 10)
        self.create_subscription(String, "/fires/state", self.on_fires, 10)     # intensities from fire_manager
        self.create_timer(1.0 / self.get_parameter("rate").value, self.tick)
        self.get_logger().info(f"{len(self.fires)} fires, sensors: {', '.join(self.sensors)}")

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_fires(self, msg):
        """fire_manager: a fire that is sprayed shrinks (weaker flame signal, less gas) and goes out."""
        state = {f["id"]: f for f in json.loads(msg.data)}
        self.fires = [dataclasses.replace(f, intensity=state[f.id]["effective"], gas_ppm=state[f.id]["gas_ppm"]) if f.id in state else f
                      for f in self.fires]

    def tick(self):
        if self.pose is None:
            return
        for name, sensor in self.sensors.items():
            sig = flame_signal(sensor, self.pose, self.fires)
            digital, analog = self.pub[name]
            digital.publish(Bool(data=sig >= 1.0))          # True = flame detected (the Arduino reads this as 0)
            analog.publish(Float32(data=float(sig)))
        ppm = gas_ppm(self.pose, self.gas_mount, self.fires)
        self.gas_pub.publish(Bool(data=ppm >= self.gas_threshold))
        self.ppm_pub.publish(Float32(data=float(ppm)))


def main():
    rclpy.init()
    node = FireSensorSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
