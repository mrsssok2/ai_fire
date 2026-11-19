"""Simulated SIM800L: reads the UART text from /sim800l/tx, publishes SMS / call events on /alerts (JSON)."""
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from fire_robot_sim.core.sim800l import Sim800l


class Sim800lNode(Node):
    def __init__(self):
        super().__init__("sim800l")
        self.modem = Sim800l()
        self.pub = self.create_publisher(String, "/alerts", 10)
        self.create_subscription(String, "/sim800l/tx", self.on_tx, 100)

    def on_tx(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        for ev in self.modem.feed(msg.data, now):
            if ev["type"] == "sms":
                self.get_logger().warn(f'SMS to {ev["to"]}: "{ev["text"]}"')
            elif ev["type"] == "call_start":
                self.get_logger().warn(f'CALL to {ev["to"]}: dialing')
            else:
                self.get_logger().warn(f'CALL to {ev["to"]}: hung up after {ev["duration_s"]} s')
            self.pub.publish(String(data=json.dumps(ev)))


def main():
    rclpy.init()
    node = Sim800lNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
