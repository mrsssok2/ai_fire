"""ROS 2 node that runs the ported fire_sensor.ino program (core/arduino_logic.py) on simulated hardware.

Sensors in:   /sensors/flame_{left,front,right}, /sensors/gas   (Bool, True = detected; read as Arduino level 0)
Outputs:      /cmd_vel (from the L298N pin states), /nozzle/cmd (servo, radians), /pump/state (Bool),
              /sim800l/tx (raw UART text), /arduino/serial (Serial.println), /arduino/motor_pins (debug)
"""
import math

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, String

from fire_robot_sim.core import arduino_logic as al


class Hardware:
    """The `io` object the ported program talks to."""

    def __init__(self, node):
        self.node = node
        self.levels = {al.LEFT: 1, al.RIGHT: 1, al.FORWARD: 1, al.GAS_SENSOR: 1}
        self.pins = (1, 1, 1, 1)      # L298N inputs after setup(): motors idle

    def digital_read(self, pin):
        return self.levels[pin]

    def motors(self, lm1, lm2, rm1, rm2):
        self.pins = (lm1, lm2, rm1, rm2)

    def pump(self, on):
        self.node.pump_pub.publish(Bool(data=bool(on)))

    def servo_write(self, angle):
        # servo 90 deg = straight ahead; joint angle (rad) = servo - 90 deg, positive = left
        self.node.servo_pub.publish(Float64(data=math.radians(angle - 90)))

    def serial_println(self, text):
        self.node.get_logger().info(f"[serial] {text}")
        self.node.serial_pub.publish(String(data=text))

    def sim800l(self, text):
        self.node.gsm_pub.publish(String(data=text))


class ArduinoBot(Node):
    def __init__(self):
        super().__init__("arduino_bot")
        self.declare_parameter("robot_yaml", "")
        self.declare_parameter("phone", "+910000000000")      # PHONE in fire_sensor.ino: set your own number
        self.declare_parameter("wheel_speed", 0.4)             # m/s of a driven side (assumed L298N + TT motors)
        self.declare_parameter("swap_motor_sides", True)       # see docs/design.md
        self.declare_parameter("firmware", "original")         # "original" = fire_sensor.ino; "improved" = call / SMS in the background
        with open(self.get_parameter("robot_yaml").value) as f:
            self.track = yaml.safe_load(f)["wheels"]["track"]
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pump_pub = self.create_publisher(Bool, "/pump/state", 10)
        self.servo_pub = self.create_publisher(Float64, "/nozzle/cmd", 10)
        self.gsm_pub = self.create_publisher(String, "/sim800l/tx", 100)
        self.serial_pub = self.create_publisher(String, "/arduino/serial", 10)
        self.io = Hardware(self)
        for pin, topic in ((al.LEFT, "left"), (al.RIGHT, "right"), (al.FORWARD, "front")):
            self.create_subscription(Bool, f"/sensors/flame_{topic}", lambda m, p=pin: self.set_level(p, m.data), 10)
        self.create_subscription(Bool, "/sensors/gas", lambda m: self.set_level(al.GAS_SENSOR, m.data), 10)
        self.prog, self.wake, self.running = None, 0.0, False
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/mission/running", self.on_running, latched)
        self.create_timer(0.002, self.tick)     # delay() resolution 2 ms (servo sweep steps are 10 ms)
        self.create_timer(0.05, self.drive)
        self.get_logger().info("fire_sensor.ino waits for the mission to start (/mission/running)")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_level(self, pin, detected):
        self.io.levels[pin] = 0 if detected else 1

    def on_running(self, msg):
        """Mission start = power on the Arduino (setup() runs, then loop()); reset = power off, everything idle."""
        if msg.data and not self.running:
            self.io.pins = (1, 1, 1, 1)
            self.prog = al.program(self.io, self.get_parameter("phone").value, self.get_parameter("firmware").value)
            self.wake = self.now()
            self.get_logger().info(f"fire_sensor.ino started, firmware={self.get_parameter('firmware').value} (setup takes 2 s)")
        elif not msg.data and self.running:
            self.prog = None
            self.io.pins = (1, 1, 1, 1)
            self.io.pump(False)
            self.io.servo_write(90)
            self.get_logger().info("fire_sensor.ino stopped (reset)")
        self.running = msg.data

    def tick(self):
        if self.prog is None:
            return
        while self.now() >= self.wake:    # run until the program blocks on the next delay()
            self.wake += next(self.prog)  # (accumulate so 10 ms servo steps keep their rhythm)
            if self.wake < self.now() - 0.05:
                self.wake = self.now()

    def drive(self):
        v, w = al.to_twist(*self.io.pins, self.get_parameter("wheel_speed").value, self.track,
                           self.get_parameter("swap_motor_sides").value)
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = ArduinoBot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
