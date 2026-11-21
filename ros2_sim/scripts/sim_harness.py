"""Shared helpers for the batch evaluations (run INSIDE the container, workspace built and sourced):
start a headless simulation as a subprocess, stop it completely, spin a probe node until a condition holds.
Use scripts/in_container.sh to build + run a script."""
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

ROOT = os.path.expanduser("~/ros2_sim")
ROBOT_YAML = f"{ROOT}/src/fire_robot_description/config/robot.yaml"


class Sim:
    def __init__(self, *launch_args, log=None):
        self.log = open(log or "/dev/null", "w")
        self.proc = subprocess.Popen(
            ["ros2", "launch", "fire_robot_bringup", "sim.launch.py", "gui:=false", "monitor_window:=false", *launch_args],
            stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)

    def stop(self):
        try:
            os.killpg(self.proc.pid, signal.SIGINT)
            self.proc.wait(20)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            pass
        for sig in (signal.SIGTERM, signal.SIGKILL):          # anything left of Gazebo / bridges / nodes
            subprocess.run(["pkill", f"-{sig.value}", "-f", "gz sim|parameter_bridge|fire_robot_sim|robot_state_publisher|sim.launch"],
                           stdout=subprocess.DEVNULL)
            time.sleep(1.0)
        self.log.close()


class Probe(Node):
    """Node on simulation time; `.sim_time` follows /clock."""

    def __init__(self, name):
        super().__init__(name, parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.sim_time = 0.0
        self.create_subscription(Clock, "/clock", lambda m: setattr(self, "sim_time", m.clock.sec + m.clock.nanosec * 1e-9), 10)

    def spin_until(self, cond, timeout_wall):
        t0 = time.time()
        while not cond() and time.time() - t0 < timeout_wall:
            rclpy.spin_once(self, timeout_sec=0.05)
        return cond()
