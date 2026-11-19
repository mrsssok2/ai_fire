"""The xacro processes and matches what the nodes assume (needs ROS xacro; skipped otherwise)."""
import math
import os
import xml.etree.ElementTree as ET

import pytest

xacro = pytest.importorskip("xacro")
HERE = os.path.dirname(__file__)
URDF = os.path.join(HERE, "..", "urdf", "fire_bot.urdf.xacro")


@pytest.fixture(scope="module")
def robot():
    # $(find fire_robot_description) needs the package on the ament path (colcon build + source install/setup.bash)
    try:
        return ET.fromstring(xacro.process_file(URDF).toxml())
    except Exception as exc:                                        # pragma: no cover
        pytest.skip(f"package not on the ament index: {exc}")


def test_four_driven_wheels_and_servo_nozzle(robot):
    joints = {j.get("name"): j for j in robot.findall("joint")}
    assert {f"wheel_{w}_joint" for w in ("fl", "fr", "rl", "rr")} <= set(joints)
    assert joints["nozzle_servo_joint"].get("type") == "revolute"


def test_nozzle_joint_covers_the_servo_sweep_50_to_110_degrees(robot):
    lim = [j for j in robot.findall("joint") if j.get("name") == "nozzle_servo_joint"][0].find("limit")
    assert float(lim.get("lower")) <= math.radians(50 - 90) and float(lim.get("upper")) >= math.radians(110 - 90)


def test_diff_drive_plugin_uses_all_wheels_and_the_yaml_track(robot):
    plugins = [p for p in robot.iter("plugin") if p.get("name", "").endswith("DiffDrive")]
    assert len(plugins) == 1
    p = plugins[0]
    assert len(p.findall("left_joint")) == len(p.findall("right_joint")) == 2
    assert float(p.find("wheel_separation").text) == pytest.approx(0.25)
