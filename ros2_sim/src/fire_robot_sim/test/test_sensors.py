import math

import pytest

from fire_robot_sim.core.sensors import Fire, FlameSensor, flame_signal, gas_ppm, to_world

DEG = math.radians
FRONT = FlameSensor(0.15, 0.0, 0.0, DEG(30), 1.0)
LEFT = FlameSensor(0.145, 0.07, DEG(40), DEG(30), 1.0)
POSE = (0.0, 0.0, 0.0)


def test_range_is_where_an_intensity_one_fire_straight_ahead_trips():
    assert flame_signal(FRONT, POSE, [Fire("a", 1.15, 0.0)]) == pytest.approx(1.0)      # sensor at x = 0.15
    assert flame_signal(FRONT, POSE, [Fire("a", 1.30, 0.0)]) < 1.0
    assert flame_signal(FRONT, POSE, [Fire("a", 0.9, 0.0)]) > 1.0


def test_inverse_square_and_intensity():
    d1 = flame_signal(FRONT, POSE, [Fire("a", 1.15, 0.0)])
    assert flame_signal(FRONT, POSE, [Fire("a", 2.15, 0.0, 4.0)]) == pytest.approx(d1)   # 2x distance, 4x intensity


def test_cone_cutoff_and_gain():
    on_axis = flame_signal(FRONT, POSE, [Fire("a", 0.65, 0.0)])
    off = flame_signal(FRONT, POSE, [Fire("b", 0.15 + 0.5 * math.cos(DEG(20)), 0.5 * math.sin(DEG(20)))])
    assert 0 < off < on_axis
    assert flame_signal(FRONT, POSE, [Fire("c", 0.15 + 0.5 * math.cos(DEG(40)), 0.5 * math.sin(DEG(40)))]) == 0


def test_first_fire_of_the_default_world_is_seen_only_by_the_left_sensor():
    f1 = [Fire("f1", 0.75, 0.45)]
    assert flame_signal(LEFT, POSE, f1) >= 1.0
    assert flame_signal(FRONT, POSE, f1) == 0.0


def test_turning_toward_the_fire_brings_it_into_the_front_cone():
    f1 = [Fire("f1", 0.75, 0.45)]
    yaw = math.atan2(0.45, 0.75 - 0.0)                         # facing the fire
    assert flame_signal(FRONT, (0.0, 0.0, yaw), f1) >= 1.0


def test_sensors_follow_the_robot_pose():
    x, y, yaw = to_world(1.0, 2.0, math.pi / 2, 0.15, 0.0, 0.0)
    assert (x, y) == pytest.approx((1.0, 2.15)) and yaw == pytest.approx(math.pi / 2)
    assert flame_signal(FRONT, (0.0, 0.0, math.pi / 2), [Fire("a", 0.0, 1.15)]) == pytest.approx(1.0)


def test_gas_decays_with_distance():
    fires = [Fire("g", 2.0, 0.0, gas_ppm=1500.0)]
    near = gas_ppm((1.6, 0.0, 0.0), (0.0, 0.0), fires)
    far = gas_ppm((0.0, 0.0, 0.0), (0.0, 0.0), fires)
    assert near > 300.0 > far
    assert gas_ppm(POSE, (0, 0), [Fire("x", 0.0, 0.0)]) == 0.0
