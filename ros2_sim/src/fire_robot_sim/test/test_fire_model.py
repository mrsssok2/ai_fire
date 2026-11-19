import math

import pytest

from fire_robot_sim.core.fire_model import FireSim, jet_effectiveness, scale_level
from fire_robot_sim.core.geometry import nozzle_pose

HALF = math.radians(6)


def test_flames_shrink_through_three_sizes_then_embers():
    assert [scale_level(i) for i in (1.0, 0.7, 0.5, 0.2, 0.05, 0.0)] == [0, 0, 1, 2, None, None]


def test_jet_hits_a_fire_straight_ahead_only_within_range_and_cone():
    origin = (0.0, 0.0)
    assert jet_effectiveness(origin, 0.0, (0.6, 0.0), 0.14, 1.1, HALF) > 0.3
    assert jet_effectiveness(origin, 0.0, (2.0, 0.0), 0.14, 1.1, HALF) == 0.0            # out of range
    assert jet_effectiveness(origin, 0.0, (0.6, 0.6), 0.14, 1.1, HALF) == 0.0            # 45 deg off
    assert jet_effectiveness(origin, 0.0, (0.6, 0.05), 0.14, 1.1, HALF) > 0.0            # within the fire's own radius
    assert jet_effectiveness(origin, 0.0, (0.3, 0.0), 0.14, 1.1, HALF) > jet_effectiveness(origin, 0.0, (1.0, 0.0), 0.14, 1.1, HALF)


def test_jet_follows_the_sweep_angle():
    fire = (0.6, 0.6 * math.tan(math.radians(20)))                  # 20 deg to the left of straight ahead
    assert jet_effectiveness((0, 0), 0.0, fire, 0.1, 1.1, HALF) == 0.0
    assert jet_effectiveness((0, 0), math.radians(20), fire, 0.1, 1.1, HALF) > 0.0


def test_fire_goes_out_after_extinguish_time_of_full_spray():
    f = FireSim("a", 0, 0)
    went_out, t = False, 0.0
    while not went_out and t < 20:
        went_out = f.step(0.05, 1.0, extinguish_time=4.0)
        t += 0.05
    assert f.out and f.intensity == 0.0
    assert t == pytest.approx(4.0, abs=0.1) and f.hit_time == pytest.approx(4.0, abs=0.1)


def test_weaker_spray_takes_longer_and_a_dry_fire_regrows_slowly():
    f = FireSim("a", 0, 0)
    for _ in range(40):                                              # 2 s at half strength
        f.step(0.05, 0.5, extinguish_time=4.0)
    assert 0.70 < f.intensity < 0.80
    low = f.intensity
    for _ in range(200):                                             # 10 s without water
        f.step(0.05, 0.0, regrow_per_s=0.004)
    assert low < f.intensity < low + 0.05


def test_smoke_fades_only_after_the_fire_is_out():
    f = FireSim("a", 0, 0)
    f.step(0.05, 1.0, extinguish_time=0.05)                          # out immediately
    assert f.out and f.smoke == 1.0
    for _ in range(100):                                             # 5 s
        f.step(0.05, 0.0, smoke_fade_s=10.0)
    assert 0.45 < f.smoke < 0.55
    for _ in range(200):
        f.step(0.05, 0.0, smoke_fade_s=10.0)
    assert f.smoke == 0.0
    assert f.effective_intensity == 0.0 and f.effective_gas == 0.0


def test_nozzle_pose_sweeps_with_the_servo_and_the_robot():
    (px, py), (tx, ty), yaw = nozzle_pose((1.0, 2.0, math.pi / 2), (0.115, 0.0), math.radians(20), 0.06)
    assert (px, py) == pytest.approx((1.0, 2.115))
    assert yaw == pytest.approx(math.pi / 2 + math.radians(20))
    assert math.hypot(tx - px, ty - py) == pytest.approx(0.06)
