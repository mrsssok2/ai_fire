"""Fusion logic, evaluation scenes, decoy world models."""
import math
import xml.etree.ElementTree as ET

import pytest

from fire_robot_sim import worldgen
from fire_robot_sim.core.fusion import Fusion, flame_bearing_deg
from fire_robot_sim.core.scenes import SCENARIOS, make_scene

NONE = {"left": 0.0, "front": 0.0, "right": 0.0}


def test_no_flame_no_alarm_even_with_a_confident_cnn():
    f = Fusion()
    f.add_cnn(0.0, 0.99)
    st = f.update(0.0, NONE)
    assert st["cnn_alarm"] and not st["flame_alarm"] and not st["fused_alarm"]


def test_flame_trip_alone_is_not_enough_the_cnn_must_confirm():
    f = Fusion()
    f.add_cnn(0.0, 0.10)
    st = f.update(0.0, {**NONE, "front": 1.5})          # hot lamp: IR yes, CNN says no
    assert st["flame_alarm"] and not st["fused_alarm"]
    f.add_cnn(0.1, 0.9)
    assert f.update(0.1, {**NONE, "front": 1.5})["fused_alarm"]


def test_confidence_weighting_strong_flame_compensates_a_weaker_cnn():
    weak_flame, strong_flame = {**NONE, "front": 1.0}, {**NONE, "front": 4.0}
    a, b = Fusion(), Fusion()
    a.add_cnn(0, 0.55)
    b.add_cnn(0, 0.55)
    assert not a.update(0, weak_flame)["fused_alarm"]
    assert b.update(0, strong_flame)["fused_alarm"]


def test_cnn_evidence_is_held_then_expires():
    f = Fusion()
    f.add_cnn(0.0, 0.95)
    assert f.update(0.9, {**NONE, "left": 2.0})["fused_alarm"]       # camera saw it 0.9 s ago, side sensor still sees it
    assert not f.update(1.5, {**NONE, "left": 2.0})["fused_alarm"]


def test_bearing_from_the_sensor_axes():
    assert flame_bearing_deg(NONE) is None
    assert flame_bearing_deg({**NONE, "front": 2.0}) == 0.0
    assert flame_bearing_deg({**NONE, "left": 1.0}) == pytest.approx(40.0)
    assert flame_bearing_deg({**NONE, "right": 1.0}) == pytest.approx(-40.0)
    assert flame_bearing_deg({"left": 1.0, "front": 1.0, "right": 0.0}) == pytest.approx(20.0)


@pytest.mark.parametrize("name", SCENARIOS)
def test_scenes_are_seeded_fixed_in_content_and_do_not_overlap(name):
    s = make_scene(name, 3)
    assert s == make_scene(name, 3) and s != make_scene(name, 4)
    assert len(s["fires"]) == SCENARIOS[name]["fires"]
    assert sorted(d["kind"] for d in s["decoys"]) == sorted(SCENARIOS[name]["decoys"])
    objs = s["fires"] + s["decoys"]
    for i, a in enumerate(objs):
        assert 0.55 <= math.hypot(a["x"], a["y"]) <= 1.35
        for b in objs[i + 1:]:
            assert math.hypot(a["x"] - b["x"], a["y"] - b["y"]) > 0.35


@pytest.mark.parametrize("kind", sorted(worldgen.DECOY_IR))
def test_every_decoy_kind_is_valid_sdf_and_faces_the_robot(kind):
    d = {"id": "d1", "kind": kind, "x": 1.0, "y": 0.0}
    root = ET.fromstring(worldgen.decoy_model(d))
    assert root.get("name") == "decoy_d1"
    assert abs(math.cos(float(root.find("pose").text.split()[5])) + 1) < 1e-3     # +x of the decoy points at the origin
    assert root.findall(".//visual")


def test_ir_decoys_are_the_lamp_and_the_window_only():
    assert {k for k, v in worldgen.DECOY_IR.items() if v > 0} == {"lamp", "window"}
    assert worldgen.decoy_ir({"kind": "red_box", "ir": 2.0}) == 2.0
