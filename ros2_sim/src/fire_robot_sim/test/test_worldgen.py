import xml.etree.ElementTree as ET

from fire_robot_sim import worldgen

FIRES = [{"id": "f1", "x": 0.75, "y": 0.45, "intensity": 1.0}, {"id": "f2", "x": 2.0, "y": -1.0, "intensity": 1.5}]


def world():
    return ET.fromstring("<world>" + worldgen.fires_sdf(FIRES, "/assets") + "</world>")


def test_every_fire_has_a_pit_and_12_flame_variants_and_a_shared_jet():
    names = [m.get("name") for m in world().findall("model")]
    assert [n for n in names if n.startswith("water_jet")] == ["water_jet_a", "water_jet_b"]
    for f in ("f1", "f2"):
        assert f"fire_{f}_pit" in names
        assert {worldgen.variant_name(f, k, j) for k in range(4) for j in range(3)} <= set(names)
    assert len(names) == 2 * (1 + 1 + 12) + 2


def test_flame_planes_use_real_photo_textures_and_start_hidden():
    root = world()
    v = [m for m in root.findall("model") if m.get("name") == "fire_f1_f2_s1"][0]
    assert float(v.find("pose").text.split()[2]) == worldgen.HIDDEN_Z
    albedo = [a.text for a in v.iter("albedo_map")]
    assert any(t.endswith("flame_3.png") for t in albedo) and len([t for t in albedo if "flame_" in t]) == 3   # 3 crossed planes


def test_the_solid_footprint_is_a_separate_model_so_it_can_be_moved_away():
    models = {m.get("name"): m for m in world().findall("model")}
    assert models["fire_f1_block"].find(".//collision") is not None
    assert models["fire_f1_pit"].find(".//collision") is None


def test_pit_has_logs_stones_and_two_particle_emitters():
    pit = [m for m in world().findall("model") if m.get("name") == "fire_f1_pit"][0]
    visuals = {v.get("name") for v in pit.iter("visual")}
    assert {"log0", "log4", "stone0", "ash"} <= visuals
    emitters = {e.get("name"): e.find("topic").text for e in pit.iter("particle_emitter")}
    assert emitters == {"flame": "/fire/f1/flame", "smoke": "/fire/f1/smoke"}


def test_each_variant_carries_its_own_uniquely_named_light_with_valid_colours():
    lights = [l for l in world().iter("light")]
    names = [l.get("name") for l in lights]
    assert len(names) == len(set(names)) == 24
    for l in lights:
        assert all(0.0 <= float(c) <= 1.0 for c in l.find("diffuse").text.split())
    assert min(float(l.find("diffuse").text.split()[0]) for l in lights) < max(float(l.find("diffuse").text.split()[0]) for l in lights)   # brightness jitter


def test_bigger_fires_are_bigger():
    a = [m for m in world().findall("model") if m.get("name") == "fire_f1_f0_s0"][0]
    b = [m for m in world().findall("model") if m.get("name") == "fire_f2_f0_s0"][0]
    width = lambda m: float(m.find(".//visual/geometry/box/size").text.split()[0])
    assert width(b) > width(a)
