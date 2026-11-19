import pytest

from fire_robot_sim.core.water import Tank


def test_level_drops_only_while_the_pump_sprays():
    t = Tank(500, 25)
    for _ in range(100):                       # 5 s with the pump off
        assert t.step(0.05, False) == (False, False)
    assert t.percent == 100.0
    for _ in range(40):                        # 2 s with the pump on
        spraying, _ = t.step(0.05, True)
        assert spraying
    assert t.level_ml == pytest.approx(450.0)
    assert t.percent == pytest.approx(90.0)


def test_empty_alert_once_and_no_flow_when_dry():
    t = Tank(100, 25)
    events = [t.step(0.05, True)[1] for _ in range(100)]               # 5 s of pump, tank lasts 4 s
    assert events.count(True) == 1
    assert t.level_ml == 0.0 and t.percent == 0.0
    assert t.step(0.05, True) == (False, False)                        # dry: pump runs but nothing sprays


def test_refill_rearms_the_alert():
    t = Tank(50, 25)
    while not t.step(0.05, True)[1]:
        pass
    t.refill()
    assert t.percent == 100.0 and t.empty_reported is False
