from fire_robot_sim.core.mission import DONE, RUNNING, TANK_EMPTY, WAITING, Mission


def test_waits_for_the_delay_with_a_countdown_then_runs():
    m = Mission(15.0)
    st = m.update(100.0, False, False)
    assert st["state"] == WAITING and st["countdown"] == 15.0 and st["message"] == "MISSION STARTS IN 15 s"
    assert not m.robot_may_run
    assert m.update(110.2, False, False)["message"] == "MISSION STARTS IN 5 s"
    st = m.update(115.0, False, False)
    assert st["state"] == RUNNING and m.robot_may_run and st["message"] == ""


def test_start_command_skips_the_countdown():
    m = Mission(60.0)
    m.update(0.0, False, False)
    m.start()
    assert m.update(1.0, False, False)["state"] == RUNNING


def test_zero_delay_starts_at_once():
    assert Mission(0.0).update(5.0, False, False)["state"] == RUNNING


def test_done_and_tank_empty_are_sticky_messages_nothing_ends():
    m = Mission(0.0)
    m.update(0.0, False, False)
    assert m.update(1.0, False, True)["message"] == "TANK EMPTY: no water left" and m.state == TANK_EMPTY
    assert m.update(2.0, False, True)["state"] == TANK_EMPTY and m.robot_may_run
    assert m.update(3.0, True, True)["message"] == "MISSION DONE: all fires out" and m.state == DONE
    assert m.update(1000.0, True, True)["state"] == DONE                      # still running much later


def test_all_out_wins_over_a_tank_that_runs_dry_at_the_same_time():
    m = Mission(0.0)
    m.update(0.0, False, False)
    assert m.update(1.0, True, True)["state"] == DONE


def test_reset_goes_back_to_waiting_with_a_fresh_countdown():
    m = Mission(10.0)
    m.update(0.0, False, False)
    m.start()
    m.update(1.0, True, False)
    assert m.state == DONE
    m.reset(50.0)
    st = m.update(52.0, False, False)
    assert st["state"] == WAITING and st["countdown"] == 8.0 and not m.robot_may_run
