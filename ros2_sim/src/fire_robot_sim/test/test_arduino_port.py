"""fire_sensor.ino port: timing and pin behaviour, run on a virtual clock with fake hardware."""
import math

import pytest

from fire_robot_sim.core import arduino_logic as al
from fire_robot_sim.core.sim800l import Sim800l

PHONE = "+911234567890"


class FakeIO:
    def __init__(self):
        self.t = 0.0
        self.levels = {al.LEFT: 1, al.RIGHT: 1, al.FORWARD: 1, al.GAS_SENSOR: 1}
        self.log = []             # (time, kind, value)
        self.motor_state = None

    def digital_read(self, pin):
        return self.levels[pin]

    def motors(self, *pins):
        self.motor_state = pins
        self.log.append((self.t, "motors", pins))

    def pump(self, on):
        self.log.append((self.t, "pump", on))

    def servo_write(self, angle):
        self.log.append((self.t, "servo", angle))

    def serial_println(self, text):
        self.log.append((self.t, "serial", text))

    def sim800l(self, text):
        self.log.append((self.t, "gsm", text))

    def events(self, kind):
        return [(t, v) for t, k, v in self.log if k == kind]


def run(io, seconds):
    """Resume the program on a virtual clock until `seconds` have elapsed."""
    prog = al.program(io, PHONE)
    io.t = 0.0
    wake = 0.0
    while True:
        io.t = wake
        if io.t > seconds:
            return
        wake += next(prog)


def test_setup_takes_two_seconds_and_configures_the_modem():
    io = FakeIO()
    run(io, 2.1)
    assert io.events("gsm")[:2] == [(0.0, "AT\r\n"), (1.0, "AT+CMGF=1\r\n")]
    assert io.events("servo")[0][0] == pytest.approx(2.0)      # myservo.write(90) after the two 1 s delays


def test_no_flame_waits_500ms_then_stops_and_idles():
    io = FakeIO()
    run(io, 6.0)
    motors = io.events("motors")
    assert all(p == (1, 1, 1, 1) for _, p in motors)
    # loop period = 0.5 + 0.4 s; first loop starts at t = 2.0 (setup)
    assert [round(t, 3) for t, _ in motors[:3]] == [2.5, 3.4, 4.3]
    assert not io.events("pump")


def test_front_flame_drives_forward_then_extinguish_sequence_and_call():
    io = FakeIO()
    io.levels[al.FORWARD] = 0
    run(io, 2.0 + 0.4 + 1.72 + 21.0 + 0.2)
    t0 = 2.0
    assert io.events("motors")[0] == (t0, (1, 0, 1, 0))                    # both motors forward
    t_fire = t0 + 0.4                                                       # delay(400) then put_off_fire
    assert io.events("motors")[1] == (pytest.approx(t_fire), (1, 1, 1, 1))  # stop
    assert io.events("pump")[0] == (pytest.approx(t_fire), True)
    t_pump_off = io.events("pump")[1][0]
    sweep = [(t, a) for t, a in io.events("servo") if t_fire < t <= t_pump_off + 1e-9]   # up, back, then write(90)
    angles = [a for _, a in sweep]
    assert angles[:2] == [50, 51] and 110 in angles and angles[-2:] == [50, 90]
    assert angles.count(110) == 2 and angles.count(50) == 2                 # up to 110, back to 50 (both ends counted)
    assert sweep[1][0] - sweep[0][0] == pytest.approx(0.010)                # 10 ms per degree
    assert t_pump_off == pytest.approx(t_fire + 0.5 + 0.61 + 0.61)
    assert [v for _, v in io.events("serial")] == ["Fire Detected.", "calling...."]
    gsm = io.events("gsm")
    assert gsm[2][1] == f"ATD{PHONE};\r\n"
    assert gsm[3][1] == "ATH\r\n" and gsm[3][0] - gsm[2][0] == pytest.approx(20.0)


@pytest.mark.parametrize("flame, pins", [(al.LEFT, (1, 0, 1, 1)), (al.RIGHT, (1, 1, 1, 0))])
def test_side_flames_turn_and_never_start_the_extinguisher(flame, pins):
    io = FakeIO()
    io.levels[flame] = 0
    run(io, 6.0)
    assert io.events("motors")[0][1] == pins
    assert not io.events("pump")


def test_front_has_priority_over_side_sensors():
    io = FakeIO()
    io.levels[al.FORWARD] = io.levels[al.LEFT] = 0
    run(io, 2.1)
    assert io.events("motors")[0][1] == (1, 0, 1, 0)


def test_motors_keep_their_pins_while_the_sms_blocks():
    """The original blocks in delay() with the motors still running: no motor write happens during the 7.15 s SMS."""
    io = FakeIO()
    io.levels[al.LEFT] = 0
    io.levels[al.GAS_SENSOR] = 0
    run(io, 2.0 + 0.4 + 7.15 + 0.1)
    assert [t for t, _ in io.events("motors")] == [2.0, pytest.approx(2.0 + 0.4 + 7.15)]
    gsm = io.events("gsm")[2:]
    assert [v for _, v in gsm] == ["AT+CMGF=1\r", f'AT+CMGS="{PHONE}"\r', "Gas Detected", "\x1a"]
    assert gsm[-1][0] - gsm[0][0] == pytest.approx(1.0 + 1.0 + 0.1)     # CMGF -> CMGS -> text -> Ctrl-Z
    assert ("Gas is Detected.") in [v for _, v in io.events("serial")]


def test_sim800l_decodes_what_the_program_sends():
    io = FakeIO()
    io.levels[al.GAS_SENSOR] = 0
    io.levels[al.FORWARD] = 0
    run(io, 2.0 + 0.4 + 7.15 + 1.72 + 21.0 + 0.5)
    modem, events = Sim800l(), []
    for t, text in io.events("gsm"):
        events += modem.feed(text, t)
    kinds = [e["type"] for e in events]
    assert kinds == ["sms", "call_start", "call_end"]
    assert events[0]["text"] == "Gas Detected" and events[0]["to"] == PHONE
    assert events[1]["to"] == PHONE and events[2]["duration_s"] == pytest.approx(20.0)


def test_pins_to_twist():
    # both forward
    assert al.to_twist(1, 0, 1, 0, 0.4, 0.25)[0] == pytest.approx(0.4)
    assert al.to_twist(1, 0, 1, 0, 0.4, 0.25)[1] == 0
    assert al.to_twist(1, 1, 1, 1, 0.4, 0.25) == (0, 0)
    # left flame pins (LM fwd, RM stop): default wiring turns LEFT (toward the fire), literal wiring turns right
    v, w = al.to_twist(1, 0, 1, 1, 0.4, 0.25)
    assert v == pytest.approx(0.2) and w == pytest.approx(1.6)
    v, w = al.to_twist(1, 0, 1, 1, 0.4, 0.25, swap_motor_sides=False)
    assert w == pytest.approx(-1.6)
    # right flame pins turn right in the default wiring
    assert al.to_twist(1, 1, 1, 0, 0.4, 0.25)[1] == pytest.approx(-1.6)
    assert al.to_twist(0, 1, 0, 1, 0.4, 0.25)[0] == pytest.approx(-0.4)   # reverse
