"""Line-by-line port of fire_sensor.ino (Arduino robot) as a coroutine.

`program(io, phone)` is a generator: every Arduino `delay(ms)` becomes `yield seconds`, every pin access goes
through the `io` object. The caller (the ROS node, or a test with a virtual clock) resumes the generator when the
delay has elapsed. Nothing here knows about ROS, so the timing quirks of the original can be tested exactly:

 * motors keep their last pin state during every blocking delay (the robot keeps driving while it "sends the SMS");
 * with no flame anywhere it waits 500 ms and then *stops* (the robot stands still until a sensor sees fire);
 * the SMS blocks for 7.15 s, the call for 21 s, `put_off_fire` takes ~1.7 s.

Sensor levels are Arduino levels: 0 = flame / gas detected, 1 = nothing.
"""
from __future__ import annotations

from typing import Generator, Protocol

# pin names (fire_sensor.ino defines)
LEFT, RIGHT, FORWARD, GAS_SENSOR = "Left", "Right", "Forward", "GAS_SENSOR"
HIGH, LOW = 1, 0


class IO(Protocol):
    def digital_read(self, pin: str) -> int: ...
    def motors(self, lm1: int, lm2: int, rm1: int, rm2: int) -> None: ...   # L298N inputs
    def pump(self, on: bool) -> None: ...
    def servo_write(self, angle: int) -> None: ...                          # degrees, 0..180
    def serial_println(self, text: str) -> None: ...                        # Serial.println (debug console)
    def sim800l(self, text: str) -> None: ...                               # raw bytes to the SIM800L UART


Delay = Generator[float, None, None]


def setup(io: IO) -> Delay:
    io.sim800l("AT\r\n")
    yield 1.0
    io.sim800l("AT+CMGF=1\r\n")
    yield 1.0
    io.servo_write(90)          # myservo.attach(13); myservo.write(90);


def put_off_fire(io: IO, state: dict) -> Delay:
    io.motors(HIGH, HIGH, HIGH, HIGH)
    io.pump(True)
    yield 0.5
    for pos in range(50, 111):  # for (pos = 50; pos <= 110; pos += 1) { write; delay(10); }
        io.servo_write(pos)
        yield 0.010
    for pos in range(110, 49, -1):
        io.servo_write(pos)
        yield 0.010
    io.pump(False)
    io.servo_write(90)
    state["fire"] = False


def make_call(io: IO, phone: str) -> Delay:
    io.serial_println("calling....")
    io.sim800l(f"ATD{phone};\r\n")
    yield 20.0                  # 20 sec delay
    io.sim800l("ATH\r\n")
    yield 1.0


def send_sms(io: IO, phone: str) -> Delay:
    io.serial_println("sending sms....")
    yield 0.050
    io.sim800l("AT+CMGF=1\r")
    yield 1.0
    io.sim800l(f'AT+CMGS="{phone}"\r')
    yield 1.0
    io.sim800l("Gas Detected")
    yield 0.100
    io.sim800l("\x1a")          # sim800L.write(0x1A)
    yield 5.0


def loop(io: IO, phone: str, state: dict) -> Delay:
    io.servo_write(90)          # myservo.write(90);
    r = io.digital_read
    if r(LEFT) == 1 and r(RIGHT) == 1 and r(FORWARD) == 1:
        yield 0.5
        io.motors(HIGH, HIGH, HIGH, HIGH)
    elif r(FORWARD) == 0:
        io.motors(HIGH, LOW, HIGH, LOW)
        state["fire"] = True
    elif r(LEFT) == 0:
        io.motors(HIGH, LOW, HIGH, HIGH)
    elif r(RIGHT) == 0:
        io.motors(HIGH, HIGH, HIGH, LOW)
    yield 0.4                   # "change this value to change the distance"

    if r(GAS_SENSOR) == 0:
        io.serial_println("Gas is Detected.")
        yield from send_sms(io, phone)

    while state["fire"]:
        yield from put_off_fire(io, state)
        io.serial_println("Fire Detected.")
        yield from make_call(io, phone)


def _program_original(io: IO, phone: str) -> Delay:
    """setup() once, then loop() forever."""
    state = {"fire": False}     # boolean fire = false;
    yield from setup(io)
    while True:
        yield from loop(io, phone, state)


# ---- "improved" firmware: the SIM800L alerts run in the background -------------------------------------------------
# NOT part of fire_sensor.ino. The original blocks in delay() for the whole call (21 s) and SMS (7.15 s), so the robot
# sprays ~1.7 s per ~23 s cycle and keeps its last motor command during the SMS. Here the modem work is a job on a
# one-job-at-a-time queue (there is one modem) that is advanced on the same clock as the main loop, so sensing, driving
# and spraying go on while the call / SMS is in progress. Same pins, same timings of each action, nothing else changed;
# a new call / SMS is not queued while one of the same kind is already queued or running (one alert per episode).

def _enqueue(queue: list, state: dict, kind: str, job: Delay) -> None:
    if kind in state["pending"]:
        return
    state["pending"].add(kind)

    def run():
        try:
            yield from job
        finally:
            state["pending"].discard(kind)
    queue.append(run())


def loop_improved(io: IO, phone: str, state: dict, queue: list) -> Delay:
    io.servo_write(90)
    r = io.digital_read
    if r(LEFT) == 1 and r(RIGHT) == 1 and r(FORWARD) == 1:
        yield 0.5
        io.motors(HIGH, HIGH, HIGH, HIGH)
    elif r(FORWARD) == 0:
        io.motors(HIGH, LOW, HIGH, LOW)
        state["fire"] = True
    elif r(LEFT) == 0:
        io.motors(HIGH, LOW, HIGH, HIGH)
    elif r(RIGHT) == 0:
        io.motors(HIGH, HIGH, HIGH, LOW)
    yield 0.4

    if r(GAS_SENSOR) == 0:
        io.serial_println("Gas is Detected.")
        _enqueue(queue, state, "sms", send_sms(io, phone))

    if state["fire"]:
        yield from put_off_fire(io, state)
        io.serial_println("Fire Detected.")
        _enqueue(queue, state, "call", make_call(io, phone))


def program_improved(io: IO, phone: str) -> Delay:
    state = {"fire": False, "pending": set()}
    queue: list = []
    main = _main_improved(io, phone, state, queue)
    t, main_wake, job, job_wake = 0.0, 0.0, None, 0.0
    eps = 1e-9
    while True:
        if job is None and queue:
            job, job_wake = queue.pop(0), t
        nxt = min(main_wake, job_wake) if job is not None else main_wake
        if nxt > t:
            yield nxt - t
            t = nxt
        if main_wake <= t + eps:
            main_wake = t + next(main)
        if job is not None and job_wake <= t + eps:
            try:
                job_wake = t + next(job)
            except StopIteration:
                job = None


def _main_improved(io: IO, phone: str, state: dict, queue: list) -> Delay:
    yield from setup(io)
    while True:
        yield from loop_improved(io, phone, state, queue)


def program(io: IO, phone: str, mode: str = "original") -> Delay:
    """mode 'original' = fire_sensor.ino exactly (default); 'improved' = same, but the call / SMS do not block."""
    if mode == "improved":
        return program_improved(io, phone)
    if mode != "original":
        raise ValueError(f"unknown firmware mode {mode!r}")
    return _program_original(io, phone)


def motor_speeds(lm1: int, lm2: int, rm1: int, rm2: int) -> tuple[float, float]:
    """L298N truth table for one channel pair -> (left_motor, right_motor) in -1..1.
    IN1 high / IN2 low = forward, IN1 low / IN2 high = reverse, equal = stop (brake or coast)."""
    side = lambda a, b: 1.0 if (a, b) == (1, 0) else -1.0 if (a, b) == (0, 1) else 0.0
    return side(lm1, lm2), side(rm1, rm2)


def to_twist(lm1, lm2, rm1, rm2, wheel_speed: float, track: float, swap_motor_sides: bool = True):
    """Motor pins -> (linear m/s, angular rad/s) for a differential drive.

    `swap_motor_sides`: see docs/design.md. fire_sensor.ino runs the LM pair when the *left* sensor sees fire, which
    only turns the robot toward the fire if LM drives the right-hand wheels, so that is the default.
    """
    lm, rm = motor_speeds(lm1, lm2, rm1, rm2)
    left, right = (rm, lm) if swap_motor_sides else (lm, rm)
    vl, vr = left * wheel_speed, right * wheel_speed
    return (vl + vr) / 2.0, (vr - vl) / track
