# Design: fire-fighting robot in ROS 2 Jazzy + Gazebo Harmonic

Simulation of the two robots in this repo (original files are untouched, everything new lives in `ros2_sim/`):

* **Part 1, Arduino robot** (`fire_sensor.ino`): 3 digital flame sensors (left / front / right), a gas sensor,
  L298N motors, a pump and a servo nozzle, SIM800L for SMS + call. No camera, pure reflexes.
* **Part 2, Raspberry Pi robot** (`fire.py`, `haar_cascade.py`, `hsv.py`): camera fire detection (Keras CNN, Haar
  cascade), pump + servo, SMS, water level through an MCP3008.

## What the repo actually contains (and what it does not)

| File | What it really does |
|---|---|
| `fire_sensor.ino` | the Arduino logic above (ported 1:1, see "Arduino port") |
| `fire.py` | Teachable-Machine-style **CNN** (`keras_model.h5`, `labels.txt`, 224x224, `(x/127)-1`); fire -> stop motors, pump, servo min/mid/max, SMS "Fire Alert" |
| `haar_cascade.py` | **Haar cascade** (`fire_detection_cascade_model.xml`): fire -> stop, pump on |
| `hsv.py` | despite its name it is a second Haar-cascade script (with servo sweep); **there is no HSV thresholding in the repo**. The README shows an HSV result image, so an HSV detector is written from scratch here |
| model files | `keras_model.h5`, `labels.txt`, `fire_detection_cascade_model.xml` are **not in the repo** -> the CNN and the cascade are retrained (see plan) |

## Fidelity rules

1. Behaviour comes from the original code, including its quirks (motors keep their last command during blocking
   `delay()`s, the SMS blocks for ~7 s, the call for 21 s, the robot stands still until a flame is seen).
2. Everything electrical is replaced by a ROS interface with the **same meaning**: digital sensors read 0 on flame,
   motor pins are (IN1, IN2) pairs, the SIM800L is driven with the same AT strings.
3. Anything invented for the simulation (sensor cone/range, fire growth, gas diffusion) is a **parameter** with a
   comment saying so.

### Assumption: which motor is which
`fire_sensor.ino` drives the *left* motor forward and stops the right one when the **left** flame sensor fires.
On a robot where `LM` is the left motor that turns it *away* from the fire. The sim assumes the pins are wired
mirrored (parameter `swap_motor_sides`, default `true`) so the robot turns **toward** the fire as described in the
project text. Set it to `false` to run the code exactly as written.

## Robot (from the photos in the README)

4-wheel skid/diff-drive chassis: flat board (~0.22 x 0.30 m), four 65 mm wheels, 3 flame sensors at the front edge
(left/right angled outward), servo + nozzle tube at the front centre, water-bottle tank at the rear left, pump,
battery, L298N, SIM800L, MQ gas sensor, USB webcam (Pi version). Differential drive via Gazebo's `DiffDrive` plugin
(all four wheels), servo via `JointPositionController`.

## Nodes and topics (step 1 in bold)

```
Gazebo (fire_world) -- ros_gz_bridge --> /clock, /fire_bot/odom (ground truth), /camera/image_raw, /joint_states
                                      <-- /cmd_vel, /nozzle/cmd (servo angle)

fire_sensor_sim   : /fire_bot/odom + fires.yaml  -> /sensors/flame_{left,front,right}[/analog], /sensors/gas[/ppm]
arduino_bot       : sensors -> /cmd_vel, /nozzle/cmd, /pump/state, /sim800l/tx     (port of fire_sensor.ino)
sim800l           : /sim800l/tx (AT commands) -> /alerts (SMS / call log, JSON)
fire_manager      : (step 2) fires: intensity, flicker, going out under the water jet, smoke; gz-transport world edits
water_system      : (step 2) tank + pump -> /water_level, /water/spraying, tank_empty alert, jet particle emitter
cnn_detector      : (step 2) /camera/image_raw -> /fire/cnn_conf, /fire/detections (user's CNN, ONNX)
fire_monitor      : (step 2) live window + /fire/annotated
detectors         : (steps 3-4) HSV, Haar as further swappable nodes on the same interface
fusion            : (step 6)
pi_bot            : (step 3) port of fire.py / haar_cascade.py / hsv.py reactions + MCP3008 water level
```

All nodes run on simulation time (`use_sim_time`), so the original `delay()`s scale with Gazebo's real-time factor.

## Sim vs real fire (honest gap)

A simulated fire is a few textured sprites and particles, not combustion: no flicker statistics, smoke, glare,
lens effects or sensor noise of a webcam, and the flame sensors are an analytic cone/inverse-square model, not
an IR photodiode. So:

* the flame-sensor logic is tested for *logic and timing*, not for real-world sensitivity;
* detectors trained on photos of real fires (Kaggle datasets in the README) will **not** automatically work on
  simulated fire. Step 6 measures exactly that (accuracy on real images vs on simulator frames) and reports it.

## Mission control (watching it)

`mission` node: the robot is idle until the countdown (`start_delay`, wall-clock seconds) ends or a `start` command arrives
(`/mission/command`: terminal Enter, window keys/buttons). `/mission/running` (latched) powers the ported Arduino program
on (setup() then loop()); on **reset** it is powered off (pump off, servo 90), the robot is teleported to the start, the tank
refilled (`/water/refill`) and the fires relit (`/fires/relight`), then the countdown starts again. The state
(`WAITING` / `RUNNING` / `DONE` / `TANK_EMPTY`) is shown as a banner in the Fire Detection window; nothing shuts down
automatically. `speed` is Gazebo's real-time factor (all nodes run on sim time), and the Gazebo GUI camera follows the
robot through the `/gui/follow` services.

## Fire, water and detection (step 2)

* **Fire** = pit with charred logs, twigs and stones; flames are **three crossed billboard planes** textured with real,
  freely licensed flame photos (alpha-keyed, credited in `assets/CREDITS.md`), 4 frames x 3 sizes. `fire_manager` swaps
  frame + random turn ~8 times a second (flicker), each variant has its own orange point light (brightness jitter),
  embers + a ground halo glow, plus Gazebo particle emitters for sparks and rising smoke. (The water jet is built from
  translucent visuals, not particles: Harmonic particle emitters can not be re-aimed at runtime, they only emit straight up.)
* **Putting it out:** the water jet is a narrow cone from the nozzle (which sweeps with the servo). Hits lower the fire's
  intensity (`extinguish_time` = 4 s of full-strength spray); the flames shrink through the three sizes, then only
  embers, then out; smoke keeps going and fades ~10 s later. A fire that is not sprayed regrows very slowly.
* **Water:** `water_system` integrates the tank (500 ml, pump 25 ml/s from `robot.yaml`) **only while the pump pin is on**
  and publishes `/water_level` in percent (like the MCP3008 reading in `fire.py`). The water body inside the transparent
  bottle is a separate visual that sinks with the level; at 0 the jet stops and `{"type": "tank_empty"}` goes to `/alerts`.
* **CNN:** the user's `keras_model.h5` is a 4-conv Keras CNN with one sigmoid output (not a Teachable-Machine model), converted
  to ONNX without TensorFlow and run with onnxruntime. Details, label/normalisation analysis and numbers: `docs/cnn_model.md`.
* **Live window:** `fire_monitor` shows the robot camera with a `FIRE DETECTED 93%` / `NO FIRE 12%` banner, boxes around
  fire regions (HSV proposals classified by the CNN) and a side panel (3 flame sensors, gas, tank, pump, servo, last alert);
  the same picture is published on `/fire/annotated`.

Honest limits: Gazebo particles are flat squares (no soft sprites), so smoke/sparks look blocky up close; flames are
cut-out photo planes, they do not light the scene volumetrically; the "gas" is still a static radial model.

## Plan

1. **Robot + world + Arduino logic** (done).
2. **Realistic fires, water system, user's CNN, live detection window, robot model like the photo** (done).
3. **Camera detector interface:** common `/fire/detections`, swappable `detector:=` launch argument, HSV detector, `pi_bot`
   (behaviour of `fire.py` / `haar_cascade.py` / `hsv.py`: stop, pump, servo sweep, SMS "Fire Alert").
4. **Haar cascade:** train with OpenCV `traincascade` (dataset from the README) and wrap as a node.
5. **CNN review:** measure the user's CNN on real images and simulator frames; retrain a small CNN only as a fallback if
   it is unusable.
6. **Fusion node (done):** flame-sensor trigger + CNN confirmation, confidence weighting, flame direction (from the sensor that
   fired, and the CNN/HSV box position) used to aim the robot and the nozzle.
7. **Decoys + evaluation (done for flame sensors / CNN / fused; HSV and Haar still to add):** red/orange non-fire objects, a sunlit window and a hot lamp in the world; false alarms and missed
   fires per method (flame sensors, HSV, Haar, CNN) versus the fused result, plus scenario tests.
8. **Docs:** README section, demo GIF, diagrams.
