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
fire_manager      : (step 2) fires: intensity, spread, gas cloud, extinguish by water
detectors         : (steps 3-5) /camera/image_raw -> /fire/detections, one node per method, swappable
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

## Plan

1. **Robot + world + Arduino logic (done in this step):** xacro robot (chassis, 4 wheels, camera, servo nozzle, pump,
   tank, flame + gas sensors), world with fires (simple placeholder visuals), flame/gas sensor model, `arduino_bot`
   port of `fire_sensor.ino`, SIM800L AT emulator, one launch command, tests.
2. **Fire model:** `fire_manager` (intensity, spread, going out when sprayed for a few seconds), spray jet
   (particles + hit test), gas cloud that spreads, realistic fire textures + particle effect on the flames,
   water tank level.
3. **Camera + detector interface:** `/fire/detections`, swappable detector launch argument, HSV detector, `pi_bot`
   (behaviour of `fire.py` / `haar_cascade.py` / `hsv.py`, MCP3008 water level, SMS "Fire Alert").
4. **Haar cascade:** train with OpenCV `traincascade` (dataset from the README) and wrap as a node.
5. **CNN:** train a small CNN on the Kaggle fire dataset (README), export to ONNX, run with onnxruntime in a node.
6. **Sim-vs-real evaluation:** each detector on real dataset images vs simulator frames, report the gap, tune the
   fire appearance where it is honest to do so.
7. **Scenarios + tests:** several worlds/fire layouts, scripted runs with pass/fail (fire out, alerts sent),
   unit tests for every node's logic.
8. **Docs:** README section, demo GIF, diagrams.
