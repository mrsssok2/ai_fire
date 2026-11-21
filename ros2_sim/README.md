# ros2_sim: fire-fighting robot in ROS 2 Jazzy + Gazebo Harmonic

Simulation of the robots in this repo (the original files one level up are untouched). Design, fidelity rules,
the sim-vs-real fire caveat and the 8-step plan are in [docs/design.md](docs/design.md).

**Status: steps 1, 2, 6 and 7 of 8** (plus mission control: start delay, speed, follow camera, reset): realistic fires, water system, the CNN from `keras_model.h5`, a live detection window, a robot
model that follows the photo, **flame-sensor + CNN fusion with decoys and an evaluation**, and an optional **non-blocking alert firmware**.
Next: HSV/Haar detectors + the Pi robot logic (see the plan in [docs/design.md](docs/design.md)).
Results: [docs/results/fusion_eval.md](docs/results/fusion_eval.md) and [docs/results/alert_modes.md](docs/results/alert_modes.md).

| | |
|---|---|
| ![fire close-up](docs/img/fire_closeup.png) | ![robot close-up](docs/img/robot_closeup.png) |
| *Fire: charred logs + stones, flames = crossed billboard planes with real flame photos, flicker, light, sparks, smoke* | *Robot: board, 4 wheels, bottle tank rear right, black battery, red L298N, SIM800L, breadboard + Arduino, gas module, webcam, flame sensors on the front edge, servo + nozzle* |
| ![spraying](docs/img/spraying.png) | ![detection window](docs/img/detection_window_spraying.png) |
| *The nozzle sits on the servo horn and sweeps; the jet is only shown while the pump sprays* | *Live "Fire Detection" window (also on `/fire/annotated`)* |
| ![tank before](docs/img/tank_before.png) | ![tank after](docs/img/tank_after.png) |
| *Tank before the fire fight...* | *...and after: the level only drops while the pump sprays* |

The simulated fire is a stack of photo cut-outs, particles and a point light. It is **not** combustion and looks
different from a real fire to a camera (no real flicker statistics, glare, smoke density or sensor noise). Anything
measured on it says little about real-world performance, see "How good is the CNN" below.

## What it does

* **Fires** (`fire_manager`): pit with logs; frame swap + random turn ~8x/s, three sizes, own flickering light, sparks and
  smoke emitters. The water jet (a narrow cone from the swept nozzle) lowers a fire's intensity: 4 s of full-strength
  spray (parameter `extinguish_time`) puts it out; the flames shrink, then only embers, then out, and the smoke fades last.
  `/fires/state` has the intensities; flame sensors and gas follow them.
* **Water** (`water_system`): 500 ml tank, pump 25 ml/s (`config/robot.yaml`), consumed **only while the pump pin is on**.
  `/water_level` is a percent like the MCP3008 reading in `fire.py`; the water body in the transparent bottle sinks with it.
  At 0 the jet stops and `{"type": "tank_empty"}` is published on `/alerts` (try `./run.sh tank_percent:=8`).
* **CNN** (`cnn_detector`): the project's `keras_model.h5`, converted to `models/fire_cnn.onnx`, see
  [docs/cnn_model.md](docs/cnn_model.md). `/fire/cnn_conf`, `/fire/detections` (HSV-proposed regions classified by the CNN).
* **Live window** (`fire_monitor`): robot camera + banner (`FIRE DETECTED 93%` red / `NO FIRE 12%` green from the CNN
  confidence) + boxes + side panel (flame sensors L/F/R, gas, tank %, pump, servo angle, last alert). It opens next to
  Gazebo (`monitor_window:=false` to disable) and is published as `/fire/annotated`.
* **Fusion** (`fusion`, `core/fusion.py`): the flame sensors trigger, the CNN confirms, the two confidences are weighted
  (`0.4 * p_flame + 0.6 * p_cnn >= 0.6`, flame sensors must have tripped, CNN reading held 1 s), and the flame sensors give a
  bearing (signal-weighted mean of the three sensor axes) for aiming. `/fusion/state` carries the flame-only, CNN-only and
  fused alarms side by side, `/fusion/alarm` the fused one, `/fusion/bearing` the direction. It does not drive the robot.
* **Decoys** (`decoys:` list in a fires yaml, `scripts/eval_fusion.py` builds seeded scenes): red box, red barrel, orange cone
  (look like fire, no IR), a sunlit window (bright, a bit of IR) and a hot lamp (bright orange, lots of IR: trips the flame sensors).
* **Firmware modes** (`firmware:=original` default, `improved`): `original` is `fire_sensor.ino` exactly (the 21 s call and 7 s SMS block
  everything); `improved` runs the call / SMS as a background job so spraying and driving continue. Not part of the real `.ino`.
* Everything from step 1 still applies (Arduino logic port, SIM800L emulator, flame/gas sensors).

## How good is the CNN (honest)

The model is a small binary Keras CNN; fire confidence = 1 - sigmoid output (determined on real photos, see
`docs/cnn_model.md`: 95% of 19 real fires flagged, but also 77% of 35 mixed non-fire photos). On the simulator camera
(`scripts/eval_cnn_sim.py`, `docs/cnn_sim_eval.json`, mean fire confidence over ~5 s per view):

| view of fire f1 | mean confidence | frames >= 0.5 |
|---|---|---|
| 1.6 m away | 0.30 | 0% |
| 1.0 m | 0.68 | 95% |
| 0.6 m | 0.98 | 100% |
| fire at the edge of the view, 1.0 m | 0.47 | 43% |
| looking away from every fire | 0.27 | 0% |
| all fires out (charred logs, fading smoke), 0.6 m | 0.39 | 0% |

So it detects a close, centred simulated fire confidently, misses a distant one, and never fires on the empty scene or the
burnt-out pit. The distant miss is why the plan has a fusion step: the flame sensors trigger and the CNN confirms.

## Run

Needs Docker Desktop (WSL2, WSLg for the Gazebo window) and the local image `gurdy2/ros:jazzy`
(ROS 2 Jazzy + Gazebo Harmonic + ros_gz; the `Dockerfile` builds on it).

```bash
cd ~/projects/ai_fire/ros2_sim
./run.sh
```

It builds in Docker and opens Gazebo (camera **follows the robot**, close enough to see the robot and the fire) plus the
**Fire Detection** window. The robot is idle at first:

| | |
|---|---|
| **Start** | after `start_delay` seconds (default 15, countdown on the window banner), or at once: **Enter** in the terminal, **Enter / Space / S** or the **START MISSION** button in the window |
| **Speed** | `speed:=0.5` (default) = Gazebo runs at half real time; `speed:=1.0` = real time |
| **Reset** | **r + Enter** in the terminal, **R** or the **RESET** button in the window: robot back to the start, tank refilled, fires relit, countdown again |
| **End** | nothing closes or resets by itself: when all fires are out the window says `MISSION DONE: all fires out`, when the tank runs dry `TANK EMPTY: no water left`, and the sim keeps running until **Ctrl+C** |

```bash
./run.sh speed:=1.0 start_delay:=5       # real time, shorter wait
./run.sh start_delay:=0                  # start at once
./run.sh gui:=false monitor_window:=false   # headless
./run.sh extinguish_time:=8              # harder fires
./run.sh tank_percent:=8                 # nearly empty tank -> TANK EMPTY
./run.sh autonomous:=false               # no Arduino program: drive with  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ...
./run.sh phone:="+91..."                 # number shown in the simulated SMS / call (default is a placeholder)
./run.sh swap_motor_sides:=false         # run the motor pins exactly as written (see design.md)
./run.sh follow:=false                   # free Gazebo camera instead of following the robot
./run.sh firmware:=improved              # call / SMS in the background (default: original = blocking, as in the .ino)
./run.sh sensor_noise:=0.1 seed:=3       # 10 % noise on the flame-sensor signal (default 0 = ideal)
```

| ![countdown](docs/img/window_waiting.png) | ![mission done](docs/img/window_done.png) |
|---|---|
| *Countdown on the window banner, START / RESET buttons* | *`MISSION DONE: all fires out`, the sim keeps running* |

The fires form a chain (`config/fires.yaml`): the robot sees f1 at the start; once it faces f1, f2 is in its right sensor's cone;
once it faces f2, f3 is in its left sensor's cone, so "all fires out" is reachable by the original (non-searching) logic.
Watch it: `ros2 topic echo /mission/state`, `/alerts`, `/sensors/flame_left/analog`, `/pump/state`, `/water_level`.

## Tests

```bash
# pure logic, fusion, scenes, decoys, overlay, world generator, ONNX interface: 69 tests (needs numpy/OpenCV/onnxruntime: easiest in the image)
docker run --rm -v $PWD:/home/ubuntu/ros2_sim fire_ros2:jazzy bash -c \
  'source /opt/ros/jazzy/setup.bash; cd ~/ros2_sim/src/fire_robot_sim && python3 -m pytest -q'
# URDF checks need the built workspace: colcon build && source install/setup.bash && cd src/fire_robot_description && python3 -m pytest test
```

Evaluations (each starts a fresh headless simulation per run; results in `docs/results/`):

```bash
docker compose run --rm -T sim /home/ubuntu/ros2_sim/scripts/in_container.sh scripts/eval_fusion.py --seeds 6        # ~10 min
docker compose run --rm -T sim /home/ubuntu/ros2_sim/scripts/in_container.sh scripts/eval_alert_modes.py --runs 3 --extinguish 2   # ~15 min
```

`scripts/check_mission.py` checks start / done / reset end to end against a running sim. Screenshots are made with `scripts/doc_shots.py` against a sim launched with `shot_camera:=true start_delay:=0 speed:=1.0`; the flame textures with
`assets/make_flame_sprites.py` (credits: [assets/CREDITS.md](assets/CREDITS.md)).

## Known limitations

* With the default 4 s `extinguish_time` the 3-fire chain layout does not always complete: the robot approaches f1 closer, f2 / f3 leave
  its side sensors' cones and it never searches, so it stands still with fires left. `extinguish_time:=2` completes (the
  alert-mode evaluation uses it). The tank (500 ml = 20 s of pump) is also tight for three fires.
* The simulation runs at about real time at best (`speed` above 1 only changes the target, the CPU is the limit; at `speed:=3` the
  firmware's timing drifts and the mission can stall), so batch evaluations run at `speed:=1.0`.
* `detection:=false` and `visuals:=false` (launch) are for batch runs: no CNN / camera / fusion, resp. no flame and jet updates.
  With `visuals:=false` a burnt-out fire keeps its solid footprint (it is moved away by the visuals update).

* Gazebo particles are flat squares and cannot be re-aimed at runtime, so smoke and sparks look blocky up close and the
  water jet is built from translucent visuals instead of particles.
* Flames are cut-out photos on crossed planes; they light the scene through point lights only.
* The Arduino robot stays stationary until a flame sensor sees fire and repeats its 21 s call cycle, so a fire takes
  several minutes of simulated time to put out (it only sprays 1.7 s per cycle).
* No occlusion for the flame sensors; the gas "cloud" is a static radial model that follows the fire's intensity.
* `models/keras_model.h5` (116 MB) is git-ignored (GitHub's 100 MB limit); the converted ONNX (39 MB) is committed.
