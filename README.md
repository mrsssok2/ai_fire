part-1 

hardware setup

![Image](https://github.com/user-attachments/assets/288571ed-6409-44a3-b5f8-e6b1c08be978)

[![Image](https://github.com/user-attachments/assets/3b467340-016c-4081-98b7-6d28d4433985)](https://github.com/user-attachments/assets/f9e05fa6-6d95-4072-93e8-d9cab5032dab)

part-2 

hardware setup 

![Image](https://github.com/user-attachments/assets/e2c91a8e-7f54-4b0f-87cf-5641dd09eb9a)


hsv fire detection 

![Image](https://github.com/user-attachments/assets/2ff3738e-379a-46db-808a-b0c6e4eb4143)


haar cascade model 


dataset := https://www.kaggle.com/datasets/atulyakumar98/test-dataset

we train the model using cascade trainer gui with the above images 

<img width="1401" height="795" alt="Image" src="https://github.com/user-attachments/assets/a9d3d52c-2ed3-4077-9070-c4a9b102f22f" />

<img width="1399" height="790" alt="Image" src="https://github.com/user-attachments/assets/f082edee-91a0-468c-b5a1-fdd1e5e90947" />

<img width="1403" height="800" alt="Image" src="https://github.com/user-attachments/assets/50b14a1e-6ae1-452f-aa8c-21aa9ce089ce" />


cnn model 

train a CNN (Convolutional Neural Network) model  using TensorFlow Keras to classify images as fire and. no fire

Database: https://www.kaggle.com/datasets/phylake1337/fire-dataset

model training : = https://colab.research.google.com/drive/1TbO_pBiTCtn0L5OkdX5SaEVEOY0zA8iE#scrollTo=bBZVol2PArKV

testing the model := https://colab.research.google.com/drive/1WOvQNlO0oBI94mtI9TmAuokz-i88wuzW?usp=sharing#scrollTo=xPUN897MaXq9


to run the model in raspberry pi 

```bash
source sttr/bin/activate
cd Desktop
python fire.py
```

![Image](https://github.com/user-attachments/assets/955407a2-d937-4914-b709-47bf708278e6)


![Image](https://github.com/user-attachments/assets/6f190dfc-c991-44fc-bef9-2fc9a0e4dee2)












---

## ROS 2 + Gazebo simulation (`ros2_sim/`)

My Arduino + Raspberry Pi fire-fighting robot rebuilt in **ROS 2 Jazzy + Gazebo Harmonic** (Docker): the Arduino logic of `fire_sensor.ino` is **ported line by line** (including its blocking `delay()`s), the camera detection runs **my own trained CNN** (`keras_model.h5`, converted to ONNX), and the robot, the water tank and the fires are simulated. Everything lives in [`ros2_sim/`](ros2_sim/); the original files above are untouched.

![Fire Detection window next to Gazebo](ros2_sim/docs/img/fire_sim_detection.png)

*1. The live Fire Detection window: robot camera with the CNN confidence banner (`FIRE DETECTED 100%`), boxes around fire regions, flame sensors L / F / R, gas level, water tank, pump, servo angle and the last SIM800L alert.*

![The robot with three fires](ros2_sim/docs/img/fire_sim_mission.png)

*2. The robot driving to three realistic fires (pit of logs, flame photos on crossed planes, flicker, light, sparks and smoke).*

![Spraying a fire](ros2_sim/docs/img/fire_sim_spraying.png)

*3. Spraying: pump ON, servo-swept nozzle, the water jet hits the fire, one fire is already out and the tank level is dropping.*

### Real robot vs simulation

| | Real robot (this repo) | Simulation (`ros2_sim/`) |
|---|---|---|
| Flame sensors | 3 digital IR modules (left / front / right), LOW = flame | 3 virtual sensors: cone + inverse-square model on the fires; **range and angle are my assumptions**, not measured |
| Gas sensor | MQ-type digital output -> SMS | virtual sensor: concentration decays with distance from a gas-leaking fire (static, no spreading cloud) |
| Detection | Pi camera: Keras CNN (`fire.py`), Haar cascade (`haar_cascade.py`, `hsv.py`) | simulated camera (320x240, 10 Hz): **my CNN** runs; HSV colour rule only proposes boxes; Haar / HSV nodes are planned |
| Control logic | `fire_sensor.ino` on the Arduino | `core/arduino_logic.py`: same pins, same delays (0.4 s loop, 0.5 s stop, servo 50-110-50 at 10 ms/deg, 7.15 s SMS, 20 s + 1 s call) |
| Pump + servo nozzle | relay + servo, sweep 50-110 deg | pump pin -> water use; the nozzle is on the servo horn and sweeps; the jet is a translucent visual (Gazebo particle emitters cannot be re-aimed) |
| Water tank | MCP3008 reading in `fire.py` | 500 ml, 25 ml/s **only while the pump runs**; `/water_level` in percent; level shown in the bottle; `tank_empty` alert |
| SMS / call | SIM800L AT commands | the same AT strings go to a SIM800L emulator that logs / publishes SMS and call events (no real message is sent) |
| Fire | real combustion | photo cut-outs + light + particles; goes out after 4 s of full spray. **Looks nothing like a real fire to a camera in detail** |
| Real vs simulated | everything | simulated: sensors, camera, fires, water, GSM modem. Not simulated: real IR/camera noise, smoke, wind, wheel slip beyond Gazebo's physics, the Pi and the Arduino hardware |

### 1. Pipeline

```mermaid
flowchart LR
  subgraph IN["Inputs (simulated)"]
    direction TB
    cam["Camera 320x240"]
    fl["3 flame sensors<br/>L / F / R"]
    gas["Gas sensor"]
  end
  subgraph DET["Detection"]
    direction TB
    ino["Flame-sensor logic<br/>(fire_sensor.ino, ported)"]
    cnn["CNN<br/>(my keras_model.h5 -> ONNX)"]
    plan["HSV / Haar / fusion<br/>(planned)"]
  end
  dec["Decision<br/>arduino_bot<br/>(loop + delays of the .ino)"]
  subgraph OUT["Actuators and alerts"]
    direction TB
    mot["Motors<br/>/cmd_vel -> Gazebo"]
    pump["Pump + water tank"]
    srv["Servo nozzle sweep 50-110"]
    gsm["SIM800L: SMS + call"]
  end
  fl --> ino
  gas --> ino
  ino --> dec
  cam --> cnn
  cnn -. "window only today" .-> win["Fire Detection window"]
  cnn -. "planned: confirm / fuse" .-> plan
  plan -.-> dec
  dec --> mot
  dec --> pump
  dec --> srv
  dec --> gsm
```

### 2. ROS 2 nodes and topics

```mermaid
flowchart LR
  gz["Gazebo Harmonic<br/>fire_world + fire_bot"]
  fs["fire_sensor_sim"]
  ab["arduino_bot"]
  gsm["sim800l"]
  fm["fire_manager"]
  ws["water_system"]
  cd["cnn_detector"]
  mon["fire_monitor<br/>window"]
  ms["mission"]
  gz -- "/fire_bot/odom, /joint_states" --> fs
  fm -- "/fires/state" --> fs
  fs -- "/sensors/flame_left, _front, _right<br/>/sensors/gas" --> ab
  ms -- "/mission/running" --> ab
  ab -- "/cmd_vel, /nozzle/cmd" --> gz
  ab -- "/pump/state" --> ws
  ab -- "/sim800l/tx" --> gsm
  gsm -- "/alerts" --> mon
  ws -- "/water_level, /water/spraying" --> fm
  ws -- "/water/level_cmd, /alerts tank_empty" --> gz
  fm -- "/alerts fire_out, /fires/state" --> ms
  gz -- "/camera/image_raw" --> cd
  cd -- "/fire/cnn_conf, /fire/detections" --> mon
  mon -- "/fire/annotated" --> out(["image topic"])
  mon -- "/mission/command start / reset" --> ms
  ms -- "/mission/state" --> mon
```

### 3. Mission state machine

The ported Arduino program does **not search**: it waits until a flame sensor sees fire.

```mermaid
stateDiagram-v2
  [*] --> Waiting
  Waiting --> Running: countdown ends or Start
  state Running {
    [*] --> WaitForFlame
    WaitForFlame --> Turn: side sensor sees flame
    Turn --> Approach: front sensor sees flame
    WaitForFlame --> Approach: front sensor sees flame
    Approach --> Spray: 0.4 s forward, then stop
    Spray --> Alert: pump + servo sweep 50-110-50
    Alert --> WaitForFlame: phone call 20 s, next loop
  }
  Running --> Done: all fires out
  Running --> TankEmpty: tank at 0 percent
  Waiting --> Waiting: Reset
  Running --> Waiting: Reset
  Done --> Waiting: Reset
  TankEmpty --> Waiting: Reset
```

`Done` shows `MISSION DONE: all fires out` and `TankEmpty` shows `TANK EMPTY: no water left`; neither closes or resets anything, the simulation keeps running until Ctrl+C. Reset puts the robot back, refills the tank and relights the fires.

### Findings

* **My CNN is a small binary Keras CNN, not a Teachable Machine model**: one sigmoid output, no labels file. On real photos the fire probability is `1 - output` (0..1 input scaling); it caught **18 of 19 real fires** but also flagged **27 of 35 hard non-fire photos** (many paintings and indoor scenes). Details: [`ros2_sim/docs/cnn_model.md`](ros2_sim/docs/cnn_model.md).
* **On the simulated fire** (robot camera, mean confidence): **0.98 at 0.6 m**, **0.68 at 1.0 m**, **0.30 at 1.6 m (missed)**; 0.27 looking away from every fire and 0.39 at the burnt-out pit ([`docs/cnn_sim_eval.json`](ros2_sim/docs/cnn_sim_eval.json)).
* **The firmware blocks for the phone call**: after each fire it waits 20 s (+1 s) in `delay()`, so it sprays only **~1.7 s per ~23 s cycle**. The motors also keep their last command during the 7 s SMS delay.
* **`hsv.py` is really a Haar cascade** (with a servo sweep); the repo contains no HSV thresholding, and the model files the scripts load (`labels.txt`, the cascade XML) are not in the repo either.
* **Motor wiring assumption:** `fire_sensor.ino` drives the left motor when the *left* sensor fires, which turns the robot away from the fire unless the motor pins are wired mirrored. The simulation assumes mirrored wiring (`swap_motor_sides`, default true; `false` runs the pins exactly as written).
* In the default three-fire layout the robot reaches all fires and the mission ends with `MISSION DONE` (276 s of simulated time in my check run with a 2 s extinguish time, `ros2_sim/scripts/check_mission.py`).
* `models/keras_model.h5` is 116 MB, above GitHub's file limit, so only the converted `fire_cnn.onnx` (39 MB) is in the repo.

### Next steps (planned, not done yet)

* HSV and Haar-cascade detector nodes on the same interface as the CNN.
* Flame-sensor + CNN fusion (sensors trigger, CNN confirms, confidence weighting, flame direction for aiming), with decoys (red / orange objects, a sunlit window, a hot lamp) and an evaluation of false alarms and missed fires per method versus fused.
* A non-blocking fix for the 20 s call (and the SMS), with before / after numbers for time under water per cycle.

### Run it

Needs Docker Desktop (WSL2 with WSLg for the Gazebo window) and the local image `gurdy2/ros:jazzy` (ROS 2 Jazzy + Gazebo Harmonic; the `ros2_sim/Dockerfile` builds on it).

```bash
cd ~/projects/ai_fire/ros2_sim
./run.sh                # Gazebo (camera follows the robot) + the Fire Detection window, half speed
```

* The robot waits 15 s (countdown on the window) or until you press **Enter** (terminal) / click **START MISSION**.
* **R** (window) or **r + Enter** (terminal) = reset: robot back at the start, tank refilled, fires relit.
* `./run.sh speed:=1.0` = real time; `./run.sh start_delay:=0` = start at once; `./run.sh gui:=false monitor_window:=false` = headless. **Ctrl+C** quits.
* Tests: `docker run --rm -v $PWD/ros2_sim:/home/ubuntu/ros2_sim fire_ros2:jazzy bash -c 'source /opt/ros/jazzy/setup.bash; cd ~/ros2_sim/src/fire_robot_sim && python3 -m pytest -q'` (50 tests). More in [`ros2_sim/README.md`](ros2_sim/README.md).

**Tech stack:** ROS 2 Jazzy, Gazebo Harmonic (`ros_gz`, DiffDrive, particle emitters), xacro/URDF, Python 3.12, onnxruntime, OpenCV, NumPy, Keras CNN (converted to ONNX), Docker (WSL2 + WSLg), pytest.
