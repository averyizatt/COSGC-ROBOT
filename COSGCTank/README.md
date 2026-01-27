# COSGC Tank — Autonomous Rover (COSGC)

## Overview

The COSGC Tank is a compact autonomous rover platform built around a Raspberry Pi 4, a Pi camera, and a dual H-bridge motor driver. The software is modular and split into perception, decision, motor-control, visualization, and monitoring layers to make it easy to test, tune, and extend.

This README documents the current architecture, module responsibilities, endpoints, expected inputs/outputs, autonomy behavior, and suggested future upgrade paths.

## Table of contents
- High-level architecture
- Modes (RC vs Autonomous)
- Perception modules (inputs/outputs)
- Decision logic and autonomy
- Live tuning (web settings)
- Motor control interface and safety
- Visualization and web dashboard
- File layout and key modules
- How to extend and upgrade

## What it does (quick)

- **RC driving from a webpage:** press-and-hold WASD/buttons, plus Xbox controller support.
- **Autonomous driving pipeline:** camera → obstacle detection + boundary detection + terrain analysis → decision → motor control.
- **SLAM integration hooks:** polls a separate SLAM process via `slam/bridge.py` (`GET /pose`) and uses pose for navigation/stuck detection.
- **Dynamic obstacle tracking (debug):** tracks moving obstacles in world space using a lightweight Kalman filter + Hungarian assignment.
- **Navigation (debug/early):** occupancy grid + A* planning with optional path smoothing.
- **Competition-friendly perception:** brightness normalization + contour fallback + texture-based “rockiness” score to help in bright sandy scenes.
- **Safety-first control:** keepalive watchdog stops motors if the network/client drops; emergency stop endpoint.
- **Live tuning:** adjust thresholds (gamma/contours/stop y-threshold/RC speed/etc.) from the web UI while the rover is running.

## High-level architecture

The system is organized into five layers:

1) Perception — processes camera frames into structured observations. Modules: `detector.py`, `boundaries.py`, `terrain.py`.
2) Decision — converts perception outputs into a short declarative command (FORWARD, TURN_LEFT, STOP_REVERSE, etc). Module: `decision.py`.
3) Motor Control — executes motor commands via GPIO, with safety checks and an abstract interface to allow swapping implementations. Module: `motor_control.py`.
4) Visualization / Overlay — draws debugging overlays on video frames (bounding boxes, lines, decision text). Module: `overlay.py`.
5) Monitoring / Dashboard — a Flask app provides manual control, live console, and log ingestion endpoints. Module: `rover_server.py`.

## Modes (RC vs Autonomous)

The rover supports multiple modes selectable from the web UI:

- `rc`: Manual RC control from the browser (WASD/buttons or Xbox controller).
- `autonomous`: General autonomous mode (your autonomy loop decides how to use it).
- `single_cam`: Autonomous mode intended for a single camera pipeline.
- `stereo_cam`: Autonomous mode intended for a stereo pipeline (debugging/expansion).

Mode is stored by the Flask server and can be queried/changed via:

- `GET /mode` → `{ "mode": "rc" }`
- `POST /mode` with JSON `{ "mode": "rc" | "autonomous" | "single_cam" | "stereo_cam" }`

When leaving `rc`, the server forces a motor stop and clears active commands.

## Perception modules (detailed)

- `camera.py` (FrameProvider)
  - Purpose: capture frames from the Pi camera (Picamera2) or fallback to OpenCV `VideoCapture`, provide consistent frames for the pipeline, handle resizing and ROI, and expose basic FPS/timestamping.
  - Inputs: none (reads camera hardware)
  - Outputs: `dict` per frame: `{ 'frame': numpy array (H x W x 3), 'timestamp': float, 'width': int, 'height': int }`

- `detector.py` (ObstacleDetector)
  - Purpose: wrapper for a TFLite SSD detector (MobileNet SSD recommended). Responsible for preprocessing, model invocation, and returning normalized detections.
  - Bright-sand hardening:
    - LAB/HSV CLAHE + gamma correction to reduce glare and blown highlights.
    - Contour-based fallback detector to catch high-contrast rocks/edges when the model misses.
    - Texture-based **rockiness score** (`rock_score` in $[0,1]$) computed from gradient/texture to help distinguish rocks vs smooth sand.
  - Inputs: RGB frame (H x W x 3)
  - Outputs: list of detections: `[{ 'class': int, 'score': float, 'box': [ymin, xmin, ymax, xmax] }, ...]` (normalized coords)
  - Additional fields (when enabled):
    - `label`: e.g. `contour` for contour detections
    - `rock_score`: float in $[0,1]$
    - `rock_like`: boolean, thresholded by `det_rock_score_threshold`

- `boundaries.py` (BoundaryDetector)
  - Purpose: detect left/right path boundaries using edge detection and Hough/line clustering. Returns averaged line segments for left/right edges.
  - Inputs: RGB frame or ROI
  - Outputs: `{ 'left_boundary': [[x1,y1,x2,y2], ...], 'right_boundary': [[...], ...] }` (image coordinates)

- `terrain.py` (TerrainAnalyzer)
  - Purpose: detect dips (holes) and inclines using lower-frame analysis (gradient, edge density, variance). Returns booleans and metric values for tuning.
  - Inputs: lower-band RGB frame
  - Outputs: `{ 'dip_detected': bool, 'incline_detected': bool, 'edge_density': float, 'variance': float }`

## Decision layer (detailed)

- `decision.py` (DecisionMaker)
  - Purpose: convert perception outputs into a prioritized command and textual reason. Implements:
    - Priority hierarchy: DIP → OBSTACLE → BOUNDARY → INCLINE → FORWARD
    - Hysteresis / smoothing to avoid oscillation (command persistence counters)
    - Emergency override: DIP causes immediate `STOP_REVERSE` and motor kill
    - Stuck detection (uses SLAM pose deltas when available): if commanded forward but pose doesn’t change, triggers an escape maneuver
    - Adjustable thresholds and tunable parameters (now tunable live from `/settings`)
  - Input: structured perception dict
  - Output: `{ 'command': <string>, 'reason': <string>, ... }` (may include `stuck: true`)

## Live tuning (web settings)

The Flask server stores tunable parameters in-memory under `SETTINGS` and exposes:

- `GET /settings` → returns the current settings JSON
- `POST /settings` → updates known keys (typed coercion for bool/int/float)

Current tuning keys include:

- Detector:
  - `det_gamma`
  - `det_contour_min_area`
  - `det_clahe_clip`
  - `det_rock_score_threshold`
- Decision:
  - `dec_obstacle_stop_ymin`
  - `dec_ultra_stop_cm`
  - `dec_ultra_turn_cm`
  - `dec_stuck_dx_m`
  - `dec_stuck_frames`
- RC:
  - `rc_speed`
  - `rc_turn_gain`

The main loop (`main.py`) polls `/settings` about once per second and:

- injects `perception['settings']` for modules that read it (navigator/decision)
- updates detector tuning live via `det.settings = settings_cache`

### Settings persistence (survives reboot)

`rover_server.py` now loads/saves settings to `COSGCTank/settings.json`.

- On startup, it loads known keys from `settings.json` if present.
The server clamps these keys to prevent unsafe/invalid values:

- `rc_speed`: $[0.2, 1.0]$
- `det_contour_min_area`: $[50, 20000]$
- `det_clahe_clip`: $[0.5, 10.0]$
- `det_rock_score_threshold`: $[0.0, 1.0]$
- `dec_obstacle_stop_ymin`: $[0.0, 1.0]$
- `dec_ultra_stop_cm`: $[2, 200]$
- `dec_stuck_dx_m`: $[0.0, 1.0]$
- `dec_stuck_frames`: $[1, 50]$
- `auto_speed`, `auto_speed_slow`, `auto_speed_turn`, `auto_speed_stuck`: $[0.0, 1.0]$


The autonomy stack supports an HC-SR04-style ultrasonic rangefinder mounted near the camera.

### GPIO choice (BCM)

To avoid conflicts with the DRV8833 motor pins (`17, 27, 23, 18, 4`), the rover uses:

- **TRIG**: `GPIO 24`
- **ECHO**: `GPIO 25`

These are the defaults in `ultrasonic.py`, and `main.py` initializes the sensor with these pins.

### Wiring (important)
- **ECHO is typically 5V** and **must be level-shifted** down to **3.3V** for Pi `GPIO 25`.
  - Use a voltage divider (example: 1k (top) + 2k (bottom) gives ~3.3V), or a logic-level shifter.

### Usage in code

`main.py` reads the sensor and injects:
- `perception['distance_cm']` (float cm or `None`)

`decision.py` uses `distance_cm` with these tunables:
- `dec_ultra_turn_cm`: if distance is below this (but above stop), bias turning/avoidance

The reader uses a small median filter (`samples=3`) and timeouts to avoid blocking the main loop.

## Stereo camera assumptions

You mentioned a **USB stereo camera** with:


Current code treats `stereo_cam` as a *mode flag* and does not yet perform true stereo depth.

MPU-6050 uses the Raspberry Pi's I2C bus (not arbitrary GPIO):

- **SDA**: `GPIO 2` (physical pin 3)
- **3.3V**: physical pin 1 (or 17)
- **GND**: physical pin 6 (or any GND)
Address selection:

- `AD0` HIGH → `0x69`


### Enable I2C on the Pi


```bash
sudo raspi-config
```


```bash
sudo apt-get update
sudo apt-get install -y i2c-tools
i2cdetect -y 1
```

You should see `68` (or `69`) in the address table.

### Python dependency

The IMU reader uses `smbus2`:

```bash
python3 -m pip install smbus2
```

### What gets added to perception

`main.py` attaches:

- `perception['imu']` → accel (m/s²), gyro (rad/s), temperature (°C), and a simple complementary-filter roll/pitch estimate.

### Tunables (saved in settings.json)

These keys are exposed under autonomy tuning and are persisted via `settings.json`:

- `imu_enabled` (bool)
- `imu_i2c_bus` (default 1)
- `imu_i2c_addr` (default 0x68)
- `imu_alpha` (complementary filter alpha, default 0.98)
- `imu_dlpf_cfg` (0..6, default 3)
- `imu_sample_div` (0..255, default 4)
- `imu_gyro_calib_seconds` (0.2..10.0, default 1.0)

## Combined pin map (quick reference)

- **DRV8833 motor driver (BCM)**: `AIN1=12`, `AIN2=13`, `BIN1=24`, `BIN2=23`, `STBY=26`
- **Ultrasonic HC-SR04 (BCM)**: `TRIG=6` (physical 31), `ECHO=12` (physical 32) — ECHO must be level-shifted to 3.3V.
- **I2C (IMU MPU-6050)**: On Jetson Nano header, `SDA=pin 3`, `SCL=pin 5` (typically `/dev/i2c-1`). The code uses `i2c_bus=1` by default.

## Motor control and safety

- `motor_control.py` (MotorController)
  - Purpose: abstract motor control operations with safety features and a simulated fallback.
  - Key functions: `forward(speed)`, `reverse(speed)`, `turn_left(speed)`, `turn_right(speed)`, `stop()`, `safe_stop()`.
  - Safety: watchdog heartbeat, immediate stop on critical errors.

  ### Network safety (keepalive watchdog)

  The server uses a keepalive watchdog:

  - The web UI sends periodic keepalives while a key/button is held.
  - If the keepalive stops (Wi‑Fi drop / browser closed), the server stops the motors after a short timeout.

  ### Emergency stop

  - `POST /emergency_stop` → sets E‑STOP and stops motors
  - `POST /clear_emergency` → clears E‑STOP
  - Implementation notes: uses `RPi.GPIO` when available; otherwise provides a no-op simulator that logs actions (useful for development on non-RPi machines).

## Visualization & Dashboard

- `overlay.py` (OverlayDrawer)
  - Purpose: draw boxes, lines, decision text, FPS and telemetry onto frames for the web console or local display.
  - Also exports the perception+decision JSON for the Flask endpoint to consume.
  - Debug overlays include:
    - contour boxes vs model boxes (different colors)
    - terrain roughness metric (`variance`)
    - stuck indicator when escape triggers
    - rockiness score (texture classifier)

- `rover_server.py` (Flask)
  - Provides these endpoints:
    - `GET /` — manual control UI (control.html)
    - `GET /cmd/<action>` — manual motor commands (f, b, l, r, s)
    - `GET /cmd/<action>/<state>` — press/hold semantics (`start` / `stop`)
    - `POST /keepalive` — safety keepalive while holding controls
    - `POST /joystick` — Xbox/gamepad analog input
    - `GET/POST /mode` — switch modes (rc/autonomous/single_cam/stereo_cam)
    - `GET/POST /settings` — live tuning
    - `GET /status` — server state: active actions, e-stop, mode, smoothing
    - `GET /led/<state>` — toggle LEDs
    - `POST /log` — ingest JSON logs from the vision pipeline (perception + decision)
    - `GET /console` — live console UI
    - `GET /console_data` — returns latest perception+decision JSON

## How autonomy works (runtime flow)

1. Frame capture: `camera.py` yields frames at configured FPS.
2. Perception: frame is passed to `detector.detect()`, `boundaries.detect()`, and `terrain.analyze()`.
3. Decision: `DecisionMaker.decide(perception)` returns a command and reason.
4. Motor control: the selected command is mapped to one of the MotorController methods.
5. Overlay and logging: `OverlayDrawer` draws overlays and the server receives JSON logs for the dashboard.

Optional navigation/SLAM:

- `main.py` polls the SLAM bridge at `http://127.0.0.1:5001/pose`.
- If a pose is available, it is added to `perception['slam_pose']`.
- `navigator.py` can use SLAM pose + obstacles to build an occupancy grid and plan with A*.

Dynamic obstacle tracking:

- `main.py` converts obstacle boxes into rough world points (heuristic) and provides `perception['dynamic_detections']`.
- `navigator.py` tracks them via `dynamic.py` and marks predicted track positions as occupied.

Typical command mapping
- `FORWARD` → `motor.forward()` (moderate speed)
- `TURN_LEFT` → `motor.turn_left()` (short duration steering)
- `TURN_RIGHT` → `motor.turn_right()`
- `STOP_REVERSE` → `motor.safe_stop(); motor.reverse(short)` (escape maneuver)
- `SLOW` → `motor.forward(low_speed)`
- `ADJUST_LEFT` / `ADJUST_RIGHT` → small steering correction while forward

## File layout (key files)
```
COSGCTank/
  camera.py
  detector.py
  boundaries.py
  terrain.py
  decision.py
  overlay.py
  rover_server.py
  main.py
  dynamic.py
  navigator.py
  utils.py
  templates/
    control.html
    console.html
  models/
    ssd_mobilenet_v1.tflite
```

## Future upgrade paths

Phase 2 — Stable driving & navigation
- Add PID steering controller that keeps the vehicle centered between left/right boundaries.
- Add adaptive speed control based on incline and path clearance.
- Add dashboard tuning values for thresholds and PID constants.

Phase 3 — Mission logic
- Introduce `navigator.py` that accepts waypoints or mission modes (explore, return-to-base).
- Add GPS + waypoint tracking; higher-level state machine for mission progress.

Phase 4 — Multi-sensor fusion
- Add IMU + wheel encoders + stereo or depth sensor and perform sensor fusion (complementary filter / EKF).
- Replace simple detector with stereo depth-based obstacle distance estimation or integrate a LiDAR unit.

Phase 5 — Safety systems
- Add software watchdog and physical E-stop wiring.
- Implement heartbeat and redundancy checks between the Pi and any microcontroller motor controller.

Getting started

1. Install dependencies:
```bash
sudo apt update
sudo apt install -y python3-opencv python3-pip
pip3 install tflite-runtime flask picamera2
```
2. Copy a TFLite model to `models/` or use the included sample.
3. Start the Flask server:
```bash
python3 rover_server.py
```
4. Run the rover pipeline (example):
```bash
python3 main.py
```

RC control tips
## Raspberry Pi Setup

This project is optimized for Raspberry Pi. Enable required interfaces and install packages:

### Enable interfaces

```bash
sudo raspi-config
# Interface Options → enable Camera and SPI
```

### Install system packages

```bash
sudo apt update
sudo apt install -y python3-opencv python3-pip python3-picamera2 python3-rpi.gpio python3-spidev
```

### Python packages

```bash
pip3 install --upgrade pip
pip3 install -r requirements.txt
```

### Camera notes

- Picamera2 is preferred; `FrameProvider` uses it automatically when available.
- USB cameras use V4L2; ensure `/dev/video0` exists.

### TFT + Switches (SPI)

- Enable SPI via `raspi-config` → Interface Options → SPI
- Wiring (BCM): TFT DC=25, RST=24, SPI0 CE0; BTN1=23, SW1=22, SW2=27 (active‑low to GND)
- Files: `tft_display.py`, `hardware_switches.py` (integrated in `main.py`)

## Jetson Nano (JetPack 4.6.4) Setup

The rover now supports NVIDIA Jetson Nano on JetPack 4.6.4 using GStreamer and the Jetson camera stack. On Jetson, the code auto-detects the platform and opens the CSI camera via a GStreamer `nvarguscamerasrc` pipeline for hardware-accelerated capture.

### Install system dependencies

```bash
sudo apt update
sudo apt install -y \
  python3-opencv python3-pip \
  gstreamer1.0-tools gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  v4l-utils python3-jetson-gpio
```

Note: `requirements.txt` skips `opencv-python` on `aarch64`. Use the system `python3-opencv` that ships with JetPack (CUDA-enabled).

### Python packages

```bash
pip3 install --upgrade pip
pip3 install -r requirements.txt
```

### Camera notes (CSI and USB)

- CSI (Raspberry Pi Camera Module / IMX219): opened via `nvarguscamerasrc` automatically. No extra config needed; the default `FrameProvider()` will use the Jetson pipeline when detected.
- USB cameras: opened via V4L2 (MJPG preferred). If using a USB camera exclusively, no changes are required.

You can sanity-check the CSI camera with GStreamer:

```bash
gst-launch-1.0 nvarguscamerasrc ! nvoverlaysink
```

### Run

```bash
python3 rover_server.py &
python3 main.py
```

If you need GStreamer debugging, you can enable it temporarily:

```bash
export GST_DEBUG=2
python3 main.py
```

### Performance tips (Jetson Nano)

- Power/perf modes:
  ```bash
  sudo nvpmodel -m 0   # max performance mode
  sudo jetson_clocks   # lock clocks to max
  ```
- CUDA/OpenCV libs are typically under `/usr/local/cuda` and `/usr/lib/aarch64-linux-gnu` on JetPack 4.6.4. If needed, export:
  ```bash
  export CUDA_HOME=/usr/local/cuda
  export LD_LIBRARY_PATH=/usr/local/cuda/lib64:/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
  ```
- GPIO access: ensure your user is in the `gpio` group:
  ```bash
  sudo usermod -aG gpio $USER
  newgrp gpio
  ```
- OpenCV: the detector enables `cv2.setUseOptimized(True)` and uses a small thread count by default. You can tune via settings key `cv_threads`.

### TensorRT Detector (optional)

For higher detector throughput on Nano, convert your SSD ONNX model to a TensorRT engine and enable the TRT backend:

1) Convert ONNX → TRT:
```bash
cd COSGC-ROBOT/COSGCTank
chmod +x tools/trtexec_convert.sh
./tools/trtexec_convert.sh models/mobilenet_ssd.onnx models/mobilenet_ssd.engine
```
2) Enable in settings (via `/settings` POST or in code):
- `use_trt: true`
- `trt_engine_path: "models/mobilenet_ssd.engine"`
- Optional: `trt_input`, `trt_outputs` if your engine uses custom binding names.

The detector will fall back to TFLite when TRT isn’t available or the engine path is missing.

### CUDA-accelerated resize

If your OpenCV build includes CUDA modules, the detector uses CUDA for image resize before inference (`use_cuda_resize: true` in settings). Color-space normalization remains on the CPU for robustness.


- **WASD/buttons**: press-and-hold sends keepalives; release stops.
- **Xbox controller**: connect it to your laptop/browser; the page will send joystick updates while in `rc` mode.
- **RC speed**: adjust `rc_speed` in the Tuning panel for faster/slower response.
