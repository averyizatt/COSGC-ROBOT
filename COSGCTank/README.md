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

- **WASD/buttons**: press-and-hold sends keepalives; release stops.
- **Xbox controller**: connect it to your laptop/browser; the page will send joystick updates while in `rc` mode.
- **RC speed**: adjust `rc_speed` in the Tuning panel for faster/slower response.
