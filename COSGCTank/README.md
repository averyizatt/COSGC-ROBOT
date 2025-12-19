# COSGC Tank — Autonomous Rover (COSGC)

Overview

The COSGC Tank is a compact autonomous rover platform built around a Raspberry Pi 4, a Pi camera, and a dual H-bridge motor driver. The software is modular and split into perception, decision, motor-control, visualization, and monitoring layers to make it easy to test, tune, and extend.

This README documents the current architecture, module responsibilities, endpoints, expected inputs/outputs, autonomy behavior, and suggested future upgrade paths.

Table of contents
- High-level architecture
- Perception modules (inputs/outputs)
- Decision logic and autonomy
- Motor control interface and safety
- Visualization and web dashboard
- File layout and key modules
- How to extend and upgrade

High-level architecture

The system is organized into five layers:

1) Perception — processes camera frames into structured observations. Modules: `detector.py`, `boundaries.py`, `terrain.py`.
2) Decision — converts perception outputs into a short declarative command (FORWARD, TURN_LEFT, STOP_REVERSE, etc). Module: `decision.py`.
3) Motor Control — executes motor commands via GPIO, with safety checks and an abstract interface to allow swapping implementations. Module: `motor_control.py`.
4) Visualization / Overlay — draws debugging overlays on video frames (bounding boxes, lines, decision text). Module: `overlay.py`.
5) Monitoring / Dashboard — a Flask app provides manual control, live console, and log ingestion endpoints. Module: `rover_server.py`.

Perception modules (detailed)

- `camera.py` (FrameProvider)
  - Purpose: capture frames from the Pi camera (Picamera2) or fallback to OpenCV `VideoCapture`, provide consistent frames for the pipeline, handle resizing and ROI, and expose basic FPS/timestamping.
  - Inputs: none (reads camera hardware)
  - Outputs: `dict` per frame: `{ 'frame': numpy array (H x W x 3), 'timestamp': float, 'width': int, 'height': int }`

- `detector.py` (ObstacleDetector)
  - Purpose: wrapper for a TFLite SSD detector (MobileNet SSD recommended). Responsible for preprocessing, model invocation, and returning normalized detections.
  - Inputs: RGB frame (H x W x 3)
  - Outputs: list of detections: `[{ 'class': int, 'score': float, 'box': [ymin, xmin, ymax, xmax] }, ...]` (normalized coords)

- `boundaries.py` (BoundaryDetector)
  - Purpose: detect left/right path boundaries using edge detection and Hough/line clustering. Returns averaged line segments for left/right edges.
  - Inputs: RGB frame or ROI
  - Outputs: `{ 'left_boundary': [[x1,y1,x2,y2], ...], 'right_boundary': [[...], ...] }` (image coordinates)

- `terrain.py` (TerrainAnalyzer)
  - Purpose: detect dips (holes) and inclines using lower-frame analysis (gradient, edge density, variance). Returns booleans and metric values for tuning.
  - Inputs: lower-band RGB frame
  - Outputs: `{ 'dip_detected': bool, 'incline_detected': bool, 'edge_density': float, 'variance': float }`

Decision layer (detailed)

- `decision.py` (DecisionMaker)
  - Purpose: convert perception outputs into a prioritized command and textual reason. Implements:
    - Priority hierarchy: DIP → OBSTACLE → BOUNDARY → INCLINE → FORWARD
    - Hysteresis / smoothing to avoid oscillation (command persistence counters)
    - Emergency override: DIP causes immediate `STOP_REVERSE` and motor kill
    - Adjustable thresholds and tunable parameters (exposed via variables)
  - Input: structured perception dict
  - Output: `{ 'command': <string>, 'reason': <string>, 'confidence': <float> }`

Motor control and safety

- `motor_control.py` (MotorController)
  - Purpose: abstract motor control operations with safety features and a simulated fallback.
  - Key functions: `forward(speed)`, `reverse(speed)`, `turn_left(speed)`, `turn_right(speed)`, `stop()`, `safe_stop()`.
  - Safety: watchdog heartbeat, immediate stop on critical errors.
  - Implementation notes: uses `RPi.GPIO` when available; otherwise provides a no-op simulator that logs actions (useful for development on non-RPi machines).

Visualization & Dashboard

- `overlay.py` (OverlayDrawer)
  - Purpose: draw boxes, lines, decision text, FPS and telemetry onto frames for the web console or local display.
  - Also exports the perception+decision JSON for the Flask endpoint to consume.

- `rover_server.py` (Flask)
  - Provides these endpoints:
    - `GET /` — manual control UI (control.html)
    - `GET /cmd/<action>` — manual motor commands (f, b, l, r, s)
    - `GET /led/<state>` — toggle LEDs
    - `POST /log` — ingest JSON logs from the vision pipeline (perception + decision)
    - `GET /console` — live console UI
    - `GET /console_data` — returns latest perception+decision JSON

How autonomy works (runtime flow)

1. Frame capture: `camera.py` yields frames at configured FPS.
2. Perception: frame is passed to `detector.detect()`, `boundaries.detect()`, and `terrain.analyze()`.
3. Decision: `DecisionMaker.decide(perception)` returns a command and reason.
4. Motor control: the selected command is mapped to one of the MotorController methods.
5. Overlay and logging: `OverlayDrawer` draws overlays and the server receives JSON logs for the dashboard.

Typical command mapping
- `FORWARD` → `motor.forward()` (moderate speed)
- `TURN_LEFT` → `motor.turn_left()` (short duration steering)
- `TURN_RIGHT` → `motor.turn_right()`
- `STOP_REVERSE` → `motor.safe_stop(); motor.reverse(short)` (escape maneuver)
- `SLOW` → `motor.forward(low_speed)`
- `ADJUST_LEFT` / `ADJUST_RIGHT` → small steering correction while forward

File layout (key files)
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
  utils.py
  templates/
    control.html
    console.html
  models/
    ssd_mobilenet_v1.tflite
```

Future upgrade paths

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
