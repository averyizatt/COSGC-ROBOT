# ESP32 Tank Rover — COSGC Competition

Firmware for an ESP32 WROOM-32 tracked rover built for the Colorado Space Grant Consortium (COSGC) competition. The rover navigates sand pits, inclines, and obstacle fields using dual ultrasonic sensors, a 6-axis IMU, hall-effect wheel encoders, and a layered safety motor controller.

---

## Table of Contents

- [Hardware Overview](#hardware-overview)
- [Pin Map](#pin-map)
- [Building & Uploading](#building--uploading)
- [Operation Modes](#operation-modes)
  - [Mode 0 — RC Control](#mode-0--rc-control-led-blue--green)
  - [Mode 1 — UART Control](#mode-1--uart-control-led-red)
  - [Mode 2 — Autonomous Navigation](#mode-2--autonomous-navigation-led-orange)
  - [Mode 3 — Simple Autonomous](#mode-3--simple-autonomous-led-purple)
  - [Mode 4 — Wall Follow](#mode-4--wall-follow-led-cyan)
  - [Mode 5 — Pre-Map Navigation](#mode-5--pre-map-navigation-led-yellow)
- [Mode Button](#mode-button)
- [Web Dashboard](#web-dashboard)
- [Sensors](#sensors)
- [Motor Safety System](#motor-safety-system)
- [Terrain Handling](#terrain-handling)
- [Self-Righting Arm](#self-righting-arm)
- [Occupancy Map & Path Planner](#occupancy-map--path-planner)
- [Hall-Effect Encoders & Odometry](#hall-effect-encoders--odometry)
- [LED Status](#led-status)
- [Serial Commands](#serial-commands)
- [UART Protocol](#uart-protocol)
- [Project Structure](#project-structure)
- [Key Tuning Constants](#key-tuning-constants)
- [Troubleshooting](#troubleshooting)

---

## Hardware Overview

| Component | Part | Interface |
|---|---|---|
| Microcontroller | ESP32 WROOM-32 DevKit V1 | — |
| Motor drivers (×2) | DRV8871 | LEDC PWM |
| Forward ultrasonic (×2) | HC-SR04 | `pulseIn()` |
| IMU | MPU6050 | I2C 0x68 |
| Wheel encoders (×2) | A3144 hall-effect | GPIO interrupt |
| Self-righting arm | Digital servo (DS3235) | LEDC PWM at 330 Hz |
| Status LED | Common-cathode RGB (3 legs) | LEDC PWM |
| Mode button | Tactile, active-LOW | GPIO, pull-up |
| Xbox controller | Xbox One/Series via BLE | Bluepad32 |

**Battery:** 16 V | **Track width:** 150 mm (center-to-center) | **Ground clearance:** 50 mm  
**PWM:** 20 kHz, 8-bit (0–255), 4 permanently attached LEDC channels  
**Max safe current:** 3.5 A per motor (DRV8871 abs max = 3.6 A) → `maxSafePWM ≈ 222` (87% duty)

---

## Pin Map

| Function | GPIO | Notes |
|---|---|---|
| **Motor A IN1** (Left fwd) | 33 | DRV8871 driver 1 |
| **Motor A IN2** (Left rev) | 25 | DRV8871 driver 1 — DAC1 disabled at boot |
| **Motor B IN1** (Right fwd) | 26 | DRV8871 driver 2 — DAC2 disabled at boot |
| **Motor B IN2** (Right rev) | 4 | DRV8871 driver 2 |
| **Ultrasonic LEFT TRIG** | 13 | HC-SR04, ~15° left of center |
| **Ultrasonic LEFT ECHO** | 34 | Input-only GPIO |
| **Ultrasonic RIGHT TRIG** | 19 | HC-SR04, ~15° right of center |
| **Ultrasonic RIGHT ECHO** | 18 | |
| **I2C SDA** (MPU6050) | 14 | |
| **I2C SCL** (MPU6050) | 27 | 100 kHz |
| **Encoder LEFT** | 35 | A3144, input-only, external 10 kΩ pull-up |
| **Encoder RIGHT** | 32 | A3144, external 10 kΩ pull-up |
| **Servo** (self-righting arm) | 15 | 330 Hz, 500–2500 µs |
| **Button** | 5 | Active-LOW with internal pull-up |
| **LED Red** | 21 | 220 Ω resistor, common-cathode RGB |
| **LED Green** | 22 | 220 Ω resistor |
| **LED Blue** | 23 | 220 Ω resistor |
| **UART TX** | 1 | 115200 baud (currently disabled — shares USB-CDC) |
| **UART RX** | 3 | |

> **GPIO 25 and 26** share silicon with DAC1/DAC2. The firmware explicitly disables both DAC channels at boot before attaching LEDC, otherwise the DAC silently overrides PWM output.

> **GPIO 12** is reserved — it sets the ESP32 flash voltage on boot. Never use it as an output.

---

## Building & Uploading

Requires [PlatformIO](https://platformio.org/) (VS Code extension or CLI).

```bash
# Build only
pio run

# Build and upload
pio run --target upload

# Serial monitor (115200 baud)
pio device monitor
```

The board target is `esp32dev` in `platformio.ini`. Build time is approximately 25–30 seconds.

---

## Operation Modes

The rover has **6 modes** cycled in order with a short button press. The LED color changes to identify the current mode.

```
UART → RC → Autonomous → Simple Auto → Wall Follow → Pre-Map Nav → UART → ...
```

---

### Mode 0 — RC Control (LED: Blue / Green)

Xbox One or Xbox Series controller over Bluetooth Low Energy, using the [Bluepad32](https://bluepad32.readthedocs.io/) library.

**Controls:**

| Input | Action |
|---|---|
| Left stick Y-axis | Drive forward / backward |
| Right stick X-axis | Steer left / right (skid steer differential) |
| Right Trigger (RT) | Self-righting arm forward (0° → 180°) |
| Left Trigger (LT) | Self-righting arm reverse (180° → 0°) |
| Both triggers released | Arm stows at 90° (center) |

**Skid steer mixing:**
```
leftMotor  = leftStick + rightStick
rightMotor = leftStick - rightStick
```
Both values are clamped to `±MAX_PWM (255)` and then passed through the full motor safety stack.

**Bluetooth lifecycle:**
- When switching **into** RC mode: BT radio is re-enabled (`btStart()`), WiFi AP remains active.
- When switching **out of** RC mode: BT is fully torn down (`btStop()` + `esp_bt_controller_deinit()`), freeing ~60 KB for the WiFi AP.
- The LED turns **blue** while waiting for a controller and **green** once connected.
- Pairing: put the controller in pairing mode (hold Xbox + Y for ~3 s), then short-press the mode button while in RC mode.
- If no controller is connected, motors are stopped but mode stays RC.

**Motor watchdog:** If no RC input is received for 500 ms (e.g. controller disconnects or signal drops), motors stop automatically.

---

### Mode 1 — UART Control (LED: Red)

Accepts binary motor commands from a host computer via the hardware UART (115200 baud). This is the **default startup mode**.

> **Note:** `Serial1` (GPIO 1/3) is currently disabled because it conflicts with the USB-CDC debug port on this WROOM-32 board. Re-enable by choosing non-overlapping pins and setting `serial1Ready = true` in `uart_comm.cpp::begin()`.

While hardware UART is disabled, you can still issue all **Serial Monitor commands** (see [Serial Commands](#serial-commands)) and receive debug telemetry. The UART command callbacks (`handleMotorCommand`, `handleCustomCommand`) remain registered and will fire if UART is re-enabled.

**Packet format:**
```
[0xFF] [TYPE] [LENGTH] [DATA...] [CHECKSUM] [0xFE]
```
Checksum = XOR of type + length + all data bytes. The receiver validates frame boundaries, length field bounds, and checksum before processing.

---

### Mode 2 — Autonomous Navigation (LED: Orange)

Full autonomous mode driven by the `AutonomousNav` state machine and fed real-time data from all sensors. This is the primary competition mode.

#### State Machine

```
NAV_CRUISE → NAV_AVOID → NAV_RECOVER → NAV_HAZARD
     ↑______________|___________|__________↑
```

| State | Description |
|---|---|
| `NAV_CRUISE` | Full-speed forward, heading-hold, waypoint following, frontier exploration |
| `NAV_AVOID` | Obstacle detected — back up then turn |
| `NAV_RECOVER` | Repeated avoidance failures — rock-and-roll recovery sequence |
| `NAV_HAZARD` | IMU detected pit, steep incline, or upside-down |
| `NAV_STOPPED` | Halted (manual or safety) |

#### Cruise Phase (`NAV_CRUISE`)

1. **Path planner waypoints:** If the wavefront path planner has a path, the robot drives toward each waypoint using proportional heading steering (0.5 PWM per degree of error, clamped ±80).
2. **Heading-hold:** After 500 ms in cruise, the IMU-integrated heading is locked. A PID correction (`HEADING_HOLD_GAIN = 1.0 PWM/°`, deadband ±2.5°, max ±40 PWM) steers the robot straight.
3. **Dead-reckoning escape bias:** If odometry shows the robot is within `DR_MIN_DIST_CM` (100 cm) of its start point and heading back toward it, a gentle steering correction (`DR_STEER_GAIN = 0.15 PWM/°`) biases the robot away from start. If it heads toward start for more than `DR_CORRECT_TIMEOUT_MS` (5 s), a 600 ms forced turn fires.
4. **Speed:** `SPEED_CRUISE = MAX_PWM (255)` — the current limiter enforces the 87% duty ceiling automatically.

#### Obstacle Avoidance (`NAV_AVOID`)

Triggered when the forward distance drops below:
- `DIST_CLOSE_ADV = 35 cm` → normal avoidance
- `DIST_CRITICAL = 15 cm` → emergency, shorter backup

**Avoidance sequence:**
1. **Backup** (`AVOID_BACKUP_MS = 400 ms` normal, `550 ms` critical)
2. **Turn** — direction chosen by comparing left vs right ultrasonic readings; the robot turns toward the more open side. If both sides are equal, direction alternates to avoid looping.
3. **Verify forward** (`AVOID_VERIFY_MS = 400 ms`) — drives forward briefly to confirm clearance.
4. If the same direction is chosen `AVOID_MAX_CYCLES (4)` times consecutively, escalates to **Recovery**.

Obstacle memory: the robot tracks the last 8 obstacle headings (° rounded to 45°) and penalizes repeat directions.

**Anti-spin "Send It" mode:** If the robot turns more than `SENDIT_TURN_THRESHOLD (6)` times within `SENDIT_WINDOW_MS (10 s)`, it enters a 2.5 s full-forward burst to break out of a loop, then resets the turn counter.

#### Recovery (`NAV_RECOVER`)

Executed when repeated avoidance fails to find a clear path.

| Step | Action | Duration |
|---|---|---|
| Rock back | Full reverse | `RECOVERY_ROCK_MS × RECOVERY_ROCK_ATTEMPTS` (300 ms × 2) |
| Rock forward | Full forward | 300 ms × 2 |
| Diagonal approach | Turn + forward | 400 ms turn + 1200 ms forward |
| Full reverse escape | Full reverse | `RECOVERY_FULL_REV_MS (2000 ms)` |

After recovery, an **adaptive ramp** activates for `ADAPTIVE_RAMP_TIMEOUT (3 s)`: PWM ramp speed is limited to `ADAPTIVE_RAMP_SLOW (30)` per cycle (vs normal 55) to prevent track dig-in on loose sand.

#### Traction Control

Requires both IMU and encoder data to be healthy. If the wheels are spinning fast (encoder > 5 cm/s) but the IMU shows no body movement:

1. Power is cut to `TC_REDUCE_PWM (80)` for 400 ms.
2. If grip recovers, normal power resumes.
3. After `TC_MAX_CUTS (4)` failed cuts, escalates to full avoidance.
4. A 5 s cooldown prevents re-triggering immediately.

If either the IMU fault flag or encoder fault flag is set, traction control is bypassed entirely.

#### IMU Pitch / Terrain Detection

| Condition | Threshold | Action |
|---|---|---|
| Positive pitch > 15° for > 500 ms | Hill detected | Mark cells as `CELL_HILL`, optionally back up if > 30° |
| Negative pitch < −15° for > 500 ms | Pit/drop-off | Mark cells as `CELL_PIT`, back up 600 ms |
| Distance jump > 80 cm between readings | Pit edge | Mark `CELL_PIT` |
| `accelZ < −0.5 g` | Upside down | Activate self-righting arm |
| `|roll| > 45°` | On its side | Activate side-righting sequence |

Minor terrain (pitch < 20° or estimated height < 15 cm) is traversed at full speed with a 2 s terrain-boost timer. Major terrain triggers backup and re-routing.

---

### Mode 3 — Simple Autonomous (LED: Purple)

A lightweight single-file autonomous mode that uses only the front ultrasonic distance reading (no path planner, no wavefront map).

**Behaviour:**

| Distance | Action |
|---|---|
| > `DISTANCE_FAR (100 cm)` | Full speed forward |
| `DISTANCE_MEDIUM (50 cm)` – FAR | Slow down proportionally |
| `DISTANCE_CLOSE (20 cm)` – MEDIUM | Turn (random direction) |
| < `DISTANCE_CLOSE (20 cm)` | Reverse then turn |

The same terrain features from Autonomous mode are also active: IMU movement verification, sand recovery, incline handling, and adaptive ramp. Useful for open environments where mapping is not needed.

---

### Mode 4 — Wall Follow (LED: Cyan)

Left-hand rule wall follower. The rover keeps the left ultrasonic sensor at a target standoff distance from the left wall and corners when it reaches a gap or blockage. Useful for perimeter mapping in competition arenas.

**PID wall-distance control:**
- Target distance: `WF_TARGET_DIST_CM (28 cm)` to the left wall
- Gains: Kp = 4.5, Kd = 1.8, Ki = 0.04
- Correction applied as a differential steering offset (positive = steer left / toward wall)
- Integral windup is limited to prevent drift accumulation

**State transitions:**

| Condition | Action |
|---|---|
| Left sensor > `WF_WALL_LOST_CM (100 cm)` | Lost wall — turn left `WF_CORNER_TURN_MS (650 ms)` |
| Front sensor < `WF_FRONT_STOP_CM (30 cm)` | Blocked — turn right `WF_BLOCK_TURN_MS (700 ms)` |
| Front sensor < `WF_FRONT_SLOW_CM (55 cm)` | Approaching — slow to `WF_SLOW_SPEED (80 PWM)` |
| No wall found after `WF_SEARCH_TIMEOUT_MS (8 s)` | Give up searching — cruise forward |

**Speeds:** Cruise `WF_CRUISE_SPEED (160 PWM)`, search/turn `WF_SEARCH_SPEED (90 PWM)`, slow `WF_SLOW_SPEED (80 PWM)`.

---

### Mode 5 — Pre-Map Navigation (LED: Yellow)

Uses the same full `AutonomousNav` state machine as Mode 2, but the occupancy map is **pre-seeded from the web UI** before the run starts. Use this when you have advance knowledge of the arena layout.

**Workflow:**
1. Connect to the `TankBot-AP` WiFi network.
2. Open the web dashboard at `http://192.168.4.1`.
3. Click **✎ Draw Pre-Map** to open the map editor.
4. Paint walls, hazards, pits, and hills on the 10×10 grid.
5. Set the robot start position by clicking on the canvas with the robot tool.
6. Click **Send to Robot** — the map is uploaded to the rover over HTTP POST.
7. Switch the rover to Mode 5 with the button.
8. The robot uses the pre-drawn map to plan paths and avoid known obstacles before it has seen them with its sensors.

Sensor readings continue to update the map live during the run, so the pre-seeded data is refined as the robot explores.

---

## Mode Button

**GPIO 5**, active-LOW with internal pull-up. Debounce: 50 ms.

| Press type | Action |
|---|---|
| **Short press** (< 800 ms) | Cycle to next mode: UART → RC → Auto → Simple → Wall → Premap → UART |
| **Long press** (≥ 800 ms), in RC mode only | Start Xbox BLE pairing |

When switching modes:
- **→ RC mode:** BT radio is enabled; WiFi stays on.
- **← from RC mode:** BT is fully shut down; WiFi stays on.
- **→ any other mode:** If BT was running, it is stopped.

The LED flashes 3× on every mode switch to confirm the change.

---

## Web Dashboard

The rover runs a WiFi Access Point called **`TankBot-AP`** (open, no password) on **`192.168.4.1`**. A captive-portal DNS server responds to all domain lookups with the AP IP, so most phones will auto-pop the dashboard on connection.

### Dashboard (`/`)

Polls `/api/status` every 500 ms and displays:

| Section | Data shown |
|---|---|
| **Control** | Current mode badge, connection status |
| **Odometry** | X/Y position (cm), heading (θ rad) from wheel encoders |
| **Encoders** | Left/right RPM, left/right speed (cm/s), left/right pulse count |
| **Motors** | Commanded speed A/B, max safe PWM, upside-down flag |
| **Ultrasonic** | Left/right distance (cm), computed center distance, wall angle, gap width |
| **Path Planner** | Planner mode, path validity, distance-to-goal (cm) |
| **IMU** | Accel X/Y/Z (g), gyro X/Y/Z (°/s), temperature (°C), ESP32 internal temp |
| **System** | Uptime (s), free heap (bytes), WiFi clients |
| **Map** | 10×10 occupancy grid rendered on a canvas (live, color-coded) |

Map cell colors: wall = red, hazard = orange, pit = purple, hill = green, frontier = cyan, unknown = dark grey, free = near-black.

### Map Editor (`/map-editor`)

Pre-draw the arena before a run. Tools:

| Tool | Cell type |
|---|---|
| 🟥 Wall | `CELL_OCCUPIED (255)` |
| 🟠 Hazard | `CELL_HAZARD (250)` |
| 🟣 Pit | `CELL_PIT (245)` |
| 🟢 Hill | `CELL_HILL (240)` |
| ⬛ Clear | `CELL_FREE (0)` |
| 🤖 Robot Start | Sets robot position in the map |

Click or drag to paint. **Send to Robot** uploads via HTTP POST to `/api/premap`. **Clear All** resets canvas to unknown. The editor shows a pixel-accurate 10×10 grid matching the on-board map dimensions.

### API Endpoints

| Endpoint | Method | Description |
|---|---|---|
| `/` | GET | Main dashboard HTML |
| `/map-editor` | GET | Pre-map editor HTML |
| `/api/status` | GET | JSON status snapshot (all sensor + nav data) |
| `/api/map` | GET | JSON occupancy grid (10×10 encoded as a flat string) |
| `/api/premap?rx=N&ry=N` | POST | Upload pre-drawn map; body = 100 ASCII chars |

---

## Sensors

### Dual HC-SR04 Ultrasonic (±15° forward-facing)

Two sensors angled ~15° left and right of center-forward. Both fire with a 500 µs stagger to avoid crosstalk. Readings are filtered with a 70/30 exponential moving average (70% new reading, 30% previous).

From the two raw readings, trig geometry computes:

| Value | Calculation |
|---|---|
| **Center distance** | Line interpolation between the two detected obstacle points at x = 0 |
| **Wall angle** | `atan2` of the vector between the two points (° from center-forward) |
| **Gap width** | Horizontal distance between the two detected points (cm) |
| **Nearest obstacle X/Y** | Cartesian position of nearest point (cm, +right / +forward) |

Health monitoring detects sensors frozen at a fixed reading for > 3 s and sets a fault flag visible in the dashboard.

### MPU6050 IMU (I2C, address 0x68)

200-sample calibration on startup (robot must be flat and still). Calibration discards the first 50 samples for settling, then averages 200 samples for zero offsets. The Z-axis accelerometer offset subtracts the expected 1 g so that `getAccelZ()` returns 0 g when flat and approximately −1 g when inverted.

**Provides:**
- Accelerometer X/Y/Z in g's — pitch, roll, upside-down, movement verification
- Gyroscope X/Y/Z in °/s — heading integration, turn confirmation
- Temperature in °C

**Partial-read protection:** If I2C times out mid-frame, the incomplete 14-byte read is **discarded** entirely rather than zero-padded, preventing false upside-down triggers from `accelZ ≈ 0`.

---

## Motor Safety System

Nine independent safety layers protect the DRV8871 motor drivers. All layers are applied in `MotorControl::safetyCheck()` (called every loop, before any mode handler) and `applySafetyLimit()` (called on every motor command):

| Layer | Description |
|---|---|
| **1. Current-based PWM cap** | `maxSafePWM` computed at boot from `DRV8871_SAFE_CURRENT_A / BATTERY_VOLTAGE / MOTOR_RESISTANCE_OHM`. Default: 3.5 A → ~222 (87% duty). |
| **2. Slew rate limiter** | PWM ramps at most `SLEW_RATE_UP (55)` or `SLEW_RATE_DOWN (50)` per 10 ms cycle. Prevents inrush spikes. |
| **3. Direction dead-time** | 15 ms coast when reversing direction. Prevents H-bridge shoot-through. |
| **4. Idle speed jump** | If command is non-zero but below `MIN_MOTOR_SPEED (60)`, it is boosted to 60 to overcome static friction. |
| **5. Thermal monitoring** | ESP32 internal temperature sensor read every 2 s. >65°C = throttle motors to 50%. >75°C = emergency stop. Resumes at <55°C. |
| **6. Duty cycle limiter** | After `MOTOR_MAX_FULL_MS (20 s)` of sustained full speed, motors throttle to 50% for `MOTOR_COOLDOWN_MS (3 s)`. |
| **7. Motor watchdog** | If no `setMotors()` call for `MOTOR_WATCHDOG_MS (500 ms)`, motors stop automatically. Resets on next command. |
| **8. Startup delay** | Motors are locked for `MOTOR_STARTUP_DELAY (1 s)` after boot while power rails stabilize. |
| **9. Emergency stop** | Any critical fault (thermal, etc.) sets a permanent `emergencyStopped` flag that requires a reboot to clear. |

**Motor calibration:** Motor B is trimmed to 95% (`MOTOR_B_CALIBRATION = 0.95f`) to compensate for a slight right-drift tendency. Both motors have their wiring polarity inverted in firmware (`MOTOR_A_INVERTED = 1`, `MOTOR_B_INVERTED = 1`).

---

## Terrain Handling

Active in both Autonomous (Mode 2) and Simple Autonomous (Mode 3).

### IMU Movement Verification

A rolling window of 8 accelerometer magnitude samples is maintained. If variance drops below `MOTION_ACCEL_VAR_THRESH (0.003)` for more than `MOTION_VERIFY_TIMEOUT (4 s)` while motors are commanded above minimum speed, the rover is declared stuck.

### Stuck Recovery Sequence

| Step | Action | Config |
|---|---|---|
| Rock back (×2) | Full reverse pulse | `RECOVERY_ROCK_MS (300 ms)` × `RECOVERY_ROCK_ATTEMPTS (2)` |
| Rock forward (×2) | Full forward pulse | same |
| Diagonal approach | Turn + drive | `RECOVERY_DIAG_TURN_MS (400 ms)` + `RECOVERY_DIAG_FWD_MS (1200 ms)` |
| Full reverse escape | Sustained reverse | `RECOVERY_FULL_REV_MS (2000 ms)` |

A `RECOVERY_COOLDOWN_MS (1500 ms)` window after recovery exit ignores stuck detection to prevent immediate re-entry.

### Incline Handling

| Threshold | Action |
|---|---|
| Pitch > `INCLINE_MAX_PITCH (30°)` for > `INCLINE_TIMEOUT_MS (1 s)` | Back up + try diagonal approach |
| Minor terrain: `|pitch|` < 20° | Terrain-boost timer active — full speed for 2 s |
| Major terrain: `|pitch|` ≥ 20° or height > 15 cm | Avoid and re-route |

### Pit / Drop-off Detection

Triggered by either:
- A distance **jump** > `PIT_DIST_JUMP_CM (80 cm)` between consecutive readings (sensor suddenly sees the far wall past the edge)
- Sustained nose-down pitch < `PIT_PITCH_THRESHOLD (−15°)` for `PIT_PITCH_SUSTAIN_MS (500 ms)`

Response: mark `PIT_MARK_DEPTH_CELLS (1)` cells deep + ±1 perpendicular cell on the map, then reverse `PIT_BACKUP_MS (600 ms)`.

### Adaptive Torque Ramp

After any recovery event, PWM ramp speed reduces to `ADAPTIVE_RAMP_SLOW (30)` per 10 ms cycle for `ADAPTIVE_RAMP_TIMEOUT (3 s)` to prevent track dig-in on sand while the rover regains traction.

---

## Self-Righting Arm

A servo-driven arm on GPIO 15 (330 Hz, 500–2500 µs). The servo is mounted horizontally; 0° = full left, 90° = stowed/center, 180° = full right.

**States:**

| State | Trigger | Behaviour |
|---|---|---|
| `ARM_STOWED` | Normal operation | Arm at 90°, servo PWM off (saves power) |
| `ARM_IDLE` | Can be triggered manually | Gentle ±10° oscillation around 90° |
| `ARM_RIGHTING` | `accelZ < −0.5 g` (upside down) | Full 0°→180° sweeps, holds 7 s at each extreme to build momentum. Repeats up to `ARM_MAX_ATTEMPTS (40)` times. |
| `ARM_SIDE_RIGHTING` | `|roll| > 45°` | Sweeps toward the ground side first (roll sign selects direction). Up to `SIDE_MAX_PUSHES (6)` pushes with 3 s hold. |
| `ARM_COOLDOWN` | After successful righting | 2 s pause before re-arming |

In RC mode, the arm is controlled manually with the Xbox triggers (RT = 0°→180°, LT = 180°→0°, both released = stow).

---

## Occupancy Map & Path Planner

### Occupancy Map

A 10×10 probabilistic grid, each cell = 50 cm × 50 cm (5 m × 5 m total coverage). Stored as two heap-allocated byte arrays (`grid[]` and `visitCount[]`), consuming ~200 bytes total.

**Cell values (0–255):**

| Value | Meaning |
|---|---|
| 0 | Confirmed free |
| 1–39 | Likely free |
| 40–79 | Probably free |
| 80–180 | Unknown (prior = 127) |
| 181–239 | Probably occupied |
| 240 | Hill |
| 245 | Pit |
| 250 | Hazard |
| 255 | Confirmed occupied / wall |

**Ray tracing:** Each ultrasonic reading traces a ray from the robot's grid cell outward, decrementing occupancy along the free path and incrementing the terminal cell if a hit is detected. Confidence scales with distance (closer = stronger update).

**Infinite map (sliding window):** When the robot approaches within `EDGE_MARGIN (2)` cells of any map edge, the entire grid (including `visitCount`) is shifted so the robot moves back to center. World-origin tracking converts grid coordinates back to absolute cm for navigation. Up to unlimited shifts are supported.

### Path Planner

A wavefront (BFS flood-fill) planner operating on a 10×10 local window centered on the goal.

**Cost function per cell:**
- Cardinal move: 10
- Diagonal move: 14
- Visit-count penalty: `min(visitCount × 10, 100)` — strongly prefers unvisited cells for exploration

**Modes:**

| Mode | Description |
|---|---|
| `PLANNER_EXPLORE` | Frontier-based exploration — finds free cells adjacent to unknown cells, paths to the nearest reachable one |
| `PLANNER_GOTO_GOAL` | Navigate to a specific world coordinate |
| `PLANNER_RETURN_HOME` | Return to the robot's boot position |
| `PLANNER_FOLLOW_PATH` | Step through a waypoint list |

**Frontiers:** A cell is a frontier if it is free (or likely free) and has at least one unknown neighbor. The planner scores all reachable frontiers by distance, paths to the closest, and advances to the next after each waypoint is reached.

If no path exists (start too far from goal), a single intermediate waypoint is placed in the goal direction (clamped to map bounds) to move the robot closer.

---

## Hall-Effect Encoders & Odometry

Two A3144 sensors on the output shafts, one magnet per trigger per half-revolution (2 magnets per shaft). ISR-driven pulse counting with 5 ms debounce (`ENCODER_DEBOUNCE_US`).

**Derived constants:**
- Drive sprocket: 26 mm diameter
- Circumference: π × 26 mm ≈ 81.7 mm
- Distance per pulse: 81.7 / 2 = **~40.8 mm per pulse**

**`update()` (called at 20 Hz):**
1. Pulse counts are read with interrupts disabled to prevent race conditions.
2. Delta pulses × cm/pulse = distance this interval.
3. Differential drive kinematics update `x`, `y`, `theta`:
   - `dCenter = (leftDist + rightDist) / 2`
   - `dTheta = (rightDist - leftDist) / wheelbase_cm`
   - `x += dCenter × sin(θ)`, `y += dCenter × cos(θ)`
4. Speed is smoothed with an EMA filter (`ENCODER_SPEED_EMA_ALPHA = 0.3`).
5. Speeds > `MAX_SPEED_CM_S (250 cm/s)` are rejected as EMI noise.
6. A complementary filter blends encoder speed with IMU-derived acceleration (`ENCODER_COMP_ALPHA = 0.85`).

The encoder pose is propagated to the shared globals `odomX`, `odomY`, `odomTheta` every sensor update cycle, feeding the web dashboard and autonomous nav.

---

## LED Status

Three-leg common-cathode RGB LED (not NeoPixel). Colors are PWM-mixed on separate LEDC channels.

| Color | Mode / State |
|---|---|
| **Red** (solid) | UART Control (Mode 1) |
| **Blue** (solid) | RC Control, waiting for controller (Mode 0) |
| **Green** (solid) | RC Control, controller connected |
| **Blue** (fast blink, 200 ms) | BLE pairing in progress |
| **Orange** (solid) | Autonomous Navigation (Mode 2) |
| **Purple** (solid) | Simple Autonomous (Mode 3) |
| **Cyan** (solid) | Wall Follow (Mode 4) |
| **Yellow** (solid) | Pre-Map Navigation (Mode 5) |
| **Red** (fast blink) | Emergency stop / critical fault |

The LED flashes white 3× when any mode transition occurs.

---

## Serial Commands

Type in the serial monitor (115200 baud) while in any mode:

| Command | Action |
|---|---|
| `EXPLORE` | Start frontier-based exploration (path planner) |
| `HOME` | Return to boot position via path planner |
| `GOTO X Y` | Go to position X Y in **meters** from start (e.g. `GOTO 2.5 1.0`) |
| `STOP` | Stop motors and path planner |
| `MAP` | Print ASCII occupancy grid to serial |
| `PATH` | Print current waypoint list |
| `SENSOR` | Print all sensor readings (distances, trig data, IMU values, heading) |
| `RESET` | Reset occupancy map and navigation state |
| `HELP` | Show all commands |

---

## UART Protocol

Binary framing for host-computer integration (re-enable `Serial1` in `uart_comm.cpp` to use):

```
[0xFF] [TYPE] [LENGTH] [DATA × LENGTH] [CHECKSUM] [0xFE]
```

- **Checksum** = XOR of `TYPE ^ LENGTH ^ DATA[0] ^ DATA[1] ^ ...`
- **Max payload:** 64 bytes (validated at parse time — oversized frames are dropped)
- **Frame validation:** start byte, end byte, checksum, and length all checked before processing

| Type | Value | Payload |
|---|---|---|
| Motor Control | 0x01 | 2 bytes: `int8_t motorA, motorB` (−100 to 100 %) |
| Sensor Data | 0x02 | `sizeof(SensorData)` bytes (~52 bytes) |
| Status | 0x03 | ASCII string (max 32 chars) |
| Command | 0x04 | Command byte + optional data |
| ACK | 0x05 | 1 byte: echoed message type |

**Commands (type 0x04):**

| Value | Command |
|---|---|
| 0x01 | Stop |
| 0x02 | Forward |
| 0x03 | Backward |
| 0x04 | Left |
| 0x05 | Right |
| 0x06 | Custom |
| 0x07 | Get Sensors |
| 0x08 | Standby |

---

## Project Structure

```
TankESP32/
├── include/
│   ├── config.h              # All pin definitions and tuning constants
│   ├── globals.h             # Shared globals (motors, encoders, map, odom)
│   ├── motor_control.h       # DRV8871 dual motor driver + 9-layer safety
│   ├── ultrasonic_sensor.h   # HC-SR04 multi-instance driver
│   ├── mpu6050_sensor.h      # MPU6050 I2C driver, calibration, partial-read guard
│   ├── uart_comm.h           # Binary packet protocol (Serial1)
│   ├── xbox_controller.h     # Bluepad32 Xbox BLE controller
│   ├── button_handler.h      # Debounced short/long press
│   ├── led_controller.h      # 3-channel RGB LED, mode colors
│   ├── autonomous_nav.h      # Full nav state machine + terrain handling
│   ├── hall_encoder.h        # ISR-driven encoder, odometry, speed fusing
│   ├── occupancy_map.h       # 10×10 probabilistic map, ray tracing, sliding window
│   ├── path_planner.h        # Wavefront BFS, frontier exploration
│   ├── self_righting.h       # Servo arm controller, state machine
│   ├── wifi_manager.h        # WiFi AP, captive-portal DNS
│   ├── web_api.h             # REST API endpoints
│   ├── web_dashboard.h       # Dashboard HTML (PROGMEM)
│   ├── web_map_editor.h      # Map editor HTML (PROGMEM)
│   └── web_status.h          # Status page HTML
├── src/
│   ├── main.cpp              # Setup, loop, all mode handlers, traction control
│   ├── globals.cpp           # Global object definitions
│   ├── motor_control.cpp     # PWM, slew, dead-time, watchdog, thermal
│   ├── ultrasonic_sensor.cpp # pulseIn timing, filtering, health monitor
│   ├── mpu6050_sensor.cpp    # I2C init, calibration, partial-read detection
│   ├── uart_comm.cpp         # Packet encode/decode, Serial1 guard
│   ├── xbox_controller.cpp   # Bluepad32 callbacks, deadzone, skid-steer mix
│   ├── button_handler.cpp    # GPIO debounce, short/long press events
│   ├── led_controller.cpp    # LEDC RGB mixing, blink patterns
│   ├── autonomous_nav.cpp    # Nav state machine, avoidance, heading-hold, DR
│   ├── hall_encoder.cpp      # ISR, atomic read, odometry integration, EMA
│   ├── occupancy_map.cpp     # Grid allocation, ray tracing, shiftMap (grid+visits)
│   ├── path_planner.cpp      # Wavefront, frontier search, waypoint tracing
│   ├── self_righting.cpp     # ARM_RIGHTING, ARM_SIDE_RIGHTING, sweep logic
│   ├── wifi_manager.cpp      # AP config, captive DNS, server lifecycle
│   ├── web_api.cpp           # /api/status, /api/map, /api/premap handlers
│   ├── web_dashboard.cpp     # Dashboard HTML generation
│   ├── web_map_editor.cpp    # Map editor HTML generation
│   └── web_status.cpp        # Simple status page
├── lib/
└── platformio.ini
```

---

## Key Tuning Constants

All in `include/config.h`.

### Motor & Power

| Constant | Default | Purpose |
|---|---|---|
| `DRV8871_SAFE_CURRENT_A` | 3.5 A | Target operating current; determines `maxSafePWM` |
| `MOTOR_RESISTANCE_OHM` | 4.0 Ω | Effective stall resistance for current calculation |
| `BATTERY_VOLTAGE` | 16.0 V | Supply voltage for PWM→current math |
| `SLEW_RATE_UP` | 55 | Max PWM increase per slew cycle (10 ms) |
| `SLEW_RATE_DOWN` | 50 | Max PWM decrease per slew cycle |
| `MIN_MOTOR_SPEED` | 60 | Minimum PWM — below this the motors don't move |
| `MOTOR_A_CALIBRATION` | 1.0 | Left motor scale factor (1.0 = no trim) |
| `MOTOR_B_CALIBRATION` | 0.95 | Right motor scale factor (trim to correct drift) |
| `DIRECTION_DEADTIME_MS` | 15 | Coast time between forward↔reverse |

### Autonomous Navigation

| Constant | Default | Purpose |
|---|---|---|
| `DIST_CRITICAL` | 15 cm | Emergency stop threshold |
| `DIST_CLOSE_ADV` | 35 cm | Standard avoidance threshold |
| `DIST_MEDIUM_ADV` | 60 cm | Caution / slow-down zone |
| `HEADING_HOLD_GAIN` | 1.0 | PWM per degree of heading error |
| `HEADING_HOLD_MAX` | 40 | Max heading correction differential |
| `HEADING_HOLD_DEADBAND` | 2.5° | Ignore small errors |
| `ROVER_MAX_SPEED_CM_S` | 100 cm/s | Calibrated top speed at MAX_PWM |

### Terrain

| Constant | Default | Purpose |
|---|---|---|
| `MOTION_ACCEL_VAR_THRESH` | 0.003 | Accel variance below this = stuck |
| `MOTION_VERIFY_TIMEOUT` | 4000 ms | Time before declaring stuck |
| `INCLINE_MAX_PITCH` | 30° | Maximum climbable incline |
| `PIT_DIST_JUMP_CM` | 80 cm | Distance jump to flag a pit |
| `RECOVERY_ROCK_ATTEMPTS` | 2 | Rock cycles before escalating |
| `ADAPTIVE_RAMP_SLOW` | 30 | Slow ramp step after recovery |
| `ADAPTIVE_RAMP_TIMEOUT` | 3000 ms | Duration of slow ramp |

### Wall Follow

| Constant | Default | Purpose |
|---|---|---|
| `WF_TARGET_DIST_CM` | 28 cm | Desired left-wall standoff |
| `WF_FRONT_STOP_CM` | 30 cm | Front obstacle stop threshold |
| `WF_PID_KP / KD / KI` | 4.5 / 1.8 / 0.04 | Wall-distance PID gains |
| `WF_CRUISE_SPEED` | 160 PWM | Normal wall-follow speed |

### Encoder / Odometry

| Constant | Default | Purpose |
|---|---|---|
| `ENCODER_DEBOUNCE_US` | 5000 µs | ISR noise filter (min pulse spacing) |
| `ENCODER_SPEED_EMA_ALPHA` | 0.3 | Speed smoothing (0 = max smooth) |
| `ENCODER_COMP_ALPHA` | 0.85 | Encoder vs IMU speed blend |
| `ENCODER_WHEEL_DIA_MM` | 26.0 mm | Drive sprocket diameter |
| `ENCODER_MAGNETS` | 2 | Magnets per shaft revolution |

### Self-Righting Arm

| Constant | Default | Purpose |
|---|---|---|
| `SERVO_FREQ_HZ` | 330 Hz | Servo PWM frequency (digital servo) |
| `SERVO_MIN_US / MAX_US` | 500 / 2500 µs | Pulse width at 0° / 180° |
| `ARM_SWEEP_STEP` | 3° | Degrees per step during sweep |
| `ARM_SWEEP_DELAY_MS` | 25 ms | Delay between steps |
| `ARM_HOLD_AT_EXTREME_MS` | 7000 ms | Hold at 0° or 180° during righting |
| `ARM_MAX_ATTEMPTS` | 40 | Max righting sweeps before cooldown |

### Safety

| Constant | Default | Purpose |
|---|---|---|
| `TEMP_WARNING_C` | 65°C | Thermal throttle threshold |
| `TEMP_CRITICAL_C` | 75°C | Emergency stop threshold |
| `MOTOR_WATCHDOG_MS` | 500 ms | Inactivity timeout before auto-stop |
| `MOTOR_MAX_FULL_MS` | 20000 ms | Max sustained full-speed before duty cycle cooldown |
| `BUTTON_DEBOUNCE_MS` | 50 ms | Button noise filter |
| `BUTTON_LONG_PRESS_MS` | 800 ms | Long-press hold time |

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Motors don't move | `maxSafePWM` too low, or below `MIN_MOTOR_SPEED` | Check battery voltage; print `[MOTOR] Computed max safe PWM` line at boot |
| Robot drifts right/left | Motor calibration off | Adjust `MOTOR_B_CALIBRATION` in config.h |
| IMU not detected | I2C wiring or address wrong | Check SDA=GPIO14, SCL=GPIO27; run `SENSOR` command |
| Ultrasonic reads −1 cm | Wiring issue or echo timeout | Check TRIG/ECHO pins; boot diagnostic prints 3 test readings |
| False "upside down" triggers | I2C partial read (old bug — fixed) | Confirm firmware includes the `readRegisters` return-value fix |
| Button unresponsive | Debounce too short / capacitive coupling | `BUTTON_DEBOUNCE_MS` is 50 ms; check pull-up is active |
| Stuck in recovery loop | Movement threshold too sensitive | Raise `MOTION_ACCEL_VAR_THRESH` (default 0.003, try 0.008) |
| Can't climb incline | Pitch limit too low | Raise `INCLINE_MAX_PITCH` (default 30°) |
| WiFi AP not appearing | BT not released before AP start (handled in code) | Power cycle; check serial for `[WiFi] AP OK` |
| Dashboard shows odom 0,0,0 | Encoders not connected | Check GPIO 35/32 wiring, 10 kΩ pull-ups, magnet alignment |
| Servo not moving | PWM channel conflict, or servo type wrong | Confirm `SERVO_FREQ_HZ = 330` only for digital servos; use 50 Hz for analog |
| Build fails with SensorData assert | `SensorData` struct > 64 bytes | Reduce fields or increase `Message::data[]` size |

---

*COSGC Robotics Competition — ESP32 WROOM-32*
