# ESP32-S3 Tank Rover — COSGC Competition

Firmware for an ESP32-S3 tracked rover built for the Colorado Space Grant Consortium (COSGC) competition. The rover navigates sand pits, inclines, and obstacle fields using dual ultrasonic sensors, an IMU, and PID-based motor control.

---

## Pin Map

| Function | GPIO | Notes |
|---|---|---|
| **Motor A IN1** (Left fwd) | 15 | DRV8871 driver 1 |
| **Motor A IN2** (Left rev) | 5 | DRV8871 driver 1 |
| **Motor B IN1** (Right fwd) | 7 | DRV8871 driver 2 |
| **Motor B IN2** (Right rev) | 6 | DRV8871 driver 2 |
| **Ultrasonic LEFT TRIG** | 12 | HC-SR04, ~15° left of center |
| **Ultrasonic LEFT ECHO** | 11 | HC-SR04 |
| **Ultrasonic RIGHT TRIG** | 38 | HC-SR04, ~15° right of center |
| **Ultrasonic RIGHT ECHO** | 37 | HC-SR04 |
| **I2C SDA** (MPU6050) | 10 | IMU accelerometer + gyroscope |
| **I2C SCL** (MPU6050) | 3 | IMU accelerometer + gyroscope |
| **Button** | 21 | Mode switch / BLE pairing |
| **LED** | 48 | WS2812 RGB onboard LED |
| **UART TX** | 43 | Host communication |
| **UART RX** | 44 | Host communication |

**PWM:** 20 kHz, 8-bit resolution, channels 0–3 permanently attached.  
**Battery:** 16V | **Track width:** 150 mm | **Ground clearance:** 50 mm

---

## Operation Modes

The rover has 4 modes, cycled with a **long button press (~1.5s)**. A **short press** starts Xbox BLE pairing.

### Mode 0 — RC Control (LED: Blue/Green)

Xbox One/Series controller over Bluetooth (Bluepad32). Left stick drives forward/backward, right stick steers. LED turns green when a controller is connected.

### Mode 1 — UART Control (LED: Red)

Motor commands from a host controller over serial (115200 baud). Accepts binary packet protocol for forward, backward, turn, stop, and sensor queries. Default startup mode.

### Mode 2 — Autonomous Navigation (LED: Orange)

Full autonomous mode using the `AutonomousNav` class:

- **PID speed control** — speed scales with obstacle distance (Kp=5, Ki=0.1, Kd=1.5)
- **Dual ultrasonic + trig geometry** — two sensors at ±15° compute center distance, wall angle, and gap width
- **IMU heading memory** — gyro-integrated heading tracks which compass directions have obstacles
- **Slope compensation** — uphill +15% power, downhill −25%
- **Terrain recovery** — rock forward/back, diagonal approach, full reverse escape (see below)
- **Incline handling** — rejects slopes >25° after timeout, tries diagonal then reverses
- **Adaptive torque ramp** — slower PWM ramp (±15 vs ±50) for 5s after a stuck event

### Mode 3 — Simple Autonomous (LED: Purple)

Lightweight autonomous mode without the full nav class:

- **PID forward** — same gains, MIN_SPEED floor of 200 PWM
- **Turn/reverse phases** — triggered by obstacle proximity or stall timer
- **Same terrain features** — IMU movement verification, sand recovery, incline handling, and adaptive ramp are all integrated inline

---

## Terrain Handling

These features activate in both autonomous modes:

| Feature | What it does | Trigger |
|---|---|---|
| **Movement verification** | Tracks accel-magnitude variance over 8 samples. If variance < 0.008 for >1.5s while motors are on, declares "not moving." | Motors on + IMU steady |
| **Stuck recovery** | 3× rock forward/back at full power → diagonal turn + forward → full reverse 1.5s | Stall detection or movement verification failure |
| **Incline limiter** | Monitors pitch. >25° for >3s → back up + try diagonal (2 attempts) → declare unclimbable + full recovery | Sustained steep pitch |
| **Adaptive ramp** | After recovery, PWM ramp slows to ±15/frame (vs ±50) for 5 seconds to prevent track dig-in on sand | Any recovery event |

---

## Sensors

### Dual HC-SR04 Ultrasonic (±15° forward-facing)

Both sensors fire with a 500 µs stagger to avoid crosstalk. Readings are filtered (70% new / 30% old). Trig math computes:

- **Center distance** — line interpolation between left/right obstacle points at x=0
- **Wall angle** — atan2 of the vector between the two detected points
- **Gap width** — horizontal span between detected points

### MPU6050 IMU (I2C 0x68)

200-sample startup calibration (robot must be still). Provides:

- Accelerometer (X/Y/Z) — pitch, roll, upside-down detection, movement verification
- Gyroscope (X/Y/Z) — heading integration, turn verification
- Temperature

---

## Project Structure

```
TankESP32/
├── include/
│   ├── config.h              # All pin definitions and tuning constants
│   ├── motor_control.h       # DRV8871 dual motor driver
│   ├── ultrasonic_sensor.h   # HC-SR04 driver (multi-instance)
│   ├── mpu6050_sensor.h      # MPU6050 IMU driver
│   ├── uart_comm.h           # UART packet protocol + SensorData struct
│   ├── xbox_controller.h     # Bluepad32 Xbox BLE controller
│   ├── button_handler.h      # Debounced button with short/long press
│   ├── led_controller.h      # WS2812 RGB status LED
│   └── autonomous_nav.h      # Full autonomous nav with terrain handling
├── src/
│   ├── main.cpp              # Setup, loop, mode handlers, sensor fusion
│   ├── motor_control.cpp     # PWM control, upside-down swap, de-jitter
│   ├── ultrasonic_sensor.cpp # pulseIn-based distance reading
│   ├── mpu6050_sensor.cpp    # I2C init, calibration, raw reads
│   ├── uart_comm.cpp         # Binary packet encode/decode
│   ├── xbox_controller.cpp   # Bluepad32 callbacks, deadzone, skid steer
│   ├── button_handler.cpp    # GPIO41 debounce + event detection
│   ├── led_controller.cpp    # NeoPixel color/blink patterns
│   └── autonomous_nav.cpp    # PID, state machine, terrain recovery
├── lib/
└── platformio.ini
```

---

## Serial Commands

Type these in the serial monitor (115200 baud) while in any mode:

| Command | Action |
|---|---|
| `SENSOR` | Print all sensor readings (distances, trig data, IMU, heading) |
| `MAP` | Print occupancy grid |
| `RESET` | Reset map and navigation state |
| `HELP` | Show all commands |

---

## Building & Uploading

Requires [PlatformIO](https://platformio.org/).

```bash
# Build
pio run

# Upload
pio run --target upload

# Serial monitor
pio device monitor
```

---

## LED Status

| Color | Meaning |
|---|---|
| Blinking Blue | BLE pairing in progress |
| Solid Green | RC mode, controller connected |
| Solid Blue | RC mode, waiting for controller |
| Solid Red | UART control mode |
| Solid Orange | Autonomous navigation |
| Solid Purple | Simple autonomous mode |

---

## UART Protocol

```
[0xFF] [TYPE] [LENGTH] [DATA...] [CHECKSUM] [0xFE]
```

| Type | Value | Description |
|---|---|---|
| Motor Control | 0x01 | Two signed speed bytes (A, B) |
| Sensor Data | 0x02 | SensorData struct (64-byte buffer) |
| Status | 0x03 | ASCII status string |
| Command | 0x04 | Command byte + optional data |
| Acknowledgment | 0x05 | ACK response |

**Commands:** Stop (0x01), Forward (0x02), Backward (0x03), Left (0x04), Right (0x05), Custom (0x06), Get Sensors (0x07), Standby (0x08)

---

## Troubleshooting

- **Robot won't move:** MIN_SPEED is 200 PWM. PID output below this gets floored. Check battery voltage.
- **Motor B can't reverse:** Known hardware issue — DRV8871 driver for Motor B has a broken reverse channel.
- **IMU not detected:** Check I2C wiring (SDA=GPIO10, SCL=GPIO3). Run `SENSOR` command to see if accel reads 0.
- **Ultrasonic reads −1:** Wiring issue or echo timeout. Check TRIG/ECHO pins. Startup diagnostic prints 3 test readings.
- **Stuck in recovery loop:** Tune `MOTION_ACCEL_VAR_THRESH` in config.h (default 0.008). Higher = less sensitive.
- **Can't climb incline:** `INCLINE_MAX_PITCH` is 25°. Increase if tracks can handle steeper slopes.

---

## Key Tuning Constants (config.h)

| Constant | Default | Purpose |
|---|---|---|
| `SPEED_CRUISE` | 240 | Forward cruising PWM |
| `SPEED_MIN_MOVE` | 200 | Minimum PWM to overcome static friction |
| `PID_KP / KI / KD` | 5.0 / 0.1 / 1.5 | PID gains for distance→speed |
| `MOTION_ACCEL_VAR_THRESH` | 0.008 | Accel variance threshold for movement detection |
| `INCLINE_MAX_PITCH` | 25° | Maximum climbable pitch before timeout |
| `RECOVERY_ROCK_ATTEMPTS` | 3 | Rock cycles before escalating to diagonal |
| `ADAPTIVE_RAMP_SLOW` | 15 | PWM step/frame during soft-terrain ramp |
| `ADAPTIVE_RAMP_TIMEOUT` | 5000 ms | How long adaptive ramp stays active |

---

*COSGC Robotics Competition*
