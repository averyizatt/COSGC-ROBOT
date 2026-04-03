# COSGC-ROBOT

> Hardware, firmware, and software for the **Colorado Space Grant Consortium (COSGC)** robotics competition platforms.

---

## Overview

COSGC-ROBOT is a multi-platform robotics repository developed for the COSGC competition. It contains everything needed to build, flash, and run autonomous and RC-controlled rover platforms — from low-level microcontroller firmware to high-level Python perception and navigation stacks running on a Raspberry Pi.

The repository is organized into self-contained platform directories, each with its own README and build instructions.

---

## Repository Structure

```
COSGC-ROBOT/
├── COSGCTank/          # Raspberry Pi 4 autonomous rover (Python + OpenCV + TFLite)
├── TankESP32/          # ESP32-S3 tracked rover firmware (C++ / PlatformIO)
├── COSGCmega/          # Arduino Mega–based robot variants (v1, v2, IEEE, R&D)
└── COSGCmini/          # Smaller Arduino-based robot variants
```

### [COSGCTank](./COSGCTank/README.md)
A compact autonomous rover built around a **Raspberry Pi 4**, Pi camera, and dual H-bridge motor driver. Features a full perception-decision-control pipeline with:
- Obstacle detection (TFLite SSD + contour fallback)
- Boundary and terrain analysis
- SLAM integration hooks (OpenVSLAM / ORB-SLAM3 / RTAB-Map)
- Live web dashboard with RC override and real-time tuning
- Ultrasonic rangefinder and IMU (MPU-6050) support

### [TankESP32](./TankESP32/README.md)
Firmware for an **ESP32-S3** tracked rover. Designed for sand pits, inclines, and obstacle fields using:
- Dual HC-SR04 ultrasonic sensors (±15° geometry)
- MPU-6050 IMU with startup calibration
- PID-based autonomous navigation with terrain recovery
- Xbox BLE controller support (Bluepad32)
- UART packet protocol for host communication

### COSGCmega
Arduino Mega–based robot variants spanning multiple hardware generations (`COSGCv1`, `COSGCv2`, `IEEE_COSGC_Robot`, and a `RnD` / `hardwareTest` workspace).

### COSGCmini
Smaller Arduino-based robot variants (`IEEE_COSGC_Mini`, `IEEEcosgcRobotV3_ServoSteering`, etc.) for lightweight competition configurations.

---

## Languages & Tools

| Layer | Technology |
|---|---|
| Pi rover software | Python 3 (Flask, OpenCV, TFLite, Picamera2) |
| ESP32 firmware | C++ via PlatformIO |
| Arduino firmware | C++ via Arduino IDE or PlatformIO |
| Build automation | Shell scripts, `platformio.ini` |

---

## Quick Start

### COSGCTank (Raspberry Pi)

```bash
# Install system dependencies
sudo apt update
sudo apt install -y python3-opencv python3-pip python3-picamera2 python3-rpi.gpio python3-spidev

# Install Python packages
pip3 install --upgrade pip
pip3 install -r COSGCTank/requirements.txt

# Start the web server
python3 COSGCTank/rover_server.py

# Run the main autonomy loop
python3 COSGCTank/main.py
```

Then open your browser to the Pi's IP address to access the live dashboard and RC controls.

### TankESP32 (PlatformIO)

```bash
# Build
pio run

# Flash to ESP32-S3
pio run --target upload

# Open serial monitor
pio device monitor
```

See the [TankESP32 README](./TankESP32/README.md) for the full pin map, tuning constants, and troubleshooting guide.

---

## Dependencies

### Raspberry Pi (COSGCTank)
- **Python:** `flask`, `opencv-python`, `tflite-runtime`, `picamera2`, `evdev`, `smbus2`
- **System:** `python3-rpi.gpio`, `python3-spidev`, `bluetooth`, `bluez`
- **Optional:** a TFLite SSD MobileNet model in `COSGCTank/models/`

### ESP32 (TankESP32)
- [PlatformIO](https://platformio.org/) with the `espressif32` platform
- Bluepad32 library (Xbox BLE controller)
- FastLED / NeoPixel (onboard WS2812 LED)

### Arduino (COSGCmega / COSGCmini)
- Arduino IDE or PlatformIO
- Platform-specific wiring and motor-control libraries (see each subfolder)

---

## Notes

- This repository is **experimental** — some branches contain WIP code and hardware test scripts.
- GPIO and wiring details are documented in each platform's README.
- All GPIO references use **BCM numbering** for Raspberry Pi targets.
- Level-shift any 5V sensor signals (e.g., ultrasonic ECHO) before connecting to 3.3V Pi GPIO pins.
