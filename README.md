# Self-Driving Robot with Autonomous & WiFi Control

## Overview
This project implements a self-driving robot that can autonomously avoid obstacles using sensor data, as well as be controlled via WiFi. The robot uses:
- A WeMos D1 (ESP8266) board as the controller.
- A Devmo L293D/Adafruit Motor Shield to drive four DC motors.
- A 180° servo for steering, with configurable angles for mild and aggressive turns.
- An IR sensor and an HC‑SR04 ultrasonic sensor for obstacle detection.
- An ESP32‑CAM module for video streaming.

## Features
- **Autonomous Mode:**  
  Uses IR and ultrasonic sensor data to adjust the steering angle dynamically:
  - **Aggressive Turn:** For very close obstacles (servo to 0° for right or 180° for left).
  - **Mild Turn:** For moderately close obstacles (servo to 60° for right or 120° for left).
  - **Center/Forward:** When no obstacles are present (servo at 90°).
- **WiFi Remote Control:**  
  Provides a web server with endpoints to control movement and steering, as well as redirecting to an ESP32‑CAM stream.
- **Integrated Hardware:**  
  Supports DC motor control, servo steering, and sensor input using readily available libraries.

## Hardware Requirements
- **Controller:** WeMos D1 (ESP8266)
- **Motor Driver:** Devmo L293D/Adafruit Motor Shield
- **Motors:** Four DC motors
- **Steering:** 180° Servo (connected to the shield’s servo port)
- **Sensors:**  
  - Analog IR sensor  
  - HC‑SR04 Ultrasonic Sensor (with proper voltage divider on the echo pin)
- **Additional:**  
  - Physical switch for mode selection  
  - ESP32‑CAM for video streaming


### Pin Assignments
- **DC Motors:** Controlled via the motor shield (pins D1–D8).
- **Servo:** Connected to pin 9.
- **IR Sensor:** Connected to A0.
- **Ultrasonic Sensor:** TRIG to D13, ECHO to D14.
- **Control Switch:** Connected to D12 (using internal pull-up).

## Software Requirements
- Arduino IDE
- Libraries:
  - ESP8266WiFi
  - ESP8266WebServer
  - Adafruit_MotorShield
  - Servo
  - HCSR04
  - (Optional: IrWidgetAggregating, IrReceiverPoll, IrReceiverSampler)

## Installation & Setup
1. Install the required libraries via the Arduino Library Manager.
2. Connect the hardware as described in the wiring diagram.
3. Update the WiFi credentials in the code if necessary.
4. Upload the code to the WeMos D1.

## Usage
- **Autonomous Mode:**  
  (Switch mode using the physical switch; the robot will adjust its steering based on sensor inputs.)
- **WiFi Remote Control Mode:**  
  Access the robot's web server at the provided IP address. Available endpoints:
  - `/forward` - Drive forward
  - `/backward` - Drive backward
  - `/stop` - Stop motors
  - `/right` - Aggressive right turn (0°)
  - `/rightmild` - Mild right turn (60°)
  - `/left` - Aggressive left turn (180°)
  - `/leftmild` - Mild left turn (120°)
  - `/center` - Center steering (90°)
  - `/camera` - Redirects to the ESP32‑CAM stream

## Autonomous Steering Logic
1. **Sensor Reading:**  
   The ultrasonic sensor measures distance and the IR sensor provides analog input.
2. **Decision Making:**  
   - If an obstacle is detected:
     - If the obstacle is very close (< 10 cm): the robot makes an aggressive right turn (servo to 0°) or left (servo to 180°).
     - If moderately close (< 20 cm): a mild right (servo to 60°) or mild left turn (servo to 120°) is performed.
   - Otherwise, steering remains centered (90°).
3. **Movement:**  
   The robot continues driving forward while the steering is adjusted.

## Power Supply Considerations
- The system can be powered using a single 12V battery pack with appropriate buck converters (or a dual-supply system) based on your motor and sensor requirements. See the documentation for recommended configurations.

## Contributions
Feel free to open issues or pull requests if you have improvements or suggestions!
