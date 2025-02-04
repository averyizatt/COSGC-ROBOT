# COSGC-ROBOT
Colorado Space Grant Consortium Autonomous Robot


How This Updated Code Works
1.	Enhanced Object Detection:
o	The IR sensor is still used (with a threshold value) for general obstacle detection.
o	An ultrasonic sensor (using trigger and echo pins) measures the distance to objects.
o	In autonomous mode, if the IR sensor reading is below its threshold or the ultrasonic sensor detects an object closer than the defined threshold (20 cm in this example), the robot stops and executes an avoidance maneuver.
2.	Camera Integration in WiFi Mode:
o	A new endpoint (/camera) is added.
o	In a real-world application, you would integrate your camera module and streaming code here. For now, it simply returns a placeholder message.
3.	Control Modes and Steering:
o	A physical switch determines whether the robot runs in autonomous mode (using sensor data for obstacle avoidance) or in WiFi remote control mode (where HTTP endpoints control driving and steering).
o	The steering positions for the Nema 17 stepper motors remain fixed at 0 (center), +81 (left), and –81 (right).
Overview of Components and Their Connections
1.	DC Motors (4 total) with H Bridge Drivers:
Each motor is driven by an H bridge (or multiple H bridges on a motor driver board) that accepts two digital control signals (one for each motor terminal/direction). In our code, each motor uses two pins.
2.	Stepper Motors (2 Nema 17):
Each Nema 17 stepper is controlled via a stepper driver (for example, an A4988, DRV8825, etc.) that needs a STEP pin and a DIR (direction) pin. The code uses the AccelStepper library in DRIVER mode.
3.	IR Sensor (Analog):
An analog IR sensor (e.g., a Sharp distance sensor) provides a voltage level on its output pin. This is connected to one of the analog input pins (A0) on the WeMos.
4.	Ultrasonic Sensor (e.g., HC SR04):
The HC SR04 uses two pins: a TRIG (trigger) output from the microcontroller and an ECHO input to the microcontroller. Note that the HC SR04 works at 5 V, so you may need a voltage divider or level shifter for the echo signal since the ESP8266’s GPIOs are 3.3 V tolerant.
5.	Control Mode Switch (Digital Input):
A physical switch (or toggle) is used to select between autonomous mode and WiFi remote control mode.
6.	Camera (Optional):
While our code only has a placeholder for the camera, you would normally add a camera module (for example, an OV2640 or ESP32-CAM module). This module might be connected over a separate interface (such as an SPI, dedicated parallel interface, or using an all-in-one board). For now, we treat it as a future expansion.
________________________________________
Example Wiring Details
Below are the connections corresponding to the pin definitions used in the sketch:
1. DC Motors with H Bridge Drivers
Assume you have a four‐channel H bridge driver board (like an L298N, TB6612FNG, or similar). Wire each motor as follows:
•	Motor 1 (Front Left):
o	IN1: Connect to WeMos pin D1
o	IN2: Connect to WeMos pin D2
•	Motor 2 (Front Right):
o	IN1: Connect to WeMos pin D3
o	IN2: Connect to WeMos pin D4
•	Motor 3 (Rear Left):
o	IN1: Connect to WeMos pin D5
o	IN2: Connect to WeMos pin D6
•	Motor 4 (Rear Right):
o	IN1: Connect to WeMos pin D7
o	IN2: Connect to WeMos pin D8
Additional Notes:
•	The H bridge driver board must be powered by a supply that meets your motor’s voltage/current requirements.
•	Connect the H bridge’s ground (GND) to the WeMos ground.
2. Stepper Motors (Nema 17) with Stepper Drivers
For each Nema 17 motor, you typically use a driver board like the A4988 or DRV8825. Wire as follows:
•	Stepper 1:
o	STEP Pin: Connect the driver’s STEP input to WeMos D0
o	DIR Pin: Connect the driver’s DIR input to WeMos D9
o	Enable (if used): You may tie the ENABLE pin on the driver to ground or a dedicated digital output if you wish to control it.
o	Motor Power: Supply the stepper driver with an appropriate voltage (commonly 12 V or as required) and ensure proper current limiting via the driver’s potentiometer.
•	Stepper 2:
o	STEP Pin: Connect the driver’s STEP input to WeMos D10
o	DIR Pin: Connect the driver’s DIR input to WeMos D11
o	Enable (if used): As above, tie the ENABLE pin as required.
Additional Notes:
•	Make sure to share a common ground between the stepper drivers, the motor power supply, and the WeMos.
3. IR Sensor
•	Signal Output: Connect the sensor’s analog output to the WeMos A0 pin.
•	Power: Connect the sensor’s Vcc (often 5 V or 3.3 V depending on the model) to the appropriate supply on the WeMos (the WeMos typically supplies 3.3 V, but some sensor modules include a regulator for 5 V).
•	Ground: Connect the sensor’s ground to the common ground.
4. Ultrasonic Sensor (HC SR04)
•	TRIG (Trigger):
o	Connect to WeMos D13.
o	You may need a resistor divider or level shifter on the echo pin as described below.
•	ECHO:
o	Connect the sensor’s Echo output to WeMos D14.
o	Important: The HC SR04 outputs 5 V on the echo pin. Since the ESP8266 (WeMos) is 3.3 V tolerant, use a voltage divider (for example, two resistors such as 1 kΩ and 2 kΩ) or a proper logic-level converter to drop the voltage safely to 3.3 V.
•	Power: Connect the HC SR04’s Vcc to 5 V (if your board or external supply provides 5 V).
•	Ground: Connect the sensor’s ground to the common ground.
5. Control Mode Switch
•	Switch Connection:
o	Connect one side of the switch to WeMos D12.
o	The other side of the switch should be connected to ground (or to 3.3 V) depending on whether you are using a pull-up or pull-down resistor configuration.
o	In the sketch, the code checks if the pin is HIGH (remote control mode). To keep it simple, you can use the WeMos’s built in pull up by changing the pinMode to INPUT_PULLUP (and then wiring your switch to ground). In our code example, the switch is assumed to be wired so that when pressed it drives the pin HIGH (you may also add an external resistor if needed).
6. (Optional) Camera Module
•	Camera:
o	Many camera modules (like an OV2640) are designed for boards such as the ESP32-CAM.
o	If you wish to add a camera to your WeMos D1, you’ll need a compatible module and appropriate wiring (often via SPI or parallel interfaces, plus additional pins for reset and power).
o	The /camera endpoint in the code is a placeholder. Once you choose a camera module, follow that module’s wiring diagram and library instructions.
o	Ensure that the power supply and logic levels for the camera are compatible with the WeMos.
