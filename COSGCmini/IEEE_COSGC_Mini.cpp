#include <Arduino.h>

// -----------------------------------------------------
// Motor Driver (DRV8833) Pin Definitions
// -----------------------------------------------------
const int leftForwardPin  = 3;  // Left side driver: forward (PWM)
const int leftReversePin  = 4;  // Left side driver: reverse (PWM)
const int rightForwardPin = 5;  // Right side driver: forward (PWM)
const int rightReversePin = 6;  // Right side driver: reverse (PWM)

// -----------------------------------------------------
// Ultrasonic Sensor Pin Definitions (digital)
// -----------------------------------------------------
// Front sensor:
const int frontTrigPin = 2;
const int frontEchoPin = 7;
// Left sensor:
const int leftTrigPin  = 8;
const int leftEchoPin  = 9;
// Right sensor:
const int rightTrigPin = 10;
const int rightEchoPin = 11;

// -----------------------------------------------------
// LED Indicator Pin Definitions
// -----------------------------------------------------
const int greenLEDPin = 12; // Green LED: Normal/straight driving
const int redLEDPin   = 13; // Red LED: Obstacle avoidance / recovery mode

// -----------------------------------------------------
// Accelerometer (GY‑61 ADXL335) Pin Definitions (Analog)
// -----------------------------------------------------
const int accelXPin = A0;
const int accelYPin = A1;
const int accelZPin = A2;

// -----------------------------------------------------
// Autonomous Parameters & Thresholds
// -----------------------------------------------------
const int ultrasonicFrontThreshold = 20;         // cm; front sensor: obstacle if <20 cm
const int ultrasonicSideThreshold  = 15;         // cm; for left/right sensors
const int ultrasonicAggressiveThreshold = 10;      // cm; if front sensor <10 cm, aggressive maneuver
const unsigned long obstacleStuckThreshold = 3000; // 3 seconds before triggering recovery
const float tiltThreshold = 0.5;                   // g; if |xG| or |yG| exceeds this, too tilted

unsigned long obstacleStartTime = 0;

// -----------------------------------------------------
// Function Prototypes
// -----------------------------------------------------
void driveForward(int speed);
void driveBackward(int speed);
void stopMotors();
void skidTurnRight(int leftSpeed, int rightSpeed);
void skidTurnLeft(int leftSpeed, int rightSpeed);
float readUltrasonic(int trigPin, int echoPin);
float readAccelerometerAxis(int pin);
void checkTiltAndRecover();
void recoverFromStuck();
void autonomousBehavior();
void debugSensors();

// -----------------------------------------------------
// Setup Function
// -----------------------------------------------------
void setup() {
  Serial.begin(9600);
  Serial.println("Autonomous Skid-Steering Robot Starting...");

  // Initialize motor driver pins as outputs
  pinMode(leftForwardPin, OUTPUT);
  pinMode(leftReversePin, OUTPUT);
  pinMode(rightForwardPin, OUTPUT);
  pinMode(rightReversePin, OUTPUT);

  // Initialize LED pins as outputs
  pinMode(greenLEDPin, OUTPUT);
  pinMode(redLEDPin, OUTPUT);
  // Default state: normal mode (green LED ON, red LED OFF)
  digitalWrite(greenLEDPin, HIGH);
  digitalWrite(redLEDPin, LOW);

  // Stop motors initially
  stopMotors();

  Serial.println("Setup complete. Entering autonomous mode.");
}

// -----------------------------------------------------
// Main Loop
// -----------------------------------------------------
void loop() {
  // First, check tilt using the accelerometer. If excessive, recover.
  checkTiltAndRecover();
  
  // Then, execute autonomous behavior based on ultrasonic sensor readings.
  autonomousBehavior();
  
  // Output debug information.
  debugSensors();
  
  delay(100);
}

// -----------------------------------------------------
// Motor Control Functions
// -----------------------------------------------------
void driveForward(int speed) {
  // To drive forward: apply PWM to forward pins and set reverse pins LOW.
  analogWrite(leftForwardPin, speed);
  digitalWrite(leftReversePin, LOW);
  analogWrite(rightForwardPin, speed);
  digitalWrite(rightReversePin, LOW);
  Serial.println("Motors: Driving forward");
}

void driveBackward(int speed) {
  // To drive backward: apply PWM to reverse pins and set forward pins LOW.
  digitalWrite(leftForwardPin, LOW);
  analogWrite(leftReversePin, speed);
  digitalWrite(rightForwardPin, LOW);
  analogWrite(rightReversePin, speed);
  Serial.println("Motors: Driving backward");
}

void stopMotors() {
  digitalWrite(leftForwardPin, LOW);
  digitalWrite(leftReversePin, LOW);
  digitalWrite(rightForwardPin, LOW);
  digitalWrite(rightReversePin, LOW);
  Serial.println("Motors: Stopped");
}

void skidTurnRight(int leftSpeed, int rightSpeed) {
  // For a right turn: drive the left side at higher speed and the right side at lower speed.
  analogWrite(leftForwardPin, leftSpeed);
  digitalWrite(leftReversePin, LOW);
  analogWrite(rightForwardPin, rightSpeed);
  digitalWrite(rightReversePin, LOW);
  Serial.println("Motors: Skid turning right");
}

void skidTurnLeft(int leftSpeed, int rightSpeed) {
  // For a left turn: drive the right side at higher speed and the left side at lower speed.
  analogWrite(leftForwardPin, leftSpeed);
  digitalWrite(leftReversePin, LOW);
  analogWrite(rightForwardPin, rightSpeed);
  digitalWrite(rightReversePin, LOW);
  Serial.println("Motors: Skid turning left");
}

// -----------------------------------------------------
// Ultrasonic Sensor Reading
// -----------------------------------------------------
float readUltrasonic(int trigPin, int echoPin) {
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  pinMode(echoPin, INPUT);
  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  float distance = (duration * 0.0343) / 2.0; // Convert to cm
  return distance;
}

// -----------------------------------------------------
// Accelerometer Reading (GY-61 ADXL335)
// -----------------------------------------------------
float readAccelerometerAxis(int pin) {
  int raw = analogRead(pin);
  // Example conversion: adjust these values based on calibration.
  return (raw - 338.0) / 100.0;
}

// -----------------------------------------------------
// Tilt Check and Recovery
// -----------------------------------------------------
void checkTiltAndRecover() {
  float xG = readAccelerometerAxis(accelXPin); // X-axis on A0
  float yG = readAccelerometerAxis(accelYPin); // Y-axis on A1
  // Ignore z-axis for tilt decisions
  if (abs(xG) > tiltThreshold || abs(yG) > tiltThreshold) {
    Serial.println("Excessive tilt detected! Initiating recovery mode.");
    digitalWrite(redLEDPin, HIGH);
    digitalWrite(greenLEDPin, LOW);
    recoverFromStuck();
    // Wait until tilt returns to normal before resuming
    while (abs(readAccelerometerAxis(accelXPin)) > tiltThreshold ||
           abs(readAccelerometerAxis(accelYPin)) > tiltThreshold) {
      Serial.print("Waiting for level... X=");
      Serial.print(readAccelerometerAxis(accelXPin), 2);
      Serial.print(", Y=");
      Serial.println(readAccelerometerAxis(accelYPin), 2);
      delay(200);
    }
    stopMotors();
    delay(500);
    driveForward(255);
  }
}

// -----------------------------------------------------
// Recovery Routine if Stuck or Tilted
// -----------------------------------------------------
void recoverFromStuck() {
  Serial.println("Recovery: Initiating unstuck routine.");
  stopMotors();
  // Reverse at full power for 1 second
  driveBackward(255);
  delay(1000);
  stopMotors();
  // Execute a skid turn (mild right turn) for 1 second
  skidTurnRight(200, 150);
  delay(1000);
  stopMotors();
  // Resume forward motion
  driveForward(255);
  obstacleStartTime = 0;
}

// -----------------------------------------------------
// Autonomous Behavior
// -----------------------------------------------------
void autonomousBehavior() {
  float frontDist = readUltrasonic(frontTrigPin, frontEchoPin);
  float leftDist  = readUltrasonic(leftTrigPin, leftEchoPin);
  float rightDist = readUltrasonic(rightTrigPin, rightEchoPin);
  
  // If the front sensor detects an obstacle
  if (frontDist > 0 && frontDist < ultrasonicFrontThreshold) {
    Serial.println("Obstacle detected ahead!");
    stopMotors();
    digitalWrite(greenLEDPin, LOW);
    digitalWrite(redLEDPin, HIGH);
    if (obstacleStartTime == 0) { obstacleStartTime = millis(); }
    // If obstacle persists too long, trigger recovery
    if (millis() - obstacleStartTime > obstacleStuckThreshold) {
      recoverFromStuck();
    } else {
      // Now, if both left and right sensors detect objects (i.e. both are below side threshold), go forward full power
      if ( (leftDist > 0 && leftDist < ultrasonicSideThreshold) &&
           (rightDist > 0 && rightDist < ultrasonicSideThreshold) ) {
        Serial.println("Both side sensors detect obstacles. Going forward at full power.");
        driveForward(255);
      } else if (leftDist > ultrasonicSideThreshold) {
        // If left side is clear, turn right
        Serial.println("Left side clear. Skid turning right.");
        skidTurnRight(255, 150);
      } else if (rightDist > ultrasonicSideThreshold) {
        // If right side is clear, turn left
        Serial.println("Right side clear. Skid turning left.");
        skidTurnLeft(150, 255);
      } else {\n        // Otherwise, if one side is slightly clearer, choose that turn\n        if (leftDist > rightDist) {\n          Serial.println(\"Turning right based on sensor comparison.\");\n          skidTurnRight(255, 150);\n        } else {\n          Serial.println(\"Turning left based on sensor comparison.\");\n          skidTurnLeft(150, 255);\n        }\n      }\n    }\n  } else {\n    // No obstacle detected at front: reset timer, set LEDs for normal mode, and drive forward\n    obstacleStartTime = 0;\n    digitalWrite(greenLEDPin, HIGH);\n    digitalWrite(redLEDPin, LOW);\n    driveForward(255);\n  }\n}\n\n// -----------------------------------------------------\n// Debug Function: Print sensor data\n// -----------------------------------------------------\nvoid debugSensors() {\n  float front = readUltrasonic(frontTrigPin, frontEchoPin);\n  float left  = readUltrasonic(leftTrigPin, leftEchoPin);\n  float right = readUltrasonic(rightTrigPin, rightEchoPin);\n  Serial.print(\"DEBUG -> Ultrasonic (cm): Front=\"); Serial.print(front);\n  Serial.print(\", Left=\"); Serial.print(left);\n  Serial.print(\", Right=\"); Serial.println(right);\n  \n  float xG = readAccelerometerAxis(accelXPin);\n  float yG = readAccelerometerAxis(accelYPin);\n  float zG = readAccelerometerAxis(accelZPin);\n  Serial.print(\"DEBUG -> Accelerometer (g): X=\"); Serial.print(xG, 2);\n  Serial.print(\", Y=\"); Serial.print(yG, 2);\n  Serial.print(\", Z=\"); Serial.println(zG, 2);\n}\n\n// -----------------------------------------------------\n// Main Loop\n// -----------------------------------------------------\nvoid loop() {\n  checkTiltAndRecover();\n  autonomousBehavior();\n  debugSensors();\n  delay(100);\n}\n```

---

### **Key Points:**

- **Autonomous Mode:**  
  On power-up, the robot drives forward. If the front ultrasonic sensor detects an obstacle (distance below 20 cm), it stops and then checks the side sensors.  
  - If **both** left and right sensors detect objects (i.e. both report a distance below the side threshold of 15 cm), the code commands the robot to continue forward at full power.  
  - Otherwise, it turns away from the side where the sensor does not detect an object (or uses a simple comparison of left versus right clearances).  
- **Tilt Protection:**  
  The accelerometer is read on analog pins A0, A1, and A2. If either the X or Y axis reading (converted to g’s) exceeds the tilt threshold, the robot enters a recovery routine until the tilt returns to normal.  
- **LED Indicators:**  
  The green LED (pin 12) is ON during normal forward driving. The red LED (pin 13) turns ON when an obstacle is detected or when the robot is recovering.
- **Debugging:**  
  Serial print statements in `debugSensors()` and throughout the code provide insight into sensor values and decision-making.

---

### **Next Steps:**

1. **Wiring:**  
   - Connect your DRV8833 motor drivers to the designated digital pins.  
   - Wire the three ultrasonic sensors (front: pins 2 & 7; left: pins 8 & 9; right: pins 10 & 11).  
   - Connect the GY‑61 accelerometer to analog pins A0, A1, and A2.  
   - Connect the green LED to digital pin 12 and the red LED to digital pin 13 (with proper current-limiting resistors).  
2. **Power:**  
   - Use your external battery (e.g. 8V) to power the motor drivers. Ensure that the Arduino’s ground, the motor drivers’ ground, and the battery ground are all common.  
3. **Testing:**  
   - Upload the sketch to your Arduino UNO R3 and open the Serial Monitor (9600 baud).  
   - Test various scenarios (obstacle in front, obstacles on one side, and both sides) and tilt conditions to validate behavior.

