#include <AFMotor.h>
#include <Servo.h>

// Motors
AF_DCMotor motorA(1);  
AF_DCMotor motorB(2);
AF_DCMotor motorC(3);
AF_DCMotor motorD(4);

// Servos
Servo leftServo;
Servo rightServo;

// Pins
#define ULTRASONIC_TRIG_PIN A2
#define ULTRASONIC_ECHO_PIN A3
#define IR_SENSOR_PIN A4

// Thresholds
const int IR_THRESHOLD = 500;
const int ULTRASONIC_THRESHOLD = 20;
const int ULTRASONIC_AGGRESSIVE_THRESHOLD = 10;
const unsigned long stuckThreshold = 3000;

unsigned long obstacleStartTime = 0;
bool autonomous = true;
char lastCommand = ' ';

// === Setup ===
void setup() {
  Serial.begin(9600);
  Serial.println("Robot Ready...");

  motorA.setSpeed(200);
  motorB.setSpeed(200);
  motorC.setSpeed(200);
  motorD.setSpeed(200);
  stopMotors();

  leftServo.attach(A0);
  rightServo.attach(A1);
  centerSteering();
}

// === Loop ===
void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    autonomous = false;
    lastCommand = command;
    Serial.print("Received Command: ");
    Serial.println(command);
    handleCommand(command);
  }

  if (autonomous) {
    runAutonomousMode();
  } else {
    debugSensors();
  }

  delay(100);
}

// === Command Handler ===
void handleCommand(char cmd) {
  switch (cmd) {
    case 'F': driveForward(); break;
    case 'B': driveBackward(); break;
    case 'L': steerLeftMild(); driveForward(); break;
    case 'R': steerRightMild(); driveForward(); break;
    case 'S': stopMotors(); break;
    case 'C': centerSteering(); break;
    default: Serial.println("Unknown Command."); stopMotors(); break;
  }
}

// === Autonomous Logic ===
void runAutonomousMode() {
  debugSensors();
  int irValue = analogRead(IR_SENSOR_PIN);
  float dist = getUltrasonicDistance();

  if (irValue < IR_THRESHOLD || (dist > 0 && dist < ULTRASONIC_THRESHOLD)) {
    stopMotors();
    if (obstacleStartTime == 0) obstacleStartTime = millis();

    if (millis() - obstacleStartTime > stuckThreshold) {
      recoverFromStuck();
    } else {
      if (dist < ULTRASONIC_AGGRESSIVE_THRESHOLD) {
        steerRightAggressive();
      } else {
        steerRightMild();
      }
      driveForward();
    }
  } else {
    obstacleStartTime = 0;
    centerSteering();
    driveForward();
  }
}

// === Motion Functions ===
void driveForward() {
  motorA.run(FORWARD);
  motorB.run(FORWARD);
  motorC.run(FORWARD);
  motorD.run(FORWARD);
  Serial.println("Driving Forward");
}

void driveBackward() {
  motorA.run(BACKWARD);
  motorB.run(BACKWARD);
  motorC.run(BACKWARD);
  motorD.run(BACKWARD);
  Serial.println("Driving Backward");
}

void stopMotors() {
  motorA.run(RELEASE);
  motorB.run(RELEASE);
  motorC.run(RELEASE);
  motorD.run(RELEASE);
  Serial.println("Motors Stopped");
}

// === Steering Functions ===
void centerSteering() {
  leftServo.write(90);
  rightServo.write(90);
  Serial.println("Steering Centered");
}

void steerRightAggressive() {
  leftServo.write(60);
  rightServo.write(120);
  Serial.println("Aggressive Right Turn");
}

void steerRightMild() {
  leftServo.write(75);
  rightServo.write(105);
  Serial.println("Mild Right Turn");
}

void steerLeftMild() {
  leftServo.write(105);
  rightServo.write(75);
  Serial.println("Mild Left Turn");
}

void steerLeftAggressive() {
  leftServo.write(120);
  rightServo.write(60);
  Serial.println("Aggressive Left Turn");
}

// === Ultrasonic ===
float getUltrasonicDistance() {
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 30000);
  float distance = (duration * 0.0343) / 2.0;
  return distance;
}

// === Recovery ===
void recoverFromStuck() {
  Serial.println("Stuck! Recovering...");
  stopMotors();
  motorA.setSpeed(255); motorB.setSpeed(255);
  motorC.setSpeed(255); motorD.setSpeed(255);
  driveBackward(); delay(1000); stopMotors();
  motorA.setSpeed(200); motorB.setSpeed(200);
  motorC.setSpeed(200); motorD.setSpeed(200);
  steerRightMild(); driveForward(); delay(1000);
  centerSteering();
  obstacleStartTime = 0;
}

// === Debug ===
void debugSensors() {
  int ir = analogRead(IR_SENSOR_PIN);
  float dist = getUltrasonicDistance();
  Serial.print("IR: "); Serial.print(ir);
  Serial.print(" | Distance: "); Serial.print(dist); Serial.println(" cm");
}
