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

char lastCommand = ' ';

void setup() {
  Serial.begin(9600);
  Serial.println("Robot Ready (Manual Control)");

  motorA.setSpeed(200);
  motorB.setSpeed(200);
  motorC.setSpeed(200);
  motorD.setSpeed(200);
  stopMotors();

  leftServo.attach(A0);
  rightServo.attach(A1);
  centerSteering();
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    lastCommand = command;
    Serial.print("Received Command: ");
    Serial.println(command);
    handleCommand(command);
  }

  delay(50);  // Optional: debounce input
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
    case 'X': steerLeftAggressive(); break;
    case 'Y': steerRightAggressive(); break;
    // Add more custom behaviors here
    default:
      Serial.println("❓ Unknown Command");
      stopMotors();
      break;
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
