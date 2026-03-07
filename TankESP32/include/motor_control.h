#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

class MotorControl {
public:
    MotorControl();
    void begin();
    
    // Motor control methods
    void setMotorA(int speed);  // -255 to 255
    void setMotorB(int speed);  // -255 to 255
    void setMotors(int speedA, int speedB);
    void stop();
    void standby(bool enable);
    
    // Movement commands (auto-adjusted for upside-down)
    void forward(int speed);
    void backward(int speed);
    void turnLeft(int speed);
    void turnRight(int speed);
    
    // Upside-down handling - call this from IMU update
    // When upside down, all motor commands are automatically inverted
    void setUpsideDown(bool flipped);
    bool isUpsideDown() { return upsideDown; }
    
    // Diagnostic test
    void motorTest();

private:
    void setupPWM();
    bool upsideDown;  // True when robot is flipped over
    int lastCommandA;
    int lastCommandB;
};

#endif // MOTOR_CONTROL_H
