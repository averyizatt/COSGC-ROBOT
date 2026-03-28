#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

class MotorControl {
public:
    MotorControl();
    void begin();
    
    // Motor control — speed range: -MAX_PWM to +MAX_PWM
    void setMotors(int speedA, int speedB);
    void stop();
    void standby(bool enable);
    
    // Movement helpers (auto-adjusted for upside-down)
    void forward(int speed);
    void backward(int speed);
    void turnLeft(int speed);
    void turnRight(int speed);
    
    // Upside-down handling
    void setUpsideDown(bool flipped);
    bool isUpsideDown() { return upsideDown; }
    
    // Safety — call from main loop every iteration
    void safetyCheck();
    
    // Status
    float getESPTemperature();       // Internal temp in °C
    bool isThermalThrottled();       // True if temp-limited
    bool isDutyCycleThrottled();     // True if duty-limited
    bool isMotorsAllowed();          // False during startup delay or emergency
    float getEstimatedCurrentA();    // Estimated motor current (A) for Motor A
    float getEstimatedCurrentB();    // Estimated motor current (A) for Motor B
    
private:
    void setMotorA(int speed);
    void setMotorB(int speed);
    void driveMotorA(int speed);
    void driveMotorB(int speed);
    int  applySlewRate(int current, int target, unsigned long &lastSlew);
    int  applyCurrentLimit(int speed);
    int  applyCalibration(int speed, float cal);
    int  applySafetyLimit(int speed);
    void emergencyStop(const char* reason);
    int  computeMaxPWMFromCurrent(float maxAmps);
    
    bool upsideDown;
    bool motorAIdle;
    bool motorBIdle;
    
    // Slew rate state (per motor)
    int  currentDutyA;         // Actual PWM duty being output on Motor A
    int  currentDutyB;         // Actual PWM duty being output on Motor B
    unsigned long lastSlewA;   // Last slew update time for A
    unsigned long lastSlewB;   // Last slew update time for B
    
    // Direction tracking (for dead-time)
    int  lastDirA;             // -1, 0, +1
    int  lastDirB;
    unsigned long dirChangeTimeA;  // Non-blocking dead-time timestamp
    unsigned long dirChangeTimeB;
    
    // Current limiting
    int  maxSafePWM;           // Max PWM computed from DRV8871 current limit
    
    // Safety state
    unsigned long bootTime;
    unsigned long lastCommandTime;
    unsigned long fullSpeedStart;
    unsigned long cooldownUntil;
    float lastTemp;
    unsigned long lastTempCheck;
    bool thermalShutdown;
    bool emergencyStopped;
    int currentLimitPWM;
};

#endif // MOTOR_CONTROL_H
