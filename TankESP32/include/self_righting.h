#ifndef SELF_RIGHTING_H
#define SELF_RIGHTING_H

#include <Arduino.h>

// Self-righting arm states
enum RightingState {
    ARM_STOWED,         // Arm up (vertical) — no motion
    ARM_IDLE,           // Gentle left-right oscillation (~10°) while driving
    ARM_RIGHTING,       // Aggressive sweep to flip robot (upside-down)
    ARM_SIDE_RIGHTING,  // Directed push to flip robot off its side
    ARM_UNSTICKING,     // Aggressive sweep to help free a stuck robot (upright)
    ARM_COOLDOWN        // Brief pause after righting/unsticking before re-checking
};

class SelfRightingArm {
public:
    SelfRightingArm();
    void begin();

    // Call every loop iteration with orientation status
    // rollDeg: roll angle in degrees (positive = tilted right, negative = tilted left)
    void update(bool isUpsideDown, bool isOnSide, float rollDeg);

    // Manual control
    void stow();          // Move arm to upright position
    void setAngle(int angle);  // Direct angle control (0-180)

private:
    void writeAngle(int angle);
    void detachServo();     // Stop PWM output (servo goes limp, saves power/heat)

    RightingState state;
    int currentAngle;       // Current servo angle (0-180)
    int sweepDirection;     // +1 or -1
    unsigned long lastSweepTime;
    unsigned long rightingStartTime;
    unsigned long cooldownUntil;
    int sweepCount;         // Number of full sweeps completed
    bool holdingAtExtreme;  // True when holding at 0° or 180°
    unsigned long holdStartTime;  // When we started holding at extreme

    // Idle oscillation state
    int idleCenter;         // Center angle for idle sweep
    int idleAmplitude;      // Half-range of idle sweep (degrees)

    // Side-righting state
    int sideTargetAngle;        // Which extreme to push toward (0 or 180)
    unsigned long sideStartTime; // When side-righting began
    int sidePushCount;          // Number of push attempts
};

#endif // SELF_RIGHTING_H
