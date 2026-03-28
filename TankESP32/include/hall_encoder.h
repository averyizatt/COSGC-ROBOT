
#ifndef HALL_ENCODER_H
#define HALL_ENCODER_H

#include <Arduino.h>

// ==================== HALL EFFECT ENCODER ====================
// A3144 hall sensors on output shafts for real odometry.
// Interrupt-driven pulse counting → RPM, speed (cm/s), distance (cm).
// Feeds into dead-reckoning for accurate localization.

class HallEncoder {
public:
    void begin();
    HallEncoder();
    // Call once in setup() — attaches interrupts
    // --- Odometry (robot pose) ---
    float getX();      // cm
    float getY();      // cm
    float getTheta();  // radians

    // Call periodically (~20Hz) to compute speed from pulse counts
    void update();

    // --- Per-wheel data ---
    // Pulse counts (total since reset)
    volatile unsigned long getLeftPulses();
    volatile unsigned long getRightPulses();

    // Speed in cm/s (updated each update() call)
    float getLeftSpeed();
    float getRightSpeed();

    // Average forward speed (cm/s) — (left + right) / 2
    float getSpeed();

    // Fused speed (cm/s) — complementary filter of encoder + IMU accelerometer
    float getFusedSpeed();

    // Feed IMU forward acceleration (in g's) each update cycle
    void setForwardAccel(float accelG);

    // Distance traveled since reset (cm)
    float getLeftDistance();
    float getRightDistance();

    // RPM
    float getLeftRPM();
    float getRightRPM();

    // Whether sensors are producing pulses (motors running but no pulses = problem)
    bool isLeftStalled();
    bool isRightStalled();

private:
    // ISR-safe pulse counters (volatile, accessed from interrupts)
    static volatile unsigned long leftPulseCount;
    static volatile unsigned long rightPulseCount;

    // Odometry state
    float x = 0;      // cm
    float y = 0;      // cm
    float theta = 0;  // radians (0 = forward, CCW+)

    // Snapshot for speed calculation
    unsigned long lastLeftPulses;
    unsigned long lastRightPulses;
    unsigned long lastUpdateTime;

    // Computed values
    float leftSpeedCmS;
    float rightSpeedCmS;
    float leftDistanceCm;
    float rightDistanceCm;
    float leftRPM;
    float rightRPM;

    // Stall detection
    unsigned long lastLeftPulseTime;
    unsigned long lastRightPulseTime;

    // IMU fusion — complementary filter
    float forwardAccelG;   // latest IMU forward accel (g's)
    float imuSpeed;        // IMU-integrated velocity (cm/s)
    float fusedSpeedCmS;   // complementary-filtered output

    // Interrupt service routines
    static void IRAM_ATTR leftISR();
    static void IRAM_ATTR rightISR();
};

#endif // HALL_ENCODER_H
