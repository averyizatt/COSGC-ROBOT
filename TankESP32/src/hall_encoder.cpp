
#include "hall_encoder.h"
#include "config.h"
// Wheelbase: use TRACK_WIDTH_MM from config.h, converted to cm
#define ENCODER_WHEELBASE_CM (TRACK_WIDTH_MM / 10.0f)

// ==================== HALL EFFECT ENCODER ====================
// A3144 sensors on output shafts — interrupt-driven pulse counting.
// 2 magnets per shaft, 26mm drive sprocket → ~4.08 cm per pulse.

// Derived constant
static const float CM_PER_PULSE = (PI * ENCODER_WHEEL_DIA_MM / 10.0f) / ENCODER_MAGNETS;
// π × 26mm / 10 = 8.168cm circumference, / 2 magnets = 4.084 cm/pulse

// Stall: no pulse for this long while motors should be running
static const unsigned long ENCODER_STALL_TIMEOUT_MS = 500;

// ISR pulse counters (must be volatile + static)
volatile unsigned long HallEncoder::leftPulseCount = 0;
volatile unsigned long HallEncoder::rightPulseCount = 0;

// Debounce, smoothing, and filter constants — pulled from config.h
// ENCODER_DEBOUNCE_US, ENCODER_SPEED_EMA_ALPHA, ENCODER_COMP_ALPHA
static volatile unsigned long lastLeftISR_us = 0;
static volatile unsigned long lastRightISR_us = 0;

// Max physically possible speed (cm/s). Anything above this is EMI noise.
// At 1500 RPM, 2 magnets, 4.08 cm/pulse → 1500/60*2*4.08 ≈ 204 cm/s
static const float MAX_SPEED_CM_S = 250.0f;
static const float G_TO_CM_S2 = 980.665f;  // 1 g = 980.665 cm/s²

// ==================== ISRs ====================

void IRAM_ATTR HallEncoder::leftISR() {
    unsigned long now = micros();
    if (now - lastLeftISR_us > ENCODER_DEBOUNCE_US) {
        leftPulseCount++;
        lastLeftISR_us = now;
    }
}

void IRAM_ATTR HallEncoder::rightISR() {
    unsigned long now = micros();
    if (now - lastRightISR_us > ENCODER_DEBOUNCE_US) {
        rightPulseCount++;
        lastRightISR_us = now;
    }
}

// ==================== CONSTRUCTOR ====================

HallEncoder::HallEncoder() {
    lastLeftPulses = 0;
    lastRightPulses = 0;
    lastUpdateTime = 0;
    leftSpeedCmS = 0;
    rightSpeedCmS = 0;
    leftDistanceCm = 0;
    rightDistanceCm = 0;
    leftRPM = 0;
    rightRPM = 0;
    lastLeftPulseTime = 0;
    lastRightPulseTime = 0;
    forwardAccelG = 0;
    imuSpeed = 0;
    fusedSpeedCmS = 0;
}

// ==================== BEGIN ====================

void HallEncoder::begin() {
    // Configure pins as inputs (external 10kΩ pull-ups to 3.3V required)
    pinMode(ENCODER_LEFT_PIN, INPUT);
    pinMode(ENCODER_RIGHT_PIN, INPUT);

    // Attach interrupts — A3144 output goes LOW when magnet passes
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), leftISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), rightISR, FALLING);

    lastUpdateTime = millis();
    lastLeftPulseTime = millis();
    lastRightPulseTime = millis();

    Serial.printf("[ENCODER] Init: pin L=%d R=%d, %d magnets, wheel=%.0fmm, %.2f cm/pulse\n",
                  ENCODER_LEFT_PIN, ENCODER_RIGHT_PIN, ENCODER_MAGNETS,
                  ENCODER_WHEEL_DIA_MM, CM_PER_PULSE);
}

// ==================== UPDATE ====================
// Call at ~20Hz from main loop. Computes speed from pulse delta.

void HallEncoder::update() {
    unsigned long now = millis();
    float dt = (now - lastUpdateTime) / 1000.0f;  // seconds

    if (dt < 0.01f) return;  // Too fast, skip

    // Snapshot pulse counts (atomic read from volatile)
    unsigned long leftNow = leftPulseCount;
    unsigned long rightNow = rightPulseCount;

    unsigned long leftDelta = leftNow - lastLeftPulses;
    unsigned long rightDelta = rightNow - lastRightPulses;

    // Distance this interval
    float leftDist = leftDelta * CM_PER_PULSE;
    float rightDist = rightDelta * CM_PER_PULSE;

    leftDistanceCm += leftDist;
    rightDistanceCm += rightDist;

    // --- ODOMETRY UPDATE ---
    // Differential drive: update x, y, theta
    float dCenter = (leftDist + rightDist) / 2.0f;
    float dTheta = (rightDist - leftDist) / ENCODER_WHEELBASE_CM; // radians
    // Integrate pose
    theta += dTheta;
    // Keep theta in -pi..pi
    while (theta > PI) theta -= 2 * PI;
    while (theta < -PI) theta += 2 * PI;

    x += dCenter * sin(theta);
    y += dCenter * cos(theta);



    // Speed (cm/s) — raw then smoothed
    float rawLeftSpeed = leftDist / dt;
    float rawRightSpeed = rightDist / dt;

    // Clamp impossible speeds (EMI noise bursts)
    if (rawLeftSpeed > MAX_SPEED_CM_S)  rawLeftSpeed = leftSpeedCmS;  // keep previous
    if (rawRightSpeed > MAX_SPEED_CM_S) rawRightSpeed = rightSpeedCmS;

    // Exponential moving average
    leftSpeedCmS  = ENCODER_SPEED_EMA_ALPHA * rawLeftSpeed  + (1.0f - ENCODER_SPEED_EMA_ALPHA) * leftSpeedCmS;
    rightSpeedCmS = ENCODER_SPEED_EMA_ALPHA * rawRightSpeed + (1.0f - ENCODER_SPEED_EMA_ALPHA) * rightSpeedCmS;

    // Kill residual drift when truly stopped
    if (leftDelta == 0 && leftSpeedCmS < 0.5f)  leftSpeedCmS = 0;
    if (rightDelta == 0 && rightSpeedCmS < 0.5f) rightSpeedCmS = 0;

    // RPM
    float leftRevs = (float)leftDelta / ENCODER_MAGNETS;
    float rightRevs = (float)rightDelta / ENCODER_MAGNETS;
    leftRPM = (leftRevs / dt) * 60.0f;
    rightRPM = (rightRevs / dt) * 60.0f;

    // Stall tracking
    if (leftDelta > 0) lastLeftPulseTime = now;
    if (rightDelta > 0) lastRightPulseTime = now;

    // --- COMPLEMENTARY FILTER: encoder + IMU fusion ---
    float encoderSpeed = (leftSpeedCmS + rightSpeedCmS) / 2.0f;
    // Integrate IMU acceleration to get IMU-predicted velocity
    imuSpeed += forwardAccelG * G_TO_CM_S2 * dt;
    // Blend: trust encoder for long-term, IMU for short-term changes
    fusedSpeedCmS = ENCODER_COMP_ALPHA * encoderSpeed + (1.0f - ENCODER_COMP_ALPHA) * imuSpeed;
    // Anchor IMU integrator to fused value (prevents unbounded drift)
    imuSpeed = fusedSpeedCmS;
    // Clamp negative (robot doesn't report reverse as negative speed here)
    if (fusedSpeedCmS < 0) fusedSpeedCmS = 0;

    // Save snapshot
    lastLeftPulses = leftNow;
    lastRightPulses = rightNow;
    lastUpdateTime = now;
}

// --- Odometry accessors ---
float HallEncoder::getX() { return x; }
float HallEncoder::getY() { return y; }
float HallEncoder::getTheta() { return theta; }


volatile unsigned long HallEncoder::getLeftPulses()  { return leftPulseCount; }
volatile unsigned long HallEncoder::getRightPulses() { return rightPulseCount; }

float HallEncoder::getLeftSpeed()   { return leftSpeedCmS; }
float HallEncoder::getRightSpeed()  { return rightSpeedCmS; }
float HallEncoder::getSpeed()       { return (leftSpeedCmS + rightSpeedCmS) / 2.0f; }
float HallEncoder::getFusedSpeed()   { return fusedSpeedCmS; }

void HallEncoder::setForwardAccel(float accelG) { forwardAccelG = accelG; }

float HallEncoder::getLeftDistance()  { return leftDistanceCm; }
float HallEncoder::getRightDistance() { return rightDistanceCm; }

float HallEncoder::getLeftRPM()  { return leftRPM; }
float HallEncoder::getRightRPM() { return rightRPM; }

bool HallEncoder::isLeftStalled() {
    return (millis() - lastLeftPulseTime) > ENCODER_STALL_TIMEOUT_MS;
}

bool HallEncoder::isRightStalled() {
    return (millis() - lastRightPulseTime) > ENCODER_STALL_TIMEOUT_MS;
}
