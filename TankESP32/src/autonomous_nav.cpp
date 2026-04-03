#include "autonomous_nav.h"
#include "config.h"
#include "globals.h"
#include "motor_control.h"
#include "path_planner.h"

// Set motor speeds helper for navigation
void setMotorSpeeds(int left, int right) {
    extern MotorControl motors;
    motors.setMotors(left, right);
}

// Navigation state variables (single definition where appropriate)
bool turningToHeading = false;
float targetTurnTheta = 0;
bool hasNavTarget = false;
float navTargetX = 0, navTargetY = 0;
// odometry variables are defined centrally in globals.cpp
extern float odomX;
extern float odomY;
extern float odomTheta;

// --- Slip/Stall detection helper ---
bool AutonomousNav::isSlippingOrStalled() {
    // If commanded speed is high but encoder speed is low, consider slip/stall
    return (currentSpeed > 100 && encoderSpeedCmS < 5.0f);
}
void AutonomousNav::setOdometry(float x, float y, float theta) {
    odomX = x;
    odomY = y;
    odomTheta = theta;
}
void AutonomousNav::turnToHeading(float targetTheta) {
    targetTurnTheta = targetTheta;
    turningToHeading = true;
}
void AutonomousNav::returnToStart() {
    // Use path planner for obstacle-aware routing back to start
    extern PathPlanner pathPlanner;
    pathPlanner.returnHome();
    setNavTarget(0, 0);  // Also set direct target as fallback
}
void AutonomousNav::setNavTarget(float x_cm, float y_cm) {
    navTargetX = x_cm;
    navTargetY = y_cm;
    hasNavTarget = true;
}

// ==================== NAVIGATION CONSTANTS ====================
// Distance/timing thresholds are in config.h (DIST_CRITICAL, DIST_CLOSE_ADV, etc.)

// Local aliases — map config.h names to the names used in this file
#define DIST_CLOSE   DIST_CLOSE_ADV
#define DIST_MEDIUM  DIST_MEDIUM_ADV
#define DIST_FAR     DIST_FAR_ADV

// Speed settings (PWM 0-255, hardware clamps to maxSafePWM=210)
#define SPEED_MAX       MAX_PWM
#define SPEED_CRUISE    MAX_PWM
#define SPEED_TURN      MAX_PWM
#define SPEED_MIN_MOVE  (MAX_PWM * 80 / 100)  // Minimum to overcome friction

// ==================== CONSTRUCTOR ====================

AutonomousNav::AutonomousNav() {
    currentState = NAV_STOPPED;
    stateStartTime = 0;
    lastStateChange = 0;
    lastTurnDirection = TURN_RANDOM;
    
    lastDistance = 0;
    historyIndex = 0;
    stuckCounter = 0;
    currentSpeed = 0;
    targetSpeed = 0;
    stallStartTime = 0;
    
    sideDistLeft = 999.0f;
    sideDistRight = 999.0f;
    detectedWallAngle = 0.0f;
    
    // IMU
    upsideDown = false;
    currentOrientation = ORIENTATION_NORMAL;
    pitch = 0; roll = 0;
    accelX = 0; accelY = 0; accelZ = 1.0f;
    gyroZ = 0;
    imuAvailable = false;
    
    // Heading / DR
    heading = 0;
    headingAtObstacle = 0;
    lastIMUUpdate = 0;
    drX = 0; drY = 0;
    headingTowardStartSince = 0;
    cruiseHeading = 0;
    headingLocked = false;
    cruiseEnteredAt = 0;
    turnStartHeading = 0;
    consecutiveSameDirection = 0;
    for (int i = 0; i < 8; i++) obstacleMemory[i] = 0;
    
    firstRun = true;
    
    // Avoidance
    avoidPhase = AVOID_BACKUP;
    avoidTurnDir = TURN_RIGHT;
    avoidAttempts = 0;
    avoidPhaseStart = 0;
    avoidPhaseDuration = 0;
    
    // Hazard
    hazardPhase = HAZARD_BACKUP;
    hazardPhaseStart = 0;
    hazardPhaseDuration = 0;
    inclineAttempts = 0;
    steepInclineStart = 0;
    onSteepIncline = false;
    
    // Motion verification
    for (int i = 0; i < 8; i++) accelMagHistory[i] = 1.0f;
    accelMagIndex = 0;
    accelMagFilled = false;
    motionVerified = true;
    noMotionStartTime = 0;
    
    // Recovery
    recoveryPhase = RECOVERY_DONE;
    rockCount = 0;
    recoveryStepStart = 0;
    recoveryStepDuration = 0;
    recoveryTurnDir = TURN_RIGHT;
    coastAfterFwd = true;
    
    // Adaptive ramp
    recentlyStuck = false;
    stuckRecoveryTime = 0;
    recoveryCooldownUntil = 0;
    
    // Terrain boost
    terrainBoostActive = false;
    terrainBoostUntil = 0;
    terrainBoostStartX = 0;
    terrainBoostStartY = 0;
    terrainBoostStuck = false;
    
    for (int i = 0; i < 5; i++) distanceHistory[i] = 999.0f;
    
    randomSeed(millis() ^ ESP.getCycleCount());
}

// ==================== IMU UPDATE ====================

void AutonomousNav::updateIMU(float ax, float ay, float az, float gx, float gy, float gz) {
    unsigned long now = millis();
    float dt = (lastIMUUpdate > 0) ? (now - lastIMUUpdate) / 1000.0f : 0;
    lastIMUUpdate = now;
    
    accelX = ax; accelY = ay; accelZ = az;
    gyroZ = gz;
    imuAvailable = true;
    
    // Integrate gyroscope for heading (subtract configured drift offset)
    if (dt > 0 && dt < 0.5f) {
        heading += (gz - GYRO_DRIFT_COMPENSATION) * dt;
        while (heading >= 360) heading -= 360;
        while (heading < 0) heading += 360;
        
        // Dead-reckoning position (only when cruising forward)
        if (currentState == NAV_CRUISE) {
            // Use odometry if available (from encoders)
            if (odomX != 0 || odomY != 0) {
                drX = odomX;
                drY = odomY;
            } else {
                float speedCmS = encoderSpeedCmS;  // Use real encoder speed
                if (speedCmS < 1.0f) {
                    // Encoder not producing pulses — fall back to PWM estimate
                    float speedFraction = currentSpeed / 255.0f;
                    speedCmS = speedFraction * ROVER_MAX_SPEED_CM_S;
                }
                float distStep = speedCmS * dt;
                float headRad = heading * PI / 180.0f;
                drX += sin(headRad) * distStep;
                drY += cos(headRad) * distStep;
            }
        }
    }
    
    // Pitch and roll from accelerometer
    pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
    roll = atan2(ay, sqrt(ax * ax + az * az)) * 180.0f / PI;
    
    // Movement verification — accel magnitude variance
    float accelMag = sqrt(ax * ax + ay * ay + az * az);
    accelMagHistory[accelMagIndex] = accelMag;
    accelMagIndex = (accelMagIndex + 1) % MOTION_VERIFY_WINDOW;
    if (accelMagIndex == 0) accelMagFilled = true;
    if (accelMagFilled) checkMotionVerification();
    
    // Upside-down detection
    bool wasUpsideDown = upsideDown;
    upsideDown = (az < UPSIDE_DOWN_THRESHOLD);
    if (upsideDown != wasUpsideDown) {
        Serial.printf("[IMU] %s\n", upsideDown ? "UPSIDE DOWN!" : "Right-side up");
    }
    
    // Orientation
    Orientation prev = currentOrientation;
    if (upsideDown) {
        currentOrientation = ORIENTATION_UPSIDE_DOWN;
    } else if (abs(roll) > 30) {
        currentOrientation = (roll > 0) ? ORIENTATION_TILTED_RIGHT : ORIENTATION_TILTED_LEFT;
    } else if (abs(pitch) > 30) {
        currentOrientation = (pitch > 0) ? ORIENTATION_TILTED_FORWARD : ORIENTATION_TILTED_BACK;
    } else {
        currentOrientation = ORIENTATION_NORMAL;
    }
    if (currentOrientation != prev && currentOrientation != ORIENTATION_NORMAL) {
        const char* s = currentOrientation == ORIENTATION_UPSIDE_DOWN ? "UPSIDE_DOWN" :
            currentOrientation == ORIENTATION_TILTED_LEFT ? "TILTED_LEFT" :
            currentOrientation == ORIENTATION_TILTED_RIGHT ? "TILTED_RIGHT" :
            currentOrientation == ORIENTATION_TILTED_FORWARD ? "TILTED_FWD" : "TILTED_BACK";
        Serial.printf("[IMU] Orientation: %s (pitch=%.1f, roll=%.1f)\n", s, pitch, roll);
    }
}

// ==================== SENSOR INPUTS ====================

void AutonomousNav::setSideDistances(float leftDist, float rightDist) {
    sideDistLeft = (leftDist < 0 || leftDist > 400) ? 999.0f : leftDist;
    sideDistRight = (rightDist < 0 || rightDist > 400) ? 999.0f : rightDist;
}

void AutonomousNav::setWallAngle(float angle) {
    detectedWallAngle = angle;
}

void AutonomousNav::setEncoderSpeed(float speedCmPerSec) {
    encoderSpeedCmS = speedCmPerSec;
}

// ==================== GETTERS ====================

bool AutonomousNav::isUpsideDown() { return upsideDown; }
float AutonomousNav::getPitch() { return pitch; }
float AutonomousNav::getRoll() { return roll; }
float AutonomousNav::getHeading() { return heading; }
bool AutonomousNav::isMotionVerified() { return motionVerified; }
bool AutonomousNav::isRecentlyStuck() { return recentlyStuck; }

const char* AutonomousNav::getStateString() {
    switch (currentState) {
        case NAV_CRUISE:  return "CRUISE";
        case NAV_AVOID:   return "AVOID";
        case NAV_RECOVER: return "RECOVER";
        case NAV_HAZARD:  return "HAZARD";
        case NAV_STOPPED: return "STOP";
        default:          return "?";
    }
}

// ==================== MAIN UPDATE ====================
// Clean dispatcher — each state handler owns its own decisions.

void AutonomousNav::update(float distance) {
    unsigned long now = millis();
    
    // Sanitize input
    if (distance < 0 || distance > 400) {
        distance = DIST_CLOSE - 1.0f;  // Fail toward caution
    }
    
    updateDistanceHistory(distance);
    
    // Adaptive ramp timeout
    if (recentlyStuck && (now - stuckRecoveryTime > ADAPTIVE_RAMP_TIMEOUT)) {
        recentlyStuck = false;
    }
    
    // First update → start cruising
    if (firstRun) {
        firstRun = false;
        enterCruise();
    }
    
    // Dispatch to current state handler
    switch (currentState) {
        case NAV_CRUISE:  handleCruise(distance);   break;
        case NAV_AVOID:   handleAvoid(distance);    break;
        case NAV_RECOVER: handleRecovery(distance);  break;
        case NAV_HAZARD:  handleHazard(distance);   break;
        case NAV_STOPPED: targetSpeed = 0;           break;
    }
    
    smoothSpeed();
    lastDistance = distance;
}

// ==================== STATE: CRUISE ====================
// Normal forward driving. Monitors for:
//   - Obstacles (→ AVOID)
//   - Stuck/no-motion (→ RECOVER)
//   - Steep incline (→ HAZARD)
//   - Heading toward start (→ correction turn via AVOID)

void AutonomousNav::handleCruise(float distance) {
                // --- TURN TO HEADING ---
                if (turningToHeading) {
                    float headingError = targetTurnTheta - odomTheta;
                    while (headingError > PI) headingError -= 2*PI;
                    while (headingError < -PI) headingError += 2*PI;
                    if (fabs(headingError) < 0.05f) { // ~3 deg
                        turningToHeading = false;
                        setMotorSpeeds(0, 0);
                        targetSpeed = 0;
                        Serial.println("[NAV] Turn complete.");
                        return;
                    }
                    int turnPWM = (headingError > 0) ? 80 : -80;
                    setMotorSpeeds(-turnPWM, turnPWM);
                    targetSpeed = 0;
                    return;
                }
            // --- HAZARD MARKING: slip/stall ---
            if (isSlippingOrStalled()) {
                envMap.markHazardCell();
            }
        // --- WAYPOINT NAVIGATION ---
        if (hasNavTarget) {
            // Obstacle guard: abort waypoint following if wall is close
            if (distance < DIST_CLOSE) {
                Serial.printf("[NAV] Waypoint aborted — obstacle %.0fcm ahead → AVOID\n", distance);
                enterAvoid(distance < DIST_CRITICAL);
                return;
            }

            float dx = navTargetX - odomX;
            float dy = navTargetY - odomY;
            float distToTarget = sqrt(dx*dx + dy*dy);
            if (distToTarget < 10.0f) { // 10cm threshold
                Serial.println("[NAV] Waypoint reached!");
                hasNavTarget = false;
                targetSpeed = 0;
                return;
            }
            float targetHeading = atan2(dx, dy); // radians
            float headingError = targetHeading - odomTheta;
            // Normalize to -pi..pi
            while (headingError > PI) headingError -= 2*PI;
            while (headingError < -PI) headingError += 2*PI;
            // Simple proportional steering
            float steer = headingError * 2.0f; // gain
            steer = constrain(steer, -40, 40); // max PWM delta
            int baseSpeed = SPEED_CRUISE;
            int left = baseSpeed - steer;
            int right = baseSpeed + steer;
            left = constrain(left, -MAX_PWM, MAX_PWM);
            right = constrain(right, -MAX_PWM, MAX_PWM);
            setMotorSpeeds(left, right);
            targetSpeed = baseSpeed;
            return;
        }
    unsigned long now = millis();
    
    // --- OBSTACLE: CRITICAL ---
    if (distance < DIST_CRITICAL) {
        Serial.printf("[CRUISE] CRITICAL %.0fcm → AVOID\n", distance);
        enterAvoid(true);
        return;
    }
    
    // --- OBSTACLE: CLOSE ---
    if (distance < DIST_CLOSE) {
        Serial.printf("[CRUISE] Obstacle %.0fcm → AVOID\n", distance);
        enterAvoid(false);
        return;
    }
    
    // --- STUCK: Distance history flat ---
    if (isStuck()) {
        Serial.println("[CRUISE] Stuck detected → RECOVER");
        enterRecover();
        return;
    }
    
    // --- STUCK: IMU no-motion ---
    bool distStagnant = (abs(distance - lastDistance) < 3.0f);
    if (imuAvailable && !motionVerified && distStagnant &&
        currentSpeed > SPEED_MIN_MOVE &&
        now > recoveryCooldownUntil) {
        unsigned long noMotionDur = (noMotionStartTime > 0) ? (now - noMotionStartTime) : 0;
        if (noMotionDur > MOTION_VERIFY_TIMEOUT) {
            Serial.printf("[CRUISE] No motion for %lums → RECOVER\n", noMotionDur);
            enterRecover();
            return;
        }
    } else if (motionVerified || !distStagnant) {
        noMotionStartTime = 0;
    }
    
    // --- INCLINE CHECK ---
    if (imuAvailable) {
        float absPitch = abs(pitch);
        if (absPitch > INCLINE_MAX_PITCH) {
            if (steepInclineStart == 0) {
                steepInclineStart = now;
                Serial.printf("[CRUISE] Steep incline: pitch=%.1f°\n", pitch);
            } else if (now - steepInclineStart > INCLINE_TIMEOUT_MS) {
                Serial.printf("[CRUISE] Incline timeout → HAZARD (attempt %d)\n", inclineAttempts + 1);
                enterHazard();
                return;
            }
        } else {
            if (steepInclineStart > 0) {
                Serial.printf("[CRUISE] Incline cleared (pitch=%.1f°)\n", pitch);
            }
            steepInclineStart = 0;
            onSteepIncline = false;
            inclineAttempts = 0;
        }
    }
    
    // --- STALL: Distance stuck in medium range too long ---
    if (distance < DIST_MEDIUM) {
        if (stallStartTime == 0) {
            stallStartTime = now;
        } else if (now - stallStartTime > STALL_TIMEOUT_MS) {
            Serial.printf("[CRUISE] Stall at %.0fcm for >%dms → AVOID\n", distance, STALL_TIMEOUT_MS);
            enterAvoid(false);
            return;
        }
    } else {
        stallStartTime = 0;
    }
    
    // --- SPEED CONTROL ---
    targetSpeed = calculateSpeed(distance);
    
    // --- DR: PROACTIVE HEADING CORRECTION ---
    // If heading roughly toward start for too long with clear path, force correction
    float distFromStart = sqrt(drX * drX + drY * drY);
    if (distFromStart > DR_MIN_DIST_CM && distance > DR_CORRECT_CLEARANCE_CM) {
        float toBearing = atan2(-drX, -drY) * 180.0f / PI;
        float toError = toBearing - heading;
        while (toError > 180) toError -= 360;
        while (toError < -180) toError += 360;
        
        if (abs(toError) < DR_HEADING_TOWARD_DEG) {
            if (headingTowardStartSince == 0) {
                headingTowardStartSince = now;
            } else if (now - headingTowardStartSince > DR_CORRECT_TIMEOUT_MS) {
                // Force correction — use avoidance mechanism for the turn
                float awayBearing = atan2(drX, drY) * 180.0f / PI;
                float awayError = awayBearing - heading;
                while (awayError > 180) awayError -= 360;
                while (awayError < -180) awayError += 360;
                
                avoidPhase = AVOID_TURN;
                avoidTurnDir = (awayError > 0) ? TURN_RIGHT : TURN_LEFT;
                avoidPhaseStart = now;
                avoidPhaseDuration = DR_CORRECT_TURN_MS;
                avoidAttempts = 0;
                currentState = NAV_AVOID;
                lastStateChange = now;
                targetSpeed = SPEED_TURN;
                headingTowardStartSince = 0;
                Serial.printf("[DR] Heading toward start for >%ds — correction turn\n",
                              DR_CORRECT_TIMEOUT_MS / 1000);
                return;
            }
        } else {
            headingTowardStartSince = 0;
        }
    } else {
        headingTowardStartSince = 0;
    }
}

// ==================== STATE: AVOID ====================
// 3-phase obstacle avoidance: BACKUP → TURN → VERIFY
// Clean sequential execution — no decisions during timed actions.
//   BACKUP:  Reverse away from obstacle
//   TURN:    Turn toward clearer side (sensor-informed)
//   VERIFY:  Brief forward to confirm path is clear
//     → CRUISE if clear
//     → Retry BACKUP if still blocked (up to AVOID_MAX_CYCLES)
//     → RECOVER if max attempts exceeded

void AutonomousNav::handleAvoid(float distance) {
    unsigned long now = millis();
    unsigned long elapsed = now - avoidPhaseStart;
    
    // Wait for current phase timer
    if (elapsed < avoidPhaseDuration) {
        return;  // Still executing — no decisions
    }
    
    // Phase complete — advance
    switch (avoidPhase) {
        case AVOID_BACKUP: {
            // Backup done → pick turn direction and turn
            avoidTurnDir = pickTurnDirection(distance);
            
            // Turn duration based on wall angle
            float absAngle = abs(detectedWallAngle);
            unsigned long turnTime;
            if (absAngle > 60) turnTime = 125;       // Glancing
            else if (absAngle > 30) turnTime = 175;   // Moderate
            else turnTime = 250;                       // Head-on
            
            avoidPhase = AVOID_TURN;
            avoidPhaseStart = now;
            avoidPhaseDuration = turnTime;
            targetSpeed = SPEED_TURN;
            Serial.printf("[AVOID] TURN %s %lums (wall:%.0f°)\n",
                          avoidTurnDir == TURN_LEFT ? "LEFT" : "RIGHT",
                          turnTime, detectedWallAngle);
            break;
        }
        
        case AVOID_TURN: {
            // Turn done → verify path is clear
            avoidPhase = AVOID_VERIFY;
            avoidPhaseStart = now;
            avoidPhaseDuration = AVOID_VERIFY_MS;
            targetSpeed = SPEED_MIN_MOVE;
            Serial.println("[AVOID] VERIFY forward");
            break;
        }
        
        case AVOID_VERIFY: {
            // Verify done — check if path is clear
            if (distance > DIST_CLOSE) {
                // Path clear! Resume cruising
                Serial.printf("[AVOID] Clear (%.0fcm) → CRUISE\n", distance);
                avoidAttempts = 0;
                enterCruise();
            } else {
                // Still blocked
                avoidAttempts++;
                if (avoidAttempts >= AVOID_MAX_CYCLES) {
                    Serial.printf("[AVOID] %d attempts failed → RECOVER\n", avoidAttempts);
                    avoidAttempts = 0;
                    enterRecover();
                } else {
                    Serial.printf("[AVOID] Still blocked (%.0fcm) — retry %d/%d\n",
                                  distance, avoidAttempts, AVOID_MAX_CYCLES);
                    avoidPhase = AVOID_BACKUP;
                    avoidPhaseStart = now;
                    avoidPhaseDuration = AVOID_BACKUP_MS;
                    targetSpeed = SPEED_CRUISE;
                }
            }
            break;
        }
    }
}

// ==================== STATE: HAZARD ====================
// Steep incline handling: backup → turn → return to CRUISE.
// CRUISE will re-detect the incline if still present, incrementing attempt counter.
// After max attempts, escalates to RECOVER.

void AutonomousNav::handleHazard(float distance) {
    unsigned long now = millis();
    unsigned long elapsed = now - hazardPhaseStart;
    
    // Check if incline cleared during handling
    if (imuAvailable && abs(pitch) < INCLINE_MAX_PITCH) {
        Serial.printf("[HAZARD] Incline cleared (pitch=%.1f°) → CRUISE\n", pitch);
        steepInclineStart = 0;
        onSteepIncline = false;
        enterCruise();
        return;
    }
    
    // Wait for current phase timer
    if (elapsed < hazardPhaseDuration) return;
    
    switch (hazardPhase) {
        case HAZARD_BACKUP:
            // Backup done → turn to try a different angle
            hazardPhase = HAZARD_TURN;
            hazardPhaseStart = now;
            hazardPhaseDuration = INCLINE_DIAG_TURN_MS;
            targetSpeed = SPEED_TURN;
            Serial.printf("[HAZARD] TURN %s\n",
                          (inclineAttempts % 2 == 0) ? "RIGHT" : "LEFT");
            break;
        
        case HAZARD_TURN:
            // Turn done — go back to CRUISE to try new angle
            if (inclineAttempts >= 3) {
                Serial.printf("[HAZARD] %d attempts — escalating to RECOVER\n", inclineAttempts);
                inclineAttempts = 0;
                steepInclineStart = 0;
                onSteepIncline = false;
                enterRecover();
            } else {
                Serial.printf("[HAZARD] Attempt %d done → CRUISE\n", inclineAttempts);
                steepInclineStart = 0;  // Reset for next detection
                enterCruise();
            }
            break;
    }
}

// ==================== STATE: RECOVER ====================
// Multi-phase stuck recovery (proven sequence):
//   1. Rock forward/backward N times (break free from sand/rut)
//   2. Diagonal approach (turn + forward at different angle)
//   3. Full reverse escape (last resort)

void AutonomousNav::handleRecovery(float distance) {
    unsigned long now = millis();
    unsigned long elapsed = now - recoveryStepStart;
    
    // Wait for current step
    if (elapsed < recoveryStepDuration) return;
    
    switch (recoveryPhase) {
        case RECOVERY_ROCK_FWD:
            recoveryPhase = RECOVERY_COAST;
            recoveryStepStart = now;
            recoveryStepDuration = RECOVERY_COAST_MS;
            targetSpeed = 0;
            coastAfterFwd = true;
            break;
            
        case RECOVERY_COAST:
            if (coastAfterFwd) {
                recoveryPhase = RECOVERY_ROCK_REV;
                recoveryStepStart = now;
                recoveryStepDuration = RECOVERY_ROCK_MS;
                targetSpeed = RECOVERY_SPEED;
                Serial.printf("[RECOVER] ROCK_REV (cycle %d/%d)\n", rockCount + 1, RECOVERY_ROCK_ATTEMPTS);
            } else {
                if (rockCount < RECOVERY_ROCK_ATTEMPTS) {
                    recoveryPhase = RECOVERY_ROCK_FWD;
                    recoveryStepStart = now;
                    recoveryStepDuration = RECOVERY_ROCK_MS;
                    targetSpeed = RECOVERY_SPEED;
                    Serial.printf("[RECOVER] ROCK_FWD (cycle %d/%d)\n", rockCount + 1, RECOVERY_ROCK_ATTEMPTS);
                } else {
                    recoveryPhase = RECOVERY_DIAGONAL_TURN;
                    recoveryStepStart = now;
                    recoveryStepDuration = RECOVERY_DIAG_TURN_MS;
                    targetSpeed = SPEED_TURN;
                    Serial.printf("[RECOVER] DIAGONAL_TURN %s\n",
                                  recoveryTurnDir == TURN_LEFT ? "LEFT" : "RIGHT");
                }
            }
            break;
            
        case RECOVERY_ROCK_REV:
            rockCount++;
            recoveryPhase = RECOVERY_COAST;
            recoveryStepStart = now;
            recoveryStepDuration = RECOVERY_COAST_MS;
            targetSpeed = 0;
            coastAfterFwd = false;
            break;
            
        case RECOVERY_DIAGONAL_TURN:
            recoveryPhase = RECOVERY_DIAGONAL_FWD;
            recoveryStepStart = now;
            recoveryStepDuration = RECOVERY_DIAG_FWD_MS;
            targetSpeed = SPEED_CRUISE;
            Serial.printf("[RECOVER] DIAGONAL_FWD %dms\n", RECOVERY_DIAG_FWD_MS);
            break;
            
        case RECOVERY_DIAGONAL_FWD:
            if (motionVerified && distance > DIST_CLOSE) {
                Serial.println("[RECOVER] === FREE! → CRUISE ===");
                recoveryPhase = RECOVERY_DONE;
                flagRecentlyStuck();
                enterCruise();
            } else {
                recoveryPhase = RECOVERY_FULL_REVERSE;
                recoveryStepStart = now;
                recoveryStepDuration = RECOVERY_FULL_REV_MS;
                targetSpeed = RECOVERY_SPEED;
                Serial.printf("[RECOVER] FULL_REVERSE %dms\n", RECOVERY_FULL_REV_MS);
            }
            break;
            
        case RECOVERY_FULL_REVERSE: {
            Serial.println("[RECOVER] === Complete — turning to new heading ===");
            recoveryPhase = RECOVERY_DONE;
            flagRecentlyStuck();
            // Exit via a turn, then cruise
            TurnDirection dir = pickTurnDirection(distance);
            avoidPhase = AVOID_TURN;
            avoidTurnDir = dir;
            avoidPhaseStart = now;
            avoidPhaseDuration = 250;
            avoidAttempts = 0;
            currentState = NAV_AVOID;
            lastStateChange = now;
            targetSpeed = SPEED_TURN;
            break;
        }
            
        case RECOVERY_DONE:
        default:
            enterCruise();
            break;
    }
}

// ==================== STATE TRANSITIONS ====================

void AutonomousNav::enterCruise() {
    currentState = NAV_CRUISE;
    stateStartTime = millis();
    lastStateChange = millis();
    targetSpeed = SPEED_CRUISE;
    stuckCounter = 0;
    stallStartTime = 0;
    
    // Heading hold — don't lock immediately, let the robot settle after a turn
    headingLocked = false;
    cruiseEnteredAt = millis();
    
    // Reset IMU motion buffer (stale data from turns/reverses)
    accelMagFilled = false;
    accelMagIndex = 0;
    noMotionStartTime = 0;
    motionVerified = true;
    
    // Decay obstacle memory — old data fades
    for (int i = 0; i < 8; i++) {
        if (obstacleMemory[i] > 0) obstacleMemory[i]--;
    }
}

void AutonomousNav::enterAvoid(bool critical) {
    // If terrain boost was active, this obstacle/stuck means terrain was too tough
    if (terrainBoostActive) {
        terrainBoostStuck = true;
        terrainBoostActive = false;
        checkTerrainOutcome();
    }
    currentState = NAV_AVOID;
    stateStartTime = millis();
    lastStateChange = millis();
    avoidPhase = AVOID_BACKUP;
    avoidPhaseStart = millis();
    avoidPhaseDuration = critical ? AVOID_BACKUP_CRIT_MS : AVOID_BACKUP_MS;
    targetSpeed = SPEED_CRUISE;
    stallStartTime = 0;
    Serial.printf("[AVOID] BACKUP %lums\n", avoidPhaseDuration);
}

void AutonomousNav::enterRecover() {
    // If terrain boost was active, this stuck means terrain was too tough
    if (terrainBoostActive) {
        terrainBoostStuck = true;
        terrainBoostActive = false;
        checkTerrainOutcome();
    }
    startRecovery();
}

void AutonomousNav::enterHazard() {
    currentState = NAV_HAZARD;
    stateStartTime = millis();
    lastStateChange = millis();
    onSteepIncline = true;
    inclineAttempts++;
    
    hazardPhase = HAZARD_BACKUP;
    hazardPhaseStart = millis();
    hazardPhaseDuration = 800;
    targetSpeed = SPEED_CRUISE;
    Serial.printf("[HAZARD] BACKUP 800ms (attempt %d)\n", inclineAttempts);
}

void AutonomousNav::enterAvoidFromPit() {
    // Only interrupt CRUISE — don't override active avoidance/recovery
    if (currentState != NAV_CRUISE) return;
    
    Serial.println("[PIT] Pit detected → AVOID backup");
    currentState = NAV_AVOID;
    stateStartTime = millis();
    lastStateChange = millis();
    avoidPhase = AVOID_BACKUP;
    avoidPhaseStart = millis();
    avoidPhaseDuration = PIT_BACKUP_MS;
    targetSpeed = SPEED_CRUISE;
    stallStartTime = 0;
}

void AutonomousNav::enterAvoidFromHill() {
    // Only interrupt CRUISE — don't override active avoidance/recovery
    if (currentState != NAV_CRUISE) return;
    
    // Cancel any active terrain boost — this is major terrain
    terrainBoostActive = false;
    
    Serial.println("[HILL] Steep hill detected → AVOID backup");
    currentState = NAV_AVOID;
    stateStartTime = millis();
    lastStateChange = millis();
    avoidPhase = AVOID_BACKUP;
    avoidPhaseStart = millis();
    avoidPhaseDuration = HILL_BACKUP_MS;
    targetSpeed = SPEED_CRUISE;
    stallStartTime = 0;
}

void AutonomousNav::enterTerrainBoost() {
    // Only activate during CRUISE
    if (currentState != NAV_CRUISE) return;
    
    terrainBoostActive = true;
    terrainBoostUntil = millis() + TERRAIN_BOOST_DURATION_MS;
    terrainBoostStartX = envMap.getRobotX();
    terrainBoostStartY = envMap.getRobotY();
    terrainBoostStuck = false;
    targetSpeed = SPEED_CRUISE;
    Serial.printf("[TERRAIN] Minor terrain → boost %dms at (%d,%d)\n",
                  TERRAIN_BOOST_DURATION_MS, terrainBoostStartX, terrainBoostStartY);
}

bool AutonomousNav::isTerrainBoostActive() {
    if (terrainBoostActive && millis() > terrainBoostUntil) {
        terrainBoostActive = false;
        checkTerrainOutcome();
    }
    return terrainBoostActive;
}

void AutonomousNav::checkTerrainOutcome() {
    // Called when terrain boost expires or rover gets stuck during boost
    int curX = envMap.getRobotX();
    int curY = envMap.getRobotY();
    int dx = curX - terrainBoostStartX;
    int dy = curY - terrainBoostStartY;
    float distMoved = sqrt(dx * dx + dy * dy) * CELL_SIZE_CM;
    
    // If rover barely moved during boost, terrain was too difficult
    bool failed = terrainBoostStuck || (distMoved < 10.0f); // < 10cm = failed
    
    if (failed) {
        // Mark the terrain cells around start as hazard (upgrade from traversable)
        for (int ox = -2; ox <= 2; ox++) {
            for (int oy = -2; oy <= 2; oy++) {
                int gx = terrainBoostStartX + ox;
                int gy = terrainBoostStartY + oy;
                if (gx >= 0 && gx < MAP_WIDTH && gy >= 0 && gy < MAP_HEIGHT) {
                    uint8_t val = envMap.getCell(gx, gy);
                    // Don't overwrite existing terrain markers
                    if (val != CELL_PIT && val != CELL_HILL && val != CELL_HAZARD) {
                        envMap.setCell(gx, gy, CELL_HAZARD);
                    }
                }
            }
        }
        Serial.printf("[TERRAIN] LEARN: traversal FAILED (moved %.0fcm) → marking hazard\n", distMoved);
    } else {
        Serial.printf("[TERRAIN] LEARN: traversal OK (moved %.0fcm)\n", distMoved);
    }
}

// ==================== TURN DECISIONS ====================

TurnDirection AutonomousNav::pickTurnDirection(float distance) {
    // Record obstacle at current heading
    if (imuAvailable) {
        int bin = getHeadingBin();
        obstacleMemory[bin]++;
        headingAtObstacle = heading;
        Serial.printf("[NAV] Obstacle at heading %.0f (bin %d, count %d)\n",
                      heading, bin, obstacleMemory[bin]);
    }
    
    TurnDirection bestDir;
    float sideDiff = sideDistLeft - sideDistRight;
    
    if (abs(sideDiff) > 5.0f) {
        // Clear difference — turn toward open space
        bestDir = (sideDistLeft > sideDistRight) ? TURN_LEFT : TURN_RIGHT;
        Serial.printf("[NAV] Turn %s (L:%.0f R:%.0f)\n",
                      bestDir == TURN_LEFT ? "LEFT" : "RIGHT", sideDistLeft, sideDistRight);
    } else {
        // Sides equal — bias away from start if far enough
        float distFromStart = sqrt(drX * drX + drY * drY);
        if (distFromStart > 30.0f) {
            float awayBearing = atan2(drX, drY) * 180.0f / PI;
            float angleDiff = awayBearing - heading;
            while (angleDiff > 180) angleDiff -= 360;
            while (angleDiff < -180) angleDiff += 360;
            bestDir = (angleDiff > 0) ? TURN_RIGHT : TURN_LEFT;
            Serial.printf("[NAV] Sides equal — bias away from start (dist:%.0f)\n", distFromStart);
        } else {
            bestDir = getBestTurnDirection();
        }
    }
    
    // Track consecutive same-direction turns
    if (bestDir == lastTurnDirection) {
        consecutiveSameDirection++;
    } else {
        consecutiveSameDirection = 1;
    }
    lastTurnDirection = bestDir;
    turnStartHeading = heading;
    
    return bestDir;
}

int AutonomousNav::getHeadingBin() {
    return (int)((heading + 22.5f) / 45.0f) % 8;
}

TurnDirection AutonomousNav::getBestTurnDirection() {
    if (!imuAvailable) {
        if (lastTurnDirection != TURN_RANDOM && consecutiveSameDirection < 3) {
            return lastTurnDirection;
        }
        return (random(2) == 0) ? TURN_LEFT : TURN_RIGHT;
    }
    
    int leftScore = 0, rightScore = 0;
    int currentBin = getHeadingBin();
    
    // Obstacle memory — prefer direction with fewer remembered obstacles
    int leftObs = obstacleMemory[(currentBin + 6) % 8] + obstacleMemory[(currentBin + 7) % 8];
    int rightObs = obstacleMemory[(currentBin + 1) % 8] + obstacleMemory[(currentBin + 2) % 8];
    leftScore -= leftObs * 10;
    rightScore -= rightObs * 10;
    
    // Tilt compensation — prefer turning uphill
    if (abs(roll) > 10) {
        if (roll < 0) rightScore += 15;
        else leftScore += 15;
    }
    
    // Anti-circle — if turned same way 3+ times, try opposite
    if (consecutiveSameDirection >= 3) {
        if (lastTurnDirection == TURN_LEFT) rightScore += 25;
        else if (lastTurnDirection == TURN_RIGHT) leftScore += 25;
    }
    
    // Consistency — slight preference to continue same direction
    if (consecutiveSameDirection < 3) {
        if (lastTurnDirection == TURN_LEFT) leftScore += 5;
        else if (lastTurnDirection == TURN_RIGHT) rightScore += 5;
    }
    
    // Small random factor
    leftScore += random(0, 8);
    rightScore += random(0, 8);
    
    Serial.printf("[NAV] Turn scores: L=%d R=%d (obs L:%d R:%d, roll:%.0f)\n",
                  leftScore, rightScore, leftObs, rightObs, roll);
    
    return (leftScore >= rightScore) ? TURN_LEFT : TURN_RIGHT;
}

// ==================== MOTOR OUTPUT ====================

void AutonomousNav::getMotorSpeeds(int& leftSpeed, int& rightSpeed) {
    int speed = currentSpeed;
    
    // Minimum to move
    if (speed > 0 && speed < SPEED_MIN_MOVE) speed = SPEED_MIN_MOVE;
    
    // Slope compensation
    if (imuAvailable && currentOrientation == ORIENTATION_NORMAL) {
        if (pitch < -15) speed = min(255, (int)(speed * 1.15f));       // Uphill boost
        else if (pitch > 15) speed = max(80, (int)(speed * 0.75f));    // Downhill reduce
    }
    
    switch (currentState) {
        case NAV_CRUISE: {
            // Lock heading after settle delay
            if (!headingLocked && imuAvailable && (millis() - cruiseEnteredAt > HEADING_HOLD_SETTLE_MS)) {
                cruiseHeading = heading;
                headingLocked = true;
                Serial.printf("[HEADING] Locked at %.1f°\n", cruiseHeading);
            }
            
            // Heading-hold: correct drift from locked heading
            int holdBias = 0;
            if (headingLocked && imuAvailable) {
                float error = cruiseHeading - heading;
                while (error > 180) error -= 360;
                while (error < -180) error += 360;
                
                if (abs(error) > HEADING_HOLD_DEADBAND) {
                    holdBias = constrain((int)(error * HEADING_HOLD_GAIN), -HEADING_HOLD_MAX, HEADING_HOLD_MAX);
                }
            }
            
            // DR steering: gently arc away from start (additive with heading-hold)
            int drBias = 0;
            float distFromStart = sqrt(drX * drX + drY * drY);
            if (distFromStart > DR_MIN_DIST_CM && lastDistance > DIST_MEDIUM) {
                float awayBearing = atan2(drX, drY) * 180.0f / PI;
                float headingError = awayBearing - heading;
                while (headingError > 180) headingError -= 360;
                while (headingError < -180) headingError += 360;
                float proxScale = constrain(1.0f - (distFromStart - DR_MIN_DIST_CM) / DR_PROXIMITY_RANGE_CM, 0.2f, 1.0f);
                drBias = constrain((int)(headingError * DR_STEER_GAIN * proxScale), -DR_STEER_MAX, DR_STEER_MAX);
                
                // DR correction also shifts the locked heading so heading-hold
                // doesn't fight the DR correction
                if (abs(drBias) > 5 && headingLocked) {
                    cruiseHeading += drBias * 0.02f;  // Gently shift target
                    while (cruiseHeading >= 360) cruiseHeading -= 360;
                    while (cruiseHeading < 0) cruiseHeading += 360;
                }
            }
            
            int totalBias = constrain(holdBias + drBias, -HEADING_HOLD_MAX, HEADING_HOLD_MAX);
            
            // Diagonal hill traversal: when terrain boost is active and pitch
            // is significant, steer to cross the slope at an angle rather than
            // straight up — reduces effective grade significantly.
            // Uses roll to pick the direction (steer toward the downhill side).
            if (terrainBoostActive && imuAvailable) {
                float absPitch = fabs(pitch);
                if (absPitch > 8.0f) {  // Only steer when pitch is noticeable
                    // Roll indicates which side is lower — steer toward it
                    // to create a diagonal traversal path
                    int diagBias = (int)(roll * 0.8f);  // ~0.8 PWM per degree of roll
                    diagBias = constrain(diagBias, -25, 25);
                    totalBias += diagBias;
                    totalBias = constrain(totalBias, -HEADING_HOLD_MAX, HEADING_HOLD_MAX);
                }
            }
            
            leftSpeed  = constrain(speed + totalBias, (int)SPEED_MIN_MOVE, 255);
            rightSpeed = constrain(speed - totalBias, (int)SPEED_MIN_MOVE, 255);
            break;
        }
        
        case NAV_AVOID:
            switch (avoidPhase) {
                case AVOID_BACKUP:
                    leftSpeed = -speed;
                    rightSpeed = -speed;
                    break;
                case AVOID_TURN:
                    if (avoidTurnDir == TURN_LEFT) {
                        leftSpeed = -speed; rightSpeed = speed;
                    } else {
                        leftSpeed = speed; rightSpeed = -speed;
                    }
                    break;
                case AVOID_VERIFY:
                    leftSpeed = speed;
                    rightSpeed = speed;
                    break;
            }
            break;
        
        case NAV_RECOVER:
            switch (recoveryPhase) {
                case RECOVERY_ROCK_FWD:
                case RECOVERY_DIAGONAL_FWD:
                    leftSpeed = speed; rightSpeed = speed;
                    break;
                case RECOVERY_COAST:
                    leftSpeed = 0; rightSpeed = 0;
                    break;
                case RECOVERY_ROCK_REV:
                case RECOVERY_FULL_REVERSE:
                    leftSpeed = -speed; rightSpeed = -speed;
                    break;
                case RECOVERY_DIAGONAL_TURN:
                    if (recoveryTurnDir == TURN_LEFT) {
                        leftSpeed = -speed; rightSpeed = speed;
                    } else {
                        leftSpeed = speed; rightSpeed = -speed;
                    }
                    break;
                default:
                    leftSpeed = 0; rightSpeed = 0;
                    break;
            }
            break;
        
        case NAV_HAZARD:
            switch (hazardPhase) {
                case HAZARD_BACKUP:
                    leftSpeed = -speed; rightSpeed = -speed;
                    break;
                case HAZARD_TURN:
                    if (inclineAttempts % 2 == 0) {
                        leftSpeed = speed; rightSpeed = -speed;
                    } else {
                        leftSpeed = -speed; rightSpeed = speed;
                    }
                    break;
            }
            break;
        
        case NAV_STOPPED:
        default:
            leftSpeed = 0; rightSpeed = 0;
            break;
    }
}

// ==================== RESET ====================

void AutonomousNav::reset() {
    currentState = NAV_STOPPED;
    stateStartTime = millis();
    lastStateChange = 0;
    lastTurnDirection = TURN_RANDOM;
    
    lastDistance = 0;
    stuckCounter = 0;
    currentSpeed = 0;
    targetSpeed = 0;
    stallStartTime = 0;
    
    sideDistLeft = 999.0f;
    sideDistRight = 999.0f;
    detectedWallAngle = 0.0f;
    
    heading = 0;
    headingAtObstacle = 0;
    consecutiveSameDirection = 0;
    drX = 0; drY = 0;
    headingTowardStartSince = 0;
    cruiseHeading = 0;
    headingLocked = false;
    cruiseEnteredAt = 0;
    for (int i = 0; i < 8; i++) obstacleMemory[i] = 0;
    
    firstRun = true;
    
    avoidPhase = AVOID_BACKUP;
    avoidAttempts = 0;
    
    steepInclineStart = 0;
    onSteepIncline = false;
    inclineAttempts = 0;
    
    for (int i = 0; i < MOTION_VERIFY_WINDOW; i++) accelMagHistory[i] = 1.0f;
    accelMagIndex = 0;
    accelMagFilled = false;
    motionVerified = true;
    noMotionStartTime = 0;
    
    recoveryPhase = RECOVERY_DONE;
    rockCount = 0;
    recentlyStuck = false;
    stuckRecoveryTime = 0;
    recoveryCooldownUntil = 0;
    
    terrainBoostActive = false;
    terrainBoostUntil = 0;
    terrainBoostStartX = 0;
    terrainBoostStartY = 0;
    terrainBoostStuck = false;
    
    for (int i = 0; i < 5; i++) distanceHistory[i] = 999.0f;
    historyIndex = 0;
    
    Serial.println("[NAV] Reset — ready to cruise");
}

// ==================== TERRAIN HANDLING ====================

void AutonomousNav::checkMotionVerification() {
    float sum = 0;
    for (int i = 0; i < MOTION_VERIFY_WINDOW; i++) sum += accelMagHistory[i];
    float mean = sum / MOTION_VERIFY_WINDOW;
    
    float variance = 0;
    for (int i = 0; i < MOTION_VERIFY_WINDOW; i++) {
        float diff = accelMagHistory[i] - mean;
        variance += diff * diff;
    }
    variance /= MOTION_VERIFY_WINDOW;
    
    bool wasVerified = motionVerified;
    
    if (currentSpeed < SPEED_MIN_MOVE) {
        motionVerified = true;
        noMotionStartTime = 0;
    } else if (variance > MOTION_ACCEL_VAR_THRESH) {
        motionVerified = true;
        noMotionStartTime = 0;
    } else {
        motionVerified = false;
        if (noMotionStartTime == 0) noMotionStartTime = millis();
    }
    
    if (wasVerified && !motionVerified) {
        Serial.printf("[TERRAIN] Motion lost! var=%.6f\n", variance);
    } else if (!wasVerified && motionVerified) {
        Serial.println("[TERRAIN] Motion restored");
        noMotionStartTime = 0;
    }
}

void AutonomousNav::startRecovery() {
    Serial.println("[RECOVER] === Starting recovery ===");
    currentState = NAV_RECOVER;
    stateStartTime = millis();
    lastStateChange = millis();
    recoveryPhase = RECOVERY_ROCK_FWD;
    rockCount = 0;
    recoveryStepStart = millis();
    recoveryStepDuration = RECOVERY_ROCK_MS;
    targetSpeed = RECOVERY_SPEED;
    
    if (sideDistLeft > sideDistRight + 5.0f) recoveryTurnDir = TURN_LEFT;
    else if (sideDistRight > sideDistLeft + 5.0f) recoveryTurnDir = TURN_RIGHT;
    else recoveryTurnDir = (random(2) == 0) ? TURN_LEFT : TURN_RIGHT;
    
    Serial.printf("[RECOVER] ROCK_FWD, diag dir: %s\n",
                  recoveryTurnDir == TURN_LEFT ? "LEFT" : "RIGHT");
}

void AutonomousNav::flagRecentlyStuck() {
    recentlyStuck = true;
    stuckRecoveryTime = millis();
    recoveryCooldownUntil = millis() + RECOVERY_COOLDOWN_MS;
    noMotionStartTime = 0;
    motionVerified = true;
    stuckCounter = 0;
    stallStartTime = 0;
    Serial.printf("[TERRAIN] Recovery cooldown %dms\n", RECOVERY_COOLDOWN_MS);
}

// ==================== HELPERS ====================

bool AutonomousNav::isStuck() {
    if (currentState != NAV_CRUISE) {
        stuckCounter = 0;
        return false;
    }
    
    float minD = distanceHistory[0], maxD = distanceHistory[0];
    for (int i = 1; i < 5; i++) {
        if (distanceHistory[i] < minD) minD = distanceHistory[i];
        if (distanceHistory[i] > maxD) maxD = distanceHistory[i];
    }
    
    if ((maxD - minD) < STUCK_DISTANCE_TOL && lastDistance < DIST_MEDIUM) {
        stuckCounter++;
        return stuckCounter >= STUCK_COUNT_THRESHOLD;
    } else {
        stuckCounter = 0;
    }
    return false;
}

void AutonomousNav::updateDistanceHistory(float distance) {
    distanceHistory[historyIndex] = distance;
    historyIndex = (historyIndex + 1) % 5;
}

int AutonomousNav::calculateSpeed(float distance) {
    // During terrain boost, maintain full cruise speed regardless of distance
    // (slope face can look like a close obstacle to ultrasonic)
    if (terrainBoostActive) {
        if (millis() > terrainBoostUntil) {
            terrainBoostActive = false;
        } else {
            return SPEED_CRUISE;
        }
    }
    if (distance >= DIST_FAR) return SPEED_CRUISE;
    if (distance > DIST_CLOSE) {
        float ratio = (distance - DIST_CLOSE) / (DIST_FAR - DIST_CLOSE);
        return SPEED_MIN_MOVE + (int)(ratio * (SPEED_CRUISE - SPEED_MIN_MOVE));
    }
    return SPEED_MIN_MOVE;
}

void AutonomousNav::smoothSpeed() {
    currentSpeed = constrain(targetSpeed, 0, SPEED_MAX);
}
