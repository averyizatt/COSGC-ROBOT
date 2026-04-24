/*
 * autonomous_modes.cpp
 * ====================
 * Implementation of all robot operating modes extracted from COSGC-ROBOT TankESP32.
 * See autonomous_modes.h for full documentation and usage instructions.
 */

#include "autonomous_modes.h"

// ===================================================================
// Internal speed aliases (relative to AM_MAX_PWM)
// ===================================================================
static const int ADV_SPEED_MAX    = AM_MAX_PWM;
static const int ADV_SPEED_CRUISE = AM_MAX_PWM;
static const int ADV_SPEED_TURN   = AM_MAX_PWM;
static const int ADV_SPEED_MIN    = (AM_MAX_PWM * 80 / 100);

// ===================================================================
// RobotModes — constructor and dispatcher
// ===================================================================

RobotModes::RobotModes()
    : _mode(MODE_RC_CONTROL), _motorFn(nullptr), _motorL(0), _motorR(0), _s(nullptr)
{
    resetSimpleAutoState();
    resetWallFollowState();
}

void RobotModes::setMotorCallback(void (*fn)(int left, int right)) {
    _motorFn = fn;
}

void RobotModes::setMode(RobotMode mode) {
    _mode = mode;
}

void RobotModes::getMotorSpeeds(int& left, int& right) const {
    left  = _motorL;
    right = _motorR;
}

void RobotModes::setMotors(int left, int right) {
    _motorL = left;
    _motorR = right;
    if (_motorFn) _motorFn(left, right);
}

void RobotModes::update(const SensorInput& sensors) {
    _s = &sensors;
    switch (_mode) {
        case MODE_RC_CONTROL:   runRC();           break;
        case MODE_UART_CONTROL: runUART();         break;
        case MODE_SOK_CONTROL:  runSOK();          break;
        case MODE_AUTONOMOUS:   runAdvancedAuto(); break;
        case MODE_SIMPLE_AUTO:  runSimpleAuto();   break;
        case MODE_WALL_FOLLOW:  runWallFollow();   break;
        case MODE_PREMAP_NAV:   runAdvancedAuto(); break; // same engine, map pre-seeded externally
        default: break;
    }
    _s = nullptr;
}

// ===================================================================
// MODE 0 — RC CONTROL
// Tank-drive mixing: left stick Y = throttle, right stick X = steering.
// Input range: -512..512 (maps to -255..255 PWM).
// ===================================================================
void RobotModes::runRC() {
    if (!_s->rcConnected) {
        setMotors(0, 0);
        return;
    }

    // Apply a simple deadzone (~10 % of range)
    auto applyDeadzone = [](int v) -> int {
        const int DZ = 50;
        if (v > -DZ && v < DZ) return 0;
        return v;
    };

    int throttle = applyDeadzone(_s->rcThrottle);
    int steering = applyDeadzone(_s->rcSteering);

    // Scale -512..512 → -255..255
    int base = (throttle * 255) / 512;
    int turn = (steering * 255) / 512;

    int left  = AM_CONSTRAIN(base + turn, -255, 255);
    int right = AM_CONSTRAIN(base - turn, -255, 255);
    setMotors(left, right);
}

// ===================================================================
// MODE 1 — UART CONTROL
// Simple full-speed forward. Your UART handler sets motor commands
// directly; this fallback keeps driving forward if nothing intervenes.
// Replace the body with your own UART command processing if needed.
// ===================================================================
void RobotModes::runUART() {
    setMotors(255, 255);
}

// ===================================================================
// MODE 2 — SOK (SOCKET) CONTROL
// Direct pass-through of socket-provided motor values.
// ===================================================================
void RobotModes::runSOK() {
    setMotors(_s->sokLeft, _s->sokRight);
}

// ===================================================================
// MODE 3 / 6 — ADVANCED AUTONOMOUS  (also PREMAP_NAV)
// Feeds sensor data into AdvNavCore state machine and applies output.
// Includes pit / hill / terrain detection and traction control logic.
// ===================================================================
void RobotModes::runAdvancedAuto() {
    const SensorInput& s = *_s;
    float dist = s.distFront;

    // Emergency: if critically close and not already avoiding, force AVOID.
    if (dist > 2.0f && dist < AM_DIST_CRITICAL) {
        AdvNavState ns = _nav.getState();
        if (ns != ADV_AVOID && ns != ADV_RECOVER) {
            AM_PRINT("[AUTO] EMERGENCY STOP %.0fcm\n", dist);
            _nav.enterAvoid(true);
        }
    }

    // Feed IMU
#if AM_IMU_ENABLED
    _nav.updateIMU(s.accelX, s.accelY, s.accelZ, s.gyroX, s.gyroY, s.gyroZ);
#endif

    // Feed sensor data
    _nav.setSideDistances(s.distLeft, s.distRight);
    _nav.setWallAngle(s.wallAngle);
    _nav.setEncoderSpeed(s.encoderSpeedCmS);
    _nav.setOdometry(s.odomX, s.odomY, s.odomTheta);

    // --- Pit detection ---
    {
        static float prevL = 0.0f, prevR = 0.0f;
        static unsigned long pitPitchStart = 0;
        static unsigned long pitCooldownUntil = 0;
        unsigned long now = AM_MILLIS();

        bool pitDetected = false;
        bool pitMajor    = false;
        float pitJump    = 0.0f;

        if (now >= pitCooldownUntil) {
            if (prevL > 2.0f && prevL < 100.0f && s.distLeft > prevL + AM_PIT_DIST_JUMP_CM) {
                float j = s.distLeft - prevL;
                pitDetected = true;
                if (j > pitJump) pitJump = j;
                AM_PRINT("[PIT] Left jump: %.0f->%.0fcm\n", prevL, s.distLeft);
            }
            if (prevR > 2.0f && prevR < 100.0f && s.distRight > prevR + AM_PIT_DIST_JUMP_CM) {
                float j = s.distRight - prevR;
                pitDetected = true;
                if (j > pitJump) pitJump = j;
                AM_PRINT("[PIT] Right jump: %.0f->%.0fcm\n", prevR, s.distRight);
            }
#if AM_IMU_ENABLED
            float navPitch = _nav.getPitch();
            if (navPitch < AM_PIT_PITCH_THRESHOLD) {
                if (pitPitchStart == 0) pitPitchStart = now;
                else if (now - pitPitchStart > AM_PIT_PITCH_SUSTAIN_MS) {
                    pitDetected = true;
                    AM_PRINT("[PIT] Sustained negative pitch %.1f\n", navPitch);
                    pitPitchStart = 0;
                }
            } else {
                pitPitchStart = 0;
            }
            if (pitDetected) {
                float absPitch = AM_ABS(_nav.getPitch());
                pitMajor = (absPitch >= AM_TERRAIN_MINOR_PITCH) ||
                           (pitJump  >= AM_TERRAIN_MINOR_PIT_JUMP_CM);
            }
#else
            pitMajor = pitDetected;
#endif
        }

        prevL = s.distLeft;
        prevR = s.distRight;

        if (pitDetected && pitMajor) {
            pitCooldownUntil = AM_MILLIS() + 4000;
            _nav.enterAvoidFromPit();
            AM_PRINT("[PIT] MAJOR -> AVOID\n");
        } else if (pitDetected) {
            pitCooldownUntil = AM_MILLIS() + 2000;
            _nav.enterTerrainBoost();
            AM_PRINT("[PIT] MINOR -> TRAVERSE\n");
        }
    }

    // --- Hill detection ---
    {
        static float hPrevL = 0.0f, hPrevR = 0.0f;
        static unsigned long hillPitchStart = 0;
        static unsigned long hillCooldownUntil = 0;
        unsigned long now = AM_MILLIS();

        bool hillDetected = false;
        bool hillMajor    = false;
        float maxHeight   = 0.0f;

        if (now >= hillCooldownUntil) {
            if (hPrevL > 50.0f && hPrevL < 400.0f && s.distLeft < hPrevL - AM_HILL_DIST_DROP_CM && s.distLeft > 2.0f) {
                float pitchRad = _nav.getPitch() * AM_PI / 180.0f;
                float h = s.distLeft * sinf(fabsf(pitchRad)) + AM_SENSOR_HEIGHT_CM;
                hillDetected = true;
                if (h > maxHeight) maxHeight = h;
                AM_PRINT("[HILL] Left drop %.0f->%.0fcm h=%.0f\n", hPrevL, s.distLeft, h);
            }
            if (hPrevR > 50.0f && hPrevR < 400.0f && s.distRight < hPrevR - AM_HILL_DIST_DROP_CM && s.distRight > 2.0f) {
                float pitchRad = _nav.getPitch() * AM_PI / 180.0f;
                float h = s.distRight * sinf(fabsf(pitchRad)) + AM_SENSOR_HEIGHT_CM;
                hillDetected = true;
                if (h > maxHeight) maxHeight = h;
                AM_PRINT("[HILL] Right drop h=%.0f\n", h);
            }
#if AM_IMU_ENABLED
            float navPitch = _nav.getPitch();
            if (navPitch > AM_HILL_PITCH_THRESHOLD) {
                if (hillPitchStart == 0) hillPitchStart = now;
                else if (now - hillPitchStart > AM_HILL_PITCH_SUSTAIN_MS) {
                    float pitchRad = navPitch * AM_PI / 180.0f;
                    float h = dist * sinf(pitchRad) + AM_SENSOR_HEIGHT_CM;
                    hillDetected = true;
                    if (h > maxHeight) maxHeight = h;
                    AM_PRINT("[HILL] Pitch %.1f h=%.0f\n", navPitch, h);
                    hillPitchStart = 0;
                }
            } else {
                hillPitchStart = 0;
            }
            if (hillDetected) {
                float absPitch = AM_ABS(_nav.getPitch());
                hillMajor = (absPitch >= AM_TERRAIN_MINOR_PITCH) ||
                            (maxHeight >= AM_TERRAIN_MINOR_HEIGHT_CM);
            }
#else
            hillMajor = hillDetected;
#endif
        }

        hPrevL = s.distLeft;
        hPrevR = s.distRight;

        if (hillDetected && hillMajor) {
            hillCooldownUntil = AM_MILLIS() + 4000;
            _nav.enterAvoidFromHill();
            AM_PRINT("[HILL] MAJOR -> AVOID\n");
        } else if (hillDetected) {
            hillCooldownUntil = AM_MILLIS() + 2000;
            _nav.enterTerrainBoost();
            AM_PRINT("[HILL] MINOR -> TRAVERSE\n");
        }
    }

    // Update navigation
    _nav.update(dist);

    // Get motor output
    int leftSpeed, rightSpeed;
    _nav.getMotorSpeeds(leftSpeed, rightSpeed);
    setMotors(leftSpeed, rightSpeed);

    // Debug
    static unsigned long lastDbg = 0;
    unsigned long now = AM_MILLIS();
    if (now - lastDbg > 2000) {
        lastDbg = now;
        AM_PRINT("[AUTO] %s | Fwd:%.0f L:%.0f R:%.0f | ML:%d MR:%d%s%s\n",
                 _nav.getStateString(), dist, s.distLeft, s.distRight,
                 leftSpeed, rightSpeed,
                 _nav.isMotionVerified() ? "" : " NO-MOTION",
                 _nav.isRecentlyStuck()  ? " SOFT-RAMP" : "");
    }
}

// ===================================================================
// MODE 4 — SIMPLE AUTONOMOUS
// PID-based forward with obstacle avoidance phases + full recovery.
// Includes heading-hold, dead reckoning, terrain detection, anti-spin.
// ===================================================================

void RobotModes::resetSimpleAutoState() {
    SimpleAutoState& st = _sa;
    memset(&st, 0, sizeof(st));
    st.phase         = 0;
    st.motionOK      = true;
    st.lastDist      = 999.0f;
    st.prevDistL     = 999.0f;
    st.prevDistR     = 999.0f;
    st.hillPrevDistL = 999.0f;
    st.hillPrevDistR = 999.0f;
    for (int i = 0; i < AM_MOTION_VERIFY_WINDOW; i++) st.accelMagBuf[i] = 1.0f;
}

void RobotModes::runSimpleAuto() {
    SimpleAutoState& st = _sa;
    const SensorInput& s = *_s;

    // PID gains
    const float KP = 5.0f, KI = 0.1f, KD = 1.5f, I_MAX = 2000.0f;

    const float OBSTACLE_CM   = 25.0f;
    const float CRITICAL_CM   = 10.0f;
    const float STALL_DIST_CM = 50.0f;
    const int   TURN_SPEED    = (AM_MAX_PWM * 90) / 100;
    const int   REVERSE_SPEED = AM_MAX_PWM;
    const int   MIN_SPEED     = (AM_MAX_PWM * 80) / 100;
    const unsigned long TURN_MS    = 400;
    const unsigned long REVERSE_MS = 300;
    const float TILT_THRESHOLD = 35.0f;
    const int   TILT_DEBOUNCE  = 3;
    const unsigned long STALL_MS = 4000;

    float distance = s.distFront;
    float distL    = s.distLeft;
    float distR    = s.distRight;
    unsigned long now = AM_MILLIS();

    if (distance < 0 || distance > 400) distance = OBSTACLE_CM - 1.0f;
    if (distL < 0 || distL > 400) distL = 999.0f;
    if (distR < 0 || distR > 400) distR = 999.0f;

    float pitch = 0.0f;

#if AM_IMU_ENABLED
    float ax = s.accelX, ay = s.accelY, az = s.accelZ;
    pitch = atan2f(ax, sqrtf(ay*ay + az*az)) * 180.0f / AM_PI;

    // Dead reckoning
    {
        float gz = s.gyroZ;
        float dt = 0.05f;
        st.drHeading += gz * dt;
        // Use encoder speed if available, otherwise skip position update
        if (s.encoderSpeedCmS > 0.5f && st.phase == 0) {
            float distCm = s.encoderSpeedCmS * dt;
            float headRad = st.drHeading * AM_PI / 180.0f;
            st.drX += distCm * sinf(headRad);
            st.drY += distCm * cosf(headRad);
        }
    }

    // IMU motion verification
    float accelMag = sqrtf(ax*ax + ay*ay + az*az);
    st.accelMagBuf[st.accelMagIdx] = accelMag;
    st.accelMagIdx = (st.accelMagIdx + 1) % AM_MOTION_VERIFY_WINDOW;
    if (st.accelMagIdx == 0) st.accelMagReady = true;
    if (st.accelMagReady) {
        float sum = 0;
        for (int i = 0; i < AM_MOTION_VERIFY_WINDOW; i++) sum += st.accelMagBuf[i];
        float mean = sum / AM_MOTION_VERIFY_WINDOW;
        float var  = 0;
        for (int i = 0; i < AM_MOTION_VERIFY_WINDOW; i++) {
            float d = st.accelMagBuf[i] - mean; var += d*d;
        }
        var /= AM_MOTION_VERIFY_WINDOW;
        st.motionOK = (var > AM_MOTION_ACCEL_VAR_THRESH);
    }
#endif

    // Tilt debounce
    if (pitch > TILT_THRESHOLD) st.tiltCount++;
    else                        st.tiltCount = 0;

    // Adaptive ramp timeout
    if (st.recentlyStuck && (now - st.stuckTime > AM_ADAPTIVE_RAMP_TIMEOUT)) {
        st.recentlyStuck = false;
        AM_PRINT("[SIMPLE] Adaptive ramp expired\n");
    }

    int leftSpeed = 0, rightSpeed = 0;

    // === PHASE 3: RECOVERY ===
    if (st.phase == 3) {
        if (now < st.recoveryUntil) {
            switch (st.recoveryStep) {
                case 0: leftSpeed =  AM_RECOVERY_SPEED; rightSpeed =  AM_RECOVERY_SPEED; break;
                case 1: case 3: leftSpeed = 0; rightSpeed = 0; break;
                case 2: leftSpeed = -AM_RECOVERY_SPEED; rightSpeed = -AM_RECOVERY_SPEED; break;
                case 4:
                    if (st.recoveryTurnDir == 1) { leftSpeed = -AM_RECOVERY_SPEED; rightSpeed =  AM_RECOVERY_SPEED; }
                    else                         { leftSpeed =  AM_RECOVERY_SPEED; rightSpeed = -AM_RECOVERY_SPEED; }
                    break;
                case 5: leftSpeed =  REVERSE_SPEED; rightSpeed =  REVERSE_SPEED; break;
                case 6: leftSpeed = -AM_RECOVERY_SPEED; rightSpeed = -AM_RECOVERY_SPEED; break;
            }
        } else {
            switch (st.recoveryStep) {
                case 0:
                    st.recoveryStep = 1;
                    st.recoveryUntil = now + AM_RECOVERY_COAST_MS;
                    leftSpeed = 0; rightSpeed = 0;
                    break;
                case 1:
                    st.recoveryStep = 2;
                    st.recoveryUntil = now + AM_RECOVERY_ROCK_MS;
                    AM_PRINT("[SIMPLE-REC] Rock REV (%d/%d)\n", st.rockCycles+1, AM_RECOVERY_ROCK_ATTEMPTS);
                    leftSpeed = -AM_RECOVERY_SPEED; rightSpeed = -AM_RECOVERY_SPEED;
                    break;
                case 2:
                    st.recoveryStep = 3;
                    st.recoveryUntil = now + AM_RECOVERY_COAST_MS;
                    leftSpeed = 0; rightSpeed = 0;
                    break;
                case 3:
                    st.rockCycles++;
                    if (st.rockCycles < AM_RECOVERY_ROCK_ATTEMPTS) {
                        st.recoveryStep = 0;
                        st.recoveryUntil = now + AM_RECOVERY_ROCK_MS;
                        AM_PRINT("[SIMPLE-REC] Rock FWD (%d/%d)\n", st.rockCycles+1, AM_RECOVERY_ROCK_ATTEMPTS);
                        leftSpeed = AM_RECOVERY_SPEED; rightSpeed = AM_RECOVERY_SPEED;
                    } else {
                        st.recoveryStep = 4;
                        st.recoveryUntil = now + AM_RECOVERY_DIAG_TURN_MS;
                        AM_PRINT("[SIMPLE-REC] Diagonal TURN\n");
                        if (st.recoveryTurnDir == 1) { leftSpeed = -AM_RECOVERY_SPEED; rightSpeed =  AM_RECOVERY_SPEED; }
                        else                         { leftSpeed =  AM_RECOVERY_SPEED; rightSpeed = -AM_RECOVERY_SPEED; }
                    }
                    break;
                case 4:
                    st.recoveryStep = 5;
                    st.recoveryUntil = now + AM_RECOVERY_DIAG_FWD_MS;
                    AM_PRINT("[SIMPLE-REC] Diagonal FWD\n");
                    leftSpeed = REVERSE_SPEED; rightSpeed = REVERSE_SPEED;
                    break;
                case 5:
                    if (st.motionOK && distance > OBSTACLE_CM) {
                        AM_PRINT("[SIMPLE-REC] === FREE! ===\n");
                        st.phase = 0; st.pidI = 0; st.pidLastErr = 0;
                        st.headingLocked = false; st.phase0EnteredAt = now;
                        st.recentlyStuck = true; st.stuckTime = now;
                        st.noMotionStart = 0;
                        st.recoveryCooldownUntil = now + AM_RECOVERY_COOLDOWN_MS;
                        leftSpeed = MIN_SPEED; rightSpeed = MIN_SPEED;
                    } else {
                        st.recoveryStep = 6;
                        st.recoveryUntil = now + AM_RECOVERY_FULL_REV_MS;
                        AM_PRINT("[SIMPLE-REC] FULL REVERSE\n");
                        leftSpeed = -AM_RECOVERY_SPEED; rightSpeed = -AM_RECOVERY_SPEED;
                    }
                    break;
                case 6:
                    AM_PRINT("[SIMPLE-REC] === Recovery complete ===\n");
                    st.phase = 1;
                    st.actionUntil = now + 800;
                    st.pidI = 0;
                    st.recentlyStuck = true; st.stuckTime = now;
                    st.noMotionStart = 0;
                    st.recoveryCooldownUntil = now + AM_RECOVERY_COOLDOWN_MS;
                    leftSpeed = -TURN_SPEED; rightSpeed = TURN_SPEED;
                    break;
            }
        }
        setMotors(leftSpeed, rightSpeed);

        static unsigned long lastRecDbg = 0;
        if (now - lastRecDbg > 2000) {
            const char* stepNames[] = {"ROCK_FWD","COAST","ROCK_REV","COAST","DIAG_TURN","DIAG_FWD","FULL_REV"};
            AM_PRINT("[SIMPLE] RECOVERY %s | Fwd:%.0f | ML:%d MR:%d\n",
                     stepNames[st.recoveryStep], distance, leftSpeed, rightSpeed);
            lastRecDbg = now;
        }
        return;
    }

    // === SEND-IT MODE ===
    if (st.sendItActive) {
        if (now >= st.sendItUntil) {
            st.sendItActive = false;
            st.sendItTurnCount = 0;
            st.sendItWindowStart = 0;
            st.sendItFwdStart = 0;
            st.phase = 0; st.pidI = 0; st.pidLastErr = 0;
            st.headingLocked = false; st.phase0EnteredAt = now;
            AM_PRINT("[SENDIT] === Send-it complete ===\n");
        } else if (distance < CRITICAL_CM) {
            leftSpeed = -REVERSE_SPEED; rightSpeed = -REVERSE_SPEED;
            setMotors(leftSpeed, rightSpeed);
            return;
        } else {
            int speed = st.recentlyStuck ? MIN_SPEED : REVERSE_SPEED;
            setMotors(speed, speed);
            return;
        }
    }

    // Check if timed action expired
    if (st.phase != 0 && now >= st.actionUntil) {
        st.phase = 0; st.pidI = 0; st.pidLastErr = 0;
        st.lastDecision = now;
        st.headingLocked = false;
        st.phase0EnteredAt = now;
        st.accelMagReady = false; st.accelMagIdx = 0;
        st.noMotionStart = 0;
        leftSpeed = MIN_SPEED; rightSpeed = MIN_SPEED;
        setMotors(leftSpeed, rightSpeed);
        return;
    }

    // Decision cooldown
    if (st.phase == 0 && (now - st.lastDecision) < AM_DECISION_COOLDOWN_MS && distance > CRITICAL_CM) {
        setMotors(_motorL, _motorR);
        return;
    }

    if (st.phase == 0) {
        // Anti-spin: track forward progress
        if (_motorL > 0 && _motorR > 0) {
            if (st.sendItFwdStart == 0) st.sendItFwdStart = now;
            if (now - st.sendItFwdStart > AM_SENDIT_FWD_PROGRESS_MS) {
                if (st.sendItTurnCount > 0) {
                    AM_PRINT("[SENDIT] Progress — clearing %d turns\n", st.sendItTurnCount);
                }
                st.sendItTurnCount = 0;
                st.sendItWindowStart = 0;
            }
        } else {
            st.sendItFwdStart = 0;
        }
        if (st.sendItWindowStart > 0 && (now - st.sendItWindowStart > AM_SENDIT_WINDOW_MS)) {
            st.sendItTurnCount = 0;
            st.sendItWindowStart = 0;
        }

#if AM_IMU_ENABLED
        // IMU movement verification — require BOTH low variance AND distance not changing
        bool distChanging = (AM_ABS(distance - st.lastDist) > 3.0f);
        st.lastDist = distance;
        if (!st.motionOK && !distChanging && _motorL >= MIN_SPEED && now >= st.recoveryCooldownUntil) {
            if (st.noMotionStart == 0) {
                st.noMotionStart = now;
            } else if (now - st.noMotionStart > AM_MOTION_VERIFY_TIMEOUT) {
                AM_PRINT("[SIMPLE] No motion -> RECOVERY\n");
                st.phase = 3; st.recoveryStep = 0; st.rockCycles = 0;
                st.recoveryUntil = now + AM_RECOVERY_ROCK_MS;
                st.recoveryTurnDir = (distL > distR) ? 1 : 0;
                st.accelMagReady = false; st.accelMagIdx = 0;
                st.noMotionStart = 0;
                setMotors(AM_RECOVERY_SPEED, AM_RECOVERY_SPEED);
                return;
            }
        } else {
            st.noMotionStart = 0;
        }

        // Incline check
        float absPitch = AM_ABS(pitch);
        if (absPitch > AM_INCLINE_MAX_PITCH) {
            if (st.steepStart == 0) {
                st.steepStart = now;
                AM_PRINT("[SIMPLE] Steep incline %.1f\n", pitch);
            } else if (now - st.steepStart > AM_INCLINE_TIMEOUT_MS) {
                if (st.inclineAttempts < 2) {
                    st.inclineAttempts++;
                    AM_PRINT("[SIMPLE] Incline -> diagonal attempt #%d\n", st.inclineAttempts);
                    st.phase = 2; st.actionUntil = now + REVERSE_MS;
                    st.steepStart = 0;
                } else {
                    AM_PRINT("[SIMPLE] Incline UNCLIMBABLE -> RECOVERY\n");
                    st.phase = 3; st.recoveryStep = 0; st.rockCycles = 0;
                    st.recoveryUntil = now + AM_RECOVERY_ROCK_MS;
                    st.recoveryTurnDir = (st.inclineAttempts % 2 == 0) ? 1 : 0;
                    st.inclineAttempts = 0; st.steepStart = 0;
                    st.accelMagReady = false; st.accelMagIdx = 0;
                    st.noMotionStart = 0;
                    setMotors(AM_RECOVERY_SPEED, AM_RECOVERY_SPEED);
                    return;
                }
            }
        } else {
            if (st.steepStart > 0) AM_PRINT("[SIMPLE] Incline cleared\n");
            st.steepStart = 0;
            st.inclineAttempts = 0;
        }

        // Pit detection (simple mode)
        {
            bool pitDetected = false;
            float pitJump = 0.0f;
            if (st.prevDistL > 2.0f && st.prevDistL < 100.0f && distL > st.prevDistL + AM_PIT_DIST_JUMP_CM) {
                pitDetected = true;
                pitJump = distL - st.prevDistL;
                AM_PRINT("[SIMPLE-PIT] Left %.0f->%.0f\n", st.prevDistL, distL);
            }
            if (st.prevDistR > 2.0f && st.prevDistR < 100.0f && distR > st.prevDistR + AM_PIT_DIST_JUMP_CM) {
                pitDetected = true;
                float j = distR - st.prevDistR;
                if (j > pitJump) pitJump = j;
                AM_PRINT("[SIMPLE-PIT] Right %.0f->%.0f\n", st.prevDistR, distR);
            }
            st.prevDistL = distL; st.prevDistR = distR;

            if (pitch < AM_PIT_PITCH_THRESHOLD) {
                if (st.pitPitchStart == 0) st.pitPitchStart = now;
                else if (now - st.pitPitchStart > AM_PIT_PITCH_SUSTAIN_MS) {
                    pitDetected = true;
                    AM_PRINT("[SIMPLE-PIT] Negative pitch %.1f\n", pitch);
                    st.pitPitchStart = 0;
                }
            } else {
                st.pitPitchStart = 0;
            }

            bool pitMajor = pitDetected && ((AM_ABS(pitch) >= AM_TERRAIN_MINOR_PITCH) || (pitJump >= AM_TERRAIN_MINOR_PIT_JUMP_CM));
            if (pitDetected && pitMajor && st.phase == 0) {
                st.phase = 2; st.actionUntil = now + AM_PIT_BACKUP_MS;
                st.pidI = 0; st.lastDecision = now;
                AM_PRINT("[SIMPLE-PIT] MAJOR -> REVERSE\n");
            }
        }

        // Hill detection (simple mode)
        {
            bool hillDetected = false;
            float maxH = 0.0f;
            if (st.hillPrevDistL > 50.0f && st.hillPrevDistL < 400.0f && distL < st.hillPrevDistL - AM_HILL_DIST_DROP_CM && distL > 2.0f) {
                float pitchRad = pitch * AM_PI / 180.0f;
                float h = distL * sinf(fabsf(pitchRad)) + AM_SENSOR_HEIGHT_CM;
                hillDetected = true; if (h > maxH) maxH = h;
            }
            if (st.hillPrevDistR > 50.0f && st.hillPrevDistR < 400.0f && distR < st.hillPrevDistR - AM_HILL_DIST_DROP_CM && distR > 2.0f) {
                float pitchRad = pitch * AM_PI / 180.0f;
                float h = distR * sinf(fabsf(pitchRad)) + AM_SENSOR_HEIGHT_CM;
                hillDetected = true; if (h > maxH) maxH = h;
            }
            st.hillPrevDistL = distL; st.hillPrevDistR = distR;

            if (pitch > AM_HILL_PITCH_THRESHOLD) {
                if (st.hillPitchStart == 0) st.hillPitchStart = now;
                else if (now - st.hillPitchStart > AM_HILL_PITCH_SUSTAIN_MS) {
                    float pitchRad = pitch * AM_PI / 180.0f;
                    float h = distance * sinf(pitchRad) + AM_SENSOR_HEIGHT_CM;
                    hillDetected = true; if (h > maxH) maxH = h;
                    st.hillPitchStart = 0;
                }
            } else {
                st.hillPitchStart = 0;
            }

            bool hillMajor = hillDetected && ((AM_ABS(pitch) >= AM_TERRAIN_MINOR_PITCH) || (maxH >= AM_TERRAIN_MINOR_HEIGHT_CM));
            if (hillDetected && hillMajor && st.phase == 0) {
                st.phase = 2; st.actionUntil = now + AM_HILL_BACKUP_MS;
                st.pidI = 0; st.lastDecision = now;
                AM_PRINT("[SIMPLE-HILL] MAJOR height=%.0f -> REVERSE\n", maxH);
            }
        }

        // Proactive heading correction (dead reckoning)
        {
            float distFromStart = sqrtf(st.drX*st.drX + st.drY*st.drY);
            if (distFromStart > AM_DR_MIN_DIST_CM && st.phase == 0) {
                float toBearing = atan2f(-st.drX, -st.drY) * 180.0f / AM_PI;
                float toError = toBearing - st.drHeading;
                while (toError >  180) toError -= 360;
                while (toError < -180) toError += 360;
                if (AM_ABS(toError) < AM_DR_HEADING_TOWARD_DEG) {
                    if (st.headingTowardStart == 0) st.headingTowardStart = now;
                    else if (now - st.headingTowardStart > AM_DR_CORRECT_TIMEOUT_MS && distance > AM_DR_CORRECT_CLEARANCE_CM) {
                        st.phase = 1; st.actionUntil = now + AM_DR_CORRECT_TURN_MS;
                        st.pidI = 0; st.headingTowardStart = 0;
                        leftSpeed = -TURN_SPEED; rightSpeed = TURN_SPEED; // turn right
                        AM_PRINT("[DR] Heading toward start -> correction RIGHT\n");
                        setMotors(leftSpeed, rightSpeed);
                        return;
                    }
                } else {
                    st.headingTowardStart = 0;
                }
            } else {
                st.headingTowardStart = 0;
            }
        }
#endif // AM_IMU_ENABLED

        // Priority 1: tilt
        if (st.tiltCount >= TILT_DEBOUNCE) {
            st.phase = 2; st.actionUntil = now + REVERSE_MS;
            st.tiltCount = 0; st.pidI = 0; st.lastDecision = now;
            AM_PRINT("[SIMPLE] Tilt -> REVERSE\n");
        }
        // Priority 2: critical distance
        else if (distance < CRITICAL_CM) {
            st.phase = 2; st.actionUntil = now + REVERSE_MS;
            st.pidI = 0; st.lastDecision = now;
            AM_PRINT("[SIMPLE] Critical %.0fcm -> REVERSE\n", distance);
        }
        // Priority 3: close → turn
        else if (distance < OBSTACLE_CM) {
            // Anti-spin tracking
            if (st.sendItWindowStart == 0) st.sendItWindowStart = now;
            st.sendItTurnCount++;
            if (st.sendItTurnCount >= AM_SENDIT_TURN_THRESHOLD) {
                AM_PRINT("[SENDIT] === SEND IT! %d turns ===\n", st.sendItTurnCount);
                st.sendItActive = true;
                st.sendItUntil = now + AM_SENDIT_DURATION_MS;
                st.sendItTurnCount = 0;
                st.sendItWindowStart = 0;
                st.sendItFwdStart = 0;
                leftSpeed = REVERSE_SPEED; rightSpeed = REVERSE_SPEED;
                st.pidI = 0;
                setMotors(leftSpeed, rightSpeed);
                return;
            }

            st.phase = 1; st.actionUntil = now + TURN_MS;
            st.pidI = 0; st.lastDecision = now;

            // Turn direction: prefer side with more room; fall back to bias away from start
            bool turnRight;
            float sideDiff = distR - distL;
            if (AM_ABS(sideDiff) > 8.0f) {
                turnRight = (sideDiff > 0);
            } else {
                float distFromStart = sqrtf(st.drX*st.drX + st.drY*st.drY);
                if (distFromStart > 30.0f) {
                    float awayBearing = atan2f(st.drX, st.drY) * 180.0f / AM_PI;
                    float angleDiff = awayBearing - st.drHeading;
                    while (angleDiff >  180) angleDiff -= 360;
                    while (angleDiff < -180) angleDiff += 360;
                    turnRight = (angleDiff > 0);
                } else {
                    turnRight = (distL <= distR);
                }
            }
            if (turnRight) { leftSpeed =  TURN_SPEED; rightSpeed = -TURN_SPEED; }
            else           { leftSpeed = -TURN_SPEED; rightSpeed =  TURN_SPEED; }
            AM_PRINT("[SIMPLE] Obstacle -> TURN %s (count %d/%d)\n",
                     turnRight ? "RIGHT" : "LEFT",
                     st.sendItTurnCount, AM_SENDIT_TURN_THRESHOLD);
        }
        // PID forward
        else {
            float error = distance - OBSTACLE_CM;
            float dt = 0.05f;
            st.pidI += error * dt;
            st.pidI = AM_CONSTRAIN(st.pidI, -I_MAX, I_MAX);
            float derivative = (error - st.pidLastErr) / dt;
            st.pidLastErr = error;
            float output = (KP * error) + (KI * st.pidI) + (KD * derivative);
            int speed = AM_CONSTRAIN((int)output, MIN_SPEED, 255);
            leftSpeed = speed; rightSpeed = speed;

#if AM_IMU_ENABLED
            // Heading-hold
            if (!st.headingLocked && st.phase0EnteredAt > 0 &&
                (now - st.phase0EnteredAt > AM_HEADING_HOLD_SETTLE_MS)) {
                st.cruiseHeading = st.drHeading;
                st.headingLocked = true;
            }
            if (st.headingLocked) {
                float err = st.cruiseHeading - st.drHeading;
                while (err >  180) err -= 360;
                while (err < -180) err += 360;
                if (AM_ABS(err) > AM_HEADING_HOLD_DEADBAND) {
                    int bias = AM_CONSTRAIN((int)(err * AM_HEADING_HOLD_GAIN),
                                           -AM_HEADING_HOLD_MAX, AM_HEADING_HOLD_MAX);
                    leftSpeed  += bias;
                    rightSpeed -= bias;
                    leftSpeed  = AM_CONSTRAIN(leftSpeed,  MIN_SPEED, 255);
                    rightSpeed = AM_CONSTRAIN(rightSpeed, MIN_SPEED, 255);
                }
            }

            // Dead-reckoning steering: gently arc away from start
            {
                float distFromStart = sqrtf(st.drX*st.drX + st.drY*st.drY);
                if (distFromStart > AM_DR_MIN_DIST_CM && distance > STALL_DIST_CM) {
                    float awayBearing = atan2f(st.drX, st.drY) * 180.0f / AM_PI;
                    float headErr = awayBearing - st.drHeading;
                    while (headErr >  180) headErr -= 360;
                    while (headErr < -180) headErr += 360;
                    float proxScale = AM_CONSTRAIN(1.0f - (distFromStart - AM_DR_MIN_DIST_CM) / AM_DR_PROXIMITY_RANGE_CM, 0.2f, 1.0f);
                    int steerBias = AM_CONSTRAIN((int)(headErr * AM_DR_STEER_GAIN * proxScale),
                                                -AM_DR_STEER_MAX, AM_DR_STEER_MAX);
                    leftSpeed  += steerBias;
                    rightSpeed -= steerBias;
                    leftSpeed  = AM_CONSTRAIN(leftSpeed,  MIN_SPEED, 255);
                    rightSpeed = AM_CONSTRAIN(rightSpeed, MIN_SPEED, 255);
                    // Shift locked heading so heading-hold doesn't fight DR
                    if (AM_ABS(steerBias) > 5 && st.headingLocked) {
                        st.cruiseHeading += steerBias * 0.02f;
                        while (st.cruiseHeading >= 360) st.cruiseHeading -= 360;
                        while (st.cruiseHeading <    0) st.cruiseHeading += 360;
                    }
                }
            }
#endif
            // Stall detection
            if (distance < STALL_DIST_CM && now >= st.recoveryCooldownUntil) {
                if (st.stallStart == 0) st.stallStart = now;
                else if (now - st.stallStart > STALL_MS) {
                    AM_PRINT("[SIMPLE] STALL -> RECOVERY\n");
                    st.phase = 3; st.recoveryStep = 0; st.rockCycles = 0;
                    st.recoveryUntil = now + AM_RECOVERY_ROCK_MS;
                    st.recoveryTurnDir = (distL > distR) ? 1 : 0;
                    st.stallStart = 0;
                    st.accelMagReady = false; st.accelMagIdx = 0;
                    st.noMotionStart = 0;
                    setMotors(AM_RECOVERY_SPEED, AM_RECOVERY_SPEED);
                    return;
                }
            } else {
                st.stallStart = 0;
            }
        }
    } // end phase == 0

    if (st.phase == 1) {
        // TURNING — keep last turn direction
        if (leftSpeed == 0 && rightSpeed == 0) {
            if (distL <= distR) { leftSpeed =  TURN_SPEED; rightSpeed = -TURN_SPEED; }
            else                { leftSpeed = -TURN_SPEED; rightSpeed =  TURN_SPEED; }
        }
    } else if (st.phase == 2) {
        // REVERSING
        leftSpeed  = -REVERSE_SPEED;
        rightSpeed = -REVERSE_SPEED;
    }

    setMotors(leftSpeed, rightSpeed);

    if (now - st.lastDebug > 2000) {
        st.lastDebug = now;
        const char* s2 = (st.phase == 0) ? "PID-FWD" : (st.phase == 1) ? "TURN" : "REVERSE";
        AM_PRINT("[SIMPLE] %s | Fwd:%.0f L:%.0f R:%.0f | pitch:%.0f | %s%s | ML:%d MR:%d\n",
                 s2, distance, distL, distR, pitch,
                 st.motionOK ? "" : "NO-MOTION ",
                 st.recentlyStuck ? "SOFT-RAMP" : "",
                 leftSpeed, rightSpeed);
    }
}

// ===================================================================
// MODE 5 — WALL FOLLOW
// Left-hand-rule PID perimeter tracer.
// States: WF_SEARCH → WF_FOLLOW ↔ WF_CORNER / WF_BLOCKED / WF_CRUISE
// ===================================================================

void RobotModes::resetWallFollowState() {
    WallFollowState& wf = _wf;
    wf.state       = WF_SEARCH;
    wf.actionUntil = 0;
    wf.searchStart = 0;
    wf.pidI        = 0.0f;
    wf.prevErr     = 0.0f;
    wf.lastDebug   = 0;
}

void RobotModes::runWallFollow() {
    WallFollowState& wf = _wf;
    const SensorInput& s = *_s;

    float fwd  = s.distFront;
    float left = s.distLeft;
    unsigned long now = AM_MILLIS();

    if (fwd  < 0) fwd  = 400.0f;
    if (left < 0) left = 400.0f;

    bool frontBlocked = (fwd  < AM_WF_FRONT_STOP_CM);
    bool frontSlow    = (fwd  > AM_WF_FRONT_STOP_CM && fwd < AM_WF_FRONT_SLOW_CM);
    bool wallPresent  = (left < AM_WF_WALL_LOST_CM);
    bool inTimedAction = (now < wf.actionUntil);

    int leftSpeed = 0, rightSpeed = 0;

    switch (wf.state) {

        // Spin counter-clockwise until a wall appears on the left
        case WF_SEARCH:
            if (wf.searchStart == 0) wf.searchStart = now;
            if (wallPresent) {
                AM_PRINT("[WF] Wall found %.0fcm -> FOLLOW\n", left);
                wf.state = WF_FOLLOW;
                wf.searchStart = 0;
                wf.pidI = 0; wf.prevErr = 0;
            } else if (now - wf.searchStart > AM_WF_SEARCH_TIMEOUT_MS) {
                AM_PRINT("[WF] No wall found -> CRUISE\n");
                wf.state = WF_CRUISE;
                wf.searchStart = 0;
            } else {
                leftSpeed  = -AM_WF_SEARCH_SPEED;
                rightSpeed =  AM_WF_SEARCH_SPEED;
            }
            break;

        // Drive forward until wall appears, then switch to FOLLOW
        case WF_CRUISE:
            if (wallPresent) {
                AM_PRINT("[WF] Wall acquired %.0fcm -> FOLLOW\n", left);
                wf.state = WF_FOLLOW;
                wf.pidI = 0; wf.prevErr = 0;
            } else if (frontBlocked) {
                AM_PRINT("[WF] Blocked while cruising -> SEARCH\n");
                wf.state = WF_SEARCH;
                wf.searchStart = 0;
            } else {
                leftSpeed  = AM_WF_CRUISE_SPEED;
                rightSpeed = AM_WF_CRUISE_SPEED;
            }
            break;

        // PID control to maintain target distance from left wall
        case WF_FOLLOW:
            if (!wallPresent && !inTimedAction) {
                AM_PRINT("[WF] Wall lost %.0fcm -> CORNER\n", left);
                wf.state = WF_CORNER;
                wf.actionUntil = now + AM_WF_CORNER_TURN_MS;
                wf.pidI = 0; wf.prevErr = 0;
                break;
            }
            if (frontBlocked && !inTimedAction) {
                AM_PRINT("[WF] Front blocked %.0fcm -> BLOCKED\n", fwd);
                wf.state = WF_BLOCKED;
                wf.actionUntil = now + AM_WF_BLOCK_TURN_MS;
                wf.pidI = 0; wf.prevErr = 0;
                break;
            }
            {
                float err = left - AM_WF_TARGET_DIST_CM;
                wf.pidI   += err * 0.05f;
                wf.pidI    = AM_CONSTRAIN(wf.pidI, -40.0f, 40.0f);
                float derr = err - wf.prevErr;
                wf.prevErr = err;
                float correction = AM_WF_PID_KP * err + AM_WF_PID_KI * wf.pidI + AM_WF_PID_KD * derr;
                correction = AM_CONSTRAIN(correction, -80.0f, 80.0f);
                int baseSpeed = frontSlow ? AM_WF_SLOW_SPEED : AM_WF_CRUISE_SPEED;
                leftSpeed  = AM_CONSTRAIN((int)(baseSpeed - correction), -255, 255);
                rightSpeed = AM_CONSTRAIN((int)(baseSpeed + correction), -255, 255);
            }
            break;

        // Left wall gone — turn left to wrap around corner
        case WF_CORNER:
            if (inTimedAction) {
                leftSpeed  = -AM_WF_TURN_SPEED;
                rightSpeed =  AM_WF_TURN_SPEED;
            } else {
                if (wallPresent) {
                    AM_PRINT("[WF] Wall re-acquired after corner -> FOLLOW\n");
                    wf.state = WF_FOLLOW;
                    wf.pidI = 0; wf.prevErr = 0;
                } else {
                    AM_PRINT("[WF] Still no wall after corner -> SEARCH\n");
                    wf.state = WF_SEARCH;
                    wf.searchStart = 0;
                }
            }
            break;

        // Front blocked — turn right to clear
        case WF_BLOCKED:
            if (inTimedAction) {
                leftSpeed  =  AM_WF_TURN_SPEED;
                rightSpeed = -AM_WF_TURN_SPEED;
            } else {
                if (wallPresent && !frontBlocked) {
                    AM_PRINT("[WF] Path clear after block -> FOLLOW\n");
                    wf.state = WF_FOLLOW;
                    wf.pidI = 0; wf.prevErr = 0;
                } else if (!wallPresent) {
                    wf.state = WF_SEARCH;
                    wf.searchStart = 0;
                } else {
                    // Still blocked — keep turning
                    wf.actionUntil = now + AM_WF_BLOCK_TURN_MS;
                }
            }
            break;
    }

#if AM_IMU_ENABLED
    // IMU steep-terrain guard: if pitching dangerously, hold position
    {
        float wfPitch = atan2f(s.accelX, sqrtf(s.accelY*s.accelY + s.accelZ*s.accelZ)) * 180.0f / AM_PI;
        if (fabsf(wfPitch) > AM_INCLINE_MAX_PITCH * 1.5f) {
            static unsigned long warnAt = 0;
            if (now - warnAt > 3000) {
                AM_PRINT("[WF] Steep terrain %.1f - holding\n", wfPitch);
                warnAt = now;
            }
            leftSpeed = 0; rightSpeed = 0;
        }
    }
#endif

    setMotors(leftSpeed, rightSpeed);

    if (now - wf.lastDebug > 2000) {
        wf.lastDebug = now;
        const char* stateStr =
            wf.state == WF_SEARCH  ? "SEARCH"  :
            wf.state == WF_FOLLOW  ? "FOLLOW"  :
            wf.state == WF_CORNER  ? "CORNER"  :
            wf.state == WF_CRUISE  ? "CRUISE"  : "BLOCKED";
        AM_PRINT("[WF] %s | Fwd:%.0f Left:%.0f | Err:%.1f | ML:%d MR:%d\n",
                 stateStr, fwd, left, left - AM_WF_TARGET_DIST_CM, leftSpeed, rightSpeed);
    }
}

// ===================================================================
// ADVANCED NAV CORE — implementation
// ===================================================================

AdvNavCore::AdvNavCore() {
    state = ADV_STOPPED;
    stateStart = lastStateChange = 0;
    firstRun   = true;

    lastDist = 0; histIdx = 0; stuckCounter = 0;
    for (int i = 0; i < 5; i++) distHist[i] = 999.0f;

    currentSpeed = targetSpeed = 0;
    stallStart   = 0;
    encSpeedCmS  = 0;

    sideL = sideR = 999.0f;
    wAngle = 0.0f;

    upsideDown   = false;
    imuAvail     = false;
    orientation  = AO_NORMAL;
    pitch = roll = iax = iay = iaz = igyroZ = 0;
    lastIMUUpdate = 0;
    motionVerified = true;
    for (int i = 0; i < AM_MOTION_VERIFY_WINDOW; i++) accelMagHist[i] = 1.0f;
    accelMagIdx = 0; accelMagFilled = false; noMotionStart = 0;

    heading = headingAtObs = 0;
    drX = drY = 0;
    headingTowardStartSince = 0;
    cruiseHeading = 0; headingLocked = false; cruiseEnteredAt = 0;

    odomX = odomY = odomTheta = 0;

    lastTurnDir = ATD_RANDOM; turnStartHeading = 0; consecSameDir = 0;
    for (int i = 0; i < 8; i++) obstMem[i] = 0;

    avoidPhase = AA_BACKUP; avoidTurnDir = ATD_RIGHT;
    avoidAttempts = 0; avoidPhaseStart = avoidPhaseDur = 0;

    hazPhase = AH_BACKUP; hazPhaseStart = hazPhaseDur = 0;
    inclineAttempts = 0; steepInclineStart = 0; onSteepIncline = false;

    recPhase = AR_DONE; rockCount = 0;
    recStepStart = recStepDur = 0; recTurnDir = ATD_RIGHT; coastAfterFwd = true;
    recentlyStuck = false; stuckRecovTime = 0; recoveryCooldownUntil = 0;

    terrainBoostActive = false; terrainBoostUntil = 0;
}

void AdvNavCore::setSideDistances(float left, float right) {
    sideL = (left  < 0 || left  > 400) ? 999.0f : left;
    sideR = (right < 0 || right > 400) ? 999.0f : right;
}

void AdvNavCore::setWallAngle(float angle) { wAngle = angle; }

void AdvNavCore::setEncoderSpeed(float cmPerSec) { encSpeedCmS = cmPerSec; }

void AdvNavCore::setOdometry(float x, float y, float theta) {
    odomX = x; odomY = y; odomTheta = theta;
}

void AdvNavCore::updateIMU(float ax, float ay, float az, float gx, float gy, float gz) {
    unsigned long now = AM_MILLIS();
    float dt = (lastIMUUpdate > 0) ? (now - lastIMUUpdate) / 1000.0f : 0;
    lastIMUUpdate = now;

    iax = ax; iay = ay; iaz = az; igyroZ = gz;
    imuAvail = true;

    if (dt > 0 && dt < 0.5f) {
        heading += (gz - AM_GYRO_DRIFT) * dt;
        while (heading >= 360) heading -= 360;
        while (heading <    0) heading += 360;

        if (state == ADV_CRUISE) {
            float speedCmS = encSpeedCmS;
            if (speedCmS < 1.0f) {
                float frac = currentSpeed / 255.0f;
                speedCmS = frac * AM_ROVER_MAX_SPEED_CM_S;
            }
            float distStep = speedCmS * dt;
            float headRad  = heading * AM_PI / 180.0f;
            drX += sinf(headRad) * distStep;
            drY += cosf(headRad) * distStep;
        }
    }

    pitch = atan2f(ax, sqrtf(ay*ay + az*az)) * 180.0f / AM_PI;
    roll  = atan2f(ay, sqrtf(ax*ax + az*az)) * 180.0f / AM_PI;

    float accelMag = sqrtf(ax*ax + ay*ay + az*az);
    accelMagHist[accelMagIdx] = accelMag;
    accelMagIdx = (accelMagIdx + 1) % AM_MOTION_VERIFY_WINDOW;
    if (accelMagIdx == 0) accelMagFilled = true;
    if (accelMagFilled) checkMotionVerification();

    bool wasUD = upsideDown;
    upsideDown = (az < AM_UPSIDE_DOWN_THRESHOLD);
    if (upsideDown != wasUD)
        AM_PRINT("[IMU] %s\n", upsideDown ? "UPSIDE DOWN!" : "Right-side up");

    AdvOrientation prev = orientation;
    if (upsideDown)            orientation = AO_UPSIDE_DOWN;
    else if (AM_ABS(roll)  > 30) orientation = (roll  > 0) ? AO_TILTED_RIGHT : AO_TILTED_LEFT;
    else if (AM_ABS(pitch) > 30) orientation = (pitch > 0) ? AO_TILTED_FWD   : AO_TILTED_BACK;
    else                         orientation = AO_NORMAL;

    if (orientation != prev && orientation != AO_NORMAL) {
        const char* s =
            orientation == AO_UPSIDE_DOWN  ? "UPSIDE_DOWN"  :
            orientation == AO_TILTED_LEFT  ? "TILTED_LEFT"  :
            orientation == AO_TILTED_RIGHT ? "TILTED_RIGHT" :
            orientation == AO_TILTED_FWD   ? "TILTED_FWD"   : "TILTED_BACK";
        AM_PRINT("[IMU] Orientation: %s (p=%.1f r=%.1f)\n", s, pitch, roll);
    }
}

const char* AdvNavCore::getStateString() const {
    switch (state) {
        case ADV_CRUISE:  return "CRUISE";
        case ADV_AVOID:   return "AVOID";
        case ADV_RECOVER: return "RECOVER";
        case ADV_HAZARD:  return "HAZARD";
        case ADV_STOPPED: return "STOP";
        default:          return "?";
    }
}

void AdvNavCore::update(float distance) {
    if (distance < 0 || distance > 400) distance = AM_DIST_CLOSE - 1.0f;
    updateDistHist(distance);

    unsigned long now = AM_MILLIS();
    if (recentlyStuck && (now - stuckRecovTime > AM_ADAPTIVE_RAMP_TIMEOUT))
        recentlyStuck = false;

    if (firstRun) { firstRun = false; enterCruise(); }

    switch (state) {
        case ADV_CRUISE:  handleCruise(distance);   break;
        case ADV_AVOID:   handleAvoid(distance);    break;
        case ADV_RECOVER: handleRecovery(distance); break;
        case ADV_HAZARD:  handleHazard(distance);   break;
        case ADV_STOPPED: targetSpeed = 0;           break;
    }

    smoothSpeed();
    lastDist = distance;
}

// --- CRUISE ---
void AdvNavCore::handleCruise(float distance) {
    unsigned long now = AM_MILLIS();

    if (distance < AM_DIST_CRITICAL) {
        AM_PRINT("[CRUISE] CRITICAL %.0fcm -> AVOID\n", distance);
        enterAvoid(true); return;
    }
    if (distance < AM_DIST_CLOSE) {
        AM_PRINT("[CRUISE] Obstacle %.0fcm -> AVOID\n", distance);
        enterAvoid(false); return;
    }
    if (isStuck()) {
        AM_PRINT("[CRUISE] Stuck -> RECOVER\n");
        enterRecover(); return;
    }

    bool distStagnant = (AM_ABS(distance - lastDist) < 3.0f);
    if (imuAvail && !motionVerified && distStagnant &&
        currentSpeed > ADV_SPEED_MIN && now > recoveryCooldownUntil) {
        unsigned long dur = (noMotionStart > 0) ? (now - noMotionStart) : 0;
        if (dur > AM_MOTION_VERIFY_TIMEOUT) {
            AM_PRINT("[CRUISE] No motion %lums -> RECOVER\n", dur);
            enterRecover(); return;
        }
    } else if (motionVerified || !distStagnant) {
        noMotionStart = 0;
    }

    // Incline check
    if (imuAvail) {
        float absPitch = AM_ABS(pitch);
        if (absPitch > AM_INCLINE_MAX_PITCH) {
            if (steepInclineStart == 0) {
                steepInclineStart = now;
                AM_PRINT("[CRUISE] Steep incline %.1f\n", pitch);
            } else if (now - steepInclineStart > AM_INCLINE_TIMEOUT_MS) {
                AM_PRINT("[CRUISE] Incline timeout -> HAZARD (attempt %d)\n", inclineAttempts+1);
                enterHazard(); return;
            }
        } else {
            if (steepInclineStart > 0)
                AM_PRINT("[CRUISE] Incline cleared (%.1f)\n", pitch);
            steepInclineStart = 0; onSteepIncline = false; inclineAttempts = 0;
        }
    }

    // Stall in medium range
    if (distance < AM_DIST_MEDIUM) {
        if (stallStart == 0) stallStart = now;
        else if (now - stallStart > AM_STALL_TIMEOUT_MS) {
            AM_PRINT("[CRUISE] Stall at %.0fcm -> AVOID\n", distance);
            enterAvoid(false); return;
        }
    } else {
        stallStart = 0;
    }

    targetSpeed = calcSpeed(distance);

    // DR heading correction: if pointing toward start too long, force a correction turn
    float distFromStart = sqrtf(drX*drX + drY*drY);
    if (distFromStart > AM_DR_MIN_DIST_CM && distance > AM_DR_CORRECT_CLEARANCE_CM) {
        float toBearing = atan2f(-drX, -drY) * 180.0f / AM_PI;
        float toError   = toBearing - heading;
        while (toError >  180) toError -= 360;
        while (toError < -180) toError += 360;
        if (AM_ABS(toError) < AM_DR_HEADING_TOWARD_DEG) {
            if (headingTowardStartSince == 0) headingTowardStartSince = now;
            else if (now - headingTowardStartSince > AM_DR_CORRECT_TIMEOUT_MS) {
                float awayBearing = atan2f(drX, drY) * 180.0f / AM_PI;
                float awayError   = awayBearing - heading;
                while (awayError >  180) awayError -= 360;
                while (awayError < -180) awayError += 360;
                avoidPhase = AA_TURN;
                avoidTurnDir = (awayError > 0) ? ATD_RIGHT : ATD_LEFT;
                avoidPhaseStart = now;
                avoidPhaseDur   = AM_DR_CORRECT_TURN_MS;
                avoidAttempts   = 0;
                state = ADV_AVOID;
                lastStateChange = now;
                targetSpeed = ADV_SPEED_TURN;
                headingTowardStartSince = 0;
                AM_PRINT("[DR] Heading toward start -> correction turn\n");
                return;
            }
        } else {
            headingTowardStartSince = 0;
        }
    } else {
        headingTowardStartSince = 0;
    }
}

// --- AVOID ---
void AdvNavCore::handleAvoid(float distance) {
    unsigned long now     = AM_MILLIS();
    unsigned long elapsed = now - avoidPhaseStart;
    if (elapsed < avoidPhaseDur) return; // Still executing

    switch (avoidPhase) {
        case AA_BACKUP: {
            if (avoidAttempts == 0) {
                avoidTurnDir = pickTurnDir(distance);
            } else {
                avoidTurnDir = (avoidTurnDir == ATD_LEFT) ? ATD_RIGHT : ATD_LEFT;
                AM_PRINT("[AVOID] Retry %d -> flip to %s\n", avoidAttempts,
                         avoidTurnDir == ATD_LEFT ? "LEFT" : "RIGHT");
            }
            float absAngle = AM_ABS(wAngle);
            unsigned long baseTurn;
            if      (absAngle > 60) baseTurn = 250;
            else if (absAngle > 30) baseTurn = 375;
            else                    baseTurn = 450;
            unsigned long turnTime = baseTurn + (unsigned long)(avoidAttempts * 100);

            avoidPhase     = AA_TURN;
            avoidPhaseStart = now;
            avoidPhaseDur  = turnTime;
            targetSpeed    = ADV_SPEED_TURN;
            AM_PRINT("[AVOID] TURN %s %lums (wall:%.0f attempt:%d)\n",
                     avoidTurnDir == ATD_LEFT ? "LEFT" : "RIGHT",
                     turnTime, wAngle, avoidAttempts);
            break;
        }
        case AA_TURN:
            avoidPhase     = AA_VERIFY;
            avoidPhaseStart = now;
            avoidPhaseDur  = AM_AVOID_VERIFY_MS;
            targetSpeed    = ADV_SPEED_MIN;
            AM_PRINT("[AVOID] VERIFY forward\n");
            break;

        case AA_VERIFY:
            if (distance > AM_DIST_CLOSE) {
                AM_PRINT("[AVOID] Clear (%.0fcm) -> CRUISE\n", distance);
                avoidAttempts = 0;
                enterCruise();
            } else {
                avoidAttempts++;
                if (avoidAttempts >= AM_AVOID_MAX_CYCLES) {
                    AM_PRINT("[AVOID] %d attempts failed -> RECOVER\n", avoidAttempts);
                    avoidAttempts = 0;
                    enterRecover();
                } else {
                    AM_PRINT("[AVOID] Still blocked %.0fcm retry %d/%d\n",
                             distance, avoidAttempts, AM_AVOID_MAX_CYCLES);
                    unsigned long retryBackup = AM_AVOID_BACKUP_MS + (unsigned long)(avoidAttempts * 150);
                    avoidPhase     = AA_BACKUP;
                    avoidPhaseStart = now;
                    avoidPhaseDur  = retryBackup;
                    targetSpeed    = ADV_SPEED_CRUISE;
                }
            }
            break;
    }
}

// --- HAZARD ---
void AdvNavCore::handleHazard(float distance) {
    unsigned long now     = AM_MILLIS();
    unsigned long elapsed = now - hazPhaseStart;

    if (imuAvail && AM_ABS(pitch) < AM_INCLINE_MAX_PITCH) {
        AM_PRINT("[HAZARD] Incline cleared -> CRUISE\n");
        steepInclineStart = 0; onSteepIncline = false;
        enterCruise(); return;
    }
    if (elapsed < hazPhaseDur) return;

    switch (hazPhase) {
        case AH_BACKUP:
            hazPhase     = AH_TURN;
            hazPhaseStart = now;
            hazPhaseDur  = AM_INCLINE_DIAG_TURN_MS;
            targetSpeed  = ADV_SPEED_TURN;
            AM_PRINT("[HAZARD] TURN %s\n", (inclineAttempts % 2 == 0) ? "RIGHT" : "LEFT");
            break;
        case AH_TURN:
            if (inclineAttempts >= 3) {
                AM_PRINT("[HAZARD] %d attempts -> RECOVER\n", inclineAttempts);
                inclineAttempts = 0; steepInclineStart = 0; onSteepIncline = false;
                enterRecover();
            } else {
                AM_PRINT("[HAZARD] Attempt %d done -> CRUISE\n", inclineAttempts);
                steepInclineStart = 0;
                enterCruise();
            }
            break;
    }
}

// --- RECOVERY ---
void AdvNavCore::handleRecovery(float distance) {
    unsigned long now     = AM_MILLIS();
    unsigned long elapsed = now - recStepStart;
    if (elapsed < recStepDur) return;

    switch (recPhase) {
        case AR_ROCK_FWD:
            recPhase = AR_COAST; recStepStart = now; recStepDur = AM_RECOVERY_COAST_MS;
            targetSpeed = 0; coastAfterFwd = true;
            break;
        case AR_COAST:
            if (coastAfterFwd) {
                recPhase = AR_ROCK_REV; recStepStart = now; recStepDur = AM_RECOVERY_ROCK_MS;
                targetSpeed = AM_RECOVERY_SPEED;
                AM_PRINT("[RECOVER] ROCK_REV (%d/%d)\n", rockCount+1, AM_RECOVERY_ROCK_ATTEMPTS);
            } else {
                if (rockCount < AM_RECOVERY_ROCK_ATTEMPTS) {
                    recPhase = AR_ROCK_FWD; recStepStart = now; recStepDur = AM_RECOVERY_ROCK_MS;
                    targetSpeed = AM_RECOVERY_SPEED;
                    AM_PRINT("[RECOVER] ROCK_FWD (%d/%d)\n", rockCount+1, AM_RECOVERY_ROCK_ATTEMPTS);
                } else {
                    recPhase = AR_DIAG_TURN; recStepStart = now; recStepDur = AM_RECOVERY_DIAG_TURN_MS;
                    targetSpeed = ADV_SPEED_TURN;
                    AM_PRINT("[RECOVER] DIAG_TURN %s\n", recTurnDir == ATD_LEFT ? "LEFT" : "RIGHT");
                }
            }
            break;
        case AR_ROCK_REV:
            rockCount++;
            recPhase = AR_COAST; recStepStart = now; recStepDur = AM_RECOVERY_COAST_MS;
            targetSpeed = 0; coastAfterFwd = false;
            break;
        case AR_DIAG_TURN:
            recPhase = AR_DIAG_FWD; recStepStart = now; recStepDur = AM_RECOVERY_DIAG_FWD_MS;
            targetSpeed = ADV_SPEED_CRUISE;
            AM_PRINT("[RECOVER] DIAG_FWD %dms\n", AM_RECOVERY_DIAG_FWD_MS);
            break;
        case AR_DIAG_FWD:
            if (motionVerified && distance > AM_DIST_CLOSE) {
                AM_PRINT("[RECOVER] === FREE! -> CRUISE ===\n");
                recPhase = AR_DONE;
                flagRecentlyStuck();
                enterCruise();
            } else {
                recPhase = AR_FULL_REV; recStepStart = now; recStepDur = AM_RECOVERY_FULL_REV_MS;
                targetSpeed = AM_RECOVERY_SPEED;
                AM_PRINT("[RECOVER] FULL_REV %dms\n", AM_RECOVERY_FULL_REV_MS);
            }
            break;
        case AR_FULL_REV: {
            AM_PRINT("[RECOVER] === Complete -> exit via turn ===\n");
            recPhase = AR_DONE;
            flagRecentlyStuck();
            AdvTurnDir dir = pickTurnDir(distance);
            avoidPhase     = AA_TURN;
            avoidTurnDir   = dir;
            avoidPhaseStart = AM_MILLIS();
            avoidPhaseDur  = 250;
            avoidAttempts  = 0;
            state = ADV_AVOID;
            lastStateChange = AM_MILLIS();
            targetSpeed = ADV_SPEED_TURN;
            break;
        }
        case AR_DONE: default:
            enterCruise();
            break;
    }
}

// --- STATE TRANSITIONS ---
void AdvNavCore::enterCruise() {
    state = ADV_CRUISE;
    stateStart = lastStateChange = AM_MILLIS();
    targetSpeed = ADV_SPEED_CRUISE;
    stuckCounter = 0; stallStart = 0;
    headingLocked = false; cruiseEnteredAt = AM_MILLIS();
    accelMagFilled = false; accelMagIdx = 0;
    noMotionStart = 0; motionVerified = true;
    for (int i = 0; i < 8; i++) if (obstMem[i] > 0) obstMem[i]--;
}

void AdvNavCore::enterAvoid(bool critical) {
    terrainBoostActive = false;
    state = ADV_AVOID;
    stateStart = lastStateChange = AM_MILLIS();
    avoidPhase     = AA_BACKUP;
    avoidPhaseStart = AM_MILLIS();
    avoidPhaseDur  = critical ? AM_AVOID_BACKUP_CRIT_MS : AM_AVOID_BACKUP_MS;
    targetSpeed    = ADV_SPEED_CRUISE;
    stallStart     = 0;
    AM_PRINT("[AVOID] BACKUP %lums\n", avoidPhaseDur);
}

void AdvNavCore::enterAvoidFromPit() {
    if (state != ADV_CRUISE) return;
    AM_PRINT("[PIT] Pit -> AVOID backup\n");
    enterAvoid(false);
    avoidPhaseDur = AM_PIT_BACKUP_MS;
}

void AdvNavCore::enterAvoidFromHill() {
    if (state != ADV_CRUISE) return;
    terrainBoostActive = false;
    AM_PRINT("[HILL] Hill -> AVOID backup\n");
    enterAvoid(false);
    avoidPhaseDur = AM_HILL_BACKUP_MS;
}

void AdvNavCore::enterRecover() {
    terrainBoostActive = false;
    startRecovery();
}

void AdvNavCore::enterHazard() {
    state = ADV_HAZARD;
    stateStart = lastStateChange = AM_MILLIS();
    onSteepIncline = true;
    inclineAttempts++;
    hazPhase    = AH_BACKUP;
    hazPhaseStart = AM_MILLIS();
    hazPhaseDur = 800;
    targetSpeed = ADV_SPEED_CRUISE;
    AM_PRINT("[HAZARD] BACKUP 800ms (attempt %d)\n", inclineAttempts);
}

void AdvNavCore::enterTerrainBoost() {
    if (state != ADV_CRUISE) return;
    terrainBoostActive = true;
    terrainBoostUntil  = AM_MILLIS() + AM_TERRAIN_BOOST_DURATION_MS;
    targetSpeed = ADV_SPEED_CRUISE;
    AM_PRINT("[TERRAIN] Boost %dms\n", AM_TERRAIN_BOOST_DURATION_MS);
}

bool AdvNavCore::isTerrainBoostActive() {
    if (terrainBoostActive && AM_MILLIS() > terrainBoostUntil)
        terrainBoostActive = false;
    return terrainBoostActive;
}

void AdvNavCore::startRecovery() {
    AM_PRINT("[RECOVER] === Starting recovery ===\n");
    state = ADV_RECOVER;
    stateStart = lastStateChange = AM_MILLIS();
    recPhase    = AR_ROCK_FWD;
    rockCount   = 0;
    recStepStart = AM_MILLIS();
    recStepDur   = AM_RECOVERY_ROCK_MS;
    targetSpeed  = AM_RECOVERY_SPEED;

    if      (sideL > sideR + 5.0f) recTurnDir = ATD_LEFT;
    else if (sideR > sideL + 5.0f) recTurnDir = ATD_RIGHT;
    else recTurnDir = (AM_RANDOM(2) == 0) ? ATD_LEFT : ATD_RIGHT;

    AM_PRINT("[RECOVER] ROCK_FWD, diag: %s\n", recTurnDir == ATD_LEFT ? "LEFT" : "RIGHT");
}

void AdvNavCore::flagRecentlyStuck() {
    recentlyStuck = true;
    stuckRecovTime = AM_MILLIS();
    recoveryCooldownUntil = AM_MILLIS() + AM_RECOVERY_COOLDOWN_MS;
    noMotionStart = 0; motionVerified = true;
    stuckCounter = 0; stallStart = 0;
    AM_PRINT("[TERRAIN] Recovery cooldown %dms\n", AM_RECOVERY_COOLDOWN_MS);
}

// --- HELPERS ---
bool AdvNavCore::isStuck() {
    if (state != ADV_CRUISE) { stuckCounter = 0; return false; }
    float minD = distHist[0], maxD = distHist[0];
    for (int i = 1; i < 5; i++) {
        if (distHist[i] < minD) minD = distHist[i];
        if (distHist[i] > maxD) maxD = distHist[i];
    }
    if ((maxD - minD) < AM_STUCK_DISTANCE_TOL && lastDist < AM_DIST_MEDIUM) {
        stuckCounter++;
        return stuckCounter >= AM_STUCK_COUNT_THRESHOLD;
    } else {
        stuckCounter = 0;
    }
    return false;
}

void AdvNavCore::updateDistHist(float dist) {
    distHist[histIdx] = dist;
    histIdx = (histIdx + 1) % 5;
}

int AdvNavCore::calcSpeed(float distance) {
    if (terrainBoostActive) {
        if (AM_MILLIS() > terrainBoostUntil) terrainBoostActive = false;
        else return ADV_SPEED_CRUISE;
    }
    if (distance >= AM_DIST_FAR)   return ADV_SPEED_CRUISE;
    if (distance >  AM_DIST_CLOSE) {
        float ratio = (distance - AM_DIST_CLOSE) / (AM_DIST_FAR - AM_DIST_CLOSE);
        return ADV_SPEED_MIN + (int)(ratio * (ADV_SPEED_CRUISE - ADV_SPEED_MIN));
    }
    return ADV_SPEED_MIN;
}

void AdvNavCore::smoothSpeed() {
    currentSpeed = AM_CONSTRAIN(targetSpeed, 0, ADV_SPEED_MAX);
}

void AdvNavCore::checkMotionVerification() {
    float sum = 0;
    for (int i = 0; i < AM_MOTION_VERIFY_WINDOW; i++) sum += accelMagHist[i];
    float mean = sum / AM_MOTION_VERIFY_WINDOW;
    float var  = 0;
    for (int i = 0; i < AM_MOTION_VERIFY_WINDOW; i++) {
        float d = accelMagHist[i] - mean; var += d*d;
    }
    var /= AM_MOTION_VERIFY_WINDOW;
    bool was = motionVerified;
    if (currentSpeed < ADV_SPEED_MIN)    { motionVerified = true; noMotionStart = 0; }
    else if (var > AM_MOTION_ACCEL_VAR_THRESH) { motionVerified = true; noMotionStart = 0; }
    else {
        motionVerified = false;
        if (noMotionStart == 0) noMotionStart = AM_MILLIS();
    }
    if (was && !motionVerified)  AM_PRINT("[TERRAIN] Motion lost! var=%.6f\n", var);
    else if (!was && motionVerified) { AM_PRINT("[TERRAIN] Motion restored\n"); noMotionStart = 0; }
}

bool AdvNavCore::isSlippingOrStalled() {
    return (currentSpeed > 100 && encSpeedCmS < 5.0f);
}

AdvTurnDir AdvNavCore::pickTurnDir(float distance) {
    if (imuAvail) {
        int bin = getHeadingBin();
        obstMem[bin]++;
        headingAtObs = heading;
        AM_PRINT("[NAV] Obstacle at heading %.0f (bin %d count %d)\n", heading, bin, obstMem[bin]);
    }
    AdvTurnDir best;
    float sideDiff = sideL - sideR;
    if (AM_ABS(sideDiff) > 5.0f) {
        best = (sideL > sideR) ? ATD_LEFT : ATD_RIGHT;
    } else {
        float distFromStart = sqrtf(drX*drX + drY*drY);
        if (distFromStart > 30.0f) {
            float awayBearing = atan2f(drX, drY) * 180.0f / AM_PI;
            float angleDiff   = awayBearing - heading;
            while (angleDiff >  180) angleDiff -= 360;
            while (angleDiff < -180) angleDiff += 360;
            best = (angleDiff > 0) ? ATD_RIGHT : ATD_LEFT;
        } else {
            best = getBestTurnDir();
        }
    }
    consecSameDir = (best == lastTurnDir) ? consecSameDir + 1 : 1;
    lastTurnDir = best;
    turnStartHeading = heading;
    return best;
}

int AdvNavCore::getHeadingBin() {
    return (int)((heading + 22.5f) / 45.0f) % 8;
}

AdvTurnDir AdvNavCore::getBestTurnDir() {
    if (!imuAvail) {
        if (lastTurnDir != ATD_RANDOM && consecSameDir < 3) return lastTurnDir;
        return (AM_RANDOM(2) == 0) ? ATD_LEFT : ATD_RIGHT;
    }
    int leftScore = 0, rightScore = 0;
    int bin = getHeadingBin();
    int leftObs  = obstMem[(bin + 6) % 8] + obstMem[(bin + 7) % 8];
    int rightObs = obstMem[(bin + 1) % 8] + obstMem[(bin + 2) % 8];
    leftScore  -= leftObs  * 10;
    rightScore -= rightObs * 10;
    if (AM_ABS(roll) > 10) { if (roll < 0) rightScore += 15; else leftScore += 15; }
    if (consecSameDir >= 3) {
        if (lastTurnDir == ATD_LEFT)  rightScore += 25;
        else if (lastTurnDir == ATD_RIGHT) leftScore += 25;
    }
    if (consecSameDir < 3) {
        if (lastTurnDir == ATD_LEFT)  leftScore  += 5;
        else if (lastTurnDir == ATD_RIGHT) rightScore += 5;
    }
    leftScore  += AM_RANDOM(8);
    rightScore += AM_RANDOM(8);
    AM_PRINT("[NAV] Turn scores L=%d R=%d\n", leftScore, rightScore);
    return (leftScore >= rightScore) ? ATD_LEFT : ATD_RIGHT;
}

// --- MOTOR OUTPUT ---
void AdvNavCore::getMotorSpeeds(int& leftSpeed, int& rightSpeed) {
    int speed = currentSpeed;
    if (speed > 0 && speed < ADV_SPEED_MIN) speed = ADV_SPEED_MIN;

    // Slope compensation
    if (imuAvail && orientation == AO_NORMAL) {
        if      (pitch < -15) speed = AM_CONSTRAIN((int)(speed * 1.15f), 0, 255);
        else if (pitch >  15) speed = AM_CONSTRAIN((int)(speed * 0.75f), 80, 255);
    }

    switch (state) {
        case ADV_CRUISE: {
            // Lock heading after settle delay
            if (!headingLocked && imuAvail && (AM_MILLIS() - cruiseEnteredAt > AM_HEADING_HOLD_SETTLE_MS)) {
                cruiseHeading = heading;
                headingLocked = true;
                AM_PRINT("[HEADING] Locked at %.1f\n", cruiseHeading);
            }
            int holdBias = 0;
            if (headingLocked && imuAvail) {
                float err = cruiseHeading - heading;
                while (err >  180) err -= 360;
                while (err < -180) err += 360;
                if (AM_ABS(err) > AM_HEADING_HOLD_DEADBAND) {
                    holdBias = AM_CONSTRAIN((int)(err * AM_HEADING_HOLD_GAIN),
                                           -AM_HEADING_HOLD_MAX, AM_HEADING_HOLD_MAX);
                }
            }
            int drBias = 0;
            float distFromStart = sqrtf(drX*drX + drY*drY);
            if (distFromStart > AM_DR_MIN_DIST_CM && lastDist > AM_DIST_MEDIUM) {
                float awayBearing = atan2f(drX, drY) * 180.0f / AM_PI;
                float headErr = awayBearing - heading;
                while (headErr >  180) headErr -= 360;
                while (headErr < -180) headErr += 360;
                float proxScale = AM_CONSTRAIN(1.0f - (distFromStart - AM_DR_MIN_DIST_CM) / AM_DR_PROXIMITY_RANGE_CM, 0.2f, 1.0f);
                drBias = AM_CONSTRAIN((int)(headErr * AM_DR_STEER_GAIN * proxScale),
                                     -AM_DR_STEER_MAX, AM_DR_STEER_MAX);
                if (AM_ABS(drBias) > 5 && headingLocked) {
                    cruiseHeading += drBias * 0.02f;
                    while (cruiseHeading >= 360) cruiseHeading -= 360;
                    while (cruiseHeading <    0) cruiseHeading += 360;
                }
            }
            int totalBias = AM_CONSTRAIN(holdBias + drBias, -AM_HEADING_HOLD_MAX, AM_HEADING_HOLD_MAX);
            // Diagonal hill traversal
            if (terrainBoostActive && imuAvail && AM_ABS(pitch) > 8.0f) {
                int diagBias = AM_CONSTRAIN((int)(roll * 0.8f), -25, 25);
                totalBias = AM_CONSTRAIN(totalBias + diagBias, -AM_HEADING_HOLD_MAX, AM_HEADING_HOLD_MAX);
            }
            leftSpeed  = AM_CONSTRAIN(speed + totalBias, ADV_SPEED_MIN, 255);
            rightSpeed = AM_CONSTRAIN(speed - totalBias, ADV_SPEED_MIN, 255);
            break;
        }
        case ADV_AVOID:
            switch (avoidPhase) {
                case AA_BACKUP: leftSpeed = -speed; rightSpeed = -speed; break;
                case AA_TURN:
                    if (avoidTurnDir == ATD_LEFT) { leftSpeed = -speed; rightSpeed =  speed; }
                    else                          { leftSpeed =  speed; rightSpeed = -speed; }
                    break;
                case AA_VERIFY: leftSpeed = speed; rightSpeed = speed; break;
            }
            break;
        case ADV_RECOVER:
            switch (recPhase) {
                case AR_ROCK_FWD: case AR_DIAG_FWD:
                    leftSpeed =  speed; rightSpeed =  speed; break;
                case AR_COAST:
                    leftSpeed = 0; rightSpeed = 0; break;
                case AR_ROCK_REV: case AR_FULL_REV:
                    leftSpeed = -speed; rightSpeed = -speed; break;
                case AR_DIAG_TURN:
                    if (recTurnDir == ATD_LEFT) { leftSpeed = -speed; rightSpeed =  speed; }
                    else                        { leftSpeed =  speed; rightSpeed = -speed; }
                    break;
                default: leftSpeed = 0; rightSpeed = 0; break;
            }
            break;
        case ADV_HAZARD:
            switch (hazPhase) {
                case AH_BACKUP:
                    leftSpeed = -speed; rightSpeed = -speed; break;
                case AH_TURN:
                    if (inclineAttempts % 2 == 0) { leftSpeed =  speed; rightSpeed = -speed; }
                    else                          { leftSpeed = -speed; rightSpeed =  speed; }
                    break;
            }
            break;
        case ADV_STOPPED: default:
            leftSpeed = 0; rightSpeed = 0; break;
    }
}

void AdvNavCore::reset() {
    state = ADV_STOPPED;
    stateStart = AM_MILLIS();
    lastStateChange = 0;
    lastTurnDir = ATD_RANDOM;
    lastDist = 0; stuckCounter = 0; currentSpeed = targetSpeed = 0; stallStart = 0;
    sideL = sideR = 999.0f; wAngle = 0;
    heading = headingAtObs = 0; consecSameDir = 0;
    drX = drY = 0; headingTowardStartSince = 0;
    cruiseHeading = 0; headingLocked = false; cruiseEnteredAt = 0;
    for (int i = 0; i < 8; i++) obstMem[i] = 0;
    firstRun = true;
    avoidPhase = AA_BACKUP; avoidAttempts = 0;
    steepInclineStart = 0; onSteepIncline = false; inclineAttempts = 0;
    for (int i = 0; i < AM_MOTION_VERIFY_WINDOW; i++) accelMagHist[i] = 1.0f;
    accelMagIdx = 0; accelMagFilled = false; motionVerified = true; noMotionStart = 0;
    recPhase = AR_DONE; rockCount = 0; recentlyStuck = false; stuckRecovTime = 0; recoveryCooldownUntil = 0;
    terrainBoostActive = false; terrainBoostUntil = 0;
    for (int i = 0; i < 5; i++) distHist[i] = 999.0f;
    histIdx = 0;
    AM_PRINT("[NAV] Reset\n");
}
