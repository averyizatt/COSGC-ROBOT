#include "self_righting.h"
#include "config.h"

// Servo timing: SERVO_FREQ_HZ, SERVO_MIN_US, SERVO_MAX_US are all defined in config.h
// With 16-bit resolution the period = 1,000,000/SERVO_FREQ_HZ µs = 65536 counts

#define SERVO_RESOLUTION 16

// Self-righting / unsticking parameters — tuning values are in config.h
// Servo mounted on side, gear output on robot-right. 0°=LEFT extreme, 90°=CENTER, 180°=RIGHT extreme.
#define ARM_STOW_ANGLE   90   // Arm centered (pointing forward) — stow/park position
#define ARM_SWEEP_MIN     0   // Full sweep left
#define ARM_SWEEP_MAX   180   // Full sweep right
// ARM_SWEEP_STEP, ARM_SWEEP_DELAY_MS, ARM_HOLD_AT_EXTREME_MS, ARM_COOLDOWN_MS → config.h

// Unstick parameters
// ARM_UNSTICK_MAX, ARM_UNSTICK_COOLDOWN_MS → config.h

// Side-righting parameters
#define SIDE_PUSH_HOLD_MS    3000 // Hold at extreme for 3s per push attempt
#define SIDE_MAX_PUSHES      6    // Max push attempts before falling back to full sweep
#define SIDE_ROCK_ANGLE      25   // Degrees to rock back before re-pushing

// Idle oscillation parameters (gentle)
#define IDLE_AMPLITUDE       10  // ±10° from center
#define IDLE_STEP             1  // 1° per step (slow, gentle)
#define IDLE_DELAY_MS        40  // 40ms between steps → smooth & slow

SelfRightingArm::SelfRightingArm() {
    state = ARM_STOWED;
    currentAngle = ARM_STOW_ANGLE;
    sweepDirection = 1;
    lastSweepTime = 0;
    rightingStartTime = 0;
    cooldownUntil = 0;
    sweepCount = 0;
    holdingAtExtreme = false;
    holdStartTime = 0;
    idleCenter = ARM_STOW_ANGLE;
    idleAmplitude = IDLE_AMPLITUDE;
    sideTargetAngle = ARM_SWEEP_MAX;
    sideStartTime = 0;
    sidePushCount = 0;
}

void SelfRightingArm::begin() {
    ledcSetup(PWM_CHANNEL_SERVO, SERVO_FREQ_HZ, SERVO_RESOLUTION);
    ledcAttachPin(SERVO_PIN, PWM_CHANNEL_SERVO);

    // Move to stow position briefly, then kill PWM to save power
    writeAngle(ARM_STOW_ANGLE);
    delay(500);  // Give servo time to reach position
    detachServo();
    Serial.printf("[SERVO] Self-righting arm on GPIO%d — stowed at %d° (PWM off)\n", SERVO_PIN, ARM_STOW_ANGLE);
}

void SelfRightingArm::writeAngle(int angle) {
    angle = constrain(angle, 0, 180);
    currentAngle = angle;

    // Convert angle to pulse width in microseconds
    long pulseUs = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);

    // Convert pulse width to LEDC duty (16-bit resolution)
    // Period (µs) = 1,000,000 / SERVO_FREQ_HZ
    // duty = pulseUs / period_us * 65536
    uint32_t periodUs = 1000000UL / SERVO_FREQ_HZ;
    uint32_t duty = (uint32_t)(pulseUs * 65536UL / periodUs);
    ledcWrite(PWM_CHANNEL_SERVO, duty);
}

void SelfRightingArm::detachServo() {
    // Write duty=0 to stop PWM output entirely
    // Servo goes limp (no holding torque) but saves power & reduces ESP32 heat
    ledcWrite(PWM_CHANNEL_SERVO, 0);
}

void SelfRightingArm::update(bool isUpsideDown, bool isOnSide, float rollDeg) {
    unsigned long now = millis();

    // Upside-down always takes highest priority
    if (isUpsideDown && state != ARM_RIGHTING && state != ARM_COOLDOWN) {
        state = ARM_RIGHTING;
        rightingStartTime = now;
        sweepCount = 0;
        sweepDirection = 1;
        holdingAtExtreme = false;
        holdStartTime = 0;
        // Start from ARM_SWEEP_MIN so the FIRST pass goes full 0→180
        // (avoids the half-sweep asymmetry caused by starting at center=90)
        currentAngle = ARM_SWEEP_MIN;
        writeAngle(currentAngle);
        lastSweepTime = now;
        Serial.println("[SERVO] *** UPSIDE DOWN — starting self-righting! ***");
    }

    // On-side takes priority over idle/stowed (but not over active righting or cooldown)
    if (isOnSide && !isUpsideDown && state != ARM_RIGHTING && state != ARM_SIDE_RIGHTING && state != ARM_COOLDOWN) {
        state = ARM_SIDE_RIGHTING;
        sideStartTime = now;
        sweepCount = 0;
        holdingAtExtreme = false;
        holdStartTime = 0;
        // Start from center. Use roll sign to sweep toward the correct extreme first:
        // rollDeg > 0 → right side down → sweep right (to 180°) first so arm pushes off ground
        // rollDeg < 0 → left side down  → sweep left  (to 0°)  first so arm pushes off ground
        currentAngle = ARM_STOW_ANGLE;
        sweepDirection = (rollDeg >= 0) ? -1 : 1;
        writeAngle(currentAngle);
        lastSweepTime = now;
        Serial.printf("[SERVO] *** ON SIDE (roll=%.1f°) — sweeping %s first ***\n",
                      rollDeg, sweepDirection > 0 ? "RIGHT" : "LEFT");
    }

    switch (state) {
        case ARM_STOWED:
            // Nothing to do — arm is parked at stow angle
            break;

        case ARM_IDLE: {
            // Gentle oscillation: ±IDLE_AMPLITUDE degrees around center
            if (isUpsideDown) break;  // Handled above

            if (now - lastSweepTime >= IDLE_DELAY_MS) {
                lastSweepTime = now;

                currentAngle += (IDLE_STEP * sweepDirection);

                int idleMin = idleCenter - idleAmplitude;
                int idleMax = idleCenter + idleAmplitude;

                if (currentAngle >= idleMax) {
                    currentAngle = idleMax;
                    sweepDirection = -1;
                } else if (currentAngle <= idleMin) {
                    currentAngle = idleMin;
                    sweepDirection = 1;
                }

                writeAngle(currentAngle);
            }
            break;
        }

        case ARM_RIGHTING:
            if (!isUpsideDown) {
                // We're upright again! Stow the arm
                Serial.printf("[SERVO] Righted after %d sweeps (%.1fs) — stowing arm\n",
                              sweepCount, (now - rightingStartTime) / 1000.0f);
                stow();
                holdingAtExtreme = false;
                state = ARM_COOLDOWN;
                cooldownUntil = now + ARM_COOLDOWN_MS;
                break;
            }

            // If holding at an extreme (0° or 180°), wait 5 seconds
            if (holdingAtExtreme) {
                if (now - holdStartTime >= ARM_HOLD_AT_EXTREME_MS) {
                    holdingAtExtreme = false;
                    Serial.printf("[SERVO] Hold done at %d° — reversing sweep\n", currentAngle);
                }
                break;  // Don't move while holding
            }

            // Aggressive full sweep side to side
            if (now - lastSweepTime >= ARM_SWEEP_DELAY_MS) {
                lastSweepTime = now;

                currentAngle += (ARM_SWEEP_STEP * sweepDirection);

                if (currentAngle >= ARM_SWEEP_MAX) {
                    currentAngle = ARM_SWEEP_MAX;
                    writeAngle(currentAngle);
                    sweepDirection = -1;
                    sweepCount++;
                    // Hold at 180° for 5 seconds
                    holdingAtExtreme = true;
                    holdStartTime = now;
                    Serial.printf("[SERVO] Holding at %d° for %ds...\n", ARM_SWEEP_MAX, ARM_HOLD_AT_EXTREME_MS / 1000);
                    break;
                } else if (currentAngle <= ARM_SWEEP_MIN) {
                    currentAngle = ARM_SWEEP_MIN;
                    writeAngle(currentAngle);
                    sweepDirection = 1;
                    sweepCount++;
                    // Hold at 0° for 5 seconds
                    holdingAtExtreme = true;
                    holdStartTime = now;
                    Serial.printf("[SERVO] Holding at %d° for %ds...\n", ARM_SWEEP_MIN, ARM_HOLD_AT_EXTREME_MS / 1000);
                    break;
                }

                writeAngle(currentAngle);

                // Debug every few sweeps
                if (sweepCount > 0 && sweepCount % 4 == 0 && currentAngle == ARM_STOW_ANGLE) {
                    Serial.printf("[SERVO] Righting sweep %d/%d...\n", sweepCount, ARM_MAX_ATTEMPTS);
                }

                // Max attempts — cooldown and retry
                if (sweepCount >= ARM_MAX_ATTEMPTS) {
                    Serial.println("[SERVO] Max sweeps reached — cooling down before retry");
                    stow();
                    holdingAtExtreme = false;
                    state = ARM_COOLDOWN;
                    cooldownUntil = now + ARM_COOLDOWN_MS;
                }
            }
            break;

        case ARM_SIDE_RIGHTING:
            // Full aggressive sweeps to flip robot off its side
            // Same as ARM_RIGHTING but exits when no longer on side
            if (!isOnSide && !isUpsideDown) {
                // We're upright! Stow the arm
                Serial.printf("[SERVO] Side-righted after %d sweeps (%.1fs) — stowing\n",
                              sweepCount, (now - sideStartTime) / 1000.0f);
                stow();
                state = ARM_COOLDOWN;
                cooldownUntil = now + ARM_COOLDOWN_MS;
                break;
            }

            if (isUpsideDown) {
                // Went from side to upside-down — let the upside-down handler take over
                break;
            }

            // Hold at extremes
            if (holdingAtExtreme) {
                if (now - holdStartTime >= ARM_HOLD_AT_EXTREME_MS) {
                    holdingAtExtreme = false;
                    Serial.printf("[SERVO] Hold done at %d° — reversing sweep\n", currentAngle);
                }
                break;
            }

            // Aggressive full sweep side to side
            if (now - lastSweepTime >= ARM_SWEEP_DELAY_MS) {
                lastSweepTime = now;

                currentAngle += (ARM_SWEEP_STEP * sweepDirection);

                if (currentAngle >= ARM_SWEEP_MAX) {
                    currentAngle = ARM_SWEEP_MAX;
                    writeAngle(currentAngle);
                    sweepDirection = -1;
                    sweepCount++;
                    holdingAtExtreme = true;
                    holdStartTime = now;
                    Serial.printf("[SERVO] Side sweep — holding at %d° for %ds...\n", ARM_SWEEP_MAX, ARM_HOLD_AT_EXTREME_MS / 1000);
                    break;
                } else if (currentAngle <= ARM_SWEEP_MIN) {
                    currentAngle = ARM_SWEEP_MIN;
                    writeAngle(currentAngle);
                    sweepDirection = 1;
                    sweepCount++;
                    holdingAtExtreme = true;
                    holdStartTime = now;
                    Serial.printf("[SERVO] Side sweep — holding at %d° for %ds...\n", ARM_SWEEP_MIN, ARM_HOLD_AT_EXTREME_MS / 1000);
                    break;
                }

                writeAngle(currentAngle);

                if (sweepCount > 0 && sweepCount % 4 == 0 && currentAngle == ARM_STOW_ANGLE) {
                    Serial.printf("[SERVO] Side-righting sweep %d/%d...\n", sweepCount, ARM_MAX_ATTEMPTS);
                }

                // Max attempts — cooldown and retry
                if (sweepCount >= ARM_MAX_ATTEMPTS) {
                    Serial.println("[SERVO] Side-righting max sweeps — cooling down before retry");
                    stow();
                    holdingAtExtreme = false;
                    state = ARM_COOLDOWN;
                    cooldownUntil = now + ARM_COOLDOWN_MS;
                }
            }
            break;

        case ARM_UNSTICKING:
            // Same aggressive sweep as righting, but used when upright & stuck
            if (isUpsideDown) break;  // Priority switch handled above

            if (now - lastSweepTime >= ARM_SWEEP_DELAY_MS) {
                lastSweepTime = now;

                currentAngle += (ARM_SWEEP_STEP * sweepDirection);

                if (currentAngle >= ARM_SWEEP_MAX) {
                    currentAngle = ARM_SWEEP_MAX;
                    sweepDirection = -1;
                    sweepCount++;
                } else if (currentAngle <= ARM_SWEEP_MIN) {
                    currentAngle = ARM_SWEEP_MIN;
                    sweepDirection = 1;
                    sweepCount++;
                }

                writeAngle(currentAngle);

                // Debug
                if (sweepCount > 0 && sweepCount % 4 == 0 && currentAngle == ARM_STOW_ANGLE) {
                    Serial.printf("[SERVO] Unstick sweep %d/%d...\n", sweepCount, ARM_UNSTICK_MAX);
                }

                // Max attempts — cooldown
                if (sweepCount >= ARM_UNSTICK_MAX) {
                    Serial.println("[SERVO] Unstick max sweeps — cooldown");
                    stow();
                    state = ARM_COOLDOWN;
                    cooldownUntil = now + ARM_UNSTICK_COOLDOWN_MS;
                }
            }
            break;

        case ARM_COOLDOWN:
            if (now >= cooldownUntil) {
                if (isUpsideDown) {
                    // Still flipped — try again from correct start angle
                    state = ARM_RIGHTING;
                    rightingStartTime = now;
                    sweepCount = 0;
                    sweepDirection = 1;
                    holdingAtExtreme = false;
                    holdStartTime = 0;
                    lastSweepTime = now;
                    currentAngle = ARM_SWEEP_MIN;
                    writeAngle(currentAngle);
                    Serial.println("[SERVO] Still upside down — retrying self-righting");
                } else if (isOnSide) {
                    // Still on side — re-read rollDeg so we push off the correct side again
                    state = ARM_SIDE_RIGHTING;
                    sideStartTime = now;
                    sweepCount = 0;
                    holdingAtExtreme = false;
                    holdStartTime = 0;
                    lastSweepTime = now;
                    currentAngle = ARM_STOW_ANGLE;
                    sweepDirection = (rollDeg >= 0) ? -1 : 1;
                    writeAngle(currentAngle);
                    Serial.printf("[SERVO] Still on side (roll=%.1f°) — retrying, sweeping %s first\n",
                                  rollDeg, sweepDirection > 0 ? "RIGHT" : "LEFT");
                } else {
                    // We're upright — go to stowed
                    state = ARM_STOWED;
                    Serial.println("[SERVO] Cooldown done — arm stowed");
                }
            }
            break;
    }
}

void SelfRightingArm::stow() {
    writeAngle(ARM_STOW_ANGLE);
    delay(300);  // Brief hold to let servo reach position
    detachServo();  // Kill PWM — saves power & heat
    state = ARM_STOWED;
}

void SelfRightingArm::setAngle(int angle) {
    writeAngle(angle);
}
