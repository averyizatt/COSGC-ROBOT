#include "motor_control.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/dac.h"

// Temperature sensor: Use internal sensor if available
// ESP32 WROOM uses the legacy API; ESP32-S3 uses driver/temperature_sensor.h
#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();  // ROM function (note: misspelling is in the SDK)
#ifdef __cplusplus
}
#endif

// ============================================================================
// MOTOR CONTROL — DRV8871 with robust current protection
//
// Safety layers (in order of application):
//   1. Current-based PWM cap — limits duty so I ≤ 3.3A (DRV8871 max = 3.6A)
//   2. Slew rate limiter — PWM ramps up/down gradually (no inrush spikes)
//   3. Direction dead-time — brief coast on fwd↔rev to prevent shoot-through
//   4. Idle speed jump — starts at MIN_MOTOR_SPEED to overcome static friction
//   5. Thermal monitoring — ESP32 internal temp sensor
//   6. Duty cycle limiter — prevents sustained max current draw
//   7. Motor watchdog — auto-stop if no command received
//   8. Startup delay — lets power rails stabilize before driving
//   9. Emergency stop — kills everything on critical fault
// ============================================================================

MotorControl::MotorControl()
    : upsideDown(false), bootTime(0), lastCommandTime(0),
      fullSpeedStart(0), cooldownUntil(0), lastTemp(25.0f),
      lastTempCheck(0), thermalShutdown(false), emergencyStopped(false),
      currentLimitPWM(MAX_PWM), motorAIdle(true), motorBIdle(true),
      currentDutyA(0), currentDutyB(0), lastSlewA(0), lastSlewB(0),
      lastDirA(0), lastDirB(0), dirChangeTimeA(0), dirChangeTimeB(0),
      maxSafePWM(MAX_PWM) {}

void MotorControl::begin() {
    bootTime = millis();
    
    Serial.println("[MOTOR] Initializing DRV8871 drivers...");
    Serial.printf("[MOTOR] A: IN1=GPIO%d IN2=GPIO%d (inverted=%d)\n", MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_INVERTED);
    Serial.printf("[MOTOR] B: IN1=GPIO%d IN2=GPIO%d (inverted=%d)\n", MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_INVERTED);
    Serial.printf("[MOTOR] LEDC channels: A=ch%d,ch%d  B=ch%d,ch%d\n",
                  PWM_CHANNEL_A_IN1, PWM_CHANNEL_A_IN2,
                  PWM_CHANNEL_B_IN1, PWM_CHANNEL_B_IN2);

    // Read ESP32 internal temperature (ROM function, no driver needed)
    {
        uint8_t raw = temprature_sens_read();
        lastTemp = (raw - 32) / 1.8f;  // Convert Fahrenheit to Celsius
        Serial.printf("[SAFETY] ESP32 internal temp: %.1f°C\n", lastTemp);
    }

    // GPIO 25 and 26 are shared with DAC1/DAC2 on ESP32.
    // The DAC peripheral can silently override LEDC output on these pins.
    // Explicitly disable DAC BEFORE any GPIO/LEDC configuration.
    dac_output_disable(DAC_CHANNEL_1);  // GPIO 25 (Motor A IN2)
    dac_output_disable(DAC_CHANNEL_2);  // GPIO 26 (Motor B IN2)
    Serial.println("[MOTOR] DAC disabled on GPIO 25 (DAC1) and GPIO 26 (DAC2)");

    // Reclaim pins from any internal peripheral
    gpio_reset_pin((gpio_num_t)MOTOR_A_IN1);
    gpio_reset_pin((gpio_num_t)MOTOR_A_IN2);
    gpio_reset_pin((gpio_num_t)MOTOR_B_IN1);
    gpio_reset_pin((gpio_num_t)MOTOR_B_IN2);

    // Set maximum drive strength on all motor pins (40mA)
    gpio_set_drive_capability((gpio_num_t)MOTOR_A_IN1, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)MOTOR_A_IN2, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)MOTOR_B_IN1, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)MOTOR_B_IN2, GPIO_DRIVE_CAP_3);

    // Verify each motor pin can physically toggle (raw GPIO, before LEDC)
    Serial.println("[MOTOR] Raw GPIO pin verification...");
    const int motorPins[] = {MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_B_IN1, MOTOR_B_IN2};
    for (int i = 0; i < 4; i++) {
        int pin = motorPins[i];
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
        delayMicroseconds(100);
        int val = digitalRead(pin);
        digitalWrite(pin, LOW);
        Serial.printf("[MOTOR]   GPIO%d: %s\n", pin, val == HIGH ? "OK" : "FAIL — cannot output HIGH!");
    }

    // Configure LEDC PWM channels
    ledcSetup(PWM_CHANNEL_A_IN1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_A_IN2, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_B_IN1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_B_IN2, PWM_FREQ, PWM_RESOLUTION);

    // Attach GPIO pins permanently
    ledcAttachPin(MOTOR_A_IN1, PWM_CHANNEL_A_IN1);
    ledcAttachPin(MOTOR_A_IN2, PWM_CHANNEL_A_IN2);
    ledcAttachPin(MOTOR_B_IN1, PWM_CHANNEL_B_IN1);
    ledcAttachPin(MOTOR_B_IN2, PWM_CHANNEL_B_IN2);

    stop();

    // Compute the max safe PWM from DRV8871 current limit and motor specs
    maxSafePWM = computeMaxPWMFromCurrent(DRV8871_SAFE_CURRENT_A);
    currentLimitPWM = maxSafePWM;

    Serial.printf("[MOTOR] PWM: %dHz %d-bit, full range=0-%d\n", PWM_FREQ, PWM_RESOLUTION, MAX_PWM);
    Serial.printf("[MOTOR] Current limit: %.1fA safe / %.1fA max, R_motor=%.1f ohm\n",
                  DRV8871_SAFE_CURRENT_A, DRV8871_MAX_CURRENT_A, MOTOR_RESISTANCE_OHM);
    Serial.printf("[MOTOR] Computed max safe PWM = %d (%.0f%% duty)\n",
                  maxSafePWM, maxSafePWM * 100.0f / 255.0f);
    Serial.printf("[MOTOR] Slew rate: +%d/-%d per %dms, dead-time=%dms (non-blocking)\n",
                  SLEW_RATE_UP, SLEW_RATE_DOWN, SLEW_INTERVAL_MS, DIRECTION_DEADTIME_MS);
    Serial.printf("[MOTOR] Idle jump: %d PWM (overcome static friction)\n", MIN_MOTOR_SPEED);
    Serial.printf("[SAFETY] Thermal: warn=%dC, crit=%dC, resume=%dC\n",
                  (int)TEMP_WARNING_C, (int)TEMP_CRITICAL_C, (int)TEMP_RESUME_C);
    Serial.printf("[SAFETY] Duty: max_full=%ds, cooldown=%ds, throttle=%d%%\n",
                  MOTOR_MAX_FULL_MS/1000, MOTOR_COOLDOWN_MS/1000, MOTOR_THROTTLE_PCT);
    Serial.printf("[SAFETY] Watchdog: %dms, startup delay: %dms\n",
                  MOTOR_WATCHDOG_MS, MOTOR_STARTUP_DELAY);
    Serial.println("[MOTOR] Ready (all protection active)");
}

// ---- Temperature ----

float MotorControl::getESPTemperature() {
    return lastTemp;
}

bool MotorControl::isThermalThrottled() {
    return thermalShutdown || lastTemp >= TEMP_WARNING_C;
}

bool MotorControl::isDutyCycleThrottled() {
    return millis() < cooldownUntil;
}

bool MotorControl::isMotorsAllowed() {
    return !emergencyStopped && (millis() - bootTime >= MOTOR_STARTUP_DELAY);
}

// ---- Safety check — call every loop iteration ----

void MotorControl::safetyCheck() {
    unsigned long now = millis();
    
    // --- Temperature monitoring ---
    if (now - lastTempCheck >= TEMP_CHECK_MS) {
        lastTempCheck = now;
        {
            uint8_t raw = temprature_sens_read();
            lastTemp = (raw - 32) / 1.8f;
        }
        
        if (lastTemp >= TEMP_CRITICAL_C && !thermalShutdown) {
            thermalShutdown = true;
            emergencyStop("CRITICAL TEMP");
            Serial.printf("[SAFETY] *** THERMAL SHUTDOWN: %.1f°C >= %d°C ***\n", lastTemp, (int)TEMP_CRITICAL_C);
        } else if (lastTemp >= TEMP_WARNING_C && !thermalShutdown) {
            int throttled = (maxSafePWM * MOTOR_THROTTLE_PCT) / 100;
            if (currentLimitPWM != throttled) {
                currentLimitPWM = throttled;
                Serial.printf("[SAFETY] Thermal throttle: %.1f°C → limit=%d PWM\n", lastTemp, currentLimitPWM);
            }
        } else if (thermalShutdown && lastTemp <= TEMP_RESUME_C) {
            thermalShutdown = false;
            emergencyStopped = false;
            currentLimitPWM = maxSafePWM;
            Serial.printf("[SAFETY] Thermal OK: %.1f°C — motors re-enabled\n", lastTemp);
        } else if (lastTemp < TEMP_WARNING_C && !thermalShutdown) {
            currentLimitPWM = maxSafePWM;
        }
    }
    
    // --- Motor watchdog — stop motors if no command received ---
    if (lastCommandTime > 0 && (now - lastCommandTime > MOTOR_WATCHDOG_MS)) {
        stop();
        // Don't spam — only print once
        static bool watchdogPrinted = false;
        if (!watchdogPrinted) {
            Serial.println("[SAFETY] Motor watchdog: no command — motors stopped");
            watchdogPrinted = true;
        }
        // Reset when new command comes in (in setMotors)
    }
    
    // --- Duty cycle management ---
    if (cooldownUntil > 0 && now >= cooldownUntil) {
        cooldownUntil = 0;
        currentLimitPWM = maxSafePWM;
        Serial.println("[SAFETY] Duty cooldown complete — full power available");
    }
}

void MotorControl::emergencyStop(const char* reason) {
    emergencyStopped = true;
    stop();
    Serial.printf("[SAFETY] *** EMERGENCY STOP: %s ***\n", reason);
}

// ---- Safety limiter applied to every motor command ----

int MotorControl::applySafetyLimit(int speed) {
    if (!isMotorsAllowed()) return 0;
    
    int limit = currentLimitPWM;
    
    // During duty cycle cooldown, reduce limit further
    if (millis() < cooldownUntil) {
        int cooldownLimit = (maxSafePWM * MOTOR_THROTTLE_PCT) / 100;
        if (cooldownLimit < limit) limit = cooldownLimit;
    }
    
    return constrain(speed, -limit, limit);
}

// ---- Current-based PWM limiting ----
// I_motor = V_battery * (duty/255) / R_motor
// Solve for duty: duty = I_max * R_motor / V_battery * 255

int MotorControl::computeMaxPWMFromCurrent(float maxAmps) {
    float maxDutyFrac = (maxAmps * MOTOR_RESISTANCE_OHM) / BATTERY_VOLTAGE;
    int maxPWM = (int)(maxDutyFrac * 255.0f);
    maxPWM = constrain(maxPWM, MIN_MOTOR_SPEED, 255);
    return maxPWM;
}

int MotorControl::applyCurrentLimit(int speed) {
    // Clamp to the current-based safe PWM
    return constrain(speed, -maxSafePWM, maxSafePWM);
}

float MotorControl::getEstimatedCurrentA() {
    float duty = abs(currentDutyA) / 255.0f;
    return (BATTERY_VOLTAGE * duty) / MOTOR_RESISTANCE_OHM;
}

float MotorControl::getEstimatedCurrentB() {
    float duty = abs(currentDutyB) / 255.0f;
    return (BATTERY_VOLTAGE * duty) / MOTOR_RESISTANCE_OHM;
}

// ---- Slew rate limiter ----
// Gradually ramps PWM up/down to prevent current spikes

int MotorControl::applySlewRate(int current, int target, unsigned long &lastSlew) {
    unsigned long now = millis();
    unsigned long elapsed = now - lastSlew;
    if (elapsed < SLEW_INTERVAL_MS) return current;  // Not time yet
    lastSlew = now;
    
    // Scale step by elapsed time so slew works correctly regardless of call frequency
    // e.g. if called every 50ms with SLEW_INTERVAL_MS=10, steps scale by 5x
    int steps = (int)(elapsed / SLEW_INTERVAL_MS);
    if (steps < 1) steps = 1;
    if (steps > 10) steps = 10;  // Cap to prevent huge jumps after long pauses
    
    int diff = target - current;
    if (diff > 0) {
        // Ramping up
        return current + min(diff, SLEW_RATE_UP * steps);
    } else if (diff < 0) {
        // Ramping down (faster for safety)
        return current + max(diff, -SLEW_RATE_DOWN * steps);
    }
    return current;
}

// ---- Non-blocking motor drivers with full protection stack ----
// No delay() calls anywhere — main loop stays responsive.
//
// On idle→moving: jumps to MIN_MOTOR_SPEED to overcome static friction,
// then slew rate ramps smoothly to target.
//
// On direction reversal: outputs 0 for DIRECTION_DEADTIME_MS (non-blocking
// via timestamp), then ramps smoothly in the new direction.

void MotorControl::driveMotorA(int speed) {
    speed = applySafetyLimit(speed);
    speed = applyCurrentLimit(speed);
    
    unsigned long now = millis();
    int targetDir = (speed > 0) ? 1 : (speed < 0) ? -1 : 0;

    // Non-blocking dead-time: if direction reversal in progress, coast
    if (dirChangeTimeA > 0) {
        if (now - dirChangeTimeA < DIRECTION_DEADTIME_MS) {
            return;  // Still in dead-time coast — pins already LOW
        }
        dirChangeTimeA = 0;  // Dead-time complete
    }
    
    // Detect direction reversal (fwd↔rev, not to/from stop)
    if (lastDirA != 0 && targetDir != 0 && targetDir != lastDirA) {
        ledcWrite(PWM_CHANNEL_A_IN1, 0);
        ledcWrite(PWM_CHANNEL_A_IN2, 0);
        currentDutyA = 0;
        dirChangeTimeA = now;
        lastDirA = 0;
        motorAIdle = false;  // Not idle — in dead-time transition
        return;
    }

    if (speed > 0) {
        // Jump past static friction dead zone when starting from idle
        if (motorAIdle) {
            motorAIdle = false;
            currentDutyA = MIN_MOTOR_SPEED;
            lastSlewA = now;
        }
        int duty = applySlewRate(abs(currentDutyA), speed, lastSlewA);
        ledcWrite(PWM_CHANNEL_A_IN2, 0);
        ledcWrite(PWM_CHANNEL_A_IN1, duty);
        currentDutyA = duty;
        lastDirA = 1;
    } else if (speed < 0) {
        if (motorAIdle) {
            motorAIdle = false;
            currentDutyA = -MIN_MOTOR_SPEED;
            lastSlewA = now;
        }
        int duty = applySlewRate(abs(currentDutyA), -speed, lastSlewA);
        ledcWrite(PWM_CHANNEL_A_IN1, 0);
        ledcWrite(PWM_CHANNEL_A_IN2, duty);
        currentDutyA = -duty;
        lastDirA = -1;
    } else {
        // Ramp down to stop
        int duty = applySlewRate(abs(currentDutyA), 0, lastSlewA);
        if (duty <= 0) {
            ledcWrite(PWM_CHANNEL_A_IN1, 0);
            ledcWrite(PWM_CHANNEL_A_IN2, 0);
            currentDutyA = 0;
            motorAIdle = true;
            lastDirA = 0;
        } else {
            if (currentDutyA > 0) {
                ledcWrite(PWM_CHANNEL_A_IN2, 0);
                ledcWrite(PWM_CHANNEL_A_IN1, duty);
            } else {
                ledcWrite(PWM_CHANNEL_A_IN1, 0);
                ledcWrite(PWM_CHANNEL_A_IN2, duty);
            }
            currentDutyA = (currentDutyA > 0) ? duty : -duty;
        }
    }
}

void MotorControl::driveMotorB(int speed) {
    speed = applySafetyLimit(speed);
    speed = applyCurrentLimit(speed);
    
    unsigned long now = millis();
    int targetDir = (speed > 0) ? 1 : (speed < 0) ? -1 : 0;

    // Non-blocking dead-time
    if (dirChangeTimeB > 0) {
        if (now - dirChangeTimeB < DIRECTION_DEADTIME_MS) {
            return;
        }
        dirChangeTimeB = 0;
    }
    
    // Detect direction reversal
    if (lastDirB != 0 && targetDir != 0 && targetDir != lastDirB) {
        ledcWrite(PWM_CHANNEL_B_IN1, 0);
        ledcWrite(PWM_CHANNEL_B_IN2, 0);
        currentDutyB = 0;
        dirChangeTimeB = now;
        lastDirB = 0;
        motorBIdle = false;
        return;
    }

    if (speed > 0) {
        if (motorBIdle) {
            motorBIdle = false;
            currentDutyB = MIN_MOTOR_SPEED;
            lastSlewB = now;
        }
        int duty = applySlewRate(abs(currentDutyB), speed, lastSlewB);
        ledcWrite(PWM_CHANNEL_B_IN2, 0);
        ledcWrite(PWM_CHANNEL_B_IN1, duty);
        currentDutyB = duty;
        lastDirB = 1;
    } else if (speed < 0) {
        if (motorBIdle) {
            motorBIdle = false;
            currentDutyB = -MIN_MOTOR_SPEED;
            lastSlewB = now;
        }
        int duty = applySlewRate(abs(currentDutyB), -speed, lastSlewB);
        ledcWrite(PWM_CHANNEL_B_IN1, 0);
        ledcWrite(PWM_CHANNEL_B_IN2, duty);
        currentDutyB = -duty;
        lastDirB = -1;
    } else {
        int duty = applySlewRate(abs(currentDutyB), 0, lastSlewB);
        if (duty <= 0) {
            ledcWrite(PWM_CHANNEL_B_IN1, 0);
            ledcWrite(PWM_CHANNEL_B_IN2, 0);
            currentDutyB = 0;
            motorBIdle = true;
            lastDirB = 0;
        } else {
            if (currentDutyB > 0) {
                ledcWrite(PWM_CHANNEL_B_IN2, 0);
                ledcWrite(PWM_CHANNEL_B_IN1, duty);
            } else {
                ledcWrite(PWM_CHANNEL_B_IN1, 0);
                ledcWrite(PWM_CHANNEL_B_IN2, duty);
            }
            currentDutyB = (currentDutyB > 0) ? duty : -duty;
        }
    }
}

// ---- Public API ----

void MotorControl::setMotorA(int speed) {
    #if MOTOR_A_INVERTED
    speed = -speed;
    #endif
    speed = applyCalibration(speed, MOTOR_A_CALIBRATION);
    driveMotorA(speed);
}

void MotorControl::setMotorB(int speed) {
    #if MOTOR_B_INVERTED
    speed = -speed;
    #endif
    speed = applyCalibration(speed, MOTOR_B_CALIBRATION);
    driveMotorB(speed);
}

void MotorControl::setMotors(int speedA, int speedB) {
    lastCommandTime = millis();
    
    // When upside down, swap sides and invert
    if (upsideDown) {
        int tmp = speedA;
        speedA = -speedB;
        speedB = -tmp;
    }
    
    // --- Duty cycle tracking ---
    int maxReq = max(abs(speedA), abs(speedB));
    int fullThreshold = (maxSafePWM * 90) / 100;  // 90% of safe max = "full speed"
    unsigned long now = millis();
    
    if (maxReq >= fullThreshold) {
        if (fullSpeedStart == 0) fullSpeedStart = now;
        else if (now - fullSpeedStart > MOTOR_MAX_FULL_MS && cooldownUntil == 0) {
            cooldownUntil = now + MOTOR_COOLDOWN_MS;
            currentLimitPWM = (maxSafePWM * MOTOR_THROTTLE_PCT) / 100;
            fullSpeedStart = 0;
            Serial.printf("[SAFETY] Duty limit: %ds at full → throttle to %d PWM for %ds\n",
                          MOTOR_MAX_FULL_MS/1000, currentLimitPWM, MOTOR_COOLDOWN_MS/1000);
        }
    } else {
        fullSpeedStart = 0;
    }

    setMotorA(speedA);
    setMotorB(speedB);
}

void MotorControl::stop() {
    ledcWrite(PWM_CHANNEL_A_IN1, 0);
    ledcWrite(PWM_CHANNEL_A_IN2, 0);
    ledcWrite(PWM_CHANNEL_B_IN1, 0);
    ledcWrite(PWM_CHANNEL_B_IN2, 0);
    fullSpeedStart = 0;
    motorAIdle = true;
    motorBIdle = true;
    currentDutyA = 0;
    currentDutyB = 0;
    lastDirA = 0;
    lastDirB = 0;
    dirChangeTimeA = 0;
    dirChangeTimeB = 0;
}

void MotorControl::setUpsideDown(bool flipped) {
    if (flipped != upsideDown) {
        upsideDown = flipped;
        Serial.printf("[MOTOR] Orientation: %s\n", flipped ? "UPSIDE DOWN" : "RIGHT-SIDE UP");
    }
}

void MotorControl::standby(bool enable) {
    if (enable) stop();
}

// ---- Convenience wrappers ----

void MotorControl::forward(int speed)  { setMotors(speed, speed); }
void MotorControl::backward(int speed) { setMotors(-speed, -speed); }
void MotorControl::turnLeft(int speed)  { setMotors(-speed, speed); }
void MotorControl::turnRight(int speed) { setMotors(speed, -speed); }

// ---- Calibration helper ----

int MotorControl::applyCalibration(int speed, float cal) {
    if (speed == 0) return 0;
    int mag = (int)(abs(speed) * cal);
    if (abs(speed) > 20 && mag > 0 && mag < MIN_MOTOR_SPEED) {
        mag = MIN_MOTOR_SPEED;
    } else if (abs(speed) <= 20) {
        return 0;
    }
    return (speed > 0) ? mag : -mag;
}
