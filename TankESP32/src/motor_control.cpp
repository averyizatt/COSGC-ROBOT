#include "motor_control.h"
#include "config.h"
#include "driver/gpio.h"

MotorControl::MotorControl() {
    upsideDown = false;
    lastCommandA = 999;
    lastCommandB = 999;
}

void MotorControl::begin() {
    Serial.println("[MOTOR] Initializing 2x DRV8871 motor drivers...");
    
    upsideDown = false;
    
    Serial.printf("[MOTOR] Motor A: IN1=%d, IN2=%d\n", MOTOR_A_IN1, MOTOR_A_IN2);
    Serial.printf("[MOTOR] Motor B: IN1=%d, IN2=%d\n", MOTOR_B_IN1, MOTOR_B_IN2);
    
    // Reset all motor pins from any internal peripheral claim
    gpio_reset_pin((gpio_num_t)MOTOR_A_IN1);
    gpio_reset_pin((gpio_num_t)MOTOR_A_IN2);
    gpio_reset_pin((gpio_num_t)MOTOR_B_IN1);
    gpio_reset_pin((gpio_num_t)MOTOR_B_IN2);
    
    // === BOOT DRIVE TEST: raw GPIO, no LEDC ===
    // Forward a little, then reverse a bit more
    Serial.println("[MOTOR] --- BOOT TEST: forward then reverse (raw GPIO) ---");
    pinMode(MOTOR_A_IN1, OUTPUT);
    pinMode(MOTOR_A_IN2, OUTPUT);
    pinMode(MOTOR_B_IN1, OUTPUT);
    pinMode(MOTOR_B_IN2, OUTPUT);
    
    // Forward: A_IN2 + B_IN1 (Motor A is inverted, so its "forward" is IN2)
    Serial.println("[MOTOR] FORWARD...");
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);  // Motor A forward (inverted)
    digitalWrite(MOTOR_B_IN1, HIGH);  // Motor B forward
    digitalWrite(MOTOR_B_IN2, LOW);
    delay(500);
    
    // Stop briefly
    digitalWrite(MOTOR_A_IN1, LOW); digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN1, LOW); digitalWrite(MOTOR_B_IN2, LOW);
    delay(200);
    
    // Reverse: A_IN1 + B_IN2
    Serial.println("[MOTOR] REVERSE...");
    digitalWrite(MOTOR_A_IN1, HIGH);  // Motor A reverse (inverted)
    digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, HIGH);  // Motor B reverse
    delay(700);
    
    // Stop
    digitalWrite(MOTOR_A_IN1, LOW); digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN1, LOW); digitalWrite(MOTOR_B_IN2, LOW);
    Serial.println("[MOTOR] --- BOOT TEST DONE ---");
    Serial.println("[MOTOR] Both tracks should have moved forward then back.");
    Serial.println("[MOTOR] If only one side moved forward, that side's forward pin is dead.");
    
    // Setup PWM channels and permanently attach pins
    ledcSetup(PWM_CHANNEL_A_IN1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_A_IN2, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_B_IN1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_B_IN2, PWM_FREQ, PWM_RESOLUTION);
    
    ledcAttachPin(MOTOR_A_IN1, PWM_CHANNEL_A_IN1);
    ledcAttachPin(MOTOR_A_IN2, PWM_CHANNEL_A_IN2);
    ledcAttachPin(MOTOR_B_IN1, PWM_CHANNEL_B_IN1);
    ledcAttachPin(MOTOR_B_IN2, PWM_CHANNEL_B_IN2);
    
    // All channels to 0 (coast)
    ledcWrite(PWM_CHANNEL_A_IN1, 0);
    ledcWrite(PWM_CHANNEL_A_IN2, 0);
    ledcWrite(PWM_CHANNEL_B_IN1, 0);
    ledcWrite(PWM_CHANNEL_B_IN2, 0);
    
    Serial.printf("[MOTOR] PWM: %dHz, %d-bit resolution, pins permanently attached\n", PWM_FREQ, PWM_RESOLUTION);
    Serial.println("[MOTOR] DRV8871 drivers ready");
}

void MotorControl::setupPWM() {
    // PWM channels are set up in begin()
}

// Apply calibration and minimum speed
static int applyMotorCalibration(int speed, float calibration) {
    if (speed == 0) return 0;
    
    // Apply calibration factor
    int calibratedSpeed = (int)(abs(speed) * calibration);
    
    // Enforce minimum speed (motors won't spin below this)
    if (calibratedSpeed > 0 && calibratedSpeed < MIN_MOTOR_SPEED) {
        calibratedSpeed = MIN_MOTOR_SPEED;
    }
    
    // Preserve direction
    return (speed > 0) ? calibratedSpeed : -calibratedSpeed;
}

// Soft-start helper: ramps PWM from current to target over ~10ms
// Prevents sudden inrush current that drops the voltage rail and causes resets
static int _lastPwmA1 = 0, _lastPwmA2 = 0;
static int _lastPwmB1 = 0, _lastPwmB2 = 0;

static void softWrite(int channel, int target, int &lastPwm) {
    // If starting from 0 or small change, just write directly
    if (lastPwm == 0 || abs(target - lastPwm) <= 30) {
        ledcWrite(channel, target);
        lastPwm = target;
        return;
    }
    // Ramp for large jumps to limit inrush current
    int current = lastPwm;
    int step = (target > current) ? 20 : -20;
    while (abs(target - current) > 20) {
        current += step;
        ledcWrite(channel, abs(current));
        delay(3);
    }
    ledcWrite(channel, target);
    lastPwm = target;
}

void MotorControl::setMotorA(int speed) {
    #if MOTOR_A_INVERTED
    speed = -speed;
    #endif

    speed = applyMotorCalibration(speed, MOTOR_A_CALIBRATION);
    speed = constrain(speed, -MAX_PWM, MAX_PWM);
    
    if (speed > 0) {
        // Forward: IN1=PWM, IN2=0
        softWrite(PWM_CHANNEL_A_IN1, speed, _lastPwmA1);
        if (_lastPwmA2 != 0) { ledcWrite(PWM_CHANNEL_A_IN2, 0); _lastPwmA2 = 0; }
    } else if (speed < 0) {
        // Reverse: IN1=0, IN2=PWM
        if (_lastPwmA1 != 0) { ledcWrite(PWM_CHANNEL_A_IN1, 0); _lastPwmA1 = 0; }
        softWrite(PWM_CHANNEL_A_IN2, -speed, _lastPwmA2);
    } else {
        // Coast: both 0
        ledcWrite(PWM_CHANNEL_A_IN1, 0); _lastPwmA1 = 0;
        ledcWrite(PWM_CHANNEL_A_IN2, 0); _lastPwmA2 = 0;
    }
}

void MotorControl::setMotorB(int speed) {
    #if MOTOR_B_INVERTED
    speed = -speed;
    #endif

    speed = applyMotorCalibration(speed, MOTOR_B_CALIBRATION);
    speed = constrain(speed, -MAX_PWM, MAX_PWM);
    
    // Motor B uses direct ledcWrite (no softWrite) for reliability
    if (speed > 0) {
        // Forward: IN1=PWM, IN2=0
        ledcWrite(PWM_CHANNEL_B_IN2, 0);
        ledcWrite(PWM_CHANNEL_B_IN1, speed);
        _lastPwmB1 = speed; _lastPwmB2 = 0;
    } else if (speed < 0) {
        // Reverse: IN1=0, IN2=PWM
        ledcWrite(PWM_CHANNEL_B_IN1, 0);
        ledcWrite(PWM_CHANNEL_B_IN2, -speed);
        _lastPwmB1 = 0; _lastPwmB2 = -speed;
    } else {
        // Coast: both 0
        ledcWrite(PWM_CHANNEL_B_IN1, 0);
        ledcWrite(PWM_CHANNEL_B_IN2, 0);
        _lastPwmB1 = 0; _lastPwmB2 = 0;
    }
}

void MotorControl::setMotors(int speedA, int speedB) {
    // TANK MODE: When upside down, the robot can still drive!
    // We swap left/right motors AND invert both directions
    // This makes "forward" still go toward the front of the robot
    if (upsideDown) {
        int tempA = speedA;
        speedA = -speedB;  // Swap and invert
        speedB = -tempA;
    }

    // Avoid PWM detach/attach churn when command is unchanged
    if (speedA == lastCommandA && speedB == lastCommandB) {
        return;
    }
    lastCommandA = speedA;
    lastCommandB = speedB;
    
    setMotorA(speedA);
    setMotorB(speedB);
}

void MotorControl::setUpsideDown(bool flipped) {
    if (flipped != upsideDown) {
        upsideDown = flipped;
        if (flipped) {
            Serial.println("[MOTOR] UPSIDE DOWN MODE - Motor commands inverted!");
        } else {
            Serial.println("[MOTOR] RIGHT-SIDE UP - Normal motor commands");
        }
    }
}

void MotorControl::stop() {
    // All PWM to 0 (coast) - pins stay attached
    ledcWrite(PWM_CHANNEL_A_IN1, 0);
    ledcWrite(PWM_CHANNEL_A_IN2, 0);
    ledcWrite(PWM_CHANNEL_B_IN1, 0);
    ledcWrite(PWM_CHANNEL_B_IN2, 0);

    // Reset soft-start tracking so next command ramps from 0
    _lastPwmA1 = 0; _lastPwmA2 = 0;
    _lastPwmB1 = 0; _lastPwmB2 = 0;

    lastCommandA = 0;
    lastCommandB = 0;
}

void MotorControl::standby(bool enable) {
    // DRV8871 has no standby pin - just stop motors
    if (enable) {
        stop();
    }
}

void MotorControl::forward(int speed) {
    setMotors(speed, speed);
}

void MotorControl::backward(int speed) {
    setMotors(-speed, -speed);
}

void MotorControl::turnLeft(int speed) {
    setMotors(-speed, speed);
}

void MotorControl::turnRight(int speed) {
    setMotors(speed, -speed);
}

void MotorControl::motorTest() {
    Serial.println("\n[MOTOR] ========== PIN-BY-PIN MOTOR TEST ==========");
    int testPwm = 120;  // Low power to avoid brownout
    
    // Test Motor A IN1 (forward for A)
    Serial.printf("[MOTOR] A-IN1 (GPIO%d, ch%d) PWM=%d...\n", MOTOR_A_IN1, PWM_CHANNEL_A_IN1, testPwm);
    ledcWrite(PWM_CHANNEL_A_IN1, testPwm);
    ledcWrite(PWM_CHANNEL_A_IN2, 0);
    delay(600);
    ledcWrite(PWM_CHANNEL_A_IN1, 0);
    delay(300);
    
    // Test Motor A IN2 (reverse for A)
    Serial.printf("[MOTOR] A-IN2 (GPIO%d, ch%d) PWM=%d...\n", MOTOR_A_IN2, PWM_CHANNEL_A_IN2, testPwm);
    ledcWrite(PWM_CHANNEL_A_IN1, 0);
    ledcWrite(PWM_CHANNEL_A_IN2, testPwm);
    delay(600);
    ledcWrite(PWM_CHANNEL_A_IN2, 0);
    delay(300);
    
    // Test Motor B IN1 (forward for B)
    Serial.printf("[MOTOR] B-IN1 (GPIO%d, ch%d) PWM=%d...\n", MOTOR_B_IN1, PWM_CHANNEL_B_IN1, testPwm);
    ledcWrite(PWM_CHANNEL_B_IN1, testPwm);
    ledcWrite(PWM_CHANNEL_B_IN2, 0);
    delay(600);
    ledcWrite(PWM_CHANNEL_B_IN1, 0);
    delay(300);
    
    // Test Motor B IN2 (reverse for B)
    Serial.printf("[MOTOR] B-IN2 (GPIO%d, ch%d) PWM=%d...\n", MOTOR_B_IN2, PWM_CHANNEL_B_IN2, testPwm);
    ledcWrite(PWM_CHANNEL_B_IN1, 0);
    ledcWrite(PWM_CHANNEL_B_IN2, testPwm);
    delay(600);
    ledcWrite(PWM_CHANNEL_B_IN2, 0);
    
    // Stop all
    ledcWrite(PWM_CHANNEL_A_IN1, 0);
    ledcWrite(PWM_CHANNEL_A_IN2, 0);
    ledcWrite(PWM_CHANNEL_B_IN1, 0);
    ledcWrite(PWM_CHANNEL_B_IN2, 0);
    _lastPwmA1 = 0; _lastPwmA2 = 0;
    _lastPwmB1 = 0; _lastPwmB2 = 0;
    
    Serial.println("[MOTOR] ========== TEST COMPLETE ==========");
    Serial.println("[MOTOR] You should have seen 4 movements:");
    Serial.println("[MOTOR]   1: Motor A one direction");
    Serial.println("[MOTOR]   2: Motor A other direction");
    Serial.println("[MOTOR]   3: Motor B one direction");
    Serial.println("[MOTOR]   4: Motor B other direction");
    Serial.println("[MOTOR] If #3 was missing, GPIO7/channel2 is broken\n");
}
