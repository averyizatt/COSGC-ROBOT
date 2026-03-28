#include "ultrasonic_sensor.h"
#include "config.h"
#include "driver/gpio.h"

// ---- Interrupt-based echo measurement ----
// pulseIn() busy-waits and gets disrupted by BLE/RTOS interrupts on ESP32-S3.
// Instead, we use attachInterrupt to capture the echo pulse edges precisely
// via hardware interrupt timestamps, which aren't affected by task scheduling.

// Pin for the ISR to digitalRead (set before each measurement)
static volatile int           _isrPin    = -1;
static volatile unsigned long _echoStart = 0;
static volatile unsigned long _echoEnd   = 0;
static volatile bool          _echoDone  = false;

static void IRAM_ATTR echoISR() {
    if (digitalRead(_isrPin) == HIGH) {
        _echoStart = micros();
        _echoDone  = false;
    } else {
        _echoEnd  = micros();
        _echoDone = true;
    }
}

UltrasonicSensor::UltrasonicSensor(int trigPin, int echoPin)
    : _trigPin(trigPin), _echoPin(echoPin),
      _medianIdx(0), _medianFilled(false),
      _health(SENSOR_OK), _consecutiveFails(0),
      _lastValidReading(30.0f), _stuckCount(0) {
    _medianBuf[0] = _medianBuf[1] = _medianBuf[2] = 0.0f;
}

void UltrasonicSensor::begin() {
    // Force-reclaim pin from any internal peripheral (e.g. SPI flash on GPIO11)
    gpio_reset_pin((gpio_num_t)_echoPin);
    gpio_reset_pin((gpio_num_t)_trigPin);

    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);
    digitalWrite(_trigPin, LOW);
    delay(50);  // Let sensor settle
}

// Single read using interrupt-based echo capture
float UltrasonicSensor::_readOnce() {
    // Set up ISR for this echo pin
    _isrPin   = _echoPin;
    _echoDone = false;
    _echoStart = 0;
    _echoEnd   = 0;

    attachInterrupt(digitalPinToInterrupt(_echoPin), echoISR, CHANGE);

    // Send trigger pulse (10µs HIGH)
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);

    // Wait for echo with timeout (non-busy: yield to RTOS)
    unsigned long waitStart = micros();
    while (!_echoDone) {
        if ((micros() - waitStart) > TIMEOUT_US + 1000) {
            detachInterrupt(digitalPinToInterrupt(_echoPin));
            return -1.0f;  // Timeout
        }
        delayMicroseconds(10);  // Yield briefly
    }

    detachInterrupt(digitalPinToInterrupt(_echoPin));

    unsigned long duration = _echoEnd - _echoStart;
    if (duration == 0 || duration > TIMEOUT_US) {
        return -1.0f;
    }

    float distance = (duration * 0.0343f) / 2.0f;
    if (distance < 2.0f || distance > 400.0f) {
        return -1.0f;
    }
    return distance;
}

float UltrasonicSensor::_median3(float a, float b, float c) {
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}

float UltrasonicSensor::readDistance() {
    float raw = _readOnce();
    
    // --- Health monitoring ---
    if (raw < 0) {
        _consecutiveFails++;
        if (_consecutiveFails >= FAIL_THRESHOLD) {
            if (_health != SENSOR_FAILED) {
                _health = SENSOR_FAILED;
                Serial.printf("[SENSOR] Pin %d FAILED (%d consecutive failures)\n",
                              _echoPin, _consecutiveFails);
            }
        } else if (_consecutiveFails >= 3) {
            _health = SENSOR_DEGRADED;
        }
        return -1.0f;  // Don't feed bad data into median
    }
    
    // Valid reading — check for stuck sensor (constant value)
    if (fabs(raw - _lastValidReading) < STUCK_TOLERANCE) {
        _stuckCount++;
        if (_stuckCount >= STUCK_THRESHOLD) {
            if (_health != SENSOR_FAILED) {
                _health = SENSOR_FAILED;
                Serial.printf("[SENSOR] Pin %d STUCK at %.1fcm (%d identical reads)\n",
                              _echoPin, raw, _stuckCount);
            }
            return -1.0f;
        }
    } else {
        _stuckCount = 0;
    }
    _lastValidReading = raw;
    _consecutiveFails = 0;
    if (_health == SENSOR_DEGRADED) _health = SENSOR_OK;
    // Note: FAILED→OK recovery requires power cycle or begin() call
    
    // --- 3-sample median filter ---
    _medianBuf[_medianIdx] = raw;
    _medianIdx = (_medianIdx + 1) % 3;
    if (!_medianFilled && _medianIdx == 0) _medianFilled = true;
    
    if (!_medianFilled) {
        return raw;  // Not enough samples yet
    }
    return _median3(_medianBuf[0], _medianBuf[1], _medianBuf[2]);
}

bool UltrasonicSensor::obstacleDetected(float threshold) {
    float distance = readDistance();
    if (distance < 0) {
        return false;
    }
    return distance < threshold;
}
