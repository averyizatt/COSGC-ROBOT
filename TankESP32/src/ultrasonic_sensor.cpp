#include "ultrasonic_sensor.h"
#include "config.h"
#include "driver/gpio.h"

UltrasonicSensor::UltrasonicSensor(int trigPin, int echoPin)
    : _trigPin(trigPin), _echoPin(echoPin),
      _medianIdx(0), _medianFilled(false),
      _health(SENSOR_OK), _consecutiveFails(0),
      _lastValidReading(30.0f), _stuckCount(0) {
    _medianBuf[0] = _medianBuf[1] = _medianBuf[2] = 0.0f;
}

void UltrasonicSensor::begin() {
    gpio_reset_pin((gpio_num_t)_echoPin);
    gpio_reset_pin((gpio_num_t)_trigPin);

    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);
    digitalWrite(_trigPin, LOW);
    delay(50);
}

// Single read using polling — reliable on ESP32 without BLE interference.
float UltrasonicSensor::_readOnce() {
    // Send 10µs trigger pulse
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(4);
    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);

    // Wait for echo pin to go HIGH (rising edge), timeout 5ms
    unsigned long t = micros();
    while (digitalRead(_echoPin) == LOW) {
        if (micros() - t > 5000UL) return -1.0f;
    }
    unsigned long echoStart = micros();

    // Wait for echo pin to go LOW (falling edge), timeout = TIMEOUT_US
    while (digitalRead(_echoPin) == HIGH) {
        if (micros() - echoStart > (unsigned long)TIMEOUT_US) return -1.0f;
    }
    unsigned long echoEnd = micros();

    unsigned long duration = echoEnd - echoStart;
    if (duration == 0) return -1.0f;

    float distance = (duration * 0.0343f) / 2.0f;
    if (distance < 2.0f || distance > 400.0f) return -1.0f;
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
        // Reading changed — reset stuck counter and recover health
        _stuckCount = 0;
        if (_health == SENSOR_FAILED) {
            _health = SENSOR_OK;
            Serial.printf("[SENSOR] Pin %d RECOVERED (reading changed to %.1fcm)\n", _echoPin, raw);
        }
    }
    _lastValidReading = raw;
    _consecutiveFails = 0;
    if (_health == SENSOR_DEGRADED) _health = SENSOR_OK;
    
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
