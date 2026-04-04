#include "ultrasonic_sensor.h"
#include "config.h"
#include "driver/gpio.h"

UltrasonicSensor::UltrasonicSensor(int trigPin, int echoPin)
    : _trigPin(trigPin), _echoPin(echoPin),
      _ambientTempC(25.0f),
      _medianIdx(0), _medianFilled(false),
      _health(SENSOR_OK), _consecutiveFails(0),
      _lastValidReading(30.0f), _pendingReading(-1.0f), _stuckCount(0) {
    for (int i = 0; i < MEDIAN_SIZE; i++) _medianBuf[i] = 0.0f;
}

void UltrasonicSensor::begin() {
    gpio_reset_pin((gpio_num_t)_echoPin);
    gpio_reset_pin((gpio_num_t)_trigPin);

    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);
    digitalWrite(_trigPin, LOW);
    delay(50);
}

// Single ping — one trigger + echo wait.
// Echo-rise timeout raised to 10ms for weak/absorbed returns on sand.
float UltrasonicSensor::_readOnce() {
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(4);
    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);

    // Wait for echo HIGH — 10ms timeout (up from 5ms) for weak sand returns
    unsigned long t = micros();
    while (digitalRead(_echoPin) == LOW) {
        if (micros() - t > 10000UL) return -1.0f;
    }
    unsigned long echoStart = micros();

    // Wait for echo LOW
    while (digitalRead(_echoPin) == HIGH) {
        if (micros() - echoStart > (unsigned long)TIMEOUT_US) return -1.0f;
    }
    unsigned long duration = micros() - echoStart;
    if (duration == 0) return -1.0f;

    // Temperature-compensated speed of sound: v = 331.3 + 0.606*T (m/s)
    // Divide by 20000 instead of hardcoding 0.0343 to incorporate temp
    float speedCmUs = (331.3f + 0.606f * _ambientTempC) / 10000.0f; // cm/µs
    float distance = (duration * speedCmUs) / 2.0f;

    if (distance < 2.0f || distance > 400.0f) return -1.0f;
    return distance;
}

// Triple-ping: fires 3 pings back-to-back with a 2ms gap, returns median.
// Rejects single-bounce failures (sand absorption, beam scatter).
// Adds ~30ms per readDistance() call but greatly reduces bad reads.
float UltrasonicSensor::_readTriple() {
    float r[3];
    int validCount = 0;
    for (int i = 0; i < 3; i++) {
        r[i] = _readOnce();
        if (r[i] > 0) validCount++;
        delayMicroseconds(2000);  // 2ms between pings — avoids echo overlap
    }
    if (validCount == 0) return -1.0f;
    if (validCount == 1) {
        // Only one valid ping — return it but treat as low-confidence
        for (int i = 0; i < 3; i++) if (r[i] > 0) return r[i];
    }
    // Sort and pick median of whichever are valid
    // Simple insertion sort on 3 elements
    for (int i = 1; i < 3; i++) {
        for (int j = i; j > 0 && r[j-1] > r[j]; j--) {
            float tmp = r[j]; r[j] = r[j-1]; r[j-1] = tmp;
        }
    }
    // Return middle valid value
    if (validCount == 3) return r[1];  // True median
    // Two valid: return the smaller (more conservative for obstacle avoidance)
    for (int i = 0; i < 3; i++) if (r[i] > 0) return r[i];
    return -1.0f;
}

// N-sample median (sorts a copy of buf).
float UltrasonicSensor::_medianN(float* buf, int n) {
    float tmp[MEDIAN_SIZE];
    for (int i = 0; i < n; i++) tmp[i] = buf[i];
    for (int i = 1; i < n; i++) {
        for (int j = i; j > 0 && tmp[j-1] > tmp[j]; j--) {
            float t = tmp[j]; tmp[j] = tmp[j-1]; tmp[j-1] = t;
        }
    }
    return tmp[n / 2];
}

float UltrasonicSensor::readDistance() {
    // Get a triple-ping median for this cycle
    float raw = _readTriple();

    // --- Health monitoring ---
    if (raw < 0) {
        _consecutiveFails++;
        if (_consecutiveFails >= FAIL_THRESHOLD) {
            if (_health != SENSOR_FAILED) {
                _health = SENSOR_FAILED;
                Serial.printf("[SENSOR] Pin %d FAILED (%d consecutive triple-pings failed)\n",
                              _echoPin, _consecutiveFails);
            }
        } else if (_consecutiveFails >= 3) {
            _health = SENSOR_DEGRADED;
        }
        return -1.0f;
    }

    // --- Spike rejection (sand grain / dust false echo) ---
    // If reading jumps >SPIKE_THRESHOLD from last accepted value, hold as pending.
    // Accept it only if the NEXT reading agrees (within 15cm of the candidate).
    float jump = fabs(raw - _lastValidReading);
    if (jump > SPIKE_THRESHOLD && _lastValidReading > 0) {
        if (_pendingReading < 0) {
            // First time seeing this spike — record and wait
            _pendingReading = raw;
            return _lastValidReading;  // Hold last good value this cycle
        } else if (fabs(raw - _pendingReading) < 15.0f) {
            // Confirmed — the jump is real (two consecutive readings agree)
            _pendingReading = -1.0f;
            // Fall through to accept raw
        } else {
            // Unconfirmed fluctuation — keep waiting
            _pendingReading = raw;
            return _lastValidReading;
        }
    } else {
        _pendingReading = -1.0f;  // No spike — clear pending
    }

    // --- Stuck sensor detection ---
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
        if (_health == SENSOR_FAILED) {
            _health = SENSOR_OK;
            Serial.printf("[SENSOR] Pin %d RECOVERED (reading changed to %.1fcm)\n", _echoPin, raw);
        }
    }

    _lastValidReading = raw;
    _consecutiveFails = 0;
    if (_health == SENSOR_DEGRADED) _health = SENSOR_OK;

    // --- 5-sample rolling median (wider window = smoother on rough terrain) ---
    _medianBuf[_medianIdx] = raw;
    _medianIdx = (_medianIdx + 1) % MEDIAN_SIZE;
    if (!_medianFilled && _medianIdx == 0) _medianFilled = true;

    if (!_medianFilled) return raw;
    return _medianN(_medianBuf, MEDIAN_SIZE);
}

bool UltrasonicSensor::obstacleDetected(float threshold) {
    float distance = readDistance();
    if (distance < 0) return false;
    return distance < threshold;
}

