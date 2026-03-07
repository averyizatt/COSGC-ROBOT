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
    : _trigPin(trigPin), _echoPin(echoPin) {
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

float UltrasonicSensor::readDistance() {
    // Try up to 3 times to get a valid reading
    for (int attempt = 0; attempt < 3; attempt++) {
        float d = _readOnce();
        if (d > 0) {
            return d;
        }
        delayMicroseconds(200);  // Brief pause between retries
    }
    return -1.0f;
}

bool UltrasonicSensor::obstacleDetected(float threshold) {
    float distance = readDistance();
    if (distance < 0) {
        return false;
    }
    return distance < threshold;
}
