#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>

// Sensor health status
enum SensorHealth {
    SENSOR_OK = 0,       // Normal operation
    SENSOR_DEGRADED,     // Some failed reads but still usable
    SENSOR_FAILED        // Stuck or consistently returning bad data
};

class UltrasonicSensor {
public:
    UltrasonicSensor(int trigPin, int echoPin);
    void begin();
    
    // Read distance in centimeters (median-filtered, spike-rejected, health monitored)
    float readDistance();
    
    // Feed ambient temperature (°C) for speed-of-sound compensation.
    // Call once per sensor update cycle if MPU6050 temp is available.
    void setAmbientTemp(float tempC) { _ambientTempC = tempC; }

    // Check if obstacle is within threshold
    bool obstacleDetected(float threshold);
    
    // Sensor health
    SensorHealth getHealth() { return _health; }
    bool isHealthy() { return _health != SENSOR_FAILED; }

private:
    int _trigPin;
    int _echoPin;
    float _ambientTempC;   // For speed-of-sound correction (default 25°C)
    
    // Single raw read — one trigger pulse + echo wait
    float _readOnce();

    // Triple-ping with immediate median to reject single-bounce failures
    float _readTriple();

    // 5-sample rolling median filter (wider window for sandy terrain)
    static constexpr int MEDIAN_SIZE = 5;
    float _medianBuf[MEDIAN_SIZE];
    int _medianIdx;
    bool _medianFilled;
    float _medianN(float* buf, int n);
    
    // Health monitoring
    SensorHealth _health;
    int _consecutiveFails;
    float _lastValidReading;   // Last accepted reading (after spike check)
    float _pendingReading;     // Candidate reading waiting for confirmation
    int _stuckCount;

    static constexpr int FAIL_THRESHOLD   = 10;    // Consecutive fails → FAILED
    static constexpr int STUCK_THRESHOLD  = 60;    // Identical readings → FAILED
    static constexpr float STUCK_TOLERANCE = 0.5f; // cm tolerance for "identical"
    static constexpr float SPIKE_THRESHOLD = 40.0f; // cm jump requiring confirmation
};

#endif // ULTRASONIC_SENSOR_H
