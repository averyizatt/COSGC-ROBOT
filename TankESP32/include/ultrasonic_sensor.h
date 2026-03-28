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
    
    // Read distance in centimeters (median-filtered, with health monitoring)
    float readDistance();
    
    // Check if obstacle is within threshold
    bool obstacleDetected(float threshold);
    
    // Sensor health
    SensorHealth getHealth() { return _health; }
    bool isHealthy() { return _health != SENSOR_FAILED; }

private:
    int _trigPin;
    int _echoPin;
    
    // Single raw read using interrupt-based echo measurement
    float _readOnce();
    
    // 3-sample median filter
    float _medianBuf[3];
    int _medianIdx;
    bool _medianFilled;
    float _median3(float a, float b, float c);
    
    // Health monitoring
    SensorHealth _health;
    int _consecutiveFails;      // Sequential failed reads
    float _lastValidReading;    // Last known-good value
    int _stuckCount;            // How many identical readings in a row
    static constexpr int FAIL_THRESHOLD = 10;  // Consecutive fails → FAILED
    static constexpr int STUCK_THRESHOLD = 20; // Identical readings → FAILED
    static constexpr float STUCK_TOLERANCE = 0.5f; // cm tolerance for "identical"
};

#endif // ULTRASONIC_SENSOR_H
