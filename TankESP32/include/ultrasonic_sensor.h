#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>

class UltrasonicSensor {
public:
    UltrasonicSensor(int trigPin, int echoPin);
    void begin();
    
    // Read distance in centimeters (interrupt-based, retries on failure)
    float readDistance();
    
    // Check if obstacle is within threshold
    bool obstacleDetected(float threshold);

private:
    int _trigPin;
    int _echoPin;
    
    // Single raw read using interrupt-based echo measurement
    float _readOnce();
};

#endif // ULTRASONIC_SENSOR_H
