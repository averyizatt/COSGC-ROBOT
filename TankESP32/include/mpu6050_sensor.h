#ifndef MPU6050_SENSOR_H
#define MPU6050_SENSOR_H

#include <Arduino.h>
#include <Wire.h>

class MPU6050Sensor {
public:
    MPU6050Sensor();
    bool begin();
    
    // Calibrate on flat surface - call once after begin() while robot is still
    void calibrate(int samples = 200);
    
    // Read sensor data
    void update();
    
    // Get accelerometer data (in g's)
    float getAccelX();
    float getAccelY();
    float getAccelZ();
    
    // Get gyroscope data (in degrees/sec)
    float getGyroX();
    float getGyroY();
    float getGyroZ();
    
    // Get temperature (in Celsius)
    float getTemperature();
    
    // Check if sensor is connected
    bool isConnected();

private:
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    uint8_t readRegisters(uint8_t reg, uint8_t count, uint8_t* dest);  // returns bytes read
    
    // Raw sensor data
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;
    int16_t temperature;
    
    // Conversion factors
    float accelScale;
    float gyroScale;
    
    // Calibration offsets (raw units)
    int32_t accelOffX, accelOffY, accelOffZ;
    int32_t gyroOffX, gyroOffY, gyroOffZ;
    bool calibrated;
    
    bool initialized;
};

#endif // MPU6050_SENSOR_H
