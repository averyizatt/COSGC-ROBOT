#include "mpu6050_sensor.h"
#include "config.h"

// MPU6050 Register Addresses
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_WHO_AM_I     0x75

MPU6050Sensor::MPU6050Sensor() {
    accelX = accelY = accelZ = 0;
    gyroX = gyroY = gyroZ = 0;
    temperature = 0;
    initialized = false;
    calibrated = false;
    accelOffX = accelOffY = accelOffZ = 0;
    gyroOffX = gyroOffY = gyroOffZ = 0;
    
    // Scale factors for ±2g and ±250°/s (default ranges)
    accelScale = 2.0 / 32768.0;  // LSB/g
    gyroScale = 250.0 / 32768.0;  // LSB/(°/s)
}

bool MPU6050Sensor::begin() {
    // Initialize I2C with custom pins (don't call Wire.end() first - causes issues)
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000); // 100kHz I2C speed for better compatibility
    
    delay(100); // Wait for sensor to stabilize
    
    // Try to detect sensor with timeout
    unsigned long timeout = millis() + 1000;  // 1 second timeout for detection
    bool found = false;
    
    while (millis() < timeout) {
        if (isConnected()) {
            found = true;
            break;
        }
        delay(50);
    }
    
    if (!found) {
        Serial.println("MPU6050 not found - continuing without IMU");
        initialized = false;
        return false;
    }
    
    // Wake up the MPU6050 (it starts in sleep mode)
    writeRegister(MPU6050_PWR_MGMT_1, 0x00);
    delay(100);
    
    // Configure gyroscope (±250°/s)
    writeRegister(MPU6050_GYRO_CONFIG, 0x00);
    
    // Configure accelerometer (±2g)
    writeRegister(MPU6050_ACCEL_CONFIG, 0x00);
    
    initialized = true;
    Serial.println("MPU6050 initialized successfully");
    return true;
}

void MPU6050Sensor::calibrate(int samples) {
    if (!initialized) {
        Serial.println("[IMU] Cannot calibrate - not initialized");
        return;
    }
    
    Serial.printf("[IMU] Calibrating on flat surface (%d samples)...\n", samples);
    Serial.println("[IMU] Keep robot still and level!");
    
    int32_t sumAx = 0, sumAy = 0, sumAz = 0;
    int32_t sumGx = 0, sumGy = 0, sumGz = 0;
    
    // Discard first 50 readings to let sensor settle
    for (int i = 0; i < 50; i++) {
        update();
        delay(5);
    }
    
    // Collect calibration samples
    for (int i = 0; i < samples; i++) {
        update();
        sumAx += accelX;
        sumAy += accelY;
        sumAz += accelZ;
        sumGx += gyroX;
        sumGy += gyroY;
        sumGz += gyroZ;
        delay(5);  // ~5ms between samples = ~1 second total
    }
    
    // Average offsets
    accelOffX = sumAx / samples;
    accelOffY = sumAy / samples;
    // For Z axis: subtract expected 1g value (16384 at ±2g range)
    // so that when flat, calibrated Z reads ~1g
    accelOffZ = (sumAz / samples) - (int32_t)(1.0f / accelScale);
    
    gyroOffX = sumGx / samples;
    gyroOffY = sumGy / samples;
    gyroOffZ = sumGz / samples;
    
    calibrated = true;
    
    Serial.println("[IMU] Calibration complete!");
    Serial.printf("[IMU] Accel offsets: X=%ld Y=%ld Z=%ld\n", accelOffX, accelOffY, accelOffZ);
    Serial.printf("[IMU] Gyro offsets:  X=%ld Y=%ld Z=%ld\n", gyroOffX, gyroOffY, gyroOffZ);
}

bool MPU6050Sensor::isConnected() {
    uint8_t whoAmI = readRegister(MPU6050_WHO_AM_I);
    return (whoAmI == 0x68);
}

void MPU6050Sensor::update() {
    if (!initialized) return;
    
    uint8_t data[14];
    readRegisters(MPU6050_ACCEL_XOUT_H, 14, data);
    
    // Combine high and low bytes
    accelX = (int16_t)((data[0] << 8) | data[1]);
    accelY = (int16_t)((data[2] << 8) | data[3]);
    accelZ = (int16_t)((data[4] << 8) | data[5]);
    temperature = (int16_t)((data[6] << 8) | data[7]);
    gyroX = (int16_t)((data[8] << 8) | data[9]);
    gyroY = (int16_t)((data[10] << 8) | data[11]);
    gyroZ = (int16_t)((data[12] << 8) | data[13]);
}

float MPU6050Sensor::getAccelX() {
    return (accelX - accelOffX) * accelScale;
}

float MPU6050Sensor::getAccelY() {
    return (accelY - accelOffY) * accelScale;
}

float MPU6050Sensor::getAccelZ() {
    return (accelZ - accelOffZ) * accelScale;
}

float MPU6050Sensor::getGyroX() {
    return (gyroX - gyroOffX) * gyroScale;
}

float MPU6050Sensor::getGyroY() {
    return (gyroY - gyroOffY) * gyroScale;
}

float MPU6050Sensor::getGyroZ() {
    return (gyroZ - gyroOffZ) * gyroScale;
}

float MPU6050Sensor::getTemperature() {
    // Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
    return (temperature / 340.0) + 36.53;
}

void MPU6050Sensor::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(value);
    uint8_t error = Wire.endTransmission();
    if (error != 0) {
        Serial.printf("MPU6050 write error: %d\n", error);
    }
}

uint8_t MPU6050Sensor::readRegister(uint8_t reg) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    
    unsigned long timeout = millis() + 100;  // 100ms timeout
    Wire.requestFrom((int)MPU6050_ADDR, (int)1);  // Cast to int to avoid ambiguity
    
    while (Wire.available() == 0 && millis() < timeout) {
        delay(1);
    }
    
    if (Wire.available()) {
        return Wire.read();
    }
    return 0xFF;  // Return error value on timeout
}

void MPU6050Sensor::readRegisters(uint8_t reg, uint8_t count, uint8_t* dest) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    
    unsigned long timeout = millis() + 100;  // 100ms timeout
    Wire.requestFrom((int)MPU6050_ADDR, (int)count);  // Cast to int to avoid ambiguity
    
    uint8_t bytesRead = 0;
    while (bytesRead < count && millis() < timeout) {
        if (Wire.available()) {
            dest[bytesRead++] = Wire.read();
        }
    }
    
    // Fill remaining bytes with zeros on timeout
    while (bytesRead < count) {
        dest[bytesRead++] = 0;
    }
}
