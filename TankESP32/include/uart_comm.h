#ifndef UART_COMM_H
#define UART_COMM_H

#include <Arduino.h>

// Message types for UART communication
enum MessageType {
    MSG_MOTOR_CONTROL = 0x01,
    MSG_SENSOR_DATA = 0x02,
    MSG_STATUS = 0x03,
    MSG_COMMAND = 0x04,
    MSG_ACK = 0x05
};

// Command types
enum CommandType {
    CMD_STOP = 0x01,
    CMD_FORWARD = 0x02,
    CMD_BACKWARD = 0x03,
    CMD_LEFT = 0x04,
    CMD_RIGHT = 0x05,
    CMD_CUSTOM = 0x06,
    CMD_GET_SENSORS = 0x07,
    CMD_STANDBY = 0x08
};

// Message structure
struct Message {
    uint8_t type;
    uint8_t length;
    uint8_t data[64];  // Must fit SensorData
    uint8_t checksum;
};

// Sensor data structure
struct SensorData {
    // Raw sensor distances
    float distanceLeft;    // Left ultrasonic raw (~15° left of center)
    float distanceRight;   // Right ultrasonic raw (~15° right of center)
    
    // Trig-computed obstacle geometry (from dual sensors at ±15°)
    float distance;        // Estimated center-forward distance (interpolated)
    float wallAngle;       // Angle of obstacle surface relative to robot (deg, +right, -left)
    float gapWidth;        // Estimated gap/passage width between obstacle points (cm)
    float obstacleX;       // X position of nearest obstacle point (cm, +right, -left)
    float obstacleY;       // Y position of nearest obstacle point (cm, forward)
    
    // IMU data
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float temperature;
};

// Motor command structure
struct MotorCommand {
    int8_t motorA;  // -100 to 100 (percentage)
    int8_t motorB;  // -100 to 100 (percentage)
};

class UARTComm {
public:
    UARTComm();
    void begin();
    
    // Send messages
    void sendSensorData(const SensorData& data);
    void sendStatus(const char* status);
    void sendAck(uint8_t messageType);
    
    // Receive and process messages
    bool receiveMessage(Message& msg);
    void processMessage(const Message& msg);
    
    // Check if data is available
    bool available();
    
    // Set callback for motor commands
    void onMotorCommand(void (*callback)(int speedA, int speedB));
    void onCustomCommand(void (*callback)(uint8_t cmd, uint8_t* data, uint8_t length));

private:
    void sendMessage(const Message& msg);
    uint8_t calculateChecksum(const Message& msg);
    bool validateChecksum(const Message& msg);
    
    // Callbacks
    void (*motorCommandCallback)(int speedA, int speedB);
    void (*customCommandCallback)(uint8_t cmd, uint8_t* data, uint8_t length);
    
    // Buffer for receiving
    uint8_t rxBuffer[64];
    uint8_t rxIndex;
};

#endif // UART_COMM_H
