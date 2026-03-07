#include "uart_comm.h"
#include "config.h"

UARTComm::UARTComm() {
    rxIndex = 0;
    motorCommandCallback = nullptr;
    customCommandCallback = nullptr;
}

void UARTComm::begin() {
    // DISABLED: Serial1 on pins 43/44 conflicts with USB-CDC on ESP32-S3
    // To re-enable, use different pins (e.g., 17/18)
    // Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);
    Serial.println("UART Communication initialized (Serial1 disabled - use USB Serial)");
}

bool UARTComm::available() {
    return Serial1.available() > 0;
}

uint8_t UARTComm::calculateChecksum(const Message& msg) {
    uint8_t checksum = 0;
    checksum ^= msg.type;
    checksum ^= msg.length;
    for (uint8_t i = 0; i < msg.length; i++) {
        checksum ^= msg.data[i];
    }
    return checksum;
}

bool UARTComm::validateChecksum(const Message& msg) {
    return calculateChecksum(msg) == msg.checksum;
}

void UARTComm::sendMessage(const Message& msg) {
    // Send start byte
    Serial1.write(0xFF);
    
    // Send message type and length
    Serial1.write(msg.type);
    Serial1.write(msg.length);
    
    // Send data
    Serial1.write(msg.data, msg.length);
    
    // Send checksum
    Serial1.write(msg.checksum);
    
    // Send end byte
    Serial1.write(0xFE);
}

void UARTComm::sendSensorData(const SensorData& data) {
    Message msg;
    msg.type = MSG_SENSOR_DATA;
    msg.length = sizeof(SensorData);
    
    // Copy sensor data to message
    memcpy(msg.data, &data, sizeof(SensorData));
    
    // Calculate and add checksum
    msg.checksum = calculateChecksum(msg);
    
    sendMessage(msg);
}

void UARTComm::sendStatus(const char* status) {
    Message msg;
    msg.type = MSG_STATUS;
    msg.length = strlen(status);
    
    // Limit to 32 bytes
    if (msg.length > 32) {
        msg.length = 32;
    }
    
    memcpy(msg.data, status, msg.length);
    msg.checksum = calculateChecksum(msg);
    
    sendMessage(msg);
}

void UARTComm::sendAck(uint8_t messageType) {
    Message msg;
    msg.type = MSG_ACK;
    msg.length = 1;
    msg.data[0] = messageType;
    msg.checksum = calculateChecksum(msg);
    
    sendMessage(msg);
}

bool UARTComm::receiveMessage(Message& msg) {
    while (Serial1.available()) {
        uint8_t byte = Serial1.read();
        
        // Look for start byte
        if (rxIndex == 0 && byte != 0xFF) {
            continue;
        }
        
        rxBuffer[rxIndex++] = byte;
        
        // Check if we have at least the header (start + type + length)
        if (rxIndex >= 3) {
            uint8_t expectedLength = rxBuffer[2] + 5; // type + length + data + checksum + end
            
            // Check if we have the complete message
            if (rxIndex >= expectedLength) {
                // Verify end byte
                if (rxBuffer[rxIndex - 1] == 0xFE) {
                    // Parse message
                    msg.type = rxBuffer[1];
                    msg.length = rxBuffer[2];
                    memcpy(msg.data, &rxBuffer[3], msg.length);
                    msg.checksum = rxBuffer[3 + msg.length];
                    
                    // Reset buffer
                    rxIndex = 0;
                    
                    // Validate checksum
                    if (validateChecksum(msg)) {
                        return true;
                    } else {
                        Serial.println("Checksum error!");
                    }
                } else {
                    // Invalid end byte, reset
                    rxIndex = 0;
                }
            }
        }
        
        // Prevent buffer overflow
        if (rxIndex >= 64) {
            rxIndex = 0;
        }
    }
    
    return false;
}

void UARTComm::processMessage(const Message& msg) {
    switch (msg.type) {
        case MSG_MOTOR_CONTROL: {
            if (msg.length >= 2 && motorCommandCallback != nullptr) {
                int8_t speedA = (int8_t)msg.data[0];
                int8_t speedB = (int8_t)msg.data[1];
                
                // Convert from percentage (-100 to 100) to PWM (255 to 255)
                int pwmA = map(speedA, -100, 100, -255, 255);
                int pwmB = map(speedB, -100, 100, -255, 255);
                
                motorCommandCallback(pwmA, pwmB);
                sendAck(MSG_MOTOR_CONTROL);
            }
            break;
        }
        
        case MSG_COMMAND: {
            if (msg.length >= 1) {
                uint8_t cmd = msg.data[0];
                
                if (cmd == CMD_GET_SENSORS) {
                    // Sensor data request will be handled in main loop
                    sendAck(MSG_COMMAND);
                } else if (customCommandCallback != nullptr) {
                    customCommandCallback(cmd, (uint8_t*)&msg.data[1], msg.length - 1);
                    sendAck(MSG_COMMAND);
                }
            }
            break;
        }
        
        case MSG_ACK: {
            // Acknowledgment received
            Serial.printf("ACK received for message type: 0x%02X\n", msg.data[0]);
            break;
        }
        
        default:
            Serial.printf("Unknown message type: 0x%02X\n", msg.type);
            break;
    }
}

void UARTComm::onMotorCommand(void (*callback)(int speedA, int speedB)) {
    motorCommandCallback = callback;
}

void UARTComm::onCustomCommand(void (*callback)(uint8_t cmd, uint8_t* data, uint8_t length)) {
    customCommandCallback = callback;
}
