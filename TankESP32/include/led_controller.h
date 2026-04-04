#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <Arduino.h>

enum LEDMode {
    LED_OFF = 0,
    LED_SOLID_RED,
    LED_SOLID_GREEN,
    LED_SOLID_BLUE,
    LED_SOLID_ORANGE,       // Autonomous mode
    LED_SOLID_PURPLE,       // Simple autonomous mode
    LED_BLINK_BLUE_FAST,    // Bluetooth pairing
    LED_SOLID_RED_MODE,     // UART mode
    LED_SOLID_CYAN,         // Wall-follow mode
    LED_SOLID_LIME          // Premap nav mode
};

class LEDController {
public:
    LEDController();
    void begin();
    
    // Set LED mode
    void setMode(LEDMode mode);
    
    // Update LED state (call in loop)
    void update();
    
    void off();

private:
    void writeRGB(uint8_t r, uint8_t g, uint8_t b);
    LEDMode currentMode;
    unsigned long lastBlinkTime;
    bool blinkState;
    uint8_t brightness;  // 0-255 global brightness scaler
};

#endif // LED_CONTROLLER_H
