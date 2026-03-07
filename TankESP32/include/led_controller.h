#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

enum LEDMode {
    LED_OFF = 0,
    LED_SOLID_RED,
    LED_SOLID_GREEN,
    LED_SOLID_BLUE,
    LED_SOLID_ORANGE,       // Autonomous mode
    LED_SOLID_PURPLE,       // Simple autonomous mode
    LED_BLINK_BLUE_FAST,    // Bluetooth pairing
    LED_SOLID_RED_MODE      // UART mode
};

class LEDController {
public:
    LEDController();
    void begin();
    
    // Set LED mode
    void setMode(LEDMode mode);
    
    // Update LED state (call in loop)
    void update();
    
    // Direct color control
    void setColor(uint8_t r, uint8_t g, uint8_t b);
    void off();

private:
    Adafruit_NeoPixel pixel;
    LEDMode currentMode;
    unsigned long lastBlinkTime;
    bool blinkState;
};

#endif // LED_CONTROLLER_H
