#include "led_controller.h"
#include "config.h"

LEDController::LEDController() : pixel(1, LED_PIN, NEO_GRB + NEO_KHZ800) {
    currentMode = LED_OFF;
    lastBlinkTime = 0;
    blinkState = false;
}

void LEDController::begin() {
    pixel.begin();
    pixel.setBrightness(50);  // 0-255, adjust brightness
    pixel.show();  // Initialize all pixels to 'off'
}

void LEDController::setMode(LEDMode mode) {
    currentMode = mode;
    blinkState = false;
    lastBlinkTime = millis();
}

void LEDController::update() {
    unsigned long currentMillis = millis();
    
    switch (currentMode) {
        case LED_OFF:
            pixel.setPixelColor(0, pixel.Color(0, 0, 0));
            pixel.show();
            break;
            
        case LED_SOLID_RED:
            pixel.setPixelColor(0, pixel.Color(255, 0, 0));
            pixel.show();
            break;
            
        case LED_SOLID_GREEN:
            pixel.setPixelColor(0, pixel.Color(0, 255, 0));
            pixel.show();
            break;
            
        case LED_SOLID_BLUE:
            pixel.setPixelColor(0, pixel.Color(0, 0, 255));
            pixel.show();
            break;
            
        case LED_SOLID_ORANGE:
            pixel.setPixelColor(0, pixel.Color(255, 165, 0));
            pixel.show();
            break;

        case LED_SOLID_PURPLE:
            pixel.setPixelColor(0, pixel.Color(180, 0, 255));
            pixel.show();
            break;
            
        case LED_BLINK_BLUE_FAST:
            // Fast blink for Bluetooth pairing
            if (currentMillis - lastBlinkTime >= LED_BLINK_FAST) {
                lastBlinkTime = currentMillis;
                blinkState = !blinkState;
                
                if (blinkState) {
                    pixel.setPixelColor(0, pixel.Color(0, 0, 255));
                } else {
                    pixel.setPixelColor(0, pixel.Color(0, 0, 0));
                }
                pixel.show();
            }
            break;
            
        case LED_SOLID_RED_MODE:
            pixel.setPixelColor(0, pixel.Color(255, 0, 0));
            pixel.show();
            break;
    }
}

void LEDController::setColor(uint8_t r, uint8_t g, uint8_t b) {
    currentMode = LED_OFF;  // Custom mode
    pixel.setPixelColor(0, pixel.Color(r, g, b));
    pixel.show();
}

void LEDController::off() {
    currentMode = LED_OFF;
    pixel.setPixelColor(0, pixel.Color(0, 0, 0));
    pixel.show();
}
