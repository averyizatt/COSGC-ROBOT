#include "led_controller.h"
#include "config.h"

LEDController::LEDController() {
    currentMode = LED_OFF;
    lastBlinkTime = 0;
    blinkState = false;
    brightness = 40;  // ~15% brightness — lower draw to avoid WiFi current-spike brownouts
}

void LEDController::begin() {
    // Setup PWM channels for each color
    ledcSetup(PWM_CHANNEL_LED_R, 5000, 8);  // 5kHz, 8-bit
    ledcSetup(PWM_CHANNEL_LED_G, 5000, 8);
    ledcSetup(PWM_CHANNEL_LED_B, 5000, 8);

    ledcAttachPin(LED_PIN_R, PWM_CHANNEL_LED_R);
    ledcAttachPin(LED_PIN_G, PWM_CHANNEL_LED_G);
    ledcAttachPin(LED_PIN_B, PWM_CHANNEL_LED_B);

    writeRGB(0, 0, 0);
    Serial.printf("[LED] RGB LED: R=GPIO%d G=GPIO%d B=GPIO%d\n", LED_PIN_R, LED_PIN_G, LED_PIN_B);
}

void LEDController::writeRGB(uint8_t r, uint8_t g, uint8_t b) {
    // Scale by brightness
    r = (r * brightness) / 255;
    g = (g * brightness) / 255;
    b = (b * brightness) / 255;

    ledcWrite(PWM_CHANNEL_LED_R, r);
    ledcWrite(PWM_CHANNEL_LED_G, g);
    ledcWrite(PWM_CHANNEL_LED_B, b);
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
            writeRGB(0, 0, 0);
            break;

        case LED_SOLID_RED:
        case LED_SOLID_RED_MODE:
            writeRGB(255, 0, 0);
            break;

        case LED_SOLID_GREEN:
            writeRGB(0, 255, 0);
            break;

        case LED_SOLID_BLUE:
            writeRGB(0, 0, 255);
            break;

        case LED_SOLID_ORANGE:
            writeRGB(255, 165, 0);
            break;

        case LED_SOLID_PURPLE:
            writeRGB(180, 0, 255);
            break;

        case LED_SOLID_CYAN:
            writeRGB(0, 255, 220);
            break;

        case LED_SOLID_LIME:
            writeRGB(80, 255, 0);   // Lime green — premap nav mode
            break;

        case LED_BLINK_BLUE_FAST:
            if (currentMillis - lastBlinkTime >= LED_BLINK_FAST) {
                lastBlinkTime = currentMillis;
                blinkState = !blinkState;
                if (blinkState) {
                    writeRGB(0, 0, 255);
                } else {
                    writeRGB(0, 0, 0);
                }
            }
            break;
    }
}

void LEDController::off() {
    currentMode = LED_OFF;
    writeRGB(0, 0, 0);
}
