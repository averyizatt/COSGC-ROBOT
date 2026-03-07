#include "button_handler.h"
#include "config.h"

ButtonHandler::ButtonHandler() {
    pressStartTime = 0;
    lastDebounceTime = 0;
    lastButtonState = HIGH;
    buttonState = HIGH;
    longPressTriggered = false;
}

void ButtonHandler::begin() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);  // Active LOW
}

ButtonEvent ButtonHandler::update() {
    bool reading = digitalRead(BUTTON_PIN);
    
    // Check for state change (with debounce)
    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }
    
    // Debounce delay passed
    if ((millis() - lastDebounceTime) > BUTTON_DEBOUNCE_MS) {
        // Button state has changed after debounce
        if (reading != buttonState) {
            buttonState = reading;
            
            // Button just pressed (active LOW)
            if (buttonState == LOW) {
                pressStartTime = millis();
                longPressTriggered = false;
            }
            // Button just released
            else {
                unsigned long pressDuration = millis() - pressStartTime;
                
                // Check if it was a long press
                if (pressDuration >= BUTTON_LONG_PRESS_MS && !longPressTriggered) {
                    longPressTriggered = true;
                    lastButtonState = reading;
                    return BUTTON_LONG_PRESS;
                }
                // Short press
                else if (pressDuration < BUTTON_LONG_PRESS_MS && !longPressTriggered) {
                    lastButtonState = reading;
                    return BUTTON_SHORT_PRESS;
                }
            }
        }
        // Button is being held down
        else if (buttonState == LOW && !longPressTriggered) {
            unsigned long pressDuration = millis() - pressStartTime;
            
            // Long press threshold reached
            if (pressDuration >= BUTTON_LONG_PRESS_MS) {
                longPressTriggered = true;
                lastButtonState = reading;
                return BUTTON_LONG_PRESS;
            }
        }
    }
    
    lastButtonState = reading;
    return BUTTON_NONE;
}

bool ButtonHandler::isPressed() {
    return buttonState == LOW;
}
