#ifndef BUTTON_HANDLER_H
#define BUTTON_HANDLER_H

#include <Arduino.h>

enum ButtonEvent {
    BUTTON_NONE = 0,
    BUTTON_SHORT_PRESS,
    BUTTON_LONG_PRESS
};

class ButtonHandler {
public:
    ButtonHandler();
    void begin();
    
    // Update button state (call in loop)
    ButtonEvent update();
    
    // Get current button state
    bool isPressed();

private:
    unsigned long pressStartTime;
    unsigned long lastDebounceTime;
    bool lastButtonState;
    bool buttonState;
    bool longPressTriggered;
};

#endif // BUTTON_HANDLER_H
