#ifndef XBOX_CONTROLLER_H
#define XBOX_CONTROLLER_H

#include <Arduino.h>

#if __has_include(<Bluepad32.h>)
#include <Bluepad32.h>
#define XBOX_BLE_AVAILABLE 1
#else
#define XBOX_BLE_AVAILABLE 0
#endif

// Xbox controller support - stub with proper feedback messages
// Full BLE gamepad support requires Bluepad32 with custom platform setup

class XboxController {
public:
    XboxController();
    void begin();
    
    // Update controller state (call in loop)
    void update();
    
    // Get controller values
    int getLeftStickY();   // -512 to 512 (forward/backward)
    int getRightStickX();  // -512 to 512 (left/right turn)
    
    // Calculate motor speeds for skid steer
    void getMotorSpeeds(int& leftSpeed, int& rightSpeed);
    
    // Check if controller is connected
    bool isConnected();
    
    // Start Bluetooth pairing
    void startPairing();
    
    // Get readable name
    const char* getControllerName();

private:
    int leftStickY;
    int rightStickX;
    
    // Deadzone for joysticks
    static const int DEADZONE = 50;
    
    // Apply deadzone to stick values
    int applyDeadzone(int value);
    
    // Pairing state
    bool pairingMode;
    unsigned long pairingStartTime;
    static const unsigned long PAIRING_TIMEOUT = 30000; // 30 seconds

#if XBOX_BLE_AVAILABLE
    GamepadPtr gamepad;
    bool connected;
#endif

    bool bleAvailable;

#if XBOX_BLE_AVAILABLE
    static void onConnectedGamepad(GamepadPtr gp);
    static void onDisconnectedGamepad(GamepadPtr gp);
#endif
};

#endif // XBOX_CONTROLLER_H
