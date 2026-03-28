#include "xbox_controller.h"
#include "config.h"

#if XBOX_BLE_AVAILABLE
static XboxController* g_xboxInstance = nullptr;
#endif

// Constructor
XboxController::XboxController() : 
    leftStickY(0), 
    rightStickX(0),
    rightTrigger(0),
    leftTrigger(0),
    pairingMode(false),
    pairingStartTime(0)
#if XBOX_BLE_AVAILABLE
    , gamepad(nullptr), connected(false)
#endif
    , bleAvailable(XBOX_BLE_AVAILABLE)
    , btInitialized(false) {
}

void XboxController::begin() {
#if XBOX_BLE_AVAILABLE
    g_xboxInstance = this;
    BP32.setup(&XboxController::onConnectedGamepad, &XboxController::onDisconnectedGamepad);
    BP32.enableVirtualDevice(false);
    btInitialized = true;
    Serial.println("[XBOX] Bluepad32 initialized - BLE pairing available");
    Serial.println("[XBOX] Put controller in pairing mode, then short-press button");
#else
    btInitialized = true;
    Serial.println("[XBOX] Controller module initialized");
    Serial.println("[XBOX] Note: Full BLE gamepad requires Bluepad32 platform");
    Serial.println("[XBOX] Stub mode - press button to simulate pairing");
#endif
}

void XboxController::stop() {
#if XBOX_BLE_AVAILABLE
    btStop();
    Serial.println("[XBOX] Bluetooth radio disabled");
#endif
}

void XboxController::start() {
#if XBOX_BLE_AVAILABLE
    if (!btInitialized) {
        begin();  // First time — full init
    } else {
        btStart();
        Serial.println("[XBOX] Bluetooth radio re-enabled");
    }
#endif
}

void XboxController::update() {
#if XBOX_BLE_AVAILABLE
    BP32.update();

    if (gamepad && gamepad->isConnected()) {
        // Bluepad32 axis range is typically -511..512
        leftStickY = -gamepad->axisY();    // Invert so up stick = forward
        
        // Right stick X for turning — amplify 3x since some controllers report small range
        int rawRX = gamepad->axisRX();
        rightStickX = constrain(rawRX * 3, -512, 512);
        
        // Triggers for servo — try analog first, fall back to digital button
        rightTrigger = gamepad->throttle(); // RT analog (0-1023)
        leftTrigger = gamepad->brake();     // LT analog (0-1023)
        // If analog brake reads 0, check l2 digital button as fallback
        if (leftTrigger < 20 && gamepad->l2()) {
            leftTrigger = 1023;  // Digital LT pressed = full throw
        }
        if (rightTrigger < 20 && gamepad->r2()) {
            rightTrigger = 1023; // Digital RT fallback
        }
        
        // Raw axis debug (every 500ms, always when connected)
        static unsigned long lastAxisDbg = 0;
        if (millis() - lastAxisDbg > 500) {
            lastAxisDbg = millis();
            Serial.printf("[XBOX-RAW] Y:%d X:%d RX:%d RY:%d brake:%d throttle:%d l2:%d r2:%d\n",
                          gamepad->axisY(), gamepad->axisX(), gamepad->axisRX(),
                          gamepad->axisRY(), gamepad->brake(), gamepad->throttle(),
                          gamepad->l2(), gamepad->r2());
        }
    } else {
        leftStickY = 0;
        rightStickX = 0;
        rightTrigger = 0;
        leftTrigger = 0;
    }
#endif

    // Check pairing timeout
    if (pairingMode) {
        if (millis() - pairingStartTime > PAIRING_TIMEOUT) {
            pairingMode = false;
            if (!isConnected()) {
                Serial.println("[XBOX] Pair failed - timeout");
            }
        }
    }
}

int XboxController::getLeftStickY() {
    return leftStickY;
}

int XboxController::getRightStickX() {
    return rightStickX;
}

int XboxController::getRightTrigger() {
    return rightTrigger;
}

int XboxController::getLeftTrigger() {
    return leftTrigger;
}

int XboxController::applyDeadzone(int value) {
    if (abs(value) < DEADZONE) {
        return 0;
    }
    return value;
}

void XboxController::getMotorSpeeds(int& leftSpeed, int& rightSpeed) {
    // Get stick values with deadzone applied
    int throttle = applyDeadzone(leftStickY);
    int steering = applyDeadzone(rightStickX);
    
    // Convert to motor speeds (-255 to 255)
    // Scale from -512..512 to -255..255
    int baseSpeed = map(throttle, -512, 512, -255, 255);
    int turnAmount = map(steering, -512, 512, -255, 255);
    
    // Tank drive mixing — full turn authority
    // Left stick Y = forward/backward, Right stick X = turn
    leftSpeed = baseSpeed + turnAmount;
    rightSpeed = baseSpeed - turnAmount;
    
    // Constrain to valid range
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
}

bool XboxController::isConnected() {
#if XBOX_BLE_AVAILABLE
    return connected && gamepad && gamepad->isConnected();
#else
    // Stub mode - always disconnected
    return false;
#endif
}

void XboxController::startPairing() {
    pairingMode = true;
    pairingStartTime = millis();
#if XBOX_BLE_AVAILABLE
    BP32.forgetBluetoothKeys();
    Serial.println("[XBOX] Pairing mode started...");
    Serial.println("[XBOX] Cleared old keys. Put controller in pairing mode now");
    Serial.println("[XBOX] Waiting for controller (30 sec timeout)");
#else
    Serial.println("[XBOX] Pairing mode started...");
    Serial.println("[XBOX] Waiting for controller (30 sec timeout)");
    Serial.println("[XBOX] Note: Stub mode - no actual BLE pairing");
#endif
}

const char* XboxController::getControllerName() {
#if XBOX_BLE_AVAILABLE
    return isConnected() ? "Bluetooth Gamepad" : "No Controller";
#else
    return "No Controller";
#endif
}

#if XBOX_BLE_AVAILABLE
void XboxController::onConnectedGamepad(GamepadPtr gp) {
    if (!g_xboxInstance) return;

    g_xboxInstance->gamepad = gp;
    g_xboxInstance->connected = true;
    g_xboxInstance->pairingMode = false;
    Serial.println("[XBOX] Controller connected!");
}

void XboxController::onDisconnectedGamepad(GamepadPtr gp) {
    if (!g_xboxInstance) return;

    if (g_xboxInstance->gamepad == gp) {
        g_xboxInstance->gamepad = nullptr;
    }
    g_xboxInstance->connected = false;
    g_xboxInstance->leftStickY = 0;
    g_xboxInstance->rightStickX = 0;
    g_xboxInstance->rightTrigger = 0;
    g_xboxInstance->leftTrigger = 0;
    Serial.println("[XBOX] Controller disconnected");
}
#endif
