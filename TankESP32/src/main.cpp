#include <Arduino.h>
#include <cstring>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_system.h"
#include "config.h"
#include "motor_control.h"
#include "ultrasonic_sensor.h"
#include "mpu6050_sensor.h"
#include "uart_comm.h"
#include "xbox_controller.h"
#include "button_handler.h"
#include "led_controller.h"
#include "autonomous_nav.h"
#include "occupancy_map.h"
#include "path_planner.h"

// Create instances of all components
MotorControl motors;
UltrasonicSensor ultrasonicLeft(ULTRASONIC_TRIG, ULTRASONIC_ECHO);    // Left sensor (~15° left)
UltrasonicSensor ultrasonicRight(ULTRASONIC2_TRIG, ULTRASONIC2_ECHO); // Right sensor (~15° right)
MPU6050Sensor imu;
UARTComm uart;
XboxController xbox;
ButtonHandler button;
LEDController led;
AutonomousNav autoNav;
OccupancyMap envMap;  // 2D occupancy grid map (25m x 25m)
PathPlanner pathPlanner;  // Wavefront path planner

// Operation mode
uint8_t currentMode = MODE_UART_CONTROL;  // Start in UART mode

// Timing variables
unsigned long lastSensorUpdate = 0;
unsigned long lastUARTSend = 0;
unsigned long lastXboxUpdate = 0;
unsigned long lastAutonomousUpdate = 0;
unsigned long lastTelemetryPrint = 0;
const unsigned long SENSOR_UPDATE_INTERVAL = 50;   // 20 Hz
const unsigned long UART_SEND_INTERVAL = 100;      // 10 Hz
const unsigned long XBOX_UPDATE_INTERVAL = 20;     // 50 Hz
const unsigned long AUTONOMOUS_UPDATE_INTERVAL = 50;  // 20 Hz
const unsigned long TELEMETRY_PRINT_INTERVAL = 10000;  // 10 seconds
const unsigned long MAP_PRINT_INTERVAL = 30000;  // Print map every 30 seconds

// Sensor data
SensorData sensorData;

// Raw ultrasonic readings (for debug output without re-triggering sensors)
float lastRawLeft = 0;
float lastRawRight = 0;

// Motor command tracking
int lastMotorSpeedA = 0;
int lastMotorSpeedB = 0;

// Function prototypes
void handleMotorCommand(int speedA, int speedB);
void handleCustomCommand(uint8_t cmd, uint8_t* data, uint8_t length);
void updateSensors();
void sendSensorDataViaUART();
void handleButtonPress();
void updateModeIndicator();
void handleXboxControl();
void handleAutonomousMode();
void handleSimpleAutonomousMode();
void printTelemetry();

// Print human-readable reset reason
static void printResetReason() {
    esp_reset_reason_t reason = esp_reset_reason();
    Serial.print("[BOOT] Reset reason: ");
    switch (reason) {
        case ESP_RST_POWERON:  Serial.println("Power-on reset"); break;
        case ESP_RST_SW:       Serial.println("Software reset"); break;
        case ESP_RST_PANIC:    Serial.println("*** PANIC (crash/exception) ***"); break;
        case ESP_RST_INT_WDT:  Serial.println("*** INTERRUPT WATCHDOG ***"); break;
        case ESP_RST_TASK_WDT: Serial.println("*** TASK WATCHDOG ***"); break;
        case ESP_RST_WDT:      Serial.println("*** OTHER WATCHDOG ***"); break;
        case ESP_RST_BROWNOUT: Serial.println("*** BROWNOUT (voltage drop) ***"); break;
        case ESP_RST_DEEPSLEEP: Serial.println("Deep sleep wakeup"); break;
        case ESP_RST_SDIO:     Serial.println("SDIO reset"); break;
        default:               Serial.printf("Unknown (%d)\n", reason); break;
    }
}

void setup() {
    // Disable brownout detector — motor current spikes cause false resets
    // Method 1: Zero entire register (covers all ESP32 variants)
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    // Method 2: Clear just the enable bit specifically
    REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);
    
    // Initialize serial for debugging
    Serial.begin(115200);
    delay(2000);  // Longer delay to ensure serial port is stable
    Serial.flush();  // Flush any pending serial data
    Serial.println("\n\nESP32-S3 Rover Control System Starting...");
    Serial.println("[BOOT] Brownout detector disabled");
    printResetReason();
    
    // Initialize button
    Serial.println("Initializing button...");
    button.begin();
    
    // Initialize LED
    Serial.println("Initializing LED...");
    led.begin();
    led.setMode(LED_SOLID_RED);  // Start with red (UART mode)
    
    // Initialize motor control
    Serial.println("Initializing motors...");
    Serial.flush();
    motors.begin();
    motors.stop();
    Serial.println("Motors initialized OK");
    Serial.flush();
    
    // Initialize ultrasonic sensors (two forward-facing, ±15°)
    Serial.println("Initializing ultrasonic sensors...");
    Serial.flush();
    ultrasonicLeft.begin();
    Serial.println("Ultrasonic LEFT (GPIO12/11) initialized OK");
    ultrasonicRight.begin();
    Serial.println("Ultrasonic RIGHT (GPIO38/37) initialized OK");
    Serial.flush();
    
    // Initialize IMU
    Serial.println("Initializing MPU6050...");
    Serial.flush();
    if (IMU_ENABLED) {
        if (!imu.begin()) {
            Serial.println("Failed to initialize MPU6050!");
            Serial.println("Check I2C connections (SDA=9, SCL=8)");
        } else {
            // Calibrate IMU on startup - robot must be flat and still!
            imu.calibrate();
        }
    } else {
        Serial.println("MPU6050 disabled in config (IMU_ENABLED=0)");
    }
    Serial.flush();
    
    // Initialize UART communication
    Serial.println("Initializing UART communication...");
    Serial.flush();
    uart.begin();
    Serial.println("UART initialized OK");
    Serial.flush();
    
    // Set up UART callbacks
    uart.onMotorCommand(handleMotorCommand);
    uart.onCustomCommand(handleCustomCommand);
    
    // Initialize Xbox controller (Bluetooth)
    Serial.println("Initializing Xbox controller support...");
    xbox.begin();
    
    // Initialize path planner with map
    Serial.println("Initializing path planner...");
    pathPlanner.begin(&envMap);
    
    Serial.println("System initialization complete!");
    Serial.printf("Current mode: %s\n", 
                  currentMode == MODE_RC_CONTROL ? "RC Control" : 
                  currentMode == MODE_UART_CONTROL ? "UART Control" :
                  currentMode == MODE_AUTONOMOUS ? "Autonomous" : "Simple Auto");
    Serial.println("Button: Short press = Start pairing, Long press (1.5s) = Switch mode");
    Serial.println("Starting in UART Control mode...");
    
    // Sensor diagnostic test
    Serial.println("\n========== SENSOR DIAGNOSTIC TEST ==========");
    
    // Ultrasonic sensor tests
    Serial.println("\n--- ULTRASONIC SENSOR TEST ---");
    Serial.printf("LEFT  sensor: TRIG=GPIO%d, ECHO=GPIO%d\n", ULTRASONIC_TRIG, ULTRASONIC_ECHO);
    Serial.printf("RIGHT sensor: TRIG=GPIO%d, ECHO=GPIO%d\n", ULTRASONIC2_TRIG, ULTRASONIC2_ECHO);
    
    Serial.println("\nTaking 3 readings from each sensor...");
    for (int i = 0; i < 3; i++) {
        float distL = ultrasonicLeft.readDistance();
        delay(30);  // Avoid echo crosstalk between sensors
        float distR = ultrasonicRight.readDistance();
        Serial.printf("  Reading %d: LEFT=%.1fcm%s  RIGHT=%.1fcm%s\n", i+1,
                      distL, distL < 0 ? "(FAIL)" : "",
                      distR, distR < 0 ? "(FAIL)" : "");
        delay(100);
    }
    
    // MPU6050 I2C scan (Wire already initialized by MPU6050)
    Serial.println("\n--- I2C BUS SCAN ---");
    Serial.printf("SDA pin: GPIO %d, SCL pin: GPIO %d\n", I2C_SDA, I2C_SCL);
    
    // Don't reinitialize Wire - just use it
    int devicesFound = 0;
    Serial.println("Scanning addresses 0x01-0x7F...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t error = Wire.endTransmission();
        if (error == 0) {
            Serial.printf("  Found device at 0x%02X", addr);
            if (addr == 0x68) Serial.print(" <- MPU6050 (AD0=LOW)");
            if (addr == 0x69) Serial.print(" <- MPU6050 (AD0=HIGH)");
            Serial.println();
            devicesFound++;
        }
    }
    if (devicesFound == 0) {
        Serial.println("  NO I2C DEVICES FOUND!");
        Serial.println("  Wiring check:");
        Serial.println("    MPU6050 VCC  -> ESP32 3.3V");
        Serial.println("    MPU6050 GND  -> ESP32 GND");
        Serial.println("    MPU6050 SDA  -> ESP32 GPIO 9");
        Serial.println("    MPU6050 SCL  -> ESP32 GPIO 8");
        Serial.println("    MPU6050 AD0  -> GND (for addr 0x68)");
    } else {
        Serial.printf("  Total: %d device(s) found\n", devicesFound);
    }
    
    // Motor test - simple forward/backward cycle
    Serial.println("\n--- MOTOR TEST ---");
    Serial.println("Forward 1s...");
    motors.forward(200);
    delay(1000);
    
    Serial.println("Backward 1s...");
    motors.backward(200);
    delay(1000);
    
    motors.stop();
    Serial.println("========== DIAGNOSTIC TEST COMPLETE ==========");
    
    // Send initial status
    uart.sendStatus("READY");
    
    Serial.println("\n*** SETUP COMPLETE - ENTERING MAIN LOOP ***\n");
}

void loop() {
    static unsigned long loopCount = 0;
    static unsigned long lastHeartbeat = 0;
    unsigned long currentMillis = millis();
    
    loopCount++;
    
    // Print heartbeat every 5 seconds to prove loop is running
    if (currentMillis - lastHeartbeat >= 5000) {
        lastHeartbeat = currentMillis;
        Serial.printf("[HEARTBEAT] Loop running - count: %lu, millis: %lu\n", loopCount, currentMillis);
    }
    
    // Check for serial commands (path planner control)
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toUpperCase();
        
        if (cmd == "EXPLORE") {
            pathPlanner.startExploration();
            Serial.println("[CMD] Starting frontier exploration");
        } else if (cmd == "HOME") {
            pathPlanner.returnHome();
            Serial.println("[CMD] Returning to start position");
        } else if (cmd == "STOP") {
            pathPlanner.stop();
            motors.stop();
            Serial.println("[CMD] Path planner and motors stopped");
        } else if (cmd == "MAP") {
            envMap.printMap();
        } else if (cmd == "PATH") {
            pathPlanner.printPath();
        } else if (cmd == "SENSOR" || cmd == "SENSORS") {
            // Print current sensor readings
            Serial.println("\n=== SENSOR READINGS ===");
            Serial.printf("Ultrasonic LEFT:  %.1f cm\n", sensorData.distanceLeft);
            Serial.printf("Ultrasonic RIGHT: %.1f cm\n", sensorData.distanceRight);
            Serial.printf("Center FWD (trig): %.1f cm\n", sensorData.distance);
            Serial.printf("Wall angle: %.1f°  Gap width: %.1f cm\n", sensorData.wallAngle, sensorData.gapWidth);
            Serial.printf("Nearest obstacle: (%.1f, %.1f) cm\n", sensorData.obstacleX, sensorData.obstacleY);
            if (IMU_ENABLED) {
                Serial.printf("Accel: X=%.2f Y=%.2f Z=%.2f g\n", 
                              sensorData.accelX, sensorData.accelY, sensorData.accelZ);
                Serial.printf("Gyro:  X=%.1f Y=%.1f Z=%.1f deg/s\n", 
                              sensorData.gyroX, sensorData.gyroY, sensorData.gyroZ);
                Serial.printf("Heading: %.1f deg\n", autoNav.getHeading());
                Serial.printf("Pitch: %.1f deg, Roll: %.1f deg\n", 
                              autoNav.getPitch(), autoNav.getRoll());
            }
            Serial.println("=======================\n");
        } else if (cmd == "RESET") {
            // Reset map and navigation
            envMap.reset();
            autoNav.reset();
            pathPlanner.stop();
            Serial.println("[CMD] Map and navigation reset");
        } else if (cmd.startsWith("GOTO ")) {
            // Parse "GOTO X Y" command (coordinates in meters)
            float x = 0, y = 0;
            if (sscanf(cmd.c_str(), "GOTO %f %f", &x, &y) == 2) {
                pathPlanner.setGoal(x * 100, y * 100);  // Convert m to cm
                Serial.printf("[CMD] Going to (%.1f, %.1f) m\n", x, y);
            } else {
                Serial.println("[CMD] Usage: GOTO X Y (in meters, e.g., GOTO 2.5 3.0)");
            }
        } else if (cmd == "HELP" || cmd == "?") {
            Serial.println("\n======== COMMANDS ========");
            Serial.println("EXPLORE  - Start frontier-based exploration");
            Serial.println("HOME     - Return to starting position");
            Serial.println("GOTO X Y - Go to position (in meters from start)");
            Serial.println("STOP     - Stop motors and path planning");
            Serial.println("MAP      - Print current occupancy map");
            Serial.println("PATH     - Print current planned path");
            Serial.println("SENSOR   - Print current sensor readings");
            Serial.println("RESET    - Reset map and navigation");
            Serial.println("HELP     - Show this help");
            Serial.println("==========================\n");
        }
    }
    
    // Update LED animation
    led.update();
    
    // Check button for mode changes
    handleButtonPress();
    
    // Update sensors periodically
    if (currentMillis - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL) {
        lastSensorUpdate = currentMillis;
        updateSensors();
    }

    // Raw ultrasonic debug print every 500ms (uses values from updateSensors, no extra reads)
    {
        static unsigned long lastUsPrint = 0;
        if (currentMillis - lastUsPrint >= 500) {
            lastUsPrint = currentMillis;
            Serial.printf("[US RAW] L: %.1f cm  R: %.1f cm  (filtered L:%.1f R:%.1f)\n",
                          lastRawLeft, lastRawRight, sensorData.distanceLeft, sensorData.distanceRight);
        }
    }

    // Print telemetry every 10 seconds
    if (currentMillis - lastTelemetryPrint >= TELEMETRY_PRINT_INTERVAL) {
        lastTelemetryPrint = currentMillis;
        printTelemetry();
    }
    
    // Handle different control modes
    if (currentMode == MODE_RC_CONTROL) {
        // Xbox controller mode - update at 50 Hz
        if (currentMillis - lastXboxUpdate >= XBOX_UPDATE_INTERVAL) {
            lastXboxUpdate = currentMillis;
            handleXboxControl();
        }
    } else if (currentMode == MODE_UART_CONTROL) {
        // UART mode - send sensor data and process commands
        if (currentMillis - lastUARTSend >= UART_SEND_INTERVAL) {
            lastUARTSend = currentMillis;
            // Disabled automatic sensor sending to prevent serial corruption
            // sendSensorDataViaUART();
        }
        
        // Check for incoming UART messages
        Message msg;
        if (uart.receiveMessage(msg)) {
            uart.processMessage(msg);
        }
    } else if (currentMode == MODE_AUTONOMOUS) {
        // Autonomous mode - update at 10 Hz
        if (currentMillis - lastAutonomousUpdate >= AUTONOMOUS_UPDATE_INTERVAL) {
            lastAutonomousUpdate = currentMillis;
            handleAutonomousMode();
        }
    } else if (currentMode == MODE_SIMPLE_AUTO) {
        // Simple autonomous mode - same update rate as autonomous
        if (currentMillis - lastAutonomousUpdate >= AUTONOMOUS_UPDATE_INTERVAL) {
            lastAutonomousUpdate = currentMillis;
            handleSimpleAutonomousMode();
        }
    }
    
    // Small delay to prevent watchdog issues
    delay(1);
}

void updateSensors() {
    // Read both ultrasonic sensors — use raw values directly, skip invalid reads
    static float lastGoodLeft = 30.0f;
    static float lastGoodRight = 30.0f;
    
    // Read left sensor
    float rawLeft = ultrasonicLeft.readDistance();
    lastRawLeft = rawLeft;  // Store for debug print
    if (rawLeft >= 2.0f && rawLeft <= 400.0f) {
        lastGoodLeft = rawLeft;
    }
    sensorData.distanceLeft = lastGoodLeft;
    
    // Small delay to avoid echo crosstalk between sensors
    delayMicroseconds(500);
    
    // Read right sensor
    float rawRight = ultrasonicRight.readDistance();
    lastRawRight = rawRight;  // Store for debug print
    if (rawRight >= 2.0f && rawRight <= 400.0f) {
        lastGoodRight = rawRight;
    }
    sensorData.distanceRight = lastGoodRight;
    
    // Effective forward distance = minimum of both sensors (conservative)
    sensorData.distance = min(sensorData.distanceLeft, sensorData.distanceRight);
    
    // ===== TRIG: Compute obstacle geometry from dual sensors at ±15° =====
    // Sensor geometry:
    //   Left sensor points 15° left of center
    //   Right sensor points 15° right of center
    //   sin(15°) ≈ 0.2588,  cos(15°) ≈ 0.9659
    {
        const float SIN15 = 0.2588f;
        const float COS15 = 0.9659f;
        
        float dL = sensorData.distanceLeft;
        float dR = sensorData.distanceRight;
        
        // Obstacle point positions in robot frame (x=right, y=forward)
        float xL = -dL * SIN15;   // Left point (negative x)
        float yL =  dL * COS15;   // Forward component
        float xR =  dR * SIN15;   // Right point (positive x)
        float yR =  dR * COS15;   // Forward component
        
        // Interpolate to find center-forward distance (x=0)
        // Line through (xL,yL) and (xR,yR), solve for y at x=0
        float dx = xR - xL;
        if (abs(dx) > 0.1f) {
            sensorData.distance = yL + (yR - yL) * (0.0f - xL) / dx;
        } else {
            sensorData.distance = min(yL, yR);  // Fallback if sensors overlap
        }
        
        // Clamp to valid range
        if (sensorData.distance < 2.0f) sensorData.distance = 2.0f;
        if (sensorData.distance > 400.0f) sensorData.distance = 400.0f;
        
        // Gap width: horizontal distance between the two detected points
        sensorData.gapWidth = xR - xL;  // Always positive since xR>0, xL<0
        
        // Wall angle: angle of the surface connecting both obstacle points
        // 0° = wall perpendicular to robot (flat ahead)
        // +° = wall angled to the right, -° = angled to the left
        sensorData.wallAngle = atan2(yR - yL, xR - xL) * 180.0f / PI;
        
        // Nearest obstacle point position
        if (dL < dR) {
            sensorData.obstacleX = xL;
            sensorData.obstacleY = yL;
        } else {
            sensorData.obstacleX = xR;
            sensorData.obstacleY = yR;
        }
    }
    
    // Update IMU data and check for upside-down state
    if (IMU_ENABLED) {
        imu.update();
        sensorData.accelX = imu.getAccelX();
        sensorData.accelY = imu.getAccelY();
        sensorData.accelZ = imu.getAccelZ();
        sensorData.gyroX = imu.getGyroX();
        sensorData.gyroY = imu.getGyroY();
        sensorData.gyroZ = imu.getGyroZ();
        sensorData.temperature = imu.getTemperature();
        
        // TANK MODE: Detect upside-down and tell motor controller
        // AccelZ < threshold means gravity is pulling "up" relative to the board
        // This works for ALL control modes (UART, Xbox, Autonomous)
        bool isFlipped = (sensorData.accelZ < UPSIDE_DOWN_THRESHOLD);
        motors.setUpsideDown(isFlipped);
    }
    
    // Debug output (optional)
    #ifdef DEBUG_SENSORS
    Serial.printf("Distance: %.2f cm | ", sensorData.distance);
    Serial.printf("Accel: X=%.2f Y=%.2f Z=%.2f | ", 
                  sensorData.accelX, sensorData.accelY, sensorData.accelZ);
    Serial.printf("Gyro: X=%.2f Y=%.2f Z=%.2f\n", 
                  sensorData.gyroX, sensorData.gyroY, sensorData.gyroZ);
    #endif
}

void sendSensorDataViaUART() {
    uart.sendSensorData(sensorData);
}

void printTelemetry() {
    Serial.println("\n========== TELEMETRY ==========");
    Serial.printf("Distance: %.2f cm\n", sensorData.distance);
    Serial.printf("Motor Speed A: %d\n", lastMotorSpeedA);
    Serial.printf("Motor Speed B: %d\n", lastMotorSpeedB);
    Serial.printf("Mode: %s\n",
                  currentMode == MODE_RC_CONTROL ? "RC Control" :
                  currentMode == MODE_UART_CONTROL ? "UART Control" :
                  currentMode == MODE_AUTONOMOUS ? "Autonomous" : "Simple Auto");
    if (IMU_ENABLED) {
        Serial.printf("Orientation: %s\n", motors.isUpsideDown() ? "UPSIDE DOWN" : "Right-side up");
    }
    Serial.println("===============================\n");
}

void handleButtonPress() {
    ButtonEvent event = button.update();
    
    if (event == BUTTON_SHORT_PRESS) {
        // Short press - start Bluetooth pairing
        Serial.println("\n*** BUTTON SHORT PRESS - Starting Bluetooth pairing ***");
        xbox.startPairing();
        led.setMode(LED_BLINK_BLUE_FAST);
    }
    else if (event == BUTTON_LONG_PRESS) {
        // Long press - cycle through modes
        motors.stop();  // Stop motors when switching modes
        lastMotorSpeedA = 0;
        lastMotorSpeedB = 0;
        
        // Cycle: UART -> RC -> Autonomous -> Simple Auto -> UART
        if (currentMode == MODE_UART_CONTROL) {
            currentMode = MODE_RC_CONTROL;
            Serial.println("\n========================================");
            Serial.println("MODE SWITCHED TO: RC CONTROL (Xbox)");
            Serial.println("========================================\n");
        } else if (currentMode == MODE_RC_CONTROL) {
            currentMode = MODE_AUTONOMOUS;
            autoNav.reset();  // Reset autonomous state
            pathPlanner.startExploration();  // Start planning immediately (avoid IDLE)
            lastAutonomousUpdate = 0;        // Force immediate autonomous cycle
            Serial.println("\n========================================");
            Serial.println("MODE SWITCHED TO: AUTONOMOUS NAVIGATION");
            Serial.println("Path planner: FRONTIER EXPLORE");
            Serial.println("========================================\n");
        } else if (currentMode == MODE_AUTONOMOUS) {
            currentMode = MODE_SIMPLE_AUTO;
            pathPlanner.stop();
            lastAutonomousUpdate = 0;
            Serial.println("\n========================================");
            Serial.println("MODE SWITCHED TO: SIMPLE AUTONOMOUS");
            Serial.println("Behavior: Forward, turn if obstacle < 25cm");
            Serial.println("========================================\n");
        } else {
            currentMode = MODE_UART_CONTROL;
            pathPlanner.stop();  // Stop planner outside autonomous mode
            Serial.println("\n========================================");
            Serial.println("MODE SWITCHED TO: UART CONTROL");
            Serial.println("========================================\n");
        }
        
        updateModeIndicator();
        
        // Flash LED to confirm mode change
        for (int i = 0; i < 3; i++) {
            led.off();
            delay(100);
            updateModeIndicator();
            delay(100);
        }
    }
}

void updateModeIndicator() {
    if (currentMode == MODE_RC_CONTROL) {
        if (xbox.isConnected()) {
            led.setMode(LED_SOLID_GREEN);  // Green when connected
        } else {
            led.setMode(LED_SOLID_BLUE);   // Blue when waiting for controller
        }
    } else if (currentMode == MODE_UART_CONTROL) {
        led.setMode(LED_SOLID_RED);  // Red for UART mode
    } else if (currentMode == MODE_AUTONOMOUS) {
        led.setMode(LED_SOLID_ORANGE);  // Orange for autonomous mode
    } else if (currentMode == MODE_SIMPLE_AUTO) {
        led.setMode(LED_SOLID_PURPLE);  // Purple for simple autonomous mode
    }
}

void handleXboxControl() {
    // Update Xbox controller state
    xbox.update();
    
    // Track connection and stop states
    static bool wasConnected = false;
    static bool wasStopped = true;
    bool isConnected = xbox.isConnected();
    
    if (isConnected != wasConnected) {
        updateModeIndicator();
        wasConnected = isConnected;
        
        if (isConnected) {
            Serial.printf("Xbox controller active: %s\n", xbox.getControllerName());
            wasStopped = false;  // Reset stop flag when controller connects
        } else {
            Serial.println("Xbox controller lost connection");
        }
    }
    
    // Control motors if connected
    if (isConnected) {
        int leftSpeed, rightSpeed;
        xbox.getMotorSpeeds(leftSpeed, rightSpeed);
        
        // Only update motors if speeds have changed significantly
        static int lastLeftSpeed = 0;
        static int lastRightSpeed = 0;
        
        if (abs(leftSpeed - lastLeftSpeed) > 5 || abs(rightSpeed - lastRightSpeed) > 5) {
            motors.setMotors(leftSpeed, rightSpeed);
            lastMotorSpeedA = leftSpeed;
            lastMotorSpeedB = rightSpeed;
            lastLeftSpeed = leftSpeed;
            lastRightSpeed = rightSpeed;
            
            #ifdef DEBUG_XBOX
            Serial.printf("Xbox Control - Left: %d, Right: %d\n", leftSpeed, rightSpeed);
            #endif
        }
    } else {
        // No controller connected - only stop once
        if (!wasStopped) {
            motors.stop();
            lastMotorSpeedA = 0;
            lastMotorSpeedB = 0;
            wasStopped = true;
            Serial.println("RC Mode: Motors stopped (no controller)");
        }
    }
}

void handleMotorCommand(int speedA, int speedB) {
    // Only accept UART commands in UART mode
    if (currentMode != MODE_UART_CONTROL) {
        Serial.println("UART command ignored - not in UART mode");
        return;
    }
    
    Serial.printf("Motor command received: A=%d, B=%d\n", speedA, speedB);
    motors.setMotors(speedA, speedB);
    lastMotorSpeedA = speedA;
    lastMotorSpeedB = speedB;
}

void handleAutonomousMode() {
    static unsigned long lastMapUpdate = 0;
    static unsigned long lastMapPrint = 0;
    static unsigned long lastPlannerKick = 0;
    
    // Get current distance from ultrasonic sensor
    float distance = sensorData.distance;
    
    // Update IMU data for orientation detection
    #if IMU_ENABLED
    autoNav.updateIMU(sensorData.accelX, sensorData.accelY, sensorData.accelZ,
                      sensorData.gyroX, sensorData.gyroY, sensorData.gyroZ);
    #endif
    
    // Get motor speeds - will be calculated after path planner update
    int leftSpeed, rightSpeed;
    
    // Update occupancy map first
    unsigned long now = millis();
    float deltaTime = (now - lastMapUpdate) / 1000.0f;
    
    // Get heading from gyro-integrated estimate in autoNav
    #if IMU_ENABLED
    float heading = autoNav.getHeading();
    #else
    float heading = 0;  // No IMU = can't track heading accurately
    #endif
    
    if (lastMapUpdate > 0 && deltaTime < 1.0f) {
        // Use last frame's motor speeds for position estimation
        float avgSpeed = (abs(lastMotorSpeedA) + abs(lastMotorSpeedB)) / 2.0f;
        float speedPercent = (avgSpeed / 255.0f) * 100.0f;
        
        // Negative if going backwards
        if (lastMotorSpeedA < 0 && lastMotorSpeedB < 0) {
            speedPercent = -speedPercent;
        }
        
        // Update robot position on map
        envMap.updatePosition(heading, speedPercent, deltaTime);
        
        // Add ultrasonic reading to map
        envMap.addReading(distance, heading);
    }
    lastMapUpdate = now;
    
    // UPDATE PATH PLANNER - disabled, reactive nav works better
    autoNav.clearTargetHeading();
    
    // Pass side distances and wall angle for informed turn decisions
    autoNav.setSideDistances(sensorData.distanceLeft, sensorData.distanceRight);
    autoNav.setWallAngle(sensorData.wallAngle);
    
    // Update autonomous navigation logic
    autoNav.update(distance);
    
    // Get motor speeds from autonomous navigator
    autoNav.getMotorSpeeds(leftSpeed, rightSpeed);
    
    // Apply motor speeds
    motors.setMotors(leftSpeed, rightSpeed);
    lastMotorSpeedA = leftSpeed;
    lastMotorSpeedB = rightSpeed;
    
    // Debug output every 500ms
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 500) {
        #if IMU_ENABLED
        const char* orientStr = 
            autoNav.isUpsideDown() ? "FLIPPED" :
            autoNav.isOnSteepIncline() ? "STEEP" :
            (abs(autoNav.getPitch()) > 15 || abs(autoNav.getRoll()) > 15) ? "TILTED" : "OK";
        Serial.printf("[AUTO] %s | Fwd:%.0f L:%.0f R:%.0f | Wall:%.0f° | ML:%d MR:%d | %s%s%s%s\n", 
                      autoNav.getStateString(), distance, sensorData.distanceLeft, sensorData.distanceRight,
                      sensorData.wallAngle, leftSpeed, rightSpeed, orientStr,
                      autoNav.isMotionVerified() ? "" : " NO-MOTION",
                      autoNav.isRecentlyStuck() ? " SOFT-RAMP" : "",
                      autoNav.isSendItActive() ? " SEND-IT" : "");
        #else
        Serial.printf("[AUTO] %s | Fwd:%.0f L:%.0f R:%.0f | Wall:%.0f° | ML:%d MR:%d\n", 
                      autoNav.getStateString(), distance, sensorData.distanceLeft, sensorData.distanceRight,
                      sensorData.wallAngle, leftSpeed, rightSpeed);
        #endif
        lastDebug = millis();
    }
    
    // Print map periodically
    if (now - lastMapPrint > MAP_PRINT_INTERVAL) {
        envMap.printMap();
        lastMapPrint = now;
    }
}

void handleSimpleAutonomousMode() {
    // PID-based speed control with terrain awareness:
    //   Phase 0 = PID forward, Phase 1 = turning, Phase 2 = reversing,
    //   Phase 3 = recovery (rock/diagonal/full-reverse), Phase 4 = incline handling
    static int phase = 0;
    static unsigned long actionUntil = 0;   // Timer for timed actions
    static float pidI = 0;                  // PID integral
    static float pidLastErr = 0;            // PID last error
    static int tiltCount = 0;
    static unsigned long stallStart = 0;    // Stall timer

    // === Terrain tracking state (persistent across calls) ===
    static float accelMagBuf[MOTION_VERIFY_WINDOW]; // Accel magnitude window
    static int accelMagIdx = 0;
    static bool accelMagReady = false;
    static bool simpleMotionOK = true;
    static unsigned long simpleNoMotionStart = 0;
    static unsigned long simpleSteepStart = 0;
    static int simpleInclineAttempts = 0;
    static bool simpleRecentlyStuck = false;
    static unsigned long simpleStuckTime = 0;
    // Recovery sub-state
    static int recoveryStep = 0;   // 0=rock_fwd,1=rock_rev,2=diag_turn,3=diag_fwd,4=full_rev
    static int rockCycles = 0;
    static unsigned long recoveryUntil = 0;
    static int recoveryTurnDir = 0; // 0=right, 1=left
    
    // Anti-spin "Send It" tracking
    static bool simpleSendItActive = false;
    static unsigned long simpleSendItUntil = 0;
    static int simpleSendItTurnCount = 0;
    static unsigned long simpleSendItWindowStart = 0;
    static unsigned long simpleSendItFwdStart = 0;

    // PID gains
    const float KP = 5.0f;
    const float KI = 0.1f;
    const float KD = 1.5f;
    const float I_MAX = 2000.0f;

    const float OBSTACLE_CM = 25.0f;
    const float CRITICAL_CM = 10.0f;
    const float STALL_DIST_CM = 50.0f;
    const unsigned long STALL_MS = 2500;
    const int TURN_SPEED = MAX_PWM;
    const int REVERSE_SPEED = MAX_PWM - 10;
    const int MIN_SPEED = MAX_PWM - 50;
    const unsigned long TURN_MS = 800;
    const unsigned long REVERSE_MS = 1000;
    const float TILT_THRESHOLD_DEG = 35.0f;
    const int TILT_DEBOUNCE = 3;
    static unsigned long lastSimpleDecision = 0;

    float distance = sensorData.distance;
    float distL = sensorData.distanceLeft;
    float distR = sensorData.distanceRight;
    unsigned long now = millis();

    // Handle invalid distance
    if (distance < 0 || distance > 400) distance = OBSTACLE_CM - 1.0f;
    if (distL < 0 || distL > 400) distL = OBSTACLE_CM - 1.0f;
    if (distR < 0 || distR > 400) distR = OBSTACLE_CM - 1.0f;

    // Calculate pitch from accelerometer
    float pitch = 0;
    #if IMU_ENABLED
    float ax = sensorData.accelX;
    float ay = sensorData.accelY;
    float az = sensorData.accelZ;
    pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
    
    // === IMU MOVEMENT VERIFICATION ===
    float accelMag = sqrt(ax*ax + ay*ay + az*az);
    accelMagBuf[accelMagIdx] = accelMag;
    accelMagIdx = (accelMagIdx + 1) % MOTION_VERIFY_WINDOW;
    if (accelMagIdx == 0) accelMagReady = true;
    
    if (accelMagReady) {
        float sum = 0;
        for (int i = 0; i < MOTION_VERIFY_WINDOW; i++) sum += accelMagBuf[i];
        float mean = sum / MOTION_VERIFY_WINDOW;
        float variance = 0;
        for (int i = 0; i < MOTION_VERIFY_WINDOW; i++) {
            float d = accelMagBuf[i] - mean;
            variance += d * d;
        }
        variance /= MOTION_VERIFY_WINDOW;
        simpleMotionOK = (variance > MOTION_ACCEL_VAR_THRESH);
    }
    #endif

    // Track consecutive tilt readings
    if (pitch > TILT_THRESHOLD_DEG) {
        tiltCount++;
    } else {
        tiltCount = 0;
    }

    // === ADAPTIVE RAMP TIMEOUT ===
    if (simpleRecentlyStuck && (now - simpleStuckTime > ADAPTIVE_RAMP_TIMEOUT)) {
        simpleRecentlyStuck = false;
        Serial.println("[SIMPLE] Adaptive ramp expired");
    }

    int leftSpeed = 0;
    int rightSpeed = 0;

    // === PHASE 3: RECOVERY STATE MACHINE ===
    if (phase == 3) {
        if (now < recoveryUntil) {
            // Still executing current recovery step — apply motor commands
            switch (recoveryStep) {
                case 0: // Rock forward
                    leftSpeed = MAX_PWM; rightSpeed = MAX_PWM; break;
                case 1: // Rock backward
                    leftSpeed = -MAX_PWM; rightSpeed = -MAX_PWM; break;
                case 2: // Diagonal turn
                    if (recoveryTurnDir == 1) { leftSpeed = -MAX_PWM; rightSpeed = MAX_PWM; }
                    else { leftSpeed = MAX_PWM; rightSpeed = -MAX_PWM; }
                    break;
                case 3: // Diagonal forward
                    leftSpeed = REVERSE_SPEED; rightSpeed = REVERSE_SPEED; break;
                case 4: // Full reverse
                    leftSpeed = -MAX_PWM; rightSpeed = -MAX_PWM; break;
            }
        } else {
            // Current step done — advance
            switch (recoveryStep) {
                case 0: // Rock fwd done → rock rev
                    recoveryStep = 1;
                    recoveryUntil = now + RECOVERY_ROCK_MS;
                    Serial.printf("[SIMPLE-REC] Rock REV (%d/%d)\n", rockCycles+1, RECOVERY_ROCK_ATTEMPTS);
                    leftSpeed = -MAX_PWM; rightSpeed = -MAX_PWM;
                    break;
                case 1: // Rock rev done → more rocks or diagonal
                    rockCycles++;
                    if (rockCycles < RECOVERY_ROCK_ATTEMPTS) {
                        recoveryStep = 0;
                        recoveryUntil = now + RECOVERY_ROCK_MS;
                        Serial.printf("[SIMPLE-REC] Rock FWD (%d/%d)\n", rockCycles+1, RECOVERY_ROCK_ATTEMPTS);
                        leftSpeed = MAX_PWM; rightSpeed = MAX_PWM;
                    } else {
                        recoveryStep = 2;
                        recoveryUntil = now + RECOVERY_DIAG_TURN_MS;
                        Serial.println("[SIMPLE-REC] Diagonal TURN");
                        if (recoveryTurnDir == 1) { leftSpeed = -MAX_PWM; rightSpeed = MAX_PWM; }
                        else { leftSpeed = MAX_PWM; rightSpeed = -MAX_PWM; }
                    }
                    break;
                case 2: // Diag turn done → diag forward
                    recoveryStep = 3;
                    recoveryUntil = now + RECOVERY_DIAG_FWD_MS;
                    Serial.println("[SIMPLE-REC] Diagonal FWD");
                    leftSpeed = REVERSE_SPEED; rightSpeed = REVERSE_SPEED;
                    break;
                case 3: // Diag fwd done → check if free
                    if (simpleMotionOK && distance > OBSTACLE_CM) {
                        Serial.println("[SIMPLE-REC] === FREE! ===");
                        phase = 0; pidI = 0; pidLastErr = 0;
                        simpleRecentlyStuck = true; simpleStuckTime = now;
                        simpleNoMotionStart = 0;
                        leftSpeed = MIN_SPEED; rightSpeed = MIN_SPEED;
                    } else {
                        recoveryStep = 4;
                        recoveryUntil = now + RECOVERY_FULL_REV_MS;
                        Serial.println("[SIMPLE-REC] FULL REVERSE");
                        leftSpeed = -MAX_PWM; rightSpeed = -MAX_PWM;
                    }
                    break;
                case 4: // Full reverse done → recovery complete
                    Serial.println("[SIMPLE-REC] === Recovery complete ===");
                    phase = 1; // Turn to new direction
                    actionUntil = now + 800;
                    pidI = 0;
                    simpleRecentlyStuck = true; simpleStuckTime = now;
                    simpleNoMotionStart = 0;
                    if (distL > distR) { leftSpeed = -TURN_SPEED; rightSpeed = TURN_SPEED; }
                    else { leftSpeed = TURN_SPEED; rightSpeed = -TURN_SPEED; }
                    break;
            }
        }
        motors.setMotors(leftSpeed, rightSpeed);
        lastMotorSpeedA = leftSpeed;
        lastMotorSpeedB = rightSpeed;
        // Debug
        static unsigned long lastRecDbg = 0;
        if (now - lastRecDbg > 500) {
            const char* stepNames[] = {"ROCK_FWD","ROCK_REV","DIAG_TURN","DIAG_FWD","FULL_REV"};
            Serial.printf("[SIMPLE] RECOVERY %s | Fwd:%.0f | ML:%d MR:%d\n",
                          stepNames[recoveryStep], distance, leftSpeed, rightSpeed);
            lastRecDbg = now;
        }
        return;
    }

    // === SEND-IT MODE: Override navigation for anti-spin ===
    if (simpleSendItActive) {
        if (now >= simpleSendItUntil) {
            // Send-it expired — resume normal navigation
            simpleSendItActive = false;
            simpleSendItTurnCount = 0;
            simpleSendItWindowStart = 0;
            simpleSendItFwdStart = 0;
            phase = 0; pidI = 0; pidLastErr = 0;
            Serial.println("[SENDIT] === Send-it complete — resuming normal nav ===");
        } else if (distance < CRITICAL_CM) {
            // Even in send-it, respect collision distance
            Serial.printf("[SENDIT] CRITICAL %.0fcm — emergency reverse\n", distance);
            leftSpeed = -REVERSE_SPEED; rightSpeed = -REVERSE_SPEED;
            // Brief reverse, don't cancel send-it
            motors.setMotors(leftSpeed, rightSpeed);
            lastMotorSpeedA = leftSpeed; lastMotorSpeedB = rightSpeed;
            return;
        } else {
            // Force forward — ignore OBSTACLE_CM
            int speed = REVERSE_SPEED;
            if (simpleRecentlyStuck) speed = MIN_SPEED;
            leftSpeed = speed; rightSpeed = speed;
            motors.setMotors(leftSpeed, rightSpeed);
            lastMotorSpeedA = leftSpeed; lastMotorSpeedB = rightSpeed;
            // Debug
            static unsigned long lastSendItDbg = 0;
            if (now - lastSendItDbg > 500) {
                unsigned long remaining = (simpleSendItUntil > now) ? (simpleSendItUntil - now) : 0;
                Serial.printf("[SENDIT] DRIVING THROUGH | Fwd:%.0f L:%.0f R:%.0f | %lums left | ML:%d MR:%d\n",
                              distance, distL, distR, remaining, leftSpeed, rightSpeed);
                lastSendItDbg = now;
            }
            return;
        }
    }

    // Check if timed action (turn/reverse) has expired
    if (phase != 0 && now >= actionUntil) {
        phase = 0;
        pidI = 0;
        pidLastErr = 0;
        lastSimpleDecision = now;
    }

    // Decision cooldown — prevent rapid state thrashing
    if (phase == 0 && (now - lastSimpleDecision) < DECISION_COOLDOWN_MS && distance > CRITICAL_CM) {
        // Keep current motor speeds, just don't make a new decision yet
        motors.setMotors(leftSpeed, rightSpeed);
        lastMotorSpeedA = leftSpeed; lastMotorSpeedB = rightSpeed;
        return;
    }

    if (phase == 0) {
        // === ANTI-SPIN: Track forward progress & window expiry ===
        // If driving forward for long enough, we're making progress — reset turn count
        if (leftSpeed > 0 && rightSpeed > 0) {
            if (simpleSendItFwdStart == 0) simpleSendItFwdStart = now;
            if (now - simpleSendItFwdStart > SENDIT_FWD_PROGRESS_MS) {
                if (simpleSendItTurnCount > 0) {
                    Serial.printf("[SENDIT] Forward progress — clearing %d turn count\n", simpleSendItTurnCount);
                }
                simpleSendItTurnCount = 0;
                simpleSendItWindowStart = 0;
            }
        } else {
            simpleSendItFwdStart = 0;
        }
        // Reset window if it expired without triggering
        if (simpleSendItWindowStart > 0 && (now - simpleSendItWindowStart > SENDIT_WINDOW_MS)) {
            simpleSendItTurnCount = 0;
            simpleSendItWindowStart = 0;
        }
        
        // === IMU MOVEMENT VERIFICATION: Trigger recovery ===
        #if IMU_ENABLED
        if (!simpleMotionOK && leftSpeed >= MIN_SPEED) {
            if (simpleNoMotionStart == 0) {
                simpleNoMotionStart = now;
            } else if (now - simpleNoMotionStart > MOTION_VERIFY_TIMEOUT) {
                Serial.printf("[SIMPLE] No motion for %lums → RECOVERY\n", now - simpleNoMotionStart);
                phase = 3;
                recoveryStep = 0; rockCycles = 0;
                recoveryUntil = now + RECOVERY_ROCK_MS;
                recoveryTurnDir = (distL > distR) ? 1 : 0;
                Serial.println("[SIMPLE-REC] === Starting recovery ===");
                motors.setMotors(MAX_PWM, MAX_PWM);
                lastMotorSpeedA = MAX_PWM; lastMotorSpeedB = MAX_PWM;
                return;
            }
        } else {
            simpleNoMotionStart = 0;
        }
        
        // === INCLINE CHECK ===
        float absPitch = abs(pitch);
        if (absPitch > INCLINE_MAX_PITCH) {
            if (simpleSteepStart == 0) {
                simpleSteepStart = now;
                Serial.printf("[SIMPLE] Steep incline: pitch=%.1f°\n", pitch);
            } else if (now - simpleSteepStart > INCLINE_TIMEOUT_MS) {
                if (simpleInclineAttempts < 2) {
                    // Try diagonal approach
                    simpleInclineAttempts++;
                    Serial.printf("[SIMPLE] Incline timeout → diagonal attempt #%d\n", simpleInclineAttempts);
                    phase = 2; // Reverse first
                    actionUntil = now + REVERSE_MS;
                    simpleSteepStart = 0;
                } else {
                    // Unclimbable — full recovery
                    Serial.println("[SIMPLE] Incline UNCLIMBABLE → RECOVERY");
                    phase = 3;
                    recoveryStep = 0; rockCycles = 0;
                    recoveryUntil = now + RECOVERY_ROCK_MS;
                    recoveryTurnDir = (simpleInclineAttempts % 2 == 0) ? 1 : 0;
                    simpleInclineAttempts = 0;
                    simpleSteepStart = 0;
                    motors.setMotors(MAX_PWM, MAX_PWM);
                    lastMotorSpeedA = MAX_PWM; lastMotorSpeedB = MAX_PWM;
                    return;
                }
            }
        } else {
            if (simpleSteepStart > 0) {
                Serial.printf("[SIMPLE] Incline cleared (pitch=%.1f°)\n", pitch);
            }
            simpleSteepStart = 0;
            simpleInclineAttempts = 0;
        }
        #endif

        // Priority 1: Tilt → reverse
        if (tiltCount >= TILT_DEBOUNCE) {
            phase = 2;
            actionUntil = now + REVERSE_MS;
            tiltCount = 0;
            pidI = 0;
            lastSimpleDecision = now;
            Serial.printf("[SIMPLE] Tilt → REVERSE %lums\n", REVERSE_MS);
        }
        // Priority 2: Critical → reverse
        else if (distance < CRITICAL_CM) {
            phase = 2;
            actionUntil = now + REVERSE_MS;
            pidI = 0;
            lastSimpleDecision = now;
            Serial.printf("[SIMPLE] Critical %.0fcm → REVERSE\n", distance);
        }
        // Priority 3: Close → turn AWAY from closer side
        else if (distance < OBSTACLE_CM) {
            // === COUNT THIS TURN FOR SPIN DETECTION ===
            if (simpleSendItWindowStart == 0) simpleSendItWindowStart = now;
            simpleSendItTurnCount++;
            
            // Check if spin threshold reached
            if (simpleSendItTurnCount >= SENDIT_TURN_THRESHOLD) {
                Serial.printf("[SENDIT] === SEND IT! %d turns in %.1fs — driving through! ===\n",
                              simpleSendItTurnCount, (now - simpleSendItWindowStart) / 1000.0f);
                simpleSendItActive = true;
                simpleSendItUntil = now + SENDIT_DURATION_MS;
                simpleSendItTurnCount = 0;
                simpleSendItWindowStart = 0;
                simpleSendItFwdStart = 0;
                leftSpeed = REVERSE_SPEED; rightSpeed = REVERSE_SPEED;
                pidI = 0;
                motors.setMotors(leftSpeed, rightSpeed);
                lastMotorSpeedA = leftSpeed; lastMotorSpeedB = rightSpeed;
                return;
            }
            
            phase = 1;
            actionUntil = now + TURN_MS;
            pidI = 0;
            lastSimpleDecision = now;
            if (distL <= distR) {
                leftSpeed = TURN_SPEED;
                rightSpeed = -TURN_SPEED;
                Serial.printf("[SIMPLE] Obstacle L:%.0f R:%.0fcm → TURN RIGHT (spin count: %d/%d)\n", 
                              distL, distR, simpleSendItTurnCount, SENDIT_TURN_THRESHOLD);
            } else {
                leftSpeed = -TURN_SPEED;
                rightSpeed = TURN_SPEED;
                Serial.printf("[SIMPLE] Obstacle L:%.0f R:%.0fcm → TURN LEFT (spin count: %d/%d)\n", 
                              distL, distR, simpleSendItTurnCount, SENDIT_TURN_THRESHOLD);
            }
        }
        // Otherwise: PID forward
        else {
            float error = distance - OBSTACLE_CM;
            float dt = 0.05f;  // ~20Hz

            pidI += error * dt;
            pidI = constrain(pidI, -I_MAX, I_MAX);

            float derivative = (error - pidLastErr) / dt;
            pidLastErr = error;

            float output = (KP * error) + (KI * pidI) + (KD * derivative);
            int speed = constrain((int)output, MIN_SPEED, 255);
            
            // === ADAPTIVE RAMP: softer acceleration after recent stuck ===
            if (simpleRecentlyStuck) {
                static int simpleCurrentSpeed = 0;
                int diff = speed - simpleCurrentSpeed;
                if (abs(diff) > ADAPTIVE_RAMP_SLOW) {
                    simpleCurrentSpeed += (diff > 0) ? ADAPTIVE_RAMP_SLOW : -ADAPTIVE_RAMP_SLOW;
                } else {
                    simpleCurrentSpeed = speed;
                }
                speed = constrain(simpleCurrentSpeed, MIN_SPEED, 255);
            }

            leftSpeed = speed;
            rightSpeed = speed;
            
            // Stall detection
            if (distance < STALL_DIST_CM) {
                if (stallStart == 0) {
                    stallStart = now;
                } else if (now - stallStart > STALL_MS) {
                    // Stall detected → enter recovery instead of simple turn
                    Serial.println("[SIMPLE] STALL → RECOVERY");
                    phase = 3;
                    recoveryStep = 0; rockCycles = 0;
                    recoveryUntil = now + RECOVERY_ROCK_MS;
                    recoveryTurnDir = (distL > distR) ? 1 : 0;
                    stallStart = 0;
                    motors.setMotors(MAX_PWM, MAX_PWM);
                    lastMotorSpeedA = MAX_PWM; lastMotorSpeedB = MAX_PWM;
                    return;
                }
            } else {
                stallStart = 0;
            }
        }
    }

    if (phase == 1) {
        // TURNING
        if (leftSpeed == 0 && rightSpeed == 0) {
            if (distL <= distR) {
                leftSpeed = TURN_SPEED;
                rightSpeed = -TURN_SPEED;
            } else {
                leftSpeed = -TURN_SPEED;
                rightSpeed = TURN_SPEED;
            }
        }
    } else if (phase == 2) {
        // REVERSING
        leftSpeed = -REVERSE_SPEED;
        rightSpeed = -REVERSE_SPEED;
    }

    motors.setMotors(leftSpeed, rightSpeed);
    lastMotorSpeedA = leftSpeed;
    lastMotorSpeedB = rightSpeed;

    static unsigned long lastDebug = 0;
    if (now - lastDebug > 500) {
        const char* stateStr = (phase == 0) ? "PID-FWD" : (phase == 1) ? "TURN" : "REVERSE";
        #if IMU_ENABLED
        Serial.printf("[SIMPLE] %s | Fwd:%.0f L:%.0f R:%.0f | pitch:%.0f° | %s%s%s | ML:%d MR:%d\n",
                      stateStr, distance, distL, distR, pitch,
                      simpleMotionOK ? "" : "NO-MOTION ",
                      simpleRecentlyStuck ? "SOFT-RAMP " : "",
                      simpleSendItActive ? "SEND-IT" : "",
                      leftSpeed, rightSpeed);
        #else
        Serial.printf("[SIMPLE] %s | Fwd:%.0f L:%.0f R:%.0f | Wall:%.0f° Gap:%.0f | ML:%d MR:%d\n",
                      stateStr, distance, distL, distR, sensorData.wallAngle, sensorData.gapWidth,
                      leftSpeed, rightSpeed);
        #endif
        lastDebug = now;
    }
}

void handleCustomCommand(uint8_t cmd, uint8_t* data, uint8_t length) {
    // Only accept UART commands in UART mode
    if (currentMode != MODE_UART_CONTROL) {
        Serial.println("UART command ignored - not in UART mode");
        return;
    }
    
    Serial.printf("Custom command received: 0x%02X\n", cmd);
    
    switch (cmd) {
        case CMD_STOP:
            motors.stop();
            lastMotorSpeedA = 0;
            lastMotorSpeedB = 0;
            Serial.println("Motors stopped");
            break;
            
        case CMD_FORWARD:
            if (length >= 1) {
                int speed = map(data[0], 0, 100, 0, 255);
                motors.forward(speed);
                lastMotorSpeedA = speed;
                lastMotorSpeedB = speed;
                Serial.printf("Moving forward at speed %d\n", speed);
            }
            break;
            
        case CMD_BACKWARD:
            if (length >= 1) {
                int speed = map(data[0], 0, 100, 0, 255);
                motors.backward(speed);
                lastMotorSpeedA = -speed;
                lastMotorSpeedB = -speed;
                Serial.printf("Moving backward at speed %d\n", speed);
            }
            break;
            
        case CMD_LEFT:
            if (length >= 1) {
                int speed = map(data[0], 0, 100, 0, 255);
                motors.turnLeft(speed);
                lastMotorSpeedA = -speed;
                lastMotorSpeedB = speed;
                Serial.printf("Turning left at speed %d\n", speed);
            }
            break;
            
        case CMD_RIGHT:
            if (length >= 1) {
                int speed = map(data[0], 0, 100, 0, 255);
                motors.turnRight(speed);
                lastMotorSpeedA = speed;
                lastMotorSpeedB = -speed;
                Serial.printf("Turning right at speed %d\n", speed);
            }
            break;
            
        case CMD_STANDBY:
            if (length >= 1) {
                bool enable = data[0] > 0;
                motors.standby(enable);
                Serial.printf("Standby mode: %s\n", enable ? "ON" : "OFF");
            }
            break;
            
        case CMD_GET_SENSORS:
            sendSensorDataViaUART();
            Serial.println("Sensor data sent");
            break;
            
        default:
            Serial.printf("Unknown command: 0x%02X\n", cmd);
            break;
    }
}
