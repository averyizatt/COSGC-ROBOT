#include <Arduino.h>
#include <cstring>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_bt.h"
#include "config.h"
#include "motor_control.h"
#include "ultrasonic_sensor.h"
#include "mpu6050_sensor.h"
#include "uart_comm.h"
#include "xbox_controller.h"
#include "button_handler.h"
#include "led_controller.h"
#include "autonomous_nav.h"
#include "self_righting.h"
#include "occupancy_map.h"
#include "path_planner.h"
#include "hall_encoder.h"
#include "web_status.h" // Web server for status page
#include "globals.h"

// Create instances of all components
UltrasonicSensor ultrasonicLeft(ULTRASONIC_TRIG, ULTRASONIC_ECHO);    // Left sensor (~15° left)
UltrasonicSensor ultrasonicRight(ULTRASONIC2_TRIG, ULTRASONIC2_ECHO); // Right sensor (~15° right)
MPU6050Sensor imu;
UARTComm uart;
XboxController xbox;
ButtonHandler button;
LEDController led;
AutonomousNav autoNav;
SelfRightingArm selfRighting;
PathPlanner pathPlanner;  // Wavefront path planner


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

// === RUNTIME FAULT FLAGS ===
// Set when a sensor produces implausible data at runtime; cleared on recovery.
bool imuFaultDetected   = false;  // IMU accel magnitude near-zero (frozen/disconnected)
bool encoderFaultDetected = false; // Both encoders silent while motors commanded high

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
void handleWallFollowMode();
void handlePremapNavMode();
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
    // Note: start Serial first so webserver setup can print status to serial
    // (setupWebServer() is called after Serial.begin below)
    // Disable brownout detector — motor current spikes cause false resets
    // Method 1: Zero entire register (covers all ESP32 variants)
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    // Method 2: Clear just the enable bit specifically
    REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);
    
    // Initialize serial for debugging
    Serial.begin(115200);
    delay(2000);  // Longer delay to ensure serial port is stable
    Serial.flush();  // Flush any pending serial data
    Serial.println("\n\nESP32 Rover Control System Starting...");
    Serial.println("[BOOT] Brownout detector disabled");
    printResetReason();
    
    // Initialize button
    Serial.println("Initializing button...");
    button.begin();
    
    // Initialize LED
    Serial.println("Initializing LED...");
    led.begin();
    led.setMode(LED_SOLID_RED);  // Start with red (UART mode)
    
    // Initialize self-righting arm
    Serial.println("Initializing self-righting arm...");
    selfRighting.begin();
    
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
    Serial.printf("Ultrasonic LEFT  (TRIG=GPIO%d ECHO=GPIO%d) initialized OK\n", ULTRASONIC_TRIG, ULTRASONIC_ECHO);
    ultrasonicRight.begin();
    Serial.printf("Ultrasonic RIGHT (TRIG=GPIO%d ECHO=GPIO%d) initialized OK\n", ULTRASONIC2_TRIG, ULTRASONIC2_ECHO);
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

    // Kill Bluetooth FIRST to free ~60KB heap before WiFi/map allocation.
    // Bluepad32's framework auto-starts BT during initArduino(), kill it now.
    // BT will start only when switching to RC Control mode.
    btStop();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();   // Release all BT memory
    Serial.printf("Bluetooth fully released (heap free: %u)\n", ESP.getFreeHeap());

    // Start WiFi AP while heap is clean (needs ~40-60KB contiguous)
    Serial.println("Starting WiFi AP and web server...");
    setupWebServer();
    Serial.println("[WiFi] Web dashboard ready at http://192.168.4.1");

    // Allocate occupancy map AFTER WiFi is up (80KB, ok with fragmented heap)
    Serial.println("Initializing occupancy map...");
    envMap.begin();
    
    // Initialize path planner with map
    Serial.println("Initializing path planner...");
    pathPlanner.begin(&envMap);
    
    Serial.println("System initialization complete!");
    Serial.printf("Current mode: %s\n", 
                  currentMode == MODE_RC_CONTROL   ? "RC Control"   : 
                  currentMode == MODE_UART_CONTROL ? "UART Control" :
                  currentMode == MODE_AUTONOMOUS   ? "Autonomous"   :
                  currentMode == MODE_SIMPLE_AUTO  ? "Simple Auto"  :
                  currentMode == MODE_WALL_FOLLOW  ? "Wall Follow"  : "Premap Nav");
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
    
    // Motor speed calibration — measure m/s at two PWM levels
    // Sequence: FWD max → pause → BACK max → pause → FWD half → pause → BACK half
    // Each phase 5s drive + 5s pause. Mark a start line and measure distance.
    #ifdef DEBUG_MOTORS
    {
        const int HALF_PWM = MAX_PWM / 2;   // ~128 PWM
        const unsigned long DRIVE_MS = 5000;
        const unsigned long PAUSE_MS = 5000;
        
        Serial.println("\n========== SPEED CALIBRATION TEST ==========");
        Serial.printf("  MAX_PWM = %d, HALF_PWM = %d\n", MAX_PWM, HALF_PWM);
        Serial.println("  Each phase: 5s drive, 5s pause");
        Serial.println("  Place robot on ground at a marked line.\n");
        
        // --- Phase 1: FORWARD at MAX_PWM ---
        Serial.printf("[CAL] Phase 1: FORWARD %d PWM for 5s...\n", MAX_PWM);
        {
            unsigned long start = millis();
            while (millis() - start < DRIVE_MS) {
                motors.forward(MAX_PWM);
                delay(SLEW_INTERVAL_MS);
            }
        }
        motors.stop();
        Serial.println("[CAL] Phase 1 DONE — measure distance from start");
        delay(PAUSE_MS);
        
        // --- Phase 2: BACKWARD at MAX_PWM ---
        Serial.printf("[CAL] Phase 2: BACKWARD %d PWM for 5s...\n", MAX_PWM);
        {
            unsigned long start = millis();
            while (millis() - start < DRIVE_MS) {
                motors.backward(MAX_PWM);
                delay(SLEW_INTERVAL_MS);
            }
        }
        motors.stop();
        Serial.println("[CAL] Phase 2 DONE — measure distance");
        delay(PAUSE_MS);
        
        // --- Phase 3: FORWARD at HALF_PWM ---
        Serial.printf("[CAL] Phase 3: FORWARD %d PWM for 5s...\n", HALF_PWM);
        {
            unsigned long start = millis();
            while (millis() - start < DRIVE_MS) {
                motors.forward(HALF_PWM);
                delay(SLEW_INTERVAL_MS);
            }
        }
        motors.stop();
        Serial.println("[CAL] Phase 3 DONE — measure distance");
        delay(PAUSE_MS);
        
        // --- Phase 4: BACKWARD at HALF_PWM ---
        Serial.printf("[CAL] Phase 4: BACKWARD %d PWM for 5s...\n", HALF_PWM);
        {
            unsigned long start = millis();
            while (millis() - start < DRIVE_MS) {
                motors.backward(HALF_PWM);
                delay(SLEW_INTERVAL_MS);
            }
        }
        motors.stop();
        Serial.println("[CAL] Phase 4 DONE — measure distance");
        
        Serial.println("\n========== CALIBRATION COMPLETE ==========");
        Serial.println("  m/s = distance(m) / 5.0  for each phase");
        Serial.println("  Record: MAX fwd, MAX back, HALF fwd, HALF back\n");
    }
    #endif  // DEBUG_MOTORS
    
    // Send initial status
    uart.sendStatus("READY");
    
    // Initialize hall effect encoders
    encoders.begin();
    
    Serial.println("\n*** SETUP COMPLETE - ENTERING MAIN LOOP ***\n");
}

void loop() {
    // Web server is async — no handleClient() needed
    static unsigned long loopCount = 0;
    static unsigned long lastHeartbeat = 0;
    unsigned long currentMillis = millis();

    loopCount++;

    // Safety check FIRST — thermal, watchdog, duty cycle
    motors.safetyCheck();

    // Print heartbeat every 30 seconds (reduced from 5s to cut serial load)
    if (currentMillis - lastHeartbeat >= 30000) {
        lastHeartbeat = currentMillis;
        Serial.printf("[HEARTBEAT] Loop count: %lu, uptime: %lus\n", loopCount, currentMillis / 1000);
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
        // Feed IMU forward acceleration to encoder before update (for complementary filter)
        if (IMU_ENABLED) {
            encoders.setForwardAccel(sensorData.accelX);
        }
        encoders.update();  // Compute speed/distance from pulse counts
        // Propagate encoder pose to shared odometry globals (used by AutonomousNav + web API)
        odomX     = encoders.getX();
        odomY     = encoders.getY();
        odomTheta = encoders.getTheta();
    }

    // Raw ultrasonic debug print (gated behind DEBUG_SENSORS to avoid serial overhead)
    #ifdef DEBUG_SENSORS
    {
        static unsigned long lastUsPrint = 0;
        if (currentMillis - lastUsPrint >= 500) {
            lastUsPrint = currentMillis;
            Serial.printf("[US RAW] L: %.1f cm  R: %.1f cm  (filtered L:%.1f R:%.1f)\n",
                          lastRawLeft, lastRawRight, sensorData.distanceLeft, sensorData.distanceRight);
        }
    }
    #endif

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
    } else if (currentMode == MODE_WALL_FOLLOW) {
        // Wall-follow mode - same update rate
        if (currentMillis - lastAutonomousUpdate >= AUTONOMOUS_UPDATE_INTERVAL) {
            lastAutonomousUpdate = currentMillis;
            handleWallFollowMode();
        }
    } else if (currentMode == MODE_PREMAP_NAV) {
        // Premap nav — identical behaviour to autonomous but map was pre-seeded via web UI
        if (currentMillis - lastAutonomousUpdate >= AUTONOMOUS_UPDATE_INTERVAL) {
            lastAutonomousUpdate = currentMillis;
            handlePremapNavMode();
        }
    }
    
    // Pump DNS captive-portal server so phones get instant responses.
    // Must run every loop; loopWiFi() is a no-op when WiFi is off.
    loopWiFi();

    // Yield to RTOS — reduces CPU heat while keeping responsive timing
    // All timed work uses millis() checks, so 10ms sleep is fine
    delay(10);
}

void updateSensors() {
    // Read ultrasonic sensors — ALTERNATE between left and right each cycle
    // to halve the worst-case blocking time (~12ms max per sensor per cycle
    // instead of ~24ms for both). Each sensor still updates at 10Hz.
    static float lastGoodLeft = 30.0f;
    static float lastGoodRight = 30.0f;
    static bool readLeftThisCycle = true;
    
    bool leftHealthy = ultrasonicLeft.isHealthy();
    bool rightHealthy = ultrasonicRight.isHealthy();
    
    if (readLeftThisCycle) {
        float rawLeft = ultrasonicLeft.readDistance();
        lastRawLeft = rawLeft;
        if (rawLeft >= 2.0f && rawLeft <= (float)MAX_DISTANCE) {
            lastGoodLeft = rawLeft;
        }
    } else {
        float rawRight = ultrasonicRight.readDistance();
        lastRawRight = rawRight;
        if (rawRight >= 2.0f && rawRight <= (float)MAX_DISTANCE) {
            lastGoodRight = rawRight;
        }
    }
    readLeftThisCycle = !readLeftThisCycle;
    
    sensorData.distanceLeft = lastGoodLeft;
    sensorData.distanceRight = lastGoodRight;
    
    // Sensor failure fallback: if one sensor failed, use the other exclusively
    if (!leftHealthy && rightHealthy) {
        sensorData.distanceLeft = lastGoodRight;  // Mirror healthy sensor
    } else if (!rightHealthy && leftHealthy) {
        sensorData.distanceRight = lastGoodLeft;  // Mirror healthy sensor
    }
    
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
        // Feed MPU6050 temperature into ultrasonic sensors for speed-of-sound compensation
        ultrasonicLeft.setAmbientTemp(sensorData.temperature);
        ultrasonicRight.setAmbientTemp(sensorData.temperature);

        // === IMU PLAUSIBILITY / FAULT DETECTION ===
        // When upright and in 1g gravity, accel magnitude should be close to 1.0.
        // If it reads near-zero for >2s the IMU has frozen or lost I2C comms.
        {
            static unsigned long imuFlatStart = 0;
            float imuMag = sqrt(sensorData.accelX * sensorData.accelX +
                                sensorData.accelY * sensorData.accelY +
                                sensorData.accelZ * sensorData.accelZ);
            bool imuFlat = (imuMag < 0.15f);  // <0.15g is physically impossible in gravity
            if (imuFlat) {
                if (imuFlatStart == 0) imuFlatStart = millis();
                if (!imuFaultDetected && millis() - imuFlatStart > 2000) {
                    imuFaultDetected = true;
                    Serial.println("[FAULT] IMU accel magnitude near-zero >2s — IMU may be disconnected. Disabling IMU-dependent features.");
                }
            } else {
                imuFlatStart = 0;
                if (imuFaultDetected) {
                    imuFaultDetected = false;
                    Serial.printf("[FAULT] IMU recovered (mag=%.2f) — re-enabling IMU features.\n", imuMag);
                }
            }
        }
        // Guard all orientation-dependent logic against IMU faults.
        // If the IMU is returning garbage, keep motors normal and arm stowed.
        if (imuFaultDetected) {
            motors.setUpsideDown(false);
            selfRighting.update(false, false, 0.0f);
        } else {
        // AccelZ < threshold means gravity is pulling "up" relative to the board
        // Debounce: must be sustained for 500ms to avoid false triggers from vibration
        static unsigned long flipDetectStart = 0;
        static bool confirmedFlipped = false;
        bool rawFlipped = (sensorData.accelZ < UPSIDE_DOWN_THRESHOLD);
        
        if (rawFlipped) {
            if (flipDetectStart == 0) {
                flipDetectStart = millis();
            } else if (millis() - flipDetectStart > 500) {
                confirmedFlipped = true;
            }
        } else {
            flipDetectStart = 0;
            // Require sustained upright to clear (200ms)
            static unsigned long uprightStart = 0;
            if (uprightStart == 0) {
                uprightStart = millis();
            } else if (millis() - uprightStart > 200) {
                confirmedFlipped = false;
                uprightStart = 0;
            }
        }
        
        bool isFlipped = confirmedFlipped;
        motors.setUpsideDown(isFlipped);
        
        // Detect "on side" state using roll angle.
        // accelX is the lateral axis: +1g = right side down, -1g = left side down.
        float imuRoll = atan2(sensorData.accelX, sensorData.accelZ) * 180.0f / PI;
        static unsigned long sideDetectStart = 0;
        static bool confirmedOnSide = false;
        bool rawOnSide = (fabs(imuRoll) > ON_SIDE_ROLL_THRESHOLD);
        
        if (rawOnSide && !isFlipped) {
            if (sideDetectStart == 0) {
                sideDetectStart = millis();
            } else if (millis() - sideDetectStart > 500) {
                confirmedOnSide = true;
            }
        } else {
            sideDetectStart = 0;
            static unsigned long sideUprightStart = 0;
            if (confirmedOnSide) {
                if (sideUprightStart == 0) {
                    sideUprightStart = millis();
                } else if (millis() - sideUprightStart > 200) {
                    confirmedOnSide = false;
                    sideUprightStart = 0;
                }
            }
        }
        
        // Self-righting arm: handle upside-down, on-side, or normal
        selfRighting.update(isFlipped, confirmedOnSide, imuRoll);
        } // end !imuFaultDetected

        // Motors run during both righting and unsticking — helps the arm flip/free the robot.
        // The upside-down motor inversion in setMotors() already handles direction.
    } // end IMU_ENABLED
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
    Serial.printf("ESP32 Temp: %.1f°C%s%s\n", motors.getESPTemperature(),
                  motors.isThermalThrottled() ? " [THROTTLED]" : "",
                  motors.isDutyCycleThrottled() ? " [DUTY-LIM]" : "");
    Serial.printf("Mode: %s\n",
                  currentMode == MODE_RC_CONTROL   ? "RC Control"   :
                  currentMode == MODE_UART_CONTROL ? "UART Control" :
                  currentMode == MODE_AUTONOMOUS   ? "Autonomous"   :
                  currentMode == MODE_SIMPLE_AUTO  ? "Simple Auto"  :
                  currentMode == MODE_WALL_FOLLOW  ? "Wall Follow"  : "Premap Nav");
    if (IMU_ENABLED) {
        Serial.printf("Orientation: %s\n", motors.isUpsideDown() ? "UPSIDE DOWN" : "Right-side up");
    }
    Serial.println("===============================");
    if (!motors.isMotorsAllowed()) {
        Serial.println("*** MOTORS DISABLED (safety) ***");
    }
    Serial.println();
}

void handleButtonPress() {
    ButtonEvent event = button.update();
    
    if (event == BUTTON_SHORT_PRESS) {
        // Short press — cycle through modes (responds immediately on release)
        motors.stop();
        lastMotorSpeedA = 0;
        lastMotorSpeedB = 0;

        uint8_t prevMode = currentMode;

        if (currentMode == MODE_UART_CONTROL) {
            currentMode = MODE_RC_CONTROL;
            Serial.println("\n========================================");
            Serial.println("MODE SWITCHED TO: RC CONTROL (Xbox)");
            Serial.println("========================================\n");
        } else if (currentMode == MODE_RC_CONTROL) {
            currentMode = MODE_AUTONOMOUS;
            autoNav.reset();
            pathPlanner.startExploration();
            lastAutonomousUpdate = 0;
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
        } else if (currentMode == MODE_SIMPLE_AUTO) {
            currentMode = MODE_WALL_FOLLOW;
            pathPlanner.stop();
            lastAutonomousUpdate = 0;
            Serial.println("\n========================================");
            Serial.println("MODE SWITCHED TO: WALL FOLLOW");
            Serial.println("Behavior: Find left wall, trace perimeter (PID)");
            Serial.println("========================================\n");
        } else if (currentMode == MODE_WALL_FOLLOW) {
            currentMode = MODE_PREMAP_NAV;
            autoNav.reset();
            pathPlanner.startExploration();
            lastAutonomousUpdate = 0;
            Serial.println("\n========================================");
            Serial.println("MODE SWITCHED TO: PREMAP NAV");
            Serial.println("Behavior: Autonomous nav using web-drawn map");
            Serial.println("Draw map at http://192.168.4.1/map/draw");
            Serial.println("========================================\n");
        } else {
            currentMode = MODE_UART_CONTROL;
            pathPlanner.stop();  // Stop planner outside autonomous mode
            Serial.println("\n========================================");
            Serial.println("MODE SWITCHED TO: UART CONTROL");
            Serial.println("========================================\n");
        }

        // --- Toggle WiFi / Bluetooth based on mode ---
        if (currentMode == MODE_RC_CONTROL && prevMode != MODE_RC_CONTROL) {
            // Entering RC mode: shut down WiFi, start Bluetooth
            stopWiFi();
            delay(200);
            xbox.start();  // Calls begin() first time, btStart() after
            delay(200);
            Serial.println("[RADIO] WiFi OFF, Bluetooth ON");
        } else if (currentMode != MODE_RC_CONTROL && prevMode == MODE_RC_CONTROL) {
            // Leaving RC mode: shut down Bluetooth, start WiFi
            xbox.stop();   // btStop()
            delay(200);
            startWiFi();
            delay(200);
            Serial.println("[RADIO] Bluetooth OFF, WiFi ON");
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
    else if (event == BUTTON_LONG_PRESS) {
        // Long hold (800ms) — start Bluetooth pairing in RC mode
        if (currentMode == MODE_RC_CONTROL) {
            Serial.println("\n*** BUTTON LONG PRESS - Starting Bluetooth pairing ***");
            xbox.startPairing();
            led.setMode(LED_BLINK_BLUE_FAST);
        } else {
            Serial.println("\n*** BUTTON LONG PRESS - hold in RC mode to pair controller ***");
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
    } else if (currentMode == MODE_WALL_FOLLOW) {
        led.setMode(LED_SOLID_CYAN);    // Cyan for wall-follow mode
    } else if (currentMode == MODE_PREMAP_NAV) {
        led.setMode(LED_SOLID_LIME);    // Lime for premap nav mode
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
        
        // Always write motor commands — no dedup filtering
        motors.setMotors(leftSpeed, rightSpeed);
        lastMotorSpeedA = leftSpeed;
        lastMotorSpeedB = rightSpeed;
        
        // === SERVO CONTROL: RT = forward (0→180), LT = reverse (180→0) ===
        int rt = xbox.getRightTrigger();  // 0-1023
        int lt = xbox.getLeftTrigger();   // 0-1023
        if (rt > 20) {  // RT pressed: sweep 0→180
            int angle = map(rt, 20, 1023, 0, 180);
            selfRighting.setAngle(angle);
        } else if (lt > 20) {  // LT pressed: sweep 180→0 (reverse)
            int angle = map(lt, 20, 1023, 180, 0);
            selfRighting.setAngle(angle);
        } else {
            selfRighting.stow();  // Both released = stow servo
        }
        
        // Debug: print motor commands every 500ms
        static unsigned long lastRcDbg = 0;
        if (millis() - lastRcDbg > 500 && (leftSpeed != 0 || rightSpeed != 0)) {
            lastRcDbg = millis();
            Serial.printf("[RC] stick_Y=%d stick_X=%d → ML:%d MR:%d\n",
                          xbox.getLeftStickY(), xbox.getRightStickX(),
                          leftSpeed, rightSpeed);
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
    // Get current distance from ultrasonic sensor
    float distance = sensorData.distance;

    // === EMERGENCY HARD STOP ===
    // If critically close, force the nav state machine into AVOID immediately
    // so the backup is handled as a timed action (not an infinite reverse loop).
    // Only trigger if we are NOT already in AVOID/RECOVER state.
    if (distance > 2.0f && distance < 15.0f) {
        NavState ns = autoNav.getCurrentState();
        if (ns != NAV_AVOID && ns != NAV_RECOVER) {
            Serial.printf("[AUTO] EMERGENCY STOP %.0fcm\n", distance);
            autoNav.enterAvoid(true);  // critical=true → shorter backup, then turn
        }
        // Still fall through so the state machine can execute the backup
    }
    
    // Update IMU data for orientation detection
    #if IMU_ENABLED
    if (!imuFaultDetected) {
    autoNav.updateIMU(sensorData.accelX, sensorData.accelY, sensorData.accelZ,
                      sensorData.gyroX, sensorData.gyroY, sensorData.gyroZ);
    }
    #endif

    // === TRACTION CONTROL ===
    // Detects free-spin: encoder sees high wheel speed but IMU sees no body movement.
    // Response stages:
    //   Stage 1 — cut power to TC_REDUCE_PWM to let wheels regain grip, hold for TC_HOLD_MS
    //   Stage 2 — slowly ramp back up; if still spinning, repeat stage 1 up to TC_MAX_CUTS times
    //   Stage 3 — after TC_MAX_CUTS failed cuts, fall back to full avoidance (back up + turn)
    #if IMU_ENABLED
    {
        static float fsMagBuf[MOTION_VERIFY_WINDOW];
        static int   fsMagIdx   = 0;
        static bool  fsMagReady = false;
        static unsigned long fsSpinStart    = 0;   // When continuous spin was first detected
        static unsigned long tcCutUntil     = 0;   // Hold reduced power until this time
        static int           tcCutCount     = 0;   // How many power cuts have been applied
        static bool          tcActive       = false; // Currently in a power-cut hold
        static unsigned long fsCooldownUntil = 0;  // Post-recovery cooldown

        // Traction control tuning
        const int   TC_REDUCE_PWM  = 80;    // Reduced power during traction cut (0-255)
        const unsigned long TC_HOLD_MS    = 400;  // How long to hold reduced power (ms)
        const unsigned long TC_CONFIRM_MS = 600;  // Spin must persist this long before cutting
        const int   TC_MAX_CUTS    = 4;     // Max cuts before escalating to avoidance

        float fsAx = sensorData.accelX, fsAy = sensorData.accelY, fsAz = sensorData.accelZ;
        float fsMag = sqrt(fsAx*fsAx + fsAy*fsAy + fsAz*fsAz);
        fsMagBuf[fsMagIdx] = fsMag;
        fsMagIdx = (fsMagIdx + 1) % MOTION_VERIFY_WINDOW;
        if (fsMagIdx == 0) fsMagReady = true;

        bool fsMotionOK = true;
        if (fsMagReady) {
            float sum = 0;
            for (int i = 0; i < MOTION_VERIFY_WINDOW; i++) sum += fsMagBuf[i];
            float mean = sum / MOTION_VERIFY_WINDOW;
            float var  = 0;
            for (int i = 0; i < MOTION_VERIFY_WINDOW; i++) {
                float d = fsMagBuf[i] - mean; var += d * d;
            }
            var /= MOTION_VERIFY_WINDOW;
            fsMotionOK = (var > MOTION_ACCEL_VAR_THRESH);
        }

        // === ENCODER FAULT GUARD ===
        // If both encoders have been silent for >2s while motors are commanded high,
        // the encoder data is unreliable. Disable TC to avoid false free-spin triggers.
        {
            static unsigned long encoderSilentStart = 0;
            bool motorsHigh = (lastMotorSpeedA >= 80 && lastMotorSpeedB >= 80);
            bool leftStalled  = encoders.isLeftStalled();
            bool rightStalled = encoders.isRightStalled();
            bool bothStalled  = leftStalled && rightStalled;
            // Warn on single-side dropout (one reading, one silent) — asymmetric traction
            if (motorsHigh && !bothStalled && (leftStalled || rightStalled)) {
                static unsigned long singleDropWarnAt = 0;
                if (millis() - singleDropWarnAt > 5000) {
                    Serial.printf("[WARN] Encoder dropout on %s side — asymmetric reading.\n",
                                  leftStalled ? "LEFT" : "RIGHT");
                    singleDropWarnAt = millis();
                }
            }
            if (motorsHigh && bothStalled) {
                if (encoderSilentStart == 0) encoderSilentStart = millis();
                if (!encoderFaultDetected && millis() - encoderSilentStart > 2000) {
                    encoderFaultDetected = true;
                    Serial.println("[FAULT] Both encoders silent >2s while motors active — disabling traction control.");
                }
            } else {
                encoderSilentStart = 0;
                if (encoderFaultDetected) {
                    encoderFaultDetected = false;
                    Serial.println("[FAULT] Encoders recovered — re-enabling traction control.");
                }
            }
        }

        float encoderSpd = encoders.getFusedSpeed();
        // Free-spin: wheels spinning fast, but IMU shows robot body not moving
        bool wheelsSpin = (encoderSpd > 5.0f && lastMotorSpeedA >= 80 && lastMotorSpeedB >= 80);
        NavState ns = autoNav.getCurrentState();
        bool drivingForward = (ns == NAV_CRUISE);
        unsigned long nowTc = millis();

        // Skip TC entirely if either IMU or encoder data can't be trusted
        if (imuFaultDetected || encoderFaultDetected) goto tc_skip;

        if (drivingForward && nowTc >= fsCooldownUntil) {
            if (!fsMotionOK && wheelsSpin) {
                // --- Spin detected ---
                if (tcActive) {
                    // Currently holding reduced power — check if grip has recovered
                    if (nowTc >= tcCutUntil) {
                        tcActive = false;
                        if (fsMotionOK || !wheelsSpin) {
                            // Grip recovered — reset cut count and resume
                            Serial.println("[TC] Grip recovered — resuming normal power");
                            tcCutCount = 0;
                            fsSpinStart = 0;
                        }
                        // If still spinning after cut, will re-enter below on next loop
                    }
                    // While cutting: override motor output with reduced power
                    motors.setMotors(TC_REDUCE_PWM, TC_REDUCE_PWM);
                    lastMotorSpeedA = TC_REDUCE_PWM;
                    lastMotorSpeedB = TC_REDUCE_PWM;
                } else {
                    // Not yet cutting — start/continue the confirmation timer
                    if (fsSpinStart == 0) fsSpinStart = nowTc;

                    if (nowTc - fsSpinStart >= TC_CONFIRM_MS) {
                        // Confirmed spin — apply power cut
                        tcCutCount++;
                        if (tcCutCount > TC_MAX_CUTS) {
                            // Exceeded max cuts — escalate to avoidance
                            Serial.printf("[TC] %d cuts failed — escalating to recovery\n", TC_MAX_CUTS);
                            autoNav.enterAvoid(false);
                            fsCooldownUntil = nowTc + 5000;
                            fsMagReady = false; fsMagIdx = 0;
                            fsSpinStart = 0; tcCutCount = 0; tcActive = false;
                        } else {
                            Serial.printf("[TC] Free-spin! Cut #%d — reducing to %d PWM for %lums\n",
                                          tcCutCount, TC_REDUCE_PWM, TC_HOLD_MS);
                            tcActive   = true;
                            tcCutUntil = nowTc + TC_HOLD_MS;
                            motors.setMotors(TC_REDUCE_PWM, TC_REDUCE_PWM);
                            lastMotorSpeedA = TC_REDUCE_PWM;
                            lastMotorSpeedB = TC_REDUCE_PWM;
                        }
                    }
                }
            } else {
                // No spin — clear state
                if (!tcActive) {
                    fsSpinStart = 0;
                    if (tcCutCount > 0) tcCutCount = 0;
                }
            }
        } else {
            fsSpinStart = 0;
            tcActive = false;
        }
        tc_skip:;
    }
    #endif

    // === DUAL ULTRASONIC FAILURE FALLBACK ===
    // If both sensors are returning invalid readings, we have no obstacle data.
    // Reduce cruise speed to a safe crawl and log periodically.
    {
        static unsigned long bothFailedLogAt = 0;
        bool leftBad  = (sensorData.distanceLeft  < 0.0f);
        bool rightBad = (sensorData.distanceRight < 0.0f);
        if (leftBad && rightBad) {
            NavState ns2 = autoNav.getCurrentState();
            if (ns2 == NAV_CRUISE) {
                // Throttle to 40% max so the robot creeps rather than cruising blind
                const int BLIND_CRAWL_PWM = 100;
                if (lastMotorSpeedA > BLIND_CRAWL_PWM || lastMotorSpeedB > BLIND_CRAWL_PWM) {
                    motors.setMotors(BLIND_CRAWL_PWM, BLIND_CRAWL_PWM);
                    lastMotorSpeedA = BLIND_CRAWL_PWM;
                    lastMotorSpeedB = BLIND_CRAWL_PWM;
                }
                if (millis() - bothFailedLogAt > 5000) {
                    Serial.println("[FAULT] Both ultrasonic sensors FAILED — crawling at reduced speed.");
                    bothFailedLogAt = millis();
                }
            }
        } else {
            bothFailedLogAt = 0;  // Reset log timer once a sensor recovers
        }
    }

    int leftSpeed, rightSpeed;
    unsigned long now = millis();
    
    // Pass side distances and wall angle for informed turn decisions
    autoNav.setSideDistances(sensorData.distanceLeft, sensorData.distanceRight);
    autoNav.setWallAngle(sensorData.wallAngle);
    
    // Feed fused encoder+IMU speed for dead-reckoning
    autoNav.setEncoderSpeed(encoders.getFusedSpeed());
    // Feed odometry (x, y, theta) for localization
    float x = encoders.getX();
    float y = encoders.getY();
    float th = encoders.getTheta();
    autoNav.setOdometry(x, y, th);
    // Update map with odometry
    envMap.setOdometryPosition(x, y, th);
    
    // Feed ultrasonic readings into the occupancy map for ray-traced cell identification
    // Front sensor along heading, left at -15°, right at +15°
    float mapHeading = autoNav.getHeading();
    if (distance >= 2.0f && distance <= 400.0f) {
        envMap.addReading(distance, mapHeading);
    }
    if (sensorData.distanceLeft >= 2.0f && sensorData.distanceLeft <= 400.0f) {
        envMap.addReading(sensorData.distanceLeft, mapHeading - 15.0f);
    }
    if (sensorData.distanceRight >= 2.0f && sensorData.distanceRight <= 400.0f) {
        envMap.addReading(sensorData.distanceRight, mapHeading + 15.0f);
    }
    
    // === PIT DETECTION ===
    // Two signals: (1) sudden distance jump (sensor looks into void beyond ground)
    //              (2) negative pitch (rover nose tilting down into a pit)
    {
        static float prevDistLeft = 0.0f;
        static float prevDistRight = 0.0f;
        static unsigned long pitPitchStart = 0;
        static unsigned long pitCooldownUntil = 0;  // Don't re-trigger while recovering

        float dL = sensorData.distanceLeft;
        float dR = sensorData.distanceRight;
        bool pitDetected = false;
        bool pitMajor = false;   // true = deep/dangerous, false = shallow/traversable
        float pitHeading = mapHeading;
        float pitDist = distance;
        float pitJump = 0.0f;    // Track largest distance jump for severity

        bool pitInCooldown = (millis() < pitCooldownUntil);

        // Signal 1: Ultrasonic distance jump — ground dropped away
        // Previous reading was short (reflecting ground/wall), now suddenly long
        if (!pitInCooldown && prevDistLeft > 2.0f && prevDistLeft < 100.0f &&
            dL > prevDistLeft + PIT_DIST_JUMP_CM) {
            float jump = dL - prevDistLeft;
            pitDetected = true;
            pitHeading = mapHeading - 15.0f;
            pitDist = prevDistLeft;
            if (jump > pitJump) pitJump = jump;
            Serial.printf("[PIT] Left sensor jump: %.0f→%.0fcm (Δ%.0f)\n", prevDistLeft, dL, jump);
        }
        if (!pitInCooldown && prevDistRight > 2.0f && prevDistRight < 100.0f &&
            dR > prevDistRight + PIT_DIST_JUMP_CM) {
            float jump = dR - prevDistRight;
            pitDetected = true;
            pitHeading = mapHeading + 15.0f;
            pitDist = prevDistRight;
            if (jump > pitJump) pitJump = jump;
            Serial.printf("[PIT] Right sensor jump: %.0f→%.0fcm (Δ%.0f)\n", prevDistRight, dR, jump);
        }

        prevDistLeft = dL;
        prevDistRight = dR;

        // Signal 2: Sustained negative pitch (tilting forward into a pit)
        #if IMU_ENABLED
        float navPitch = autoNav.getPitch();
        if (!pitInCooldown && !imuFaultDetected && navPitch < PIT_PITCH_THRESHOLD) {
            if (pitPitchStart == 0) {
                pitPitchStart = now;
            } else if (now - pitPitchStart > PIT_PITCH_SUSTAIN_MS) {
                pitDetected = true;
                pitDist = distance > 5.0f ? distance : 20.0f;
                Serial.printf("[PIT] Negative pitch %.1f° sustained → pit ahead\n", navPitch);
                pitPitchStart = 0;  // Reset after marking
            }
        } else {
            pitPitchStart = 0;
        }
        #endif

        // Classify severity: major if steep pitch OR large distance jump
        if (pitDetected) {
            float absPitch = fabs(autoNav.getPitch());
            pitMajor = (absPitch >= TERRAIN_MINOR_PITCH) ||
                       (pitJump >= TERRAIN_MINOR_PIT_JUMP_CM);
        }

        // React based on severity
        if (pitDetected && pitMajor) {
            pitCooldownUntil = millis() + 4000;  // 4s cooldown — let backup/turn complete
            envMap.markPitAhead(pitHeading, pitDist);
            autoNav.enterAvoidFromPit();
            Serial.println("[PIT] MAJOR → AVOID");
        } else if (pitDetected) {
            pitCooldownUntil = millis() + 2000;  // 2s cooldown — let it traverse
            autoNav.enterTerrainBoost();
            Serial.printf("[PIT] MINOR (pitch=%.1f° jump=%.0fcm) → TRAVERSE\n",
                          autoNav.getPitch(), pitJump);
        }
    }
    
    // === HILL DETECTION ===
    // Mirror of pit detection: (1) distance *decrease* (hitting slope face)
    //                          (2) positive pitch (rover tilting back climbing)
    // Uses pitch + distance to estimate terrain height via trigonometry
    {
        static float prevDistLeftH = 0.0f;
        static float prevDistRightH = 0.0f;
        static unsigned long hillPitchStart = 0;
        static unsigned long hillCooldownUntil = 0;  // Don't re-trigger while recovering

        float dL = sensorData.distanceLeft;
        float dR = sensorData.distanceRight;
        bool hillDetected = false;
        float hillDist = distance;
        float maxHeightCm = 0.0f;  // Track max estimated height for severity

        // Suppress new hill detections while in cooldown
        bool hillInCooldown = (millis() < hillCooldownUntil);
        // Signal 1: Ultrasonic distance drop — beam hitting a rising slope
        // Previous reading was long (flat ground), now suddenly short (slope face)
        if (!hillInCooldown &&
            prevDistLeftH > 50.0f && prevDistLeftH < 400.0f &&
            dL < prevDistLeftH - HILL_DIST_DROP_CM && dL > 2.0f) {
            float pitchRad = autoNav.getPitch() * PI / 180.0f;
            float heightCm = dL * sin(fabs(pitchRad)) + SENSOR_HEIGHT_CM;
            hillDetected = true;
            hillDist = dL;
            if (heightCm > maxHeightCm) maxHeightCm = heightCm;
            Serial.printf("[HILL] Left sensor drop: %.0f→%.0fcm, est height %.0fcm\n",
                          prevDistLeftH, dL, heightCm);
        }
        if (!hillInCooldown &&
            prevDistRightH > 50.0f && prevDistRightH < 400.0f &&
            dR < prevDistRightH - HILL_DIST_DROP_CM && dR > 2.0f) {
            float pitchRad = autoNav.getPitch() * PI / 180.0f;
            float heightCm = dR * sin(fabs(pitchRad)) + SENSOR_HEIGHT_CM;
            hillDetected = true;
            hillDist = dR;
            if (heightCm > maxHeightCm) maxHeightCm = heightCm;
            Serial.printf("[HILL] Right sensor drop: %.0f→%.0fcm, est height %.0fcm\n",
                          prevDistRightH, dR, heightCm);
        }
        
        prevDistLeftH = dL;
        prevDistRightH = dR;
        
        // Signal 2: Sustained positive pitch (climbing)
        #if IMU_ENABLED
        float navPitch = autoNav.getPitch();
        if (!hillInCooldown && !imuFaultDetected && navPitch > HILL_PITCH_THRESHOLD) {
            if (hillPitchStart == 0) {
                hillPitchStart = now;
            } else if (now - hillPitchStart > HILL_PITCH_SUSTAIN_MS) {
                float pitchRad = navPitch * PI / 180.0f;
                float heightCm = distance * sin(pitchRad) + SENSOR_HEIGHT_CM;
                hillDetected = true;
                if (heightCm > maxHeightCm) maxHeightCm = heightCm;
                Serial.printf("[HILL] Positive pitch %.1f° sustained, est height %.0fcm\n",
                              navPitch, heightCm);
                hillPitchStart = 0;  // Reset after marking
            }
        } else {
            hillPitchStart = 0;
        }
        #endif
        
        // Classify severity: major if steep pitch OR tall estimated height
        bool hillMajor = false;
        if (hillDetected) {
            float absPitch = fabs(autoNav.getPitch());
            hillMajor = (absPitch >= TERRAIN_MINOR_PITCH) ||
                        (maxHeightCm >= TERRAIN_MINOR_HEIGHT_CM);
        }
        
        // React based on severity
        if (hillDetected && hillMajor) {
            // Major hill → mark on map + avoid (backup + turn)
            hillCooldownUntil = millis() + 4000;  // 4s cooldown — let backup/turn complete
            envMap.markHillAhead(hillDist < distance ? mapHeading : mapHeading,
                                hillDist > 5.0f ? hillDist : 20.0f, maxHeightCm);
            autoNav.enterAvoidFromHill();
            Serial.printf("[HILL] MAJOR (pitch=%.1f° height=%.0fcm) → AVOID\n",
                          autoNav.getPitch(), maxHeightCm);
        } else if (hillDetected) {
            // Minor hill → maintain speed and drive over it
            hillCooldownUntil = millis() + 2000;  // 2s cooldown — let it traverse
            autoNav.enterTerrainBoost();
            Serial.printf("[HILL] MINOR (pitch=%.1f° height=%.0fcm) → TRAVERSE\n",
                          autoNav.getPitch(), maxHeightCm);
        }
    }
    
    // === PATH PLANNER INTEGRATION ===
    // If the planner has a mode (explore, return home, goto goal), run it
    // and feed waypoints into autoNav for obstacle-aware path following
    if (pathPlanner.getMode() != PLANNER_IDLE) {
        int rx = envMap.getRobotX();
        int ry = envMap.getRobotY();
        float plannerHeading = pathPlanner.update(rx, ry, autoNav.getHeading());
        
        // If planner has a valid waypoint, set it as nav target (in cm from origin)
        int wpX, wpY;
        if (pathPlanner.getCurrentWaypoint(wpX, wpY)) {
            float targetX_cm = (wpX - MAP_CENTER_X) * CELL_SIZE_CM;
            float targetY_cm = (wpY - MAP_CENTER_Y) * CELL_SIZE_CM;
            autoNav.setNavTarget(targetX_cm, targetY_cm);
        } else if (pathPlanner.isGoalReached()) {
            autoNav.hasNavTarget = false;
        }
    }
    
    // Update autonomous navigation logic
    autoNav.update(distance);
    
    // Get motor speeds from autonomous navigator
    autoNav.getMotorSpeeds(leftSpeed, rightSpeed);
    
    // Apply motor speeds
    motors.setMotors(leftSpeed, rightSpeed);
    lastMotorSpeedA = leftSpeed;
    lastMotorSpeedB = rightSpeed;

    // === SELF-RIGHTING MOTOR PAUSE ===
    // When the arm is actively trying to flip the robot, stop the drive motors so
    // they don't fight the arm's leverage. Resume once the arm returns to stowed/idle.
    {
        static bool rightingMotorsPaused = false;
        RightingState rs = selfRighting.getState();
        bool armActive = (rs == ARM_RIGHTING || rs == ARM_SIDE_RIGHTING);
        if (armActive) {
            if (!rightingMotorsPaused) {
                Serial.println("[RIGHTING] Pausing drive motors — arm has control.");
                rightingMotorsPaused = true;
            }
            motors.setMotors(0, 0);
            lastMotorSpeedA = 0;
            lastMotorSpeedB = 0;
        } else if (rightingMotorsPaused) {
            Serial.println("[RIGHTING] Arm done — resuming drive motors.");
            rightingMotorsPaused = false;
        }
    }
    
    // Debug output every 2s
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 2000) {
        float x = encoders.getX();
        float y = encoders.getY();
        float th = encoders.getTheta();
        float lspd = encoders.getLeftSpeed();
        float rspd = encoders.getRightSpeed();
        float ldist = encoders.getLeftDistance();
        float rdist = encoders.getRightDistance();
        Serial.printf("[ODOM] x=%.1fcm y=%.1fcm th=%.2frad | Lspd=%.1f Rspd=%.1f | Ldist=%.1f Rdist=%.1f\n",
            x, y, th, lspd, rspd, ldist, rdist);
        #if IMU_ENABLED
        const char* orientStr = 
            autoNav.isUpsideDown() ? "FLIPPED" :
            (abs(autoNav.getPitch()) > 15 || abs(autoNav.getRoll()) > 15) ? "TILTED" : "OK";
        Serial.printf("[AUTO] %s | Fwd:%.0f L:%.0f R:%.0f | Wall:%.0f° | ML:%d MR:%d | %s%s%s\n", 
                      autoNav.getStateString(), distance, sensorData.distanceLeft, sensorData.distanceRight,
                      sensorData.wallAngle, leftSpeed, rightSpeed, orientStr,
                      autoNav.isMotionVerified() ? "" : " NO-MOTION",
                      autoNav.isRecentlyStuck() ? " SOFT-RAMP" : "");
        #else
        Serial.printf("[AUTO] %s | Fwd:%.0f L:%.0f R:%.0f | Wall:%.0f° | ML:%d MR:%d\n", 
                      autoNav.getStateString(), distance, sensorData.distanceLeft, sensorData.distanceRight,
                      sensorData.wallAngle, leftSpeed, rightSpeed);
        #endif
        lastDebug = millis();
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
    // Steps: 0=rock_fwd, 1=coast, 2=rock_rev, 3=coast, 4=diag_turn, 5=diag_fwd, 6=full_rev
    static int recoveryStep = 0;
    static int rockCycles = 0;
    static unsigned long recoveryUntil = 0;
    static int recoveryTurnDir = 0; // 0=right, 1=left
    static unsigned long simpleRecoveryCooldownUntil = 0;  // Post-recovery cooldown
    static float simpleLastDist = 999.0f;  // Track distance changes for stuck verify
    
    // === DEAD RECKONING: Track position relative to start ===
    // Simple estimate: integrate heading (from gyro) and speed to get (x, y)
    // Used to bias turn decisions AWAY from starting point
    static float drX = 0.0f;           // Estimated X position (cm, right = +)
    static float drY = 0.0f;           // Estimated Y position (cm, forward = +)
    static float drHeading = 0.0f;     // Heading in degrees (0 = initial forward)
    
    // Anti-spin "Send It" tracking
    static bool simpleSendItActive = false;
    static unsigned long simpleSendItUntil = 0;
    static int simpleSendItTurnCount = 0;
    static unsigned long simpleSendItWindowStart = 0;
    static unsigned long simpleSendItFwdStart = 0;
    
    // Dead-reckoning heading correction tracking
    static unsigned long simpleHeadingTowardStart = 0;  // When started heading toward start
    
    // Heading-hold straight-line correction
    static float simpleCruiseHeading = 0.0f;   // Locked heading target
    static bool simpleHeadingLocked = false;    // Whether heading is locked
    static unsigned long simplePhase0EnteredAt = 0; // When we last entered phase 0

    // PID gains
    const float KP = 5.0f;
    const float KI = 0.1f;
    const float KD = 1.5f;
    const float I_MAX = 2000.0f;

    const float OBSTACLE_CM = 25.0f;
    const float CRITICAL_CM = 10.0f;
    const float STALL_DIST_CM = 50.0f;
    const unsigned long STALL_MS = 4000;
    const int TURN_SPEED = (MAX_PWM * 90) / 100;  // 90% power turns
    const int REVERSE_SPEED = MAX_PWM;             // Clamped to 210 by current limiter
    const int MIN_SPEED = (MAX_PWM * 80) / 100;   // Minimum to overcome friction
    const unsigned long TURN_MS = 400;    // Short turn to prevent spinning
    const unsigned long REVERSE_MS = 300;
    const float TILT_THRESHOLD_DEG = 35.0f;
    const int TILT_DEBOUNCE = 3;
    static unsigned long lastSimpleDecision = 0;

    float distance = sensorData.distance;
    float distL = sensorData.distanceLeft;
    float distR = sensorData.distanceRight;
    unsigned long now = millis();

    // Handle invalid distance — side sensors default to FAR (safe/forward-biased)
    // Only front sensor defaults to close (conservative for collision avoidance)
    if (distance < 0 || distance > 400) distance = OBSTACLE_CM - 1.0f;
    if (distL < 0 || distL > 400) distL = 999.0f;  // Unknown = assume clear
    if (distR < 0 || distR > 400) distR = 999.0f;  // Unknown = assume clear

    // Calculate pitch from accelerometer
    float pitch = 0;
    #if IMU_ENABLED
    float ax = sensorData.accelX;
    float ay = sensorData.accelY;
    float az = sensorData.accelZ;
    pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
    
    // === DEAD RECKONING UPDATE ===
    {
        float gz = sensorData.gyroZ;
        float dt = 0.05f;  // ~20Hz update
        drHeading += gz * dt;  // Integrate gyro for heading
        
        // Use fused encoder+IMU speed for DR (falls back to PWM estimate if no pulses)
        if (lastMotorSpeedA > 0 && lastMotorSpeedB > 0 && phase == 0) {
            float speedCmS = encoders.getFusedSpeed();  // Fused encoder+IMU speed
            if (speedCmS < 1.0f) {
                // Encoder not producing pulses — fall back to PWM estimate
                float speedFrac = (float)min(lastMotorSpeedA, lastMotorSpeedB) / 255.0f;
                speedCmS = speedFrac * ROVER_MAX_SPEED_CM_S;
            }
            float distCm = speedCmS * dt;
            float headRad = drHeading * PI / 180.0f;
            drX += distCm * sin(headRad);
            drY += distCm * cos(headRad);
        }
    }
    
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
                    leftSpeed = RECOVERY_SPEED; rightSpeed = RECOVERY_SPEED; break;
                case 1: // Coast (fwd→rev transition)
                case 3: // Coast (rev→fwd or diagonal transition)
                    leftSpeed = 0; rightSpeed = 0; break;
                case 2: // Rock backward
                    leftSpeed = -RECOVERY_SPEED; rightSpeed = -RECOVERY_SPEED; break;
                case 4: // Diagonal turn
                    if (recoveryTurnDir == 1) { leftSpeed = -RECOVERY_SPEED; rightSpeed = RECOVERY_SPEED; }
                    else { leftSpeed = RECOVERY_SPEED; rightSpeed = -RECOVERY_SPEED; }
                    break;
                case 5: // Diagonal forward
                    leftSpeed = REVERSE_SPEED; rightSpeed = REVERSE_SPEED; break;
                case 6: // Full reverse
                    leftSpeed = -RECOVERY_SPEED; rightSpeed = -RECOVERY_SPEED; break;
            }
        } else {
            // Current step done — advance
            switch (recoveryStep) {
                case 0: // Rock fwd done → coast before reversing
                    recoveryStep = 1;
                    recoveryUntil = now + RECOVERY_COAST_MS;
                    leftSpeed = 0; rightSpeed = 0;
                    break;
                case 1: // Coast done → rock rev
                    recoveryStep = 2;
                    recoveryUntil = now + RECOVERY_ROCK_MS;
                    Serial.printf("[SIMPLE-REC] Rock REV (%d/%d)\n", rockCycles+1, RECOVERY_ROCK_ATTEMPTS);
                    leftSpeed = -RECOVERY_SPEED; rightSpeed = -RECOVERY_SPEED;
                    break;
                case 2: // Rock rev done → coast
                    recoveryStep = 3;
                    recoveryUntil = now + RECOVERY_COAST_MS;
                    leftSpeed = 0; rightSpeed = 0;
                    break;
                case 3: // Coast done → more rocks or diagonal
                    rockCycles++;
                    if (rockCycles < RECOVERY_ROCK_ATTEMPTS) {
                        recoveryStep = 0;
                        recoveryUntil = now + RECOVERY_ROCK_MS;
                        Serial.printf("[SIMPLE-REC] Rock FWD (%d/%d)\n", rockCycles+1, RECOVERY_ROCK_ATTEMPTS);
                        leftSpeed = RECOVERY_SPEED; rightSpeed = RECOVERY_SPEED;
                    } else {
                        recoveryStep = 4;
                        recoveryUntil = now + RECOVERY_DIAG_TURN_MS;
                        Serial.println("[SIMPLE-REC] Diagonal TURN");
                        if (recoveryTurnDir == 1) { leftSpeed = -RECOVERY_SPEED; rightSpeed = RECOVERY_SPEED; }
                        else { leftSpeed = RECOVERY_SPEED; rightSpeed = -RECOVERY_SPEED; }
                    }
                    break;
                case 4: // Diag turn done → diag forward
                    recoveryStep = 5;
                    recoveryUntil = now + RECOVERY_DIAG_FWD_MS;
                    Serial.println("[SIMPLE-REC] Diagonal FWD");
                    leftSpeed = REVERSE_SPEED; rightSpeed = REVERSE_SPEED;
                    break;
                case 5: // Diag fwd done → check if free
                    if (simpleMotionOK && distance > OBSTACLE_CM) {
                        Serial.println("[SIMPLE-REC] === FREE! ===");
                        phase = 0; pidI = 0; pidLastErr = 0;
                        simpleHeadingLocked = false; simplePhase0EnteredAt = now;
                        simpleRecentlyStuck = true; simpleStuckTime = now;
                        simpleNoMotionStart = 0;
                        simpleRecoveryCooldownUntil = now + RECOVERY_COOLDOWN_MS;
                        leftSpeed = MIN_SPEED; rightSpeed = MIN_SPEED;
                    } else {
                        recoveryStep = 6;
                        recoveryUntil = now + RECOVERY_FULL_REV_MS;
                        Serial.println("[SIMPLE-REC] FULL REVERSE");
                        leftSpeed = -RECOVERY_SPEED; rightSpeed = -RECOVERY_SPEED;
                    }
                    break;
                case 6: // Full reverse done → recovery complete
                    Serial.println("[SIMPLE-REC] === Recovery complete ===");
                    phase = 1; // Turn to new direction
                    actionUntil = now + 800;
                    pidI = 0;
                    simpleRecentlyStuck = true; simpleStuckTime = now;
                    simpleNoMotionStart = 0;
                    simpleRecoveryCooldownUntil = now + RECOVERY_COOLDOWN_MS;
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
        if (now - lastRecDbg > 2000) {
            const char* stepNames[] = {"ROCK_FWD","COAST","ROCK_REV","COAST","DIAG_TURN","DIAG_FWD","FULL_REV"};
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
            simpleHeadingLocked = false; simplePhase0EnteredAt = now;
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
            if (now - lastSendItDbg > 2000) {
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
        // Unlock heading — will re-lock after settle delay
        simpleHeadingLocked = false;
        simplePhase0EnteredAt = now;
        // Reset motion detection — give IMU time to see forward movement
        // Prevents false no-motion trigger immediately after a turn
        accelMagReady = false;
        accelMagIdx = 0;
        simpleNoMotionStart = 0;
        // Force a brief forward burst after every turn to prevent endless spinning
        leftSpeed = MIN_SPEED;
        rightSpeed = MIN_SPEED;
        motors.setMotors(leftSpeed, rightSpeed);
        lastMotorSpeedA = leftSpeed;
        lastMotorSpeedB = rightSpeed;
        return;  // Skip decision this cycle — commit to forward
    }

    // Decision cooldown — prevent rapid state thrashing
    if (phase == 0 && (now - lastSimpleDecision) < DECISION_COOLDOWN_MS && distance > CRITICAL_CM) {
        // Keep LAST motor speeds during cooldown (don't zero them!)
        motors.setMotors(lastMotorSpeedA, lastMotorSpeedB);
        return;
    }

    if (phase == 0) {
        // === ANTI-SPIN: Track forward progress & window expiry ===
        // If driving forward for long enough, we're making progress — reset turn count
        if (lastMotorSpeedA > 0 && lastMotorSpeedB > 0) {
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
        // Requires BOTH low IMU variance AND distance not changing
        // This prevents false triggers on smooth surfaces where IMU variance is naturally low
        #if IMU_ENABLED
        bool distChanging = (abs(distance - simpleLastDist) > 3.0f);  // >3cm = moving
        simpleLastDist = distance;
        
        if (!simpleMotionOK && !distChanging && lastMotorSpeedA >= MIN_SPEED && now >= simpleRecoveryCooldownUntil) {
            if (simpleNoMotionStart == 0) {
                simpleNoMotionStart = now;
            } else if (now - simpleNoMotionStart > MOTION_VERIFY_TIMEOUT) {
                Serial.printf("[SIMPLE] No motion for %lums → RECOVERY\n", now - simpleNoMotionStart);
                phase = 3;
                recoveryStep = 0; rockCycles = 0;
                recoveryUntil = now + RECOVERY_ROCK_MS;
                recoveryTurnDir = (distL > distR) ? 1 : 0;
                accelMagReady = false; accelMagIdx = 0;  // Reset IMU buffer
                simpleNoMotionStart = 0;
                Serial.println("[SIMPLE-REC] === Starting recovery ===");
                motors.setMotors(RECOVERY_SPEED, RECOVERY_SPEED);
                lastMotorSpeedA = RECOVERY_SPEED; lastMotorSpeedB = RECOVERY_SPEED;
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
                    accelMagReady = false; accelMagIdx = 0;  // Reset IMU buffer
                    simpleNoMotionStart = 0;
                    motors.setMotors(RECOVERY_SPEED, RECOVERY_SPEED);
                    lastMotorSpeedA = RECOVERY_SPEED; lastMotorSpeedB = RECOVERY_SPEED;
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

        // === PIT DETECTION (Simple mode) ===
        {
            static float simplePrevDistL = 0.0f;
            static float simplePrevDistR = 0.0f;
            static unsigned long simplePitPitchStart = 0;
            bool simplePitDetected = false;
            float simplePitJump = 0.0f;
            
            // Signal 1: Ultrasonic distance jump
            if (simplePrevDistL > 2.0f && simplePrevDistL < 100.0f &&
                distL > simplePrevDistL + PIT_DIST_JUMP_CM) {
                float jump = distL - simplePrevDistL;
                simplePitDetected = true;
                if (jump > simplePitJump) simplePitJump = jump;
                Serial.printf("[SIMPLE-PIT] Left jump: %.0f→%.0fcm (Δ%.0f)\n", simplePrevDistL, distL, jump);
            }
            if (simplePrevDistR > 2.0f && simplePrevDistR < 100.0f &&
                distR > simplePrevDistR + PIT_DIST_JUMP_CM) {
                float jump = distR - simplePrevDistR;
                simplePitDetected = true;
                if (jump > simplePitJump) simplePitJump = jump;
                Serial.printf("[SIMPLE-PIT] Right jump: %.0f→%.0fcm (Δ%.0f)\n", simplePrevDistR, distR, jump);
            }
            simplePrevDistL = distL;
            simplePrevDistR = distR;
            
            // Signal 2: Sustained negative pitch
            #if IMU_ENABLED
            if (pitch < PIT_PITCH_THRESHOLD) {
                if (simplePitPitchStart == 0) {
                    simplePitPitchStart = now;
                } else if (now - simplePitPitchStart > PIT_PITCH_SUSTAIN_MS) {
                    simplePitDetected = true;
                    Serial.printf("[SIMPLE-PIT] Negative pitch %.1f° → pit ahead\n", pitch);
                    simplePitPitchStart = 0;
                }
            } else {
                simplePitPitchStart = 0;
            }
            #endif
            
            // Classify severity
            bool simplePitMajor = false;
            if (simplePitDetected) {
                simplePitMajor = (fabs(pitch) >= TERRAIN_MINOR_PITCH) ||
                                 (simplePitJump >= TERRAIN_MINOR_PIT_JUMP_CM);
            }
            
            // React based on severity
            if (simplePitDetected && simplePitMajor && phase == 0) {
                envMap.markPitAhead(drHeading, distance > 5.0f ? distance : 20.0f);
                phase = 2;  // Reverse
                actionUntil = now + PIT_BACKUP_MS;
                pidI = 0;
                lastSimpleDecision = now;
                Serial.println("[SIMPLE-PIT] MAJOR → REVERSE");
            } else if (simplePitDetected && phase == 0) {
                // Minor pit — keep driving, don't reverse
                Serial.printf("[SIMPLE-PIT] MINOR (pitch=%.1f° jump=%.0f) → TRAVERSE\n",
                              pitch, simplePitJump);
            }
        }

        // === HILL DETECTION (Simple mode) ===
        {
            static float simpleHPrevDistL = 0.0f;
            static float simpleHPrevDistR = 0.0f;
            static unsigned long simpleHillPitchStart = 0;
            bool simpleHillDetected = false;
            float simpleMaxHeight = 0.0f;
            
            // Signal 1: Ultrasonic distance drop (hitting slope face)
            if (simpleHPrevDistL > 50.0f && simpleHPrevDistL < 400.0f &&
                distL < simpleHPrevDistL - HILL_DIST_DROP_CM && distL > 2.0f) {
                float pitchRad = pitch * PI / 180.0f;
                float heightCm = distL * sin(fabs(pitchRad)) + SENSOR_HEIGHT_CM;
                simpleHillDetected = true;
                if (heightCm > simpleMaxHeight) simpleMaxHeight = heightCm;
                Serial.printf("[SIMPLE-HILL] Left drop: %.0f→%.0fcm, height %.0fcm\n",
                              simpleHPrevDistL, distL, heightCm);
            }
            if (simpleHPrevDistR > 50.0f && simpleHPrevDistR < 400.0f &&
                distR < simpleHPrevDistR - HILL_DIST_DROP_CM && distR > 2.0f) {
                float pitchRad = pitch * PI / 180.0f;
                float heightCm = distR * sin(fabs(pitchRad)) + SENSOR_HEIGHT_CM;
                simpleHillDetected = true;
                if (heightCm > simpleMaxHeight) simpleMaxHeight = heightCm;
                Serial.printf("[SIMPLE-HILL] Right drop: %.0f→%.0fcm, height %.0fcm\n",
                              simpleHPrevDistR, distR, heightCm);
            }
            simpleHPrevDistL = distL;
            simpleHPrevDistR = distR;
            
            // Signal 2: Sustained positive pitch (climbing)
            #if IMU_ENABLED
            if (pitch > HILL_PITCH_THRESHOLD) {
                if (simpleHillPitchStart == 0) {
                    simpleHillPitchStart = now;
                } else if (now - simpleHillPitchStart > HILL_PITCH_SUSTAIN_MS) {
                    float pitchRad = pitch * PI / 180.0f;
                    float heightCm = distance * sin(pitchRad) + SENSOR_HEIGHT_CM;
                    simpleHillDetected = true;
                    if (heightCm > simpleMaxHeight) simpleMaxHeight = heightCm;
                    Serial.printf("[SIMPLE-HILL] Pitch %.1f° sustained, height %.0fcm\n", pitch, heightCm);
                    simpleHillPitchStart = 0;
                }
            } else {
                simpleHillPitchStart = 0;
            }
            #endif
            
            // Classify severity
            bool simpleHillMajor = false;
            if (simpleHillDetected) {
                simpleHillMajor = (fabs(pitch) >= TERRAIN_MINOR_PITCH) ||
                                  (simpleMaxHeight >= TERRAIN_MINOR_HEIGHT_CM);
            }
            
            // React based on severity
            if (simpleHillDetected && simpleHillMajor && phase == 0) {
                envMap.markHillAhead(drHeading, distance > 5.0f ? distance : 20.0f, simpleMaxHeight);
                phase = 2;  // Reverse
                actionUntil = now + HILL_BACKUP_MS;
                pidI = 0;
                lastSimpleDecision = now;
                Serial.printf("[SIMPLE-HILL] MAJOR (height=%.0fcm) → REVERSE\n", simpleMaxHeight);
            } else if (simpleHillDetected && phase == 0) {
                // Minor hill — keep driving with momentum
                Serial.printf("[SIMPLE-HILL] MINOR (pitch=%.1f° height=%.0fcm) → TRAVERSE\n",
                              pitch, simpleMaxHeight);
            }
        }

        // === PROACTIVE HEADING CORRECTION ===
        // If heading roughly toward start for too long with clear path, force a turn
        {
            float distFromStart = sqrt(drX * drX + drY * drY);
            if (distFromStart > DR_MIN_DIST_CM && phase == 0) {
                float toBearing = atan2(-drX, -drY) * 180.0f / PI;
                float toError = toBearing - drHeading;
                while (toError > 180) toError -= 360;
                while (toError < -180) toError += 360;
                
                if (abs(toError) < DR_HEADING_TOWARD_DEG) {
                    if (simpleHeadingTowardStart == 0) {
                        simpleHeadingTowardStart = now;
                    } else if (now - simpleHeadingTowardStart > DR_CORRECT_TIMEOUT_MS &&
                               distance > DR_CORRECT_CLEARANCE_CM) {
                        // Force correction turn away from start
                        float awayBearing = atan2(drX, drY) * 180.0f / PI;
                        float awayError = awayBearing - drHeading;
                        while (awayError > 180) awayError -= 360;
                        while (awayError < -180) awayError += 360;
                        
                        phase = 1;
                        actionUntil = now + DR_CORRECT_TURN_MS;
                        pidI = 0;
                        simpleHeadingTowardStart = 0;
                        bool turnRight = (awayError > 0);
                        if (turnRight) {
                            leftSpeed = TURN_SPEED; rightSpeed = -TURN_SPEED;
                        } else {
                            leftSpeed = -TURN_SPEED; rightSpeed = TURN_SPEED;
                        }
                        Serial.printf("[DR] Heading toward start >%ds — correction turn %s (dist:%.0f)\n",
                                      DR_CORRECT_TIMEOUT_MS / 1000, turnRight ? "RIGHT" : "LEFT", distFromStart);
                        motors.setMotors(leftSpeed, rightSpeed);
                        lastMotorSpeedA = leftSpeed; lastMotorSpeedB = rightSpeed;
                        return;
                    }
                } else {
                    simpleHeadingTowardStart = 0;
                }
            } else {
                simpleHeadingTowardStart = 0;
            }
        }

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
            
            // Decide turn direction: prefer AWAY from start position
            // drHeading = current heading, atan2(drX, drY) = bearing TO start
            // We want to face AWAY from start, so opposite of bearing-to-start
            bool turnRight;
            float sideDiff = distR - distL;  // positive = more room on right
            
            if (abs(sideDiff) > 8.0f) {
                // Clear sensor difference — trust sensors
                turnRight = (sideDiff > 0);
            } else {
                // Sensors roughly equal — bias away from start
                float distFromStart = sqrt(drX * drX + drY * drY);
                if (distFromStart > 30.0f) {  // Only bias when we have enough tracking data
                    // Bearing FROM start (direction we should go)
                    float awayBearing = atan2(drX, drY) * 180.0f / PI;  // degrees
                    // Difference: which way do we need to turn to face away from start?
                    float angleDiff = awayBearing - drHeading;
                    // Normalize to -180..180
                    while (angleDiff > 180) angleDiff -= 360;
                    while (angleDiff < -180) angleDiff += 360;
                    turnRight = (angleDiff > 0);  // Positive = turn right to face away
                    Serial.printf("[SIMPLE] Bias AWAY from start (%.0f,%.0f) dist=%.0fcm → %s\n",
                                  drX, drY, distFromStart, turnRight ? "RIGHT" : "LEFT");
                } else {
                    turnRight = (distL <= distR);  // Fallback to sensor
                }
            }
            
            if (turnRight) {
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
            
            // Hardware slew rate in motor_control.cpp handles ramping safely
            leftSpeed = speed;
            rightSpeed = speed;
            
            // === HEADING-HOLD: correct drift when going straight ===
            #if IMU_ENABLED
            {
                // Lock heading after settling into phase 0
                if (!simpleHeadingLocked && simplePhase0EnteredAt > 0 &&
                    (now - simplePhase0EnteredAt > HEADING_HOLD_SETTLE_MS)) {
                    simpleCruiseHeading = drHeading;
                    simpleHeadingLocked = true;
                }
                
                int holdBias = 0;
                if (simpleHeadingLocked) {
                    float error = simpleCruiseHeading - drHeading;
                    while (error > 180) error -= 360;
                    while (error < -180) error += 360;
                    if (abs(error) > HEADING_HOLD_DEADBAND) {
                        holdBias = constrain((int)(error * HEADING_HOLD_GAIN), -HEADING_HOLD_MAX, HEADING_HOLD_MAX);
                    }
                }
                leftSpeed  += holdBias;
                rightSpeed -= holdBias;
                leftSpeed  = constrain(leftSpeed, MIN_SPEED, 255);
                rightSpeed = constrain(rightSpeed, MIN_SPEED, 255);
            }
            #endif
            
            // === DEAD-RECKONING STEERING: gently arc away from start ===
            // Only apply in clear space — don't fight obstacle avoidance
            {
                float distFromStart = sqrt(drX * drX + drY * drY);
                if (distFromStart > DR_MIN_DIST_CM && distance > STALL_DIST_CM) {
                    float awayBearing = atan2(drX, drY) * 180.0f / PI;
                    float headingError = awayBearing - drHeading;
                    while (headingError > 180) headingError -= 360;
                    while (headingError < -180) headingError += 360;
                    // Stronger correction near start, fades with distance
                    float proxScale = constrain(1.0f - (distFromStart - DR_MIN_DIST_CM) / DR_PROXIMITY_RANGE_CM, 0.2f, 1.0f);
                    int steerBias = constrain((int)(headingError * DR_STEER_GAIN * proxScale), -DR_STEER_MAX, DR_STEER_MAX);
                    leftSpeed  += steerBias;
                    rightSpeed -= steerBias;
                    leftSpeed  = constrain(leftSpeed, MIN_SPEED, 255);
                    rightSpeed = constrain(rightSpeed, MIN_SPEED, 255);
                    
                    // Shift locked heading so heading-hold doesn't fight DR
                    if (abs(steerBias) > 5 && simpleHeadingLocked) {
                        simpleCruiseHeading += steerBias * 0.02f;
                        while (simpleCruiseHeading >= 360) simpleCruiseHeading -= 360;
                        while (simpleCruiseHeading < 0) simpleCruiseHeading += 360;
                    }
                }
            }
            
            // Stall detection (skip if within post-recovery cooldown)
            if (distance < STALL_DIST_CM && now >= simpleRecoveryCooldownUntil) {
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
                    accelMagReady = false; accelMagIdx = 0;  // Reset IMU buffer
                    simpleNoMotionStart = 0;
                    motors.setMotors(RECOVERY_SPEED, RECOVERY_SPEED);
                    lastMotorSpeedA = RECOVERY_SPEED; lastMotorSpeedB = RECOVERY_SPEED;
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
    if (now - lastDebug > 2000) {
        const char* stateStr = (phase == 0) ? "PID-FWD" : (phase == 1) ? "TURN" : "REVERSE";
        #if IMU_ENABLED
        float x = encoders.getX();
        float y = encoders.getY();
        float th = encoders.getTheta();
        float lspd = encoders.getLeftSpeed();
        float rspd = encoders.getRightSpeed();
        float ldist = encoders.getLeftDistance();
        float rdist = encoders.getRightDistance();
        Serial.printf("[ODOM] x=%.1fcm y=%.1fcm th=%.2frad | Lspd=%.1f Rspd=%.1f | Ldist=%.1f Rdist=%.1f\n",
            x, y, th, lspd, rspd, ldist, rdist);
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

// =============================================================================
// WALL FOLLOW MODE — Left-Hand Rule Perimeter Tracer
// Strategy: find a wall on the left side, then keep it at WF_TARGET_DIST_CM
// using a PID controller. Steer right for corners where the wall disappears,
// or right when the front sensor is blocked. All terrain / self-righting logic
// still runs via updateSensors() / selfRighting.update().
//
// States:
//   WF_SEARCH  — spinning to find an initial left wall
//   WF_FOLLOW  — PID wall-distance hold; drive parallel to wall
//   WF_CORNER  — left wall lost; turning left to wrap around a corner
//   WF_BLOCKED — front obstacle reached; turning right to go around it
// =============================================================================
void handleWallFollowMode() {
    enum WFState { WF_SEARCH, WF_FOLLOW, WF_CORNER, WF_BLOCKED, WF_CRUISE };
    static WFState wfState    = WF_SEARCH;
    static unsigned long wfActionUntil = 0;
    static unsigned long wfSearchStart = 0;  // When WF_SEARCH began (for timeout)
    static float wfPidI       = 0.0f;
    static float wfPrevErr    = 0.0f;
    static unsigned long lastWFDebug = 0;

    // Self-righting motor pause (same as autonomous mode)
    {
        RightingState rs = selfRighting.getState();
        if (rs == ARM_RIGHTING || rs == ARM_SIDE_RIGHTING) {
            motors.setMotors(0, 0);
            lastMotorSpeedA = 0;
            lastMotorSpeedB = 0;
            return;
        }
    }

    float fwd  = sensorData.distance;         // front sensor
    float left = sensorData.distanceLeft;     // left sensor (wall reference)
    unsigned long now = millis();

    // Treat invalid readings as "very far" so they don't trigger false walls
    if (fwd  < 0) fwd  = 400.0f;
    if (left < 0) left = 400.0f;

    bool frontBlocked  = (fwd  < WF_FRONT_STOP_CM);
    bool frontSlow     = (fwd  > WF_FRONT_STOP_CM && fwd < WF_FRONT_SLOW_CM);
    bool wallPresent   = (left < WF_WALL_LOST_CM);
    bool inTimedAction = (now < wfActionUntil);

    int leftSpeed = 0, rightSpeed = 0;

    switch (wfState) {

        // ── SEARCH: spin left until we detect a wall on the left ──────────────
        case WF_SEARCH:
            if (wfSearchStart == 0) wfSearchStart = now;
            if (wallPresent) {
                Serial.printf("[WF] Wall found at %.0fcm — switching to FOLLOW\n", left);
                wfState = WF_FOLLOW;
                wfSearchStart = 0;
                wfPidI  = 0; wfPrevErr = 0;
            } else if (now - wfSearchStart > WF_SEARCH_TIMEOUT_MS) {
                // No wall after timeout — open area. Cruise forward looking for one.
                Serial.println("[WF] No wall found — switching to CRUISE (open area)");
                wfState = WF_CRUISE;
                wfSearchStart = 0;
            } else {
                // Spin counter-clockwise (left) to sweep until we find something
                leftSpeed  = -WF_SEARCH_SPEED;
                rightSpeed =  WF_SEARCH_SPEED;
            }
            break;

        // ── CRUISE: open area, drive forward until a wall appears on the left ─
        case WF_CRUISE:
            if (wallPresent) {
                Serial.printf("[WF] Wall acquired at %.0fcm while cruising — FOLLOW\n", left);
                wfState = WF_FOLLOW;
                wfPidI = 0; wfPrevErr = 0;
            } else if (frontBlocked) {
                // Hit something ahead while cruising — spin to find left wall
                Serial.println("[WF] Blocked while cruising — SEARCH");
                wfState = WF_SEARCH;
                wfSearchStart = 0;
            } else {
                // Drive forward at cruise speed
                leftSpeed  = WF_CRUISE_SPEED;
                rightSpeed = WF_CRUISE_SPEED;
            }
            break;

        // ── FOLLOW: PID to maintain target distance from the left wall ─────────
        case WF_FOLLOW:
            if (!wallPresent && !inTimedAction) {
                // Wall disappeared — corner! Turn left to stay with it
                Serial.printf("[WF] Wall lost (%.0fcm) — CORNER turn\n", left);
                wfState = WF_CORNER;
                wfActionUntil = now + WF_CORNER_TURN_MS;
                wfPidI = 0; wfPrevErr = 0;
                break;
            }
            if (frontBlocked && !inTimedAction) {
                // Front blocked while following — turn right
                Serial.printf("[WF] Front blocked %.0fcm — BLOCKED turn right\n", fwd);
                wfState = WF_BLOCKED;
                wfActionUntil = now + WF_BLOCK_TURN_MS;
                wfPidI = 0; wfPrevErr = 0;
                break;
            }
            {
                // PID on wall error: positive error → too far from wall → steer left
                float err = left - WF_TARGET_DIST_CM;
                wfPidI   += err * 0.05f;  // integrate at ~20Hz update rate
                wfPidI    = constrain(wfPidI, -40.0f, 40.0f);
                float derr = err - wfPrevErr;
                wfPrevErr = err;
                float correction = WF_PID_KP * err + WF_PID_KI * wfPidI + WF_PID_KD * derr;
                correction = constrain(correction, -80.0f, 80.0f);

                int baseSpeed = frontSlow ? WF_SLOW_SPEED : WF_CRUISE_SPEED;
                // Positive correction → too far right of wall → slow left, add right
                leftSpeed  = constrain((int)(baseSpeed - correction), -255, 255);
                rightSpeed = constrain((int)(baseSpeed + correction), -255, 255);
            }
            break;

        // ── CORNER: left wall gone; turn left to wrap around corner ───────────
        case WF_CORNER:
            if (inTimedAction) {
                leftSpeed  = -WF_TURN_SPEED;
                rightSpeed =  WF_TURN_SPEED;
            } else {
                // After turn, check if we found the wall again
                if (wallPresent) {
                    Serial.println("[WF] Wall re-acquired after corner — FOLLOW");
                    wfState = WF_FOLLOW;
                    wfPidI = 0; wfPrevErr = 0;
                } else {
                    // Still no wall — search (resets timeout)
                    Serial.println("[WF] Still no wall after corner — SEARCH");
                    wfState = WF_SEARCH;
                    wfSearchStart = 0;
                }
            }
            break;

        // ── BLOCKED: front obstacle; turn right to clear path ─────────────────
        case WF_BLOCKED:
            if (inTimedAction) {
                leftSpeed  =  WF_TURN_SPEED;
                rightSpeed = -WF_TURN_SPEED;
            } else {
                // After right turn, resume following if wall is still there
                if (wallPresent && !frontBlocked) {
                    Serial.println("[WF] Path clear after block — FOLLOW");
                    wfState = WF_FOLLOW;
                    wfPidI = 0; wfPrevErr = 0;
                } else if (!wallPresent) {
                    wfState = WF_SEARCH;
                }
                // else: still blocked, stay in BLOCKED and turn more (wfActionUntil reset below)
                else {
                    wfActionUntil = now + WF_BLOCK_TURN_MS;
                }
            }
            break;
    }

    // IMU hill/pit guard: if pitching dangerously, halt and log
    #if IMU_ENABLED
    if (!imuFaultDetected) {
        float wfPitch = 0;
        // Use autoNav pitch if available (needs IMU update)
        autoNav.updateIMU(sensorData.accelX, sensorData.accelY, sensorData.accelZ,
                          sensorData.gyroX, sensorData.gyroY, sensorData.gyroZ);
        wfPitch = autoNav.getPitch();
        if (fabs(wfPitch) > HILL_PITCH_THRESHOLD * 1.5f) {
            // Very steep — don't follow, stop and let driver/simple logic handle it
            static unsigned long wfTerrainWarnAt = 0;
            if (millis() - wfTerrainWarnAt > 3000) {
                Serial.printf("[WF] Steep terrain %.1f° — holding position\n", wfPitch);
                wfTerrainWarnAt = millis();
            }
            leftSpeed = 0; rightSpeed = 0;
        }
    }
    #endif

    motors.setMotors(leftSpeed, rightSpeed);
    lastMotorSpeedA = leftSpeed;
    lastMotorSpeedB = rightSpeed;

    // Occupancy map update with current readings
    float wfHeading = autoNav.getHeading();
    if (fwd  >= 2.0f && fwd  <= 400.0f) envMap.addReading(fwd,  wfHeading);
    if (left >= 2.0f && left <= 400.0f) envMap.addReading(left, wfHeading - 15.0f);

    if (now - lastWFDebug > 2000) {
        lastWFDebug = now;
        const char* stateStr = wfState == WF_SEARCH  ? "SEARCH"  :
                               wfState == WF_FOLLOW  ? "FOLLOW"  :
                               wfState == WF_CORNER  ? "CORNER"  :
                               wfState == WF_CRUISE  ? "CRUISE"  : "BLOCKED";
        Serial.printf("[WF] %s | Fwd:%.0f Left:%.0f | Err:%.1f | ML:%d MR:%d\n",
                      stateStr, fwd, left, left - WF_TARGET_DIST_CM, leftSpeed, rightSpeed);
    }
}

// =============================================================================
// PREMAP NAV MODE — Autonomous navigation with a web-drawn pre-seeded map.
// Behaviour is identical to handleAutonomousMode() — the autonomous navigator
// reads the occupancy map and avoids whatever is marked in it. The difference
// is purely that the map was seeded by the user via the web editor before the
// run, so the robot already "knows" the arena layout from the very first cycle
// instead of discovering it by bumping into things.
//
// The map can be drawn at http://192.168.4.1/map/draw while the robot is idle,
// then the robot is switched to this mode to start the run.
// =============================================================================
void handlePremapNavMode() {
    // Delegate entirely to the autonomous nav handler — it already reads from
    // envMap which is pre-seeded by the POST /api/premap endpoint.
    extern void handleAutonomousMode();
    handleAutonomousMode();
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
