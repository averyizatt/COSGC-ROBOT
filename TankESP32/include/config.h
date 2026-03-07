#ifndef CONFIG_H
#define CONFIG_H

// ==================== HARDWARE PINS ====================

// Motor Driver Pins (2x DRV8871)
// DRV8871: IN1=PWM + IN2=LOW for forward, IN1=LOW + IN2=PWM for reverse
// Motor A (Driver 1) - Left side
#define MOTOR_A_IN1  15  // Motor A Input 1
#define MOTOR_A_IN2  5   // Motor A Input 2
// Motor B (Driver 2) - Right side
#define MOTOR_B_IN1  7   // Motor B Input 1
#define MOTOR_B_IN2  6   // Motor B Input 2 (moved from GPIO4 - JTAG conflict)

// Ultrasonic Sensor Pins (HC-SR04)
// Sensor 1 (left-angled, ~15° left of center)
#define ULTRASONIC_TRIG    12
#define ULTRASONIC_ECHO    11
// Sensor 2 (right-angled, ~15° right of center)
#define ULTRASONIC2_TRIG   35
#define ULTRASONIC2_ECHO   36

// I2C Pins for MPU6050
#define I2C_SDA      10
#define I2C_SCL      3

// Button and LED Pins
#define BUTTON_PIN   21  // Mode switch / Bluetooth pairing button
#define LED_PIN      48  // Onboard RGB LED (WS2812 on most ESP32-S3 DevKits)

// UART Configuration
#define UART_TX      43  // Default UART TX for ESP32-S3
#define UART_RX      44  // Default UART RX for ESP32-S3
#define UART_BAUD    115200

// ==================== PWM CONFIG ====================

#define PWM_FREQ     20000  // 20 kHz (DRV8871 works best at higher freq)
#define PWM_RESOLUTION 8    // 8-bit (0-255)
#define PWM_CHANNEL_A_IN1 0 // Motor A IN1 PWM channel
#define PWM_CHANNEL_A_IN2 1 // Motor A IN2 PWM channel
#define PWM_CHANNEL_B_IN1 2 // Motor B IN1 PWM channel
#define PWM_CHANNEL_B_IN2 3 // Motor B IN2 PWM channel

// ==================== SENSOR CONFIG ====================

// Ultrasonic Sensor Configuration
#define MAX_DISTANCE 400  // Maximum distance in cm
#define TIMEOUT_US   23200  // Timeout for ultrasonic sensor (23200 us = ~400cm)

// MPU6050 Configuration
#define MPU6050_ADDR 0x68

// IMU Enable (set to 1 to enable MPU6050 init/update)
#define IMU_ENABLED 1

// IMU Thresholds
#define UPSIDE_DOWN_THRESHOLD -0.5f  // AccelZ below this = upside down
#define SLOPE_STEEP_THRESHOLD 0.5f   // Tilt angle for steep slope detection
#define TILT_DANGER_THRESHOLD 0.7f   // About to tip over

// ==================== UI CONFIG ====================

// Button Configuration
#define BUTTON_DEBOUNCE_MS 20
#define BUTTON_LONG_PRESS_MS 1500

// LED Configuration
#define LED_BLINK_FAST 200   // Fast blink for pairing (ms)
#define LED_BLINK_SLOW 1000  // Slow blink (ms)

// ==================== OPERATION MODES ====================

#define MODE_RC_CONTROL   0  // Xbox controller (Bluetooth)
#define MODE_UART_CONTROL 1  // UART commands
#define MODE_AUTONOMOUS   2  // Autonomous navigation
#define MODE_SIMPLE_AUTO  3  // Simple autonomous (distance-only)

// ==================== ROBOT PHYSICAL SPECS ====================

#define BATTERY_VOLTAGE     16.0f   // Battery voltage (V)
#define TRACK_WIDTH_MM      150     // Center-to-center track width (mm)
#define GROUND_CLEARANCE_MM 50      // Ground clearance (mm)
#define ULTRASONIC_HEIGHT_MM 76     // Ultrasonic sensor height above ground (mm)
// Ultrasonic is forward-facing (no angle), MPU6050 at center of mass

// ==================== AUTONOMOUS NAV CONFIG ====================

// Distance thresholds (cm)
#define DISTANCE_CLOSE    20.0   // cm - back up
#define DISTANCE_MEDIUM   50.0   // cm - turn
#define DISTANCE_FAR      100.0  // cm - go forward
#define AUTONOMOUS_SPEED  255    // Base speed for autonomous mode (0-255)

// Motor calibration - adjust if robot drifts left/right
// Values from 0.8 to 1.2, 1.0 = no adjustment
#define MOTOR_A_CALIBRATION 1.0f  // Left motor multiplier
#define MOTOR_B_CALIBRATION 1.0f  // Right motor multiplier
// Set to 1 if motor wiring polarity is reversed
#define MOTOR_A_INVERTED 1
#define MOTOR_B_INVERTED 0

// ==================== TUNING PARAMETERS ====================

// Gyro drift compensation (degrees/second)
// If robot thinks it's turning when stationary, adjust this
#define GYRO_DRIFT_COMPENSATION 0.0f

// Minimum motor speed (motors won't spin below this PWM value)
#define MIN_MOTOR_SPEED 60

// Maximum PWM cap — lowers peak current draw to prevent brownout
// Adjust up/down to find the sweet spot (255 = full, 230 = ~90%)
#define MAX_PWM 255

// ==================== TERRAIN HANDLING ====================

// IMU-based movement verification
#define MOTION_VERIFY_WINDOW    8       // Accel samples in rolling window
#define MOTION_VERIFY_TIMEOUT   1500    // ms with no motion before flagging stuck
#define MOTION_ACCEL_VAR_THRESH 0.008f  // Accel variance below this = not moving
#define MOTION_GYRO_TURN_THRESH 5.0f    // deg/s — expected gyro when turning

// Incline handling
#define INCLINE_MAX_PITCH       25.0f   // Max climbable pitch (degrees)
#define INCLINE_TIMEOUT_MS      3000    // Time on steep incline before giving up
#define INCLINE_DIAG_TURN_MS    400     // Turn duration for diagonal incline attempt

// Stuck recovery sequence
#define RECOVERY_ROCK_MS        500     // Duration of each rock forward/back pulse
#define RECOVERY_ROCK_ATTEMPTS  3       // Rock back-and-forth cycles before escalating
#define RECOVERY_DIAG_TURN_MS   600     // Diagonal approach turn duration
#define RECOVERY_DIAG_FWD_MS    800     // Diagonal approach forward duration
#define RECOVERY_FULL_REV_MS    1500    // Full reverse escape duration

// Adaptive torque ramp (soft terrain)
#define ADAPTIVE_RAMP_SLOW      15      // Speed ramp step after stuck (vs normal 50)
#define ADAPTIVE_RAMP_NORMAL    50      // Normal speed ramp step
#define ADAPTIVE_RAMP_TIMEOUT   5000    // How long slow ramp lasts after stuck (ms)

// Decision pacing
#define DECISION_COOLDOWN_MS    300     // Min ms between state changes (prevents jitter)

// Anti-spin "Send It" mode (narrow corridors)
#define SENDIT_TURN_THRESHOLD   4       // Turns within window to trigger send-it
#define SENDIT_WINDOW_MS        8000    // Time window for counting turns (ms)
#define SENDIT_DURATION_MS      2500    // How long to drive forward in send-it (ms)
#define SENDIT_FWD_PROGRESS_MS  1500    // Sustained forward time to reset turn count

// Debug flags
// #define DEBUG_SENSORS    // Uncomment to print all sensor readings
// #define DEBUG_XBOX       // Uncomment to print Xbox controller data
// #define DEBUG_MOTORS     // Uncomment to print motor commands
// #define DEBUG_NAV        // Uncomment for verbose navigation debug

#endif // CONFIG_H
