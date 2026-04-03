#ifndef CONFIG_H
#define CONFIG_H

// ==================== HARDWARE PINS ====================
// ESP32 DevKit V1 (WROOM-32, 30-pin)
// Available GPIOs: 0,1,2,3,4,5,12,13,14,15,16,17,18,19,21,22,23,25,26,27,32,33,34*,35*,36*,39*
// (* = input only)
// Avoid: GPIO0 (boot), GPIO1 (TX0), GPIO2 (boot strap), GPIO3 (RX0), GPIO12 (flash voltage strap!)

// Motor Driver Pins (2x DRV8871)
// DRV8871: IN1=PWM + IN2=LOW for forward, IN1=LOW + IN2=PWM for reverse
// Motor A (Driver 1) - Left side
#define MOTOR_A_IN1  33  // Left motor Input 1
#define MOTOR_A_IN2  25  // Left motor Input 2
// Motor B (Driver 2) - Right side
// GPIO 32 was dead — moved IN1 to GPIO 4
#define MOTOR_B_IN1  26   // Right motor Input 1 (was GPIO32, didn't output)
#define MOTOR_B_IN2  4  // Right motor Input 2

// --- ESP32-S3 pins (commented out) ---
// #define MOTOR_B_IN1  7   // S3: Motor B Input 1
// #define MOTOR_B_IN2  6   // S3: Motor B Input 2

// Ultrasonic Sensor Pins (HC-SR04)
// Sensor 1 (left-angled)
#define ULTRASONIC_TRIG    13  // Left ultrasonic trigger
#define ULTRASONIC_ECHO    34  // Left ultrasonic echo (GPIO34 = input-only, safe)
                               // ** MOVED from GPIO12 — GPIO12 is flash voltage strap! **
// Sensor 2 (right-angled)
#define ULTRASONIC2_TRIG   19  // Right ultrasonic trigger
#define ULTRASONIC2_ECHO   18  // Right ultrasonic echo

// --- ESP32-S3 ultrasonic pins (commented out) ---
// #define ULTRASONIC_TRIG    12
// #define ULTRASONIC_ECHO    11
// #define ULTRASONIC2_TRIG   35
// #define ULTRASONIC2_ECHO   36

// I2C Pins for MPU6050
#define I2C_SDA      14  // MPU6050 SDA
#define I2C_SCL      27  // MPU6050 SCL

// --- ESP32-S3 I2C pins (commented out) ---
// #define I2C_SDA      10
// #define I2C_SCL      3

// Button Pin
#define BUTTON_PIN   5   // Mode switch / Bluetooth pairing button

// RGB LED Pins (4-leg common cathode: R, G, B + GND)
#define LED_PIN_R    21  // Red leg   (with 220Ω resistor)
#define LED_PIN_G    22  // Green leg (with 220Ω resistor)
#define LED_PIN_B    23  // Blue leg  (with 220Ω resistor)
// Longest leg (common cathode) → GND

// LED PWM channels (motors use 0-3)
#define PWM_CHANNEL_LED_R  4
#define PWM_CHANNEL_LED_G  5
#define PWM_CHANNEL_LED_B  6

// Self-righting servo
// Mounting: servo on its side, gear output on robot-right (looking forward), centered on robot.
// Arm sweeps LEFT (0°/SERVO_MIN_US) to RIGHT (180°/SERVO_MAX_US) in the horizontal plane.
// If physical direction is reversed after installation, swap SERVO_MIN_US and SERVO_MAX_US.
#define SERVO_PIN          15     // Servo signal wire
#define PWM_CHANNEL_SERVO  8      // LEDC channel for servo (low-speed group, separate timer from LED ch6)
// Servo PWM frequency:
//   Analog servo  → use 50  Hz only (going higher damages analog servos)
//   Digital servo → 50-333 Hz (200-330 Hz gives faster response & smoother hold)
//   If unsure: leave at 50 Hz — works safely for both types
#define SERVO_FREQ_HZ      330    // Hz — DS3235 digital servo supports up to 330Hz
#define SERVO_MIN_US       500    // Pulse width (µs) at 0°  — arm at LEFT extreme
#define SERVO_MAX_US       2500   // Pulse width (µs) at 180° — arm at RIGHT extreme

// Hall effect encoders (A3144, one per output shaft, 10kΩ pull-up to 3.3V)
#define ENCODER_LEFT_PIN   35  // Left motor hall sensor  (GPIO35 — input-only, D35, external pull-up required)
#define ENCODER_RIGHT_PIN  32  // Right motor hall sensor (GPIO32 — input, D32, external pull-up required)
#define ENCODER_MAGNETS    2       // Magnets per shaft revolution
#define ENCODER_WHEEL_DIA_MM 26.0f // Drive sprocket diameter (mm)
// Derived: circumference = π × 26mm ≈ 81.7mm, distance per pulse ≈ 40.8mm

// Keep for backward compat (some code references LED_PIN)
#define LED_PIN      LED_PIN_R

// UART Configuration (WROOM default UART0)
#define UART_TX      1   // was GPIO43 on S3
#define UART_RX      3   // was GPIO44 on S3
#define UART_BAUD    115200

// --- ESP32-S3 UART pins (commented out) ---
// #define UART_TX      43
// #define UART_RX      44

// ==================== PWM CONFIG ====================

#define PWM_FREQ     20000  // 20 kHz (DRV8871 works best at higher freq)
#define PWM_RESOLUTION 8    // 8-bit (0-255)
#define PWM_CHANNEL_A_IN1 0 // Motor A IN1 PWM channel
#define PWM_CHANNEL_A_IN2 1 // Motor A IN2 PWM channel
#define PWM_CHANNEL_B_IN1 2 // Motor B IN1 PWM channel
#define PWM_CHANNEL_B_IN2 3 // Motor B IN2 PWM channel

// ==================== SENSOR CONFIG ====================

// Ultrasonic Sensor Configuration
#define MAX_DISTANCE 200  // Maximum distance in cm (200cm = plenty for ground rover)
#define TIMEOUT_US   12000  // Timeout for ultrasonic sensor (12000 us ≈ 200cm)

// MPU6050 Configuration
#define MPU6050_ADDR 0x68

// IMU Enable (set to 1 to enable MPU6050 init/update)
#define IMU_ENABLED 1

// IMU Thresholds
#define UPSIDE_DOWN_THRESHOLD -0.5f  // AccelZ below this = upside down (needs to be solidly inverted)
#define ON_SIDE_ROLL_THRESHOLD 45.0f // Roll angle (degrees) above which rover is considered on its side
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

// Distance thresholds (cm) — used by advanced autonomous nav
#define DISTANCE_CLOSE    20.0   // cm - back up  (simple auto)
#define DISTANCE_MEDIUM   50.0   // cm - turn      (simple auto)
#define DISTANCE_FAR      100.0  // cm - go forward (simple auto)
#define AUTONOMOUS_SPEED  255    // Base speed for autonomous mode (0-255)

// Advanced autonomous distance thresholds (cm)
#define DIST_CRITICAL         15.0f  // Emergency — reverse immediately
#define DIST_CLOSE_ADV        35.0f  // Obstacle — enter avoidance
#define DIST_MEDIUM_ADV       60.0f  // Caution — speed reduction zone
#define DIST_FAR_ADV         100.0f  // Cruising — full speed

// Avoidance timing
#define AVOID_BACKUP_MS       250    // Reverse duration on normal obstacle (ms)
#define AVOID_BACKUP_CRIT_MS  350    // Reverse duration on critical obstacle (ms)
#define AVOID_VERIFY_MS       300    // Forward check duration after turn (ms)
#define AVOID_MAX_CYCLES      4      // Avoid attempts before escalating to recovery

// Stall / stuck detection
#define STALL_TIMEOUT_MS      2500   // Time stuck under DIST_MEDIUM before avoidance (ms)
#define STUCK_COUNT_THRESHOLD  8      // Consecutive history readings to flag stuck
#define STUCK_DISTANCE_TOL     5.0f   // Distance change tolerance to consider stuck (cm)

// Motor calibration - adjust if robot drifts left/right
// Values from 0.8 to 1.2, 1.0 = no adjustment
#define MOTOR_A_CALIBRATION 1.0f  // Left motor multiplier
#define MOTOR_B_CALIBRATION 0.95f // Right motor multiplier (trimmed down — robot drifts right)
// Set to 1 if motor wiring polarity is reversed
#define MOTOR_A_INVERTED 1
#define MOTOR_B_INVERTED 1  // Changed: new driver has reversed wiring

// ==================== TUNING PARAMETERS ====================

// Gyro drift compensation (degrees/second)
// If robot thinks it's turning when stationary, adjust this
#define GYRO_DRIFT_COMPENSATION 0.0f

// Minimum motor speed (motors won't spin below this PWM value)
#define MIN_MOTOR_SPEED 60

// Maximum PWM — full 8-bit range available (safety enforced by current limiter)
#define MAX_PWM 255

// ==================== DRV8871 CURRENT PROTECTION ====================
// DRV8871 absolute max continuous current = 3.6A
// We estimate motor current as: I = V_battery * (duty/255) / R_motor
// and clamp PWM so estimated current never exceeds the limit.
//
// If you don't know your motor's winding resistance, start with
// MOTOR_RESISTANCE_OHM = 1.0 and tune up until robot has enough torque.

#define DRV8871_MAX_CURRENT_A  3.6f   // Absolute max continuous current (A)
#define DRV8871_SAFE_CURRENT_A 3.3f   // Target operating current (A) — 92% of max, brief peaks OK
#define MOTOR_RESISTANCE_OHM   4.0f   // Effective stall resistance (ohms)
                                       // Multimeter reads ~8ohm static, but stall draws ~4A at 16V
                                       // → effective R = 16V/4A = 4ohm (use this for safety calc)
                                       // At 4ohm: safe PWM = 210 (82%), brief peaks to 3.6A OK

// Slew rate limiter — max PWM change per update cycle
// Prevents sudden current spikes that cause voltage sag / brownout
#define SLEW_RATE_UP    30     // Max PWM increase per cycle (ramp up)
#define SLEW_RATE_DOWN  35     // Max PWM decrease per cycle (ramp down — faster for safety)
#define SLEW_INTERVAL_MS 10    // How often slew rate is applied (ms)

// Direction-change dead-time — brief coast between fwd↔rev
// Prevents both FETs conducting simultaneously (shoot-through)
#define DIRECTION_DEADTIME_MS  15   // Coast time when reversing direction (non-blocking)

// ==================== TERRAIN HANDLING ====================

// IMU-based movement verification
#define MOTION_VERIFY_WINDOW    8       // Accel samples in rolling window
#define MOTION_VERIFY_TIMEOUT   4000    // ms with no motion before flagging stuck (was 2s, raised to avoid false triggers)
#define MOTION_ACCEL_VAR_THRESH 0.003f  // Accel variance below this = not moving (lower = less sensitive)
#define MOTION_GYRO_TURN_THRESH 5.0f    // deg/s — expected gyro when turning

// Incline handling
#define INCLINE_MAX_PITCH       30.0f   // Max climbable pitch (degrees)
#define INCLINE_TIMEOUT_MS      1000    // Time on steep incline before giving up
#define INCLINE_DIAG_TURN_MS    200     // Turn duration for diagonal incline attempt

// Pit / drop-off detection
#define PIT_DIST_JUMP_CM        80.0f   // Distance jump between consecutive readings to flag pit
#define PIT_PITCH_THRESHOLD    -15.0f   // Negative pitch (nose down) indicates approaching pit edge
#define PIT_PITCH_SUSTAIN_MS    500     // Sustained negative pitch to confirm pit
#define PIT_MARK_DEPTH_CELLS    1       // How many cells deep to mark as pit ahead of detection point
#define PIT_BACKUP_MS           600     // Reverse duration after pit detection

// Hill / slope detection
#define HILL_PITCH_THRESHOLD    15.0f   // Positive pitch (nose up) indicates climbing a hill
#define HILL_PITCH_SUSTAIN_MS   500     // Sustained positive pitch to confirm hill
#define HILL_DIST_DROP_CM       40.0f   // Distance *decrease* between readings (hitting slope face)
#define HILL_MARK_DEPTH_CELLS   4       // How many cells deep to mark as hill
#define HILL_BACKUP_MS          600     // Reverse duration after steep hill detection
#define SENSOR_HEIGHT_CM        7.6f    // Ultrasonic sensor height above ground (76mm)

// Terrain severity classification
// Minor terrain → traverse with full speed (momentum); Major → avoid (backup + turn)
#define TERRAIN_MINOR_PITCH      20.0f   // |pitch| below this = minor terrain (traversable)
#define TERRAIN_MINOR_HEIGHT_CM  15.0f   // Hill below this estimated height = minor (cm)
#define TERRAIN_MINOR_PIT_JUMP_CM 120.0f // Distance jump below this = minor pit (cm)
#define TERRAIN_BOOST_DURATION_MS 2000   // Maintain full speed through minor terrain (ms)

// Stuck recovery sequence
#define RECOVERY_ROCK_MS        300     // Duration of each rock forward/back pulse
#define RECOVERY_ROCK_ATTEMPTS  2       // Rock back-and-forth cycles before escalating
#define RECOVERY_DIAG_TURN_MS   400     // Diagonal approach turn duration
#define RECOVERY_DIAG_FWD_MS    1200    // Diagonal approach forward duration
#define RECOVERY_FULL_REV_MS    2000    // Full reverse escape duration

// Adaptive torque ramp (soft terrain)
#define RECOVERY_COAST_MS       100     // Coast (motors off) between direction reversals
#define RECOVERY_SPEED          200     // PWM during recovery rocks (driver clamps to 210 max)
#define RECOVERY_COOLDOWN_MS    1500    // Ignore stuck detection after recovery exit

#define ADAPTIVE_RAMP_SLOW      30      // Speed ramp step after stuck (vs normal 50)
#define ADAPTIVE_RAMP_NORMAL    50      // Normal speed ramp step
#define ADAPTIVE_RAMP_TIMEOUT   3000    // How long slow ramp lasts after stuck (ms)

// Decision pacing
#define DECISION_COOLDOWN_MS    200     // Min ms between state changes (prevents oscillation)

// Anti-spin "Send It" mode (narrow corridors)
#define SENDIT_TURN_THRESHOLD   6       // Turns within window to trigger send-it
#define SENDIT_WINDOW_MS        10000   // Time window for counting turns (ms)
#define SENDIT_DURATION_MS      2500    // How long to drive forward in send-it (ms)
#define SENDIT_FWD_PROGRESS_MS  2000    // Sustained forward time to reset turn count

// Calibrated rover velocity (measured: 5m in 5s at MAX_PWM = 1 m/s)
#define ROVER_MAX_SPEED_CM_S    100.0f  // cm/s at MAX_PWM (measured)

// Heading-hold straight-line correction (IMU-based)
#define HEADING_HOLD_GAIN       1.0f    // PWM per degree of heading error
#define HEADING_HOLD_MAX        40      // Max steering differential (PWM)
#define HEADING_HOLD_DEADBAND   2.5f    // Ignore errors smaller than this (degrees)
#define HEADING_HOLD_SETTLE_MS  500     // Wait this long after entering CRUISE before locking heading

// Dead-reckoning "escape from start" steering
#define DR_STEER_GAIN           0.15f   // Steering PWM per degree of heading error
#define DR_STEER_MAX            30      // Max steering differential (PWM)
#define DR_MIN_DIST_CM          100.0f  // Min distance from start before bias activates (~1s driving)
#define DR_PROXIMITY_RANGE_CM   1000.0f // Range over which proximity scaling fades out (~10s driving)
#define DR_HEADING_TOWARD_DEG   60.0f   // Within this many degrees of start = "heading toward"
#define DR_CORRECT_TIMEOUT_MS   5000    // Heading toward start this long = force correction turn
#define DR_CORRECT_CLEARANCE_CM 150.0f  // Min front distance to allow correction turn (well into clear space)
#define DR_CORRECT_TURN_MS      600     // Duration of proactive correction turn

// ==================== SELF-RIGHTING SERVO TUNING ====================
// Servo sweeps LEFT (0°) → CENTER (90°) → RIGHT (180°) in horizontal plane.
// Stow angle is 90° (centered / pointing forward).

#define ARM_SWEEP_STEP          3     // Degrees per step — lower = slower but more torque
#define ARM_SWEEP_DELAY_MS      25    // Milliseconds between steps — lower = faster sweep
#define ARM_HOLD_AT_EXTREME_MS  7000  // How long to hold at 0° or 180° extremes (ms)
#define ARM_COOLDOWN_MS         2000  // Pause between righting attempt cycles (ms)
#define ARM_MAX_ATTEMPTS        40    // Max full sweeps before cooldown
#define ARM_UNSTICK_MAX         12    // Max sweeps during unstick mode before cooldown
#define ARM_UNSTICK_COOLDOWN_MS 1500  // Cooldown after unstick max attempts (ms)

// ==================== ENCODER / ODOMETRY TUNING ====================

#define ENCODER_DEBOUNCE_US     5000  // ISR noise filter — min µs between valid pulses
                                      // Lower = more sensitive, higher = rejects more noise
                                      // With 2 magnets at 1000 RPM, max legit period = ~30000µs
#define ENCODER_SPEED_EMA_ALPHA 0.3f  // Speed smoothing factor (0.0=max smooth, 1.0=no smooth)
#define ENCODER_COMP_ALPHA      0.85f // Encoder vs IMU speed blend (higher = trust encoder more)

// ==================== SAFETY PROTECTION ====================

// ESP32 internal temperature limits (Celsius)
#define TEMP_WARNING_C       65.0f   // Start throttling motors
#define TEMP_CRITICAL_C      75.0f   // Kill motors entirely
#define TEMP_RESUME_C        55.0f   // Resume after cooldown (hysteresis)
#define TEMP_CHECK_MS        2000    // How often to read temp sensor (ms)

// Motor duty cycle protection
#define MOTOR_MAX_FULL_MS    20000   // Max continuous full-speed (20s)
#define MOTOR_COOLDOWN_MS    3000    // Forced throttle-down period after max
#define MOTOR_THROTTLE_PCT   50      // % of MAX_PWM during cooldown (0-100)

// Motor watchdog — auto-stop if no new command received
#define MOTOR_WATCHDOG_MS    500     // Stop motors if no command in 500ms

// Startup safety
#define MOTOR_STARTUP_DELAY  1000    // Wait 1s after boot before allowing motors

// Debug flags
// #define DEBUG_SENSORS    // Uncomment to print all sensor readings
// #define DEBUG_XBOX       // Uncomment to print Xbox controller data
// #define DEBUG_MOTORS     // Uncomment to print motor commands
// #define DEBUG_NAV        // Uncomment for verbose navigation debug

#endif // CONFIG_H
