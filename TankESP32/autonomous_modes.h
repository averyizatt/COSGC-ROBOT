#pragma once
/*
 * autonomous_modes.h / autonomous_modes.cpp
 * ==========================================
 * Portable robot mode library — extracted from COSGC-ROBOT TankESP32.
 *
 * MODES:
 *   0  RC_CONTROL    Tank-drive via joystick (Xbox / gamepad)
 *   1  UART_CONTROL  Full-speed forward (UART-commanded)
 *   2  SOK_CONTROL   Pass-through socket-commanded motor values
 *   3  AUTONOMOUS    Advanced state machine (CRUISE / AVOID / RECOVER / HAZARD)
 *                    Heading-hold, dead-reckoning, terrain detection, traction ctrl
 *   4  SIMPLE_AUTO   PID forward + turn/reverse phases + full recovery sequence
 *                    Incline / pit / hill detection, "send-it" anti-spin mode
 *   5  WALL_FOLLOW   Left-hand-rule PID wall follower (SEARCH / FOLLOW / CORNER / BLOCKED)
 *   6  PREMAP_NAV    Identical to AUTONOMOUS (map pre-seeded by caller externally)
 *
 * USAGE:
 *   1. Provide millis() — on Arduino/ESP32 it is automatic; elsewhere #define AM_MILLIS().
 *   2. Fill a SensorInput struct each loop iteration with current sensor values.
 *   3. Instantiate RobotModes and call setMotorCallback(fn) with your motor output fn.
 *   4. Call modes.setMode(MODE_xxx) to choose a mode.
 *   5. Call modes.update(sensors) at ~20 Hz (every 50 ms).
 *   6. Your motor callback receives (leftPWM, rightPWM), range -255..255.
 *
 * PLATFORM:
 *   Arduino / ESP32 : millis(), Serial.printf(), random() provided automatically.
 *   Other C++       : #define AM_MILLIS(), AM_RANDOM(n), AM_PRINT(fmt,...) before including.
 *
 * TUNING:
 *   #define any AM_xxx constant BEFORE including this header to override defaults.
 */

#include <math.h>
#include <stdint.h>
#include <string.h>

// ===== PLATFORM COMPATIBILITY =====
#ifndef AM_MILLIS
  #ifdef ARDUINO
    #include <Arduino.h>
    #define AM_MILLIS()   millis()
    #define AM_RANDOM(n)  random(n)
    #define AM_PRINT(...) Serial.printf(__VA_ARGS__)
  #else
    #include <cstdlib>
    // Non-Arduino: implement am_millis_impl() to return milliseconds since startup.
    extern "C" unsigned long am_millis_impl();
    #define AM_MILLIS()   am_millis_impl()
    #define AM_RANDOM(n)  (rand() % (int)(n))
    #define AM_PRINT(...) /* define to printf or your logger if you want debug output */
  #endif
#endif

#ifndef AM_CONSTRAIN
  #define AM_CONSTRAIN(x,lo,hi) ((x)<(lo)?(lo):((x)>(hi)?(hi):(x)))
#endif
#ifndef AM_PI
  #define AM_PI 3.14159265358979323846f
#endif
#ifndef AM_ABS
  #define AM_ABS(x) ((x)<0?-(x):(x))
#endif

// ===== IMU ENABLE =====
// Set 0 to compile without any IMU-dependent features.
#ifndef AM_IMU_ENABLED
  #define AM_IMU_ENABLED 1
#endif

// ===== CONFIG DEFAULTS =====
// Every constant is guarded — #define before including to override.

// General motor limits
#ifndef AM_MAX_PWM
  #define AM_MAX_PWM               255
#endif
#ifndef AM_RECOVERY_SPEED
  #define AM_RECOVERY_SPEED        200
#endif

// Advanced-auto distance thresholds (cm)
#ifndef AM_DIST_CRITICAL
  #define AM_DIST_CRITICAL         15.0f
#endif
#ifndef AM_DIST_CLOSE
  #define AM_DIST_CLOSE            35.0f
#endif
#ifndef AM_DIST_MEDIUM
  #define AM_DIST_MEDIUM           60.0f
#endif
#ifndef AM_DIST_FAR
  #define AM_DIST_FAR             100.0f
#endif

// Avoidance timing (ms)
#ifndef AM_AVOID_BACKUP_MS
  #define AM_AVOID_BACKUP_MS       400
#endif
#ifndef AM_AVOID_BACKUP_CRIT_MS
  #define AM_AVOID_BACKUP_CRIT_MS  550
#endif
#ifndef AM_AVOID_VERIFY_MS
  #define AM_AVOID_VERIFY_MS       400
#endif
#ifndef AM_AVOID_MAX_CYCLES
  #define AM_AVOID_MAX_CYCLES      4
#endif

// Stall / stuck detection
#ifndef AM_STALL_TIMEOUT_MS
  #define AM_STALL_TIMEOUT_MS      2500
#endif
#ifndef AM_STUCK_COUNT_THRESHOLD
  #define AM_STUCK_COUNT_THRESHOLD 8
#endif
#ifndef AM_STUCK_DISTANCE_TOL
  #define AM_STUCK_DISTANCE_TOL    5.0f
#endif

// Recovery sequence timing (ms)
#ifndef AM_RECOVERY_ROCK_MS
  #define AM_RECOVERY_ROCK_MS      300
#endif
#ifndef AM_RECOVERY_COAST_MS
  #define AM_RECOVERY_COAST_MS     100
#endif
#ifndef AM_RECOVERY_ROCK_ATTEMPTS
  #define AM_RECOVERY_ROCK_ATTEMPTS 2
#endif
#ifndef AM_RECOVERY_DIAG_TURN_MS
  #define AM_RECOVERY_DIAG_TURN_MS 400
#endif
#ifndef AM_RECOVERY_DIAG_FWD_MS
  #define AM_RECOVERY_DIAG_FWD_MS  1200
#endif
#ifndef AM_RECOVERY_FULL_REV_MS
  #define AM_RECOVERY_FULL_REV_MS  2000
#endif
#ifndef AM_RECOVERY_COOLDOWN_MS
  #define AM_RECOVERY_COOLDOWN_MS  1500
#endif
#ifndef AM_ADAPTIVE_RAMP_TIMEOUT
  #define AM_ADAPTIVE_RAMP_TIMEOUT 3000
#endif

// IMU / motion verification
#ifndef AM_UPSIDE_DOWN_THRESHOLD
  #define AM_UPSIDE_DOWN_THRESHOLD -0.5f
#endif
#ifndef AM_MOTION_VERIFY_WINDOW
  #define AM_MOTION_VERIFY_WINDOW  8
#endif
#ifndef AM_MOTION_VERIFY_TIMEOUT
  #define AM_MOTION_VERIFY_TIMEOUT 4000
#endif
#ifndef AM_MOTION_ACCEL_VAR_THRESH
  #define AM_MOTION_ACCEL_VAR_THRESH 0.003f
#endif

// Incline handling
#ifndef AM_INCLINE_MAX_PITCH
  #define AM_INCLINE_MAX_PITCH     30.0f
#endif
#ifndef AM_INCLINE_TIMEOUT_MS
  #define AM_INCLINE_TIMEOUT_MS    1000
#endif
#ifndef AM_INCLINE_DIAG_TURN_MS
  #define AM_INCLINE_DIAG_TURN_MS  200
#endif

// Gyro / odometry
#ifndef AM_GYRO_DRIFT
  #define AM_GYRO_DRIFT            0.0f
#endif
#ifndef AM_ROVER_MAX_SPEED_CM_S
  #define AM_ROVER_MAX_SPEED_CM_S  100.0f
#endif

// Heading-hold (straight-line correction)
#ifndef AM_HEADING_HOLD_GAIN
  #define AM_HEADING_HOLD_GAIN     1.0f
#endif
#ifndef AM_HEADING_HOLD_MAX
  #define AM_HEADING_HOLD_MAX      40
#endif
#ifndef AM_HEADING_HOLD_DEADBAND
  #define AM_HEADING_HOLD_DEADBAND 2.5f
#endif
#ifndef AM_HEADING_HOLD_SETTLE_MS
  #define AM_HEADING_HOLD_SETTLE_MS 500
#endif

// Dead-reckoning "escape from start" steering
#ifndef AM_DR_STEER_GAIN
  #define AM_DR_STEER_GAIN         0.15f
#endif
#ifndef AM_DR_STEER_MAX
  #define AM_DR_STEER_MAX          30
#endif
#ifndef AM_DR_MIN_DIST_CM
  #define AM_DR_MIN_DIST_CM        100.0f
#endif
#ifndef AM_DR_PROXIMITY_RANGE_CM
  #define AM_DR_PROXIMITY_RANGE_CM 1000.0f
#endif
#ifndef AM_DR_HEADING_TOWARD_DEG
  #define AM_DR_HEADING_TOWARD_DEG 60.0f
#endif
#ifndef AM_DR_CORRECT_TIMEOUT_MS
  #define AM_DR_CORRECT_TIMEOUT_MS 5000
#endif
#ifndef AM_DR_CORRECT_CLEARANCE_CM
  #define AM_DR_CORRECT_CLEARANCE_CM 150.0f
#endif
#ifndef AM_DR_CORRECT_TURN_MS
  #define AM_DR_CORRECT_TURN_MS    600
#endif

// Decision pacing
#ifndef AM_DECISION_COOLDOWN_MS
  #define AM_DECISION_COOLDOWN_MS  200
#endif

// Pit / drop-off detection
#ifndef AM_PIT_DIST_JUMP_CM
  #define AM_PIT_DIST_JUMP_CM      80.0f
#endif
#ifndef AM_PIT_PITCH_THRESHOLD
  #define AM_PIT_PITCH_THRESHOLD  -15.0f
#endif
#ifndef AM_PIT_PITCH_SUSTAIN_MS
  #define AM_PIT_PITCH_SUSTAIN_MS  500
#endif
#ifndef AM_PIT_BACKUP_MS
  #define AM_PIT_BACKUP_MS         600
#endif

// Hill / slope detection
#ifndef AM_HILL_PITCH_THRESHOLD
  #define AM_HILL_PITCH_THRESHOLD  15.0f
#endif
#ifndef AM_HILL_PITCH_SUSTAIN_MS
  #define AM_HILL_PITCH_SUSTAIN_MS 500
#endif
#ifndef AM_HILL_DIST_DROP_CM
  #define AM_HILL_DIST_DROP_CM     40.0f
#endif
#ifndef AM_HILL_BACKUP_MS
  #define AM_HILL_BACKUP_MS        600
#endif
#ifndef AM_SENSOR_HEIGHT_CM
  #define AM_SENSOR_HEIGHT_CM      7.6f
#endif
#ifndef AM_TERRAIN_MINOR_PITCH
  #define AM_TERRAIN_MINOR_PITCH   20.0f
#endif
#ifndef AM_TERRAIN_MINOR_HEIGHT_CM
  #define AM_TERRAIN_MINOR_HEIGHT_CM 15.0f
#endif
#ifndef AM_TERRAIN_MINOR_PIT_JUMP_CM
  #define AM_TERRAIN_MINOR_PIT_JUMP_CM 120.0f
#endif
#ifndef AM_TERRAIN_BOOST_DURATION_MS
  #define AM_TERRAIN_BOOST_DURATION_MS 2000
#endif

// Anti-spin "send-it" (simple auto)
#ifndef AM_SENDIT_TURN_THRESHOLD
  #define AM_SENDIT_TURN_THRESHOLD 6
#endif
#ifndef AM_SENDIT_WINDOW_MS
  #define AM_SENDIT_WINDOW_MS      10000
#endif
#ifndef AM_SENDIT_DURATION_MS
  #define AM_SENDIT_DURATION_MS    2500
#endif
#ifndef AM_SENDIT_FWD_PROGRESS_MS
  #define AM_SENDIT_FWD_PROGRESS_MS 2000
#endif

// Wall follow
#ifndef AM_WF_TARGET_DIST_CM
  #define AM_WF_TARGET_DIST_CM     28.0f
#endif
#ifndef AM_WF_WALL_LOST_CM
  #define AM_WF_WALL_LOST_CM      100.0f
#endif
#ifndef AM_WF_FRONT_STOP_CM
  #define AM_WF_FRONT_STOP_CM      30.0f
#endif
#ifndef AM_WF_FRONT_SLOW_CM
  #define AM_WF_FRONT_SLOW_CM      55.0f
#endif
#ifndef AM_WF_SEARCH_SPEED
  #define AM_WF_SEARCH_SPEED       90
#endif
#ifndef AM_WF_CRUISE_SPEED
  #define AM_WF_CRUISE_SPEED       160
#endif
#ifndef AM_WF_SLOW_SPEED
  #define AM_WF_SLOW_SPEED         80
#endif
#ifndef AM_WF_TURN_SPEED
  #define AM_WF_TURN_SPEED         120
#endif
#ifndef AM_WF_PID_KP
  #define AM_WF_PID_KP             4.5f
#endif
#ifndef AM_WF_PID_KD
  #define AM_WF_PID_KD             1.8f
#endif
#ifndef AM_WF_PID_KI
  #define AM_WF_PID_KI             0.04f
#endif
#ifndef AM_WF_CORNER_TURN_MS
  #define AM_WF_CORNER_TURN_MS     650
#endif
#ifndef AM_WF_BLOCK_TURN_MS
  #define AM_WF_BLOCK_TURN_MS      700
#endif
#ifndef AM_WF_SEARCH_TIMEOUT_MS
  #define AM_WF_SEARCH_TIMEOUT_MS  8000
#endif

// ===================================================================
// SENSOR INPUT STRUCT
// Fill this every loop iteration before calling RobotModes::update().
// ===================================================================
struct SensorInput {
    // Ultrasonic distances (cm). Use 0 or negative for invalid / no reading.
    float distFront;
    float distLeft;
    float distRight;
    float wallAngle;        // Angle of nearest wall to front sensor (degrees). 0 if unknown.

    // IMU — set all to 0 if no IMU is fitted; leave AM_IMU_ENABLED 1 and
    // the code will automatically skip IMU-based features when accelZ == 0.
    float accelX, accelY, accelZ;   // Accelerometer (g). Flat/upright = (0, 0, ~1).
    float gyroX,  gyroY,  gyroZ;    // Gyroscope (deg/s). Positive Z = turning right.

    // Odometry / encoder (optional — set all to 0 if unused).
    float encoderSpeedCmS;  // Fused speed from encoders (cm/s).
    float odomX, odomY;     // Position relative to start (cm).
    float odomTheta;        // Heading in radians (0 = initial forward).

    // RC gamepad inputs — used by MODE_RC_CONTROL.
    // Range: -512..512 (maps to -255..255 motor PWM).
    int   rcThrottle;       // Left stick Y  — positive = forward.
    int   rcSteering;       // Right stick X — positive = right.
    bool  rcConnected;      // True when a controller is connected.

    // Socket control inputs — used by MODE_SOK_CONTROL.
    int   sokLeft;          // -255..255
    int   sokRight;         // -255..255
};

// ===================================================================
// MODE IDs  (matches original config.h values)
// ===================================================================
enum RobotMode {
    MODE_RC_CONTROL   = 0,
    MODE_UART_CONTROL = 1,
    MODE_SOK_CONTROL  = 2,
    MODE_AUTONOMOUS   = 3,
    MODE_SIMPLE_AUTO  = 4,
    MODE_WALL_FOLLOW  = 5,
    MODE_PREMAP_NAV   = 6
};

// ===================================================================
// ADVANCED NAV CORE — state-machine enums
// ===================================================================
enum AdvNavState   { ADV_CRUISE=0, ADV_AVOID, ADV_RECOVER, ADV_HAZARD, ADV_STOPPED };
enum AdvAvoidPhase { AA_BACKUP=0, AA_TURN, AA_VERIFY };
enum AdvRecovPhase { AR_ROCK_FWD=0, AR_COAST, AR_ROCK_REV,
                     AR_DIAG_TURN, AR_DIAG_FWD, AR_FULL_REV, AR_DONE };
enum AdvHazPhase   { AH_BACKUP=0, AH_TURN };
enum AdvTurnDir    { ATD_LEFT=0, ATD_RIGHT, ATD_RANDOM };
enum AdvOrientation {
    AO_NORMAL=0, AO_UPSIDE_DOWN,
    AO_TILTED_LEFT, AO_TILTED_RIGHT, AO_TILTED_FWD, AO_TILTED_BACK
};

// ===================================================================
// ADVANCED NAV CORE CLASS
// Embedded, self-contained autonomous navigation state machine.
// Extracted from AutonomousNav in autonomous_nav.h / autonomous_nav.cpp.
// Dependencies removed: OccupancyMap, PathPlanner, MotorControl.
// Call update(distFront), then getMotorSpeeds(l, r) each cycle.
// ===================================================================
class AdvNavCore {
public:
    AdvNavCore();

    // Main update — call every loop with front ultrasonic reading (cm).
    void update(float distance);

    // Sensor feed-ins (call before update()).
    void setSideDistances(float leftCm, float rightCm);
    void setWallAngle(float angle);
    void updateIMU(float ax, float ay, float az,
                   float gx=0, float gy=0, float gz=0);
    void setEncoderSpeed(float cmPerSec);
    void setOdometry(float x, float y, float theta);

    // Motor output — call after update().
    void getMotorSpeeds(int& left, int& right);

    // State queries.
    AdvNavState    getState()       const { return state; }
    const char*    getStateString() const;
    bool           isUpsideDown()   const { return upsideDown; }
    float          getPitch()       const { return pitch; }
    float          getRoll()        const { return roll; }
    float          getHeading()     const { return heading; }
    bool           isMotionVerified() const { return motionVerified; }
    bool           isRecentlyStuck()  const { return recentlyStuck; }
    AdvOrientation getOrientation()   const { return orientation; }

    // External triggers (call from terrain detection logic in your loop).
    void enterAvoid(bool critical);
    void enterAvoidFromPit();
    void enterAvoidFromHill();
    void enterTerrainBoost();
    bool isTerrainBoostActive();

    // Reset to stopped state.
    void reset();

private:
    // === STATE ===
    AdvNavState   state;
    unsigned long stateStart, lastStateChange;
    bool          firstRun;

    // === DISTANCE HISTORY ===
    float         lastDist, distHist[5];
    int           histIdx, stuckCounter;

    // === SPEED ===
    int           currentSpeed, targetSpeed;
    unsigned long stallStart;

    // === ENCODER ===
    float         encSpeedCmS;

    // === SENSORS ===
    float         sideL, sideR, wAngle;

    // === IMU ===
    bool          upsideDown, imuAvail;
    AdvOrientation orientation;
    float         pitch, roll, iax, iay, iaz, igyroZ;
    unsigned long lastIMUUpdate;
    bool          motionVerified;
    float         accelMagHist[AM_MOTION_VERIFY_WINDOW];
    int           accelMagIdx;
    bool          accelMagFilled;
    unsigned long noMotionStart;

    // === HEADING / DEAD RECKONING ===
    float         heading, headingAtObs;
    float         drX, drY;
    unsigned long headingTowardStartSince;
    float         cruiseHeading;
    bool          headingLocked;
    unsigned long cruiseEnteredAt;

    // === ODOMETRY ===
    float         odomX, odomY, odomTheta;

    // === TURN TRACKING ===
    AdvTurnDir    lastTurnDir;
    float         turnStartHeading;
    int           consecSameDir, obstMem[8];

    // === AVOIDANCE ===
    AdvAvoidPhase avoidPhase;
    AdvTurnDir    avoidTurnDir;
    int           avoidAttempts;
    unsigned long avoidPhaseStart, avoidPhaseDur;

    // === HAZARD ===
    AdvHazPhase   hazPhase;
    unsigned long hazPhaseStart, hazPhaseDur;
    int           inclineAttempts;
    unsigned long steepInclineStart;
    bool          onSteepIncline;

    // === RECOVERY ===
    AdvRecovPhase recPhase;
    int           rockCount;
    unsigned long recStepStart, recStepDur;
    AdvTurnDir    recTurnDir;
    bool          coastAfterFwd;
    bool          recentlyStuck;
    unsigned long stuckRecovTime, recoveryCooldownUntil;

    // === TERRAIN BOOST ===
    bool          terrainBoostActive;
    unsigned long terrainBoostUntil;

    // --- Internal handlers ---
    void handleCruise(float dist);
    void handleAvoid(float dist);
    void handleHazard(float dist);
    void handleRecovery(float dist);

    void enterCruise();
    void enterRecover();
    void enterHazard();
    void startRecovery();
    void flagRecentlyStuck();

    bool       isStuck();
    void       updateDistHist(float dist);
    int        calcSpeed(float dist);
    void       smoothSpeed();
    void       checkMotionVerification();
    AdvTurnDir pickTurnDir(float dist);
    AdvTurnDir getBestTurnDir();
    int        getHeadingBin();
    bool       isSlippingOrStalled();
};

// ===================================================================
// SIMPLE AUTO STATE
// Holds all persistent state for MODE_SIMPLE_AUTO so it lives in the
// RobotModes object instead of static locals (allows clean resets).
// ===================================================================
struct SimpleAutoState {
    int           phase;            // 0=PID fwd, 1=turn, 2=reverse, 3=recovery
    unsigned long actionUntil;
    float         pidI, pidLastErr;
    int           tiltCount;
    unsigned long stallStart;

    // IMU motion-verification window
    float         accelMagBuf[AM_MOTION_VERIFY_WINDOW];
    int           accelMagIdx;
    bool          accelMagReady;
    bool          motionOK;
    unsigned long noMotionStart;

    // Terrain
    unsigned long steepStart;
    int           inclineAttempts;
    bool          recentlyStuck;
    unsigned long stuckTime;

    // Recovery sub-state
    // Steps: 0=rock_fwd 1=coast 2=rock_rev 3=coast 4=diag_turn 5=diag_fwd 6=full_rev
    int           recoveryStep, rockCycles;
    unsigned long recoveryUntil;
    int           recoveryTurnDir;      // 0=right, 1=left
    unsigned long recoveryCooldownUntil;
    float         lastDist;

    // Dead reckoning
    float         drX, drY, drHeading;

    // Anti-spin "send-it"
    bool          sendItActive;
    unsigned long sendItUntil;
    int           sendItTurnCount;
    unsigned long sendItWindowStart, sendItFwdStart;

    // Heading-hold straight-line correction
    unsigned long headingTowardStart;
    float         cruiseHeading;
    bool          headingLocked;
    unsigned long phase0EnteredAt;

    // Pit / hill previous readings
    float         prevDistL, prevDistR;
    unsigned long pitPitchStart;
    float         hillPrevDistL, hillPrevDistR;
    unsigned long hillPitchStart;

    unsigned long lastDecision;
    unsigned long lastDebug;
};

// ===================================================================
// WALL FOLLOW STATE
// ===================================================================
enum WFState { WF_SEARCH=0, WF_FOLLOW, WF_CORNER, WF_BLOCKED, WF_CRUISE };

struct WallFollowState {
    WFState       state;
    unsigned long actionUntil;
    unsigned long searchStart;
    float         pidI, prevErr;
    unsigned long lastDebug;
};

// ===================================================================
// ROBOT MODES — top-level dispatcher
// ===================================================================
class RobotModes {
public:
    RobotModes();

    // Register a motor output callback: fn(leftPWM, rightPWM), range -255..255.
    // Called automatically by update(). You may also poll getMotorSpeeds().
    void setMotorCallback(void (*fn)(int left, int right));

    // Select a mode. Safe to call at any time.
    void setMode(RobotMode mode);
    RobotMode getMode() const { return _mode; }

    // Call every ~50 ms (20 Hz) with fresh sensor data.
    void update(const SensorInput& sensors);

    // Read the most recent motor output directly.
    void getMotorSpeeds(int& left, int& right) const;

    // Direct access to the advanced nav core.
    // Use to call enterAvoidFromPit(), enterAvoidFromHill(), enterTerrainBoost(), etc.
    AdvNavCore& advNav() { return _nav; }

    // Reset persistent state for a mode (call after setMode if desired).
    void resetSimpleAutoState();
    void resetWallFollowState();

private:
    RobotMode       _mode;
    void          (*_motorFn)(int, int);
    int             _motorL, _motorR;
    const SensorInput* _s;   // pointer to current sensor frame (valid during update())

    AdvNavCore      _nav;
    SimpleAutoState _sa;
    WallFollowState _wf;

    void setMotors(int left, int right);
    void runRC();
    void runUART();
    void runSOK();
    void runAdvancedAuto();
    void runSimpleAuto();
    void runWallFollow();
};
