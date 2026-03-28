#ifndef AUTONOMOUS_NAV_H
#define AUTONOMOUS_NAV_H

#include <Arduino.h>

// ==================== COSGC COMPETITION ROVER ====================
// High-level autonomous navigation state machine
//
// States:
//   CRUISE    → Normal forward driving, covering maximum ground
//   AVOID     → 3-phase obstacle avoidance (backup → turn → verify)
//   RECOVER   → Multi-phase stuck recovery (rock → diagonal → full reverse)
//   HAZARD    → Steep incline handling (backup → turn → reattempt)
//   STOPPED   → Motors off (initial / commanded stop)
//
// Design principles:
//   - Each state has ONE job with clear entry/exit conditions
//   - Sub-phases within states prevent oscillation
//   - No decisions during timed actions (sequential execution)
//   - Timeouts on every state prevent infinite loops
//   - Clean transitions: only the state handler decides the next state

// === Primary navigation states ===
enum NavState {
    NAV_CRUISE = 0,     // Forward driving — covering ground
    NAV_AVOID,          // Obstacle avoidance sequence
    NAV_RECOVER,        // Stuck recovery sequence
    NAV_HAZARD,         // Steep incline handling
    NAV_STOPPED         // Motors off
};

// === Avoidance sub-phases ===
enum AvoidPhase {
    AVOID_BACKUP = 0,   // Reversing away from obstacle
    AVOID_TURN,         // Turning to clear direction
    AVOID_VERIFY        // Brief forward to confirm path is clear
};

// === Recovery sub-phases ===
enum RecoveryPhase {
    RECOVERY_ROCK_FWD = 0,
    RECOVERY_COAST,
    RECOVERY_ROCK_REV,
    RECOVERY_DIAGONAL_TURN,
    RECOVERY_DIAGONAL_FWD,
    RECOVERY_FULL_REVERSE,
    RECOVERY_DONE
};

// === Hazard sub-phases ===
enum HazardPhase {
    HAZARD_BACKUP = 0,  // Reversing from incline
    HAZARD_TURN         // Turning to try different angle
};

// Internal turn direction
enum TurnDirection {
    TURN_LEFT = 0,
    TURN_RIGHT,
    TURN_RANDOM
};

// Orientation state
enum Orientation {
    ORIENTATION_NORMAL = 0,
    ORIENTATION_UPSIDE_DOWN,
    ORIENTATION_TILTED_LEFT,
    ORIENTATION_TILTED_RIGHT,
    ORIENTATION_TILTED_FORWARD,
    ORIENTATION_TILTED_BACK
};

class AutonomousNav {
    // Helper: slip/stall detection
    bool isSlippingOrStalled();
    // Odometry input (from encoders)
public:
    AutonomousNav();

    // Main update — call every loop iteration with front distance (cm)
    void update(float distance);

    // Sensor inputs
    void setSideDistances(float leftDist, float rightDist);
    void setWallAngle(float angle);
    void updateIMU(float accelX, float accelY, float accelZ,
                   float gyroX = 0, float gyroY = 0, float gyroZ = 0);
    void setEncoderSpeed(float speedCmPerSec);  // From hall effect encoders

    // Odometry input (from encoders)
    void setOdometry(float x, float y, float theta);
    float odomX = 0, odomY = 0, odomTheta = 0;

    // Waypoint navigation
    void setNavTarget(float x_cm, float y_cm);
    bool hasNavTarget = false;
    float navTargetX = 0, navTargetY = 0;

    // Command: return to start (origin)
    void returnToStart();

    // Pit detected externally — enter avoidance backup
    void enterAvoidFromPit();

    // Hill detected externally — enter avoidance backup
    void enterAvoidFromHill();

    // Minor terrain detected — maintain full speed to cross it
    void enterTerrainBoost();
    bool isTerrainBoostActive();
    
    // Terrain learning — check outcome after traversal attempt
    void checkTerrainOutcome();

    // Command: turn to absolute heading (radians, odometry frame)
    void turnToHeading(float targetTheta);
    bool turningToHeading = false;
    float targetTurnTheta = 0;

    // Motor output
    void getMotorSpeeds(int& leftSpeed, int& rightSpeed);

    // Control
    void reset();

    // State access for external emergency handling
    NavState getCurrentState() { return currentState; }
    void enterAvoid(bool critical);  // Trigger avoidance from outside (e.g. e-stop)

    // Status
    const char* getStateString();

    // Orientation
    bool isUpsideDown();
    Orientation getOrientation();
    float getPitch();
    float getRoll();
    float getHeading();

    // Terrain awareness
    bool isMotionVerified();
    bool isRecentlyStuck();

private:
    // === STATE ===
    NavState currentState;
    unsigned long stateStartTime;
    unsigned long lastStateChange;
    
    // === DISTANCE ===
    float lastDistance;
    float distanceHistory[5];
    int historyIndex;
    int stuckCounter;
    
    // === SPEED ===
    int currentSpeed;
    int targetSpeed;
    
    // === STALL ===
    unsigned long stallStartTime;
    
    // === ENCODER ===
    float encoderSpeedCmS;  // Last reported encoder speed (cm/s)
    
    // === SENSORS ===
    float sideDistLeft;
    float sideDistRight;
    float detectedWallAngle;
    
    // === IMU ===
    bool upsideDown;
    Orientation currentOrientation;
    float pitch, roll;
    float accelX, accelY, accelZ;
    float gyroZ;
    bool imuAvailable;
    
    // === HEADING / DEAD RECKONING ===
    float heading;
    float headingAtObstacle;
    unsigned long lastIMUUpdate;
    float drX, drY;
    unsigned long headingTowardStartSince;
    
    // === HEADING HOLD (straight-line correction) ===
    float cruiseHeading;        // Target heading locked when entering CRUISE
    bool headingLocked;         // Whether cruiseHeading is valid
    unsigned long cruiseEnteredAt; // When we entered CRUISE (for settle delay)
    
    // === TURN TRACKING ===
    TurnDirection lastTurnDirection;
    float turnStartHeading;
    int consecutiveSameDirection;
    int obstacleMemory[8];
    
    bool firstRun;
    
    // === AVOIDANCE STATE ===
    AvoidPhase avoidPhase;
    TurnDirection avoidTurnDir;
    int avoidAttempts;
    unsigned long avoidPhaseStart;
    unsigned long avoidPhaseDuration;
    
    // === HAZARD STATE ===
    HazardPhase hazardPhase;
    unsigned long hazardPhaseStart;
    unsigned long hazardPhaseDuration;
    int inclineAttempts;
    unsigned long steepInclineStart;
    bool onSteepIncline;
    
    // === TERRAIN / MOTION VERIFICATION ===
    float accelMagHistory[8];
    int accelMagIndex;
    bool accelMagFilled;
    bool motionVerified;
    unsigned long noMotionStartTime;
    
    // === RECOVERY STATE ===
    RecoveryPhase recoveryPhase;
    int rockCount;
    unsigned long recoveryStepStart;
    unsigned long recoveryStepDuration;
    TurnDirection recoveryTurnDir;
    bool coastAfterFwd;
    
    // === ADAPTIVE RAMP ===
    bool recentlyStuck;
    unsigned long stuckRecoveryTime;
    unsigned long recoveryCooldownUntil;
    
    // === TERRAIN BOOST ===
    bool terrainBoostActive;
    unsigned long terrainBoostUntil;
    int terrainBoostStartX;   // Grid position when boost started
    int terrainBoostStartY;
    bool terrainBoostStuck;   // Set if motion fails during boost
    
    // === STATE HANDLERS ===
    void handleCruise(float distance);
    void handleAvoid(float distance);
    void handleRecovery(float distance);
    void handleHazard(float distance);
    
    // === STATE TRANSITIONS ===
    void enterCruise();
    void enterRecover();
    void enterHazard();
    
    // === TURN DECISIONS ===
    TurnDirection pickTurnDirection(float distance);
    TurnDirection getBestTurnDirection();
    int getHeadingBin();
    
    // === TERRAIN ===
    void checkMotionVerification();
    void startRecovery();
    void flagRecentlyStuck();
    
    // === HELPERS ===
    bool isStuck();
    void updateDistanceHistory(float distance);
    int calculateSpeed(float distance);
    void smoothSpeed();
};

#endif // AUTONOMOUS_NAV_H
