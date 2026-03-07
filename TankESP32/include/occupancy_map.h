#ifndef OCCUPANCY_MAP_H
#define OCCUPANCY_MAP_H

#include <Arduino.h>

// Map configuration - 15m x 15m coverage at 5cm resolution
// Uses ~90KB RAM (300x300 grid) - reduced to fit with Bluepad32 BT stack
// Persistent map - no shifting, keeps entire course in memory
#define MAP_WIDTH       300     // Grid cells wide
#define MAP_HEIGHT      300     // Grid cells tall
#define CELL_SIZE_CM    5.0f    // Each cell = 5cm x 5cm
#define MAP_CENTER_X    150     // Robot starts at center
#define MAP_CENTER_Y    150

// Cell values (0-255)
#define CELL_UNKNOWN    127     // Unexplored
#define CELL_FREE       0       // Confirmed empty
#define CELL_OCCUPIED   255     // Confirmed obstacle
#define CELL_ROBOT      200     // Robot's current position (for visualization)

// Confidence adjustment per observation
#define OCCUPIED_INCREMENT  40  // How much to increase on obstacle detection
#define FREE_DECREMENT      10  // How much to decrease when ray passes through

class OccupancyMap {
public:
    OccupancyMap();
    
    // Update robot position (call frequently)
    void updatePosition(float heading, float speedPercent, float deltaTime);
    
    // Add ultrasonic reading to map
    void addReading(float distanceCm, float heading);
    
    // Get map info
    uint8_t getCell(int x, int y);
    void setCell(int x, int y, uint8_t value);
    
    // Robot position in grid coordinates
    int getRobotX() { return (int)robotX; }
    int getRobotY() { return (int)robotY; }
    float getRobotHeading() { return robotHeading; }
    
    // World position (cm from original start, accounts for map shifts)
    float getWorldX() { return (robotX - MAP_CENTER_X) * CELL_SIZE_CM + worldOriginX; }
    float getWorldY() { return (robotY - MAP_CENTER_Y) * CELL_SIZE_CM + worldOriginY; }
    
    // Reset map and position
    void reset();
    
    // Check if a direction is clear (for navigation)
    // Returns estimated clear distance in that direction (cm)
    float getEstimatedClearance(float heading, float maxRange = 100);
    
    // Get best direction to turn (lowest obstacle density)
    // Returns heading offset in degrees (-180 to +180)
    float getBestTurnDirection();
    
    // Debug: print map to serial (downsampled)
    void printMap();
    
    // Statistics
    int getExploredCells();
    int getOccupiedCells();
    float getExplorationPercent();
    int getMapShiftCount() { return mapShiftCount; }

private:
    // The map grid - uses ~40KB RAM for 200x200
    uint8_t grid[MAP_WIDTH][MAP_HEIGHT];
    
    // Robot position in grid coordinates (floating point for precision)
    float robotX;
    float robotY;
    float robotHeading;  // degrees, 0 = North/forward
    
    // World origin offset (tracks total map shifts)
    float worldOriginX;  // cm offset from original start
    float worldOriginY;
    int mapShiftCount;   // How many times map has shifted
    
    // Distance traveled estimation
    float totalDistanceCm;
    unsigned long lastUpdateTime;
    
    // Speed to cm/s conversion (approximate)
    // At 100% speed, roughly 30 cm/s (adjust based on your motors)
    static constexpr float MAX_SPEED_CM_S = 30.0f;
    
    // Map boundary margin - disabled for persistent map (set larger than possible)
    static constexpr int EDGE_MARGIN = 5;  // Only shift if within 5 cells of edge (25cm)
    
    // Helper: convert world coords to grid coords
    bool worldToGrid(float worldX, float worldY, int& gridX, int& gridY);
    
    // Helper: trace a ray and update cells
    void traceRay(float startX, float startY, float angle, float distanceCm, bool hitObstacle);
    
    // Helper: check bounds
    bool inBounds(int x, int y);
    
    // Helper: shift map when robot approaches edge
    void shiftMap(int shiftX, int shiftY);
    
    // Helper: check if map needs shifting
    void checkAndShiftMap();
};

#endif // OCCUPANCY_MAP_H
