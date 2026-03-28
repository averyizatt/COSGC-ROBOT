
#ifndef OCCUPANCY_MAP_H
#define OCCUPANCY_MAP_H

#include <Arduino.h>

// Map configuration - 5m x 5m coverage at 50cm resolution
// Uses ~200 bytes RAM (10x10 grid x2 arrays) - ultra lightweight
#define MAP_WIDTH       10      // Grid cells wide
#define MAP_HEIGHT      10      // Grid cells tall
#define CELL_SIZE_CM    50.0f   // Each cell = 50cm x 50cm
#define MAP_CENTER_X    5       // Robot starts at center
#define MAP_CENTER_Y    5

// ── Cell value encoding (0-255 probabilistic occupancy) ──
// 0 = certainly free ... 127 = unknown ... 255 = certainly occupied
// Special sentinels: 240 = hill, 245 = pit, 250 = hazard (treated as non-traversable)
#define CELL_UNKNOWN    127     // Unexplored (prior)
#define CELL_FREE       0       // Confirmed empty
#define CELL_OCCUPIED   255     // Confirmed obstacle
#define CELL_HILL       240     // Significant hill / steep slope
#define CELL_PIT        245     // Pit / drop-off / cliff edge
#define CELL_HAZARD     250     // Hazard zone (steep incline, stall, etc.)
#define CELL_ROBOT      200     // Robot's current position (for visualization only)

// ── Classification thresholds ──
// These define confidence bands for interpreting cell probability values
#define THRESH_FREE         40  // Below this = confident free
#define THRESH_LIKELY_FREE  80  // Below this = probably free
#define THRESH_LIKELY_OCC  180  // Above this = probably occupied
#define THRESH_OCCUPIED    220  // Above this = confident occupied

// ── Confidence adjustment (base values, scaled by distance) ──
#define OCCUPIED_INCREMENT  15  // Base increase on obstacle detection (needs multiple hits to confirm)
#define FREE_DECREMENT      25  // Base decrease when ray passes through (clears false positives faster)

// Classified cell types for navigation decisions
enum CellType {
    CTYPE_FREE = 0,         // High confidence free (cell < 40)
    CTYPE_LIKELY_FREE,      // Moderate confidence free (40-80)
    CTYPE_UNKNOWN,          // Uncertain (80-180)
    CTYPE_LIKELY_OCCUPIED,  // Moderate confidence obstacle (180-220)
    CTYPE_OCCUPIED,         // High confidence obstacle (>220)
    CTYPE_HAZARD,           // Hazard zone (cell == 250)
    CTYPE_PIT,              // Pit / drop-off (cell == 245)
    CTYPE_HILL              // Steep hill (cell == 240)
};

// Terrain height reading (recent estimates from pitch + ultrasonic)
struct TerrainReading {
    int16_t gridX, gridY;   // Map cell coordinates
    int16_t heightCm;       // Estimated height relative to start (+up, -down)
};
#define TERRAIN_RING_SIZE 32  // Circular buffer of recent height readings

class OccupancyMap {
public:
    OccupancyMap();
    void begin();  // Allocate grid buffers — call from setup(), not constructor
    void addReading(float distanceCm, float heading);
    uint8_t getCell(int x, int y);
    CellType getCellType(int x, int y);
    void setCell(int x, int y, uint8_t value);
    int getRobotX() { return (int)robotX; }
    int getRobotY() { return (int)robotY; }
    float getRobotHeading() { return robotHeading; }
    void reset();
    void printMap();
    float getExplorationPercent();
    int getMapShiftCount() { return mapShiftCount; }
    volatile bool busy = false;  // Set during grid mutations to block async readers
    uint8_t getVisitCount(int x, int y);
    void markHazardCell();
    void markPitAhead(float heading, float distanceCm);
    void markHillAhead(float heading, float distanceCm, float estimatedHeightCm);
    void addTerrainReading(int gx, int gy, int16_t heightCm);
    void setOdometryPosition(float x_cm, float y_cm, float theta_rad);
private:
    // Internal-only accessors (used by printMap / getExplorationPercent)
    float getWorldX() { return (robotX - MAP_CENTER_X) * CELL_SIZE_CM + worldOriginX; }
    float getWorldY() { return (robotY - MAP_CENTER_Y) * CELL_SIZE_CM + worldOriginY; }
    float getEstimatedClearance(float heading, float maxRange = 100);
    int getExploredCells();
    int getOccupiedCells();
    int getHazardCells();
    int getPitCells();
    int getHillCells();

    // Use heap-allocated 1D buffers to avoid large .bss usage
    uint8_t *grid;            // size MAP_WIDTH * MAP_HEIGHT
    uint8_t *visitCount;      // size MAP_WIDTH * MAP_HEIGHT
    float robotX;
    float robotY;
    float robotHeading;
    float worldOriginX;
    float worldOriginY;
    int mapShiftCount;
    float totalDistanceCm;
    unsigned long lastUpdateTime;
    // Terrain height ring buffer
    TerrainReading terrainRing[TERRAIN_RING_SIZE];
    int terrainRingIdx;
    int terrainRingCount;
    static constexpr float MAX_SPEED_CM_S = 100.0f;
    static constexpr int EDGE_MARGIN = 2;
    bool worldToGrid(float worldX, float worldY, int& gridX, int& gridY);
    void traceRay(float startX, float startY, float angle, float distanceCm, bool hitObstacle);
    bool inBounds(int x, int y);
    void shiftMap(int shiftX, int shiftY);
    void checkAndShiftMap();
};

#endif // OCCUPANCY_MAP_H
