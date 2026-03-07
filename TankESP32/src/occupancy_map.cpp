#include "occupancy_map.h"
#include <math.h>

OccupancyMap::OccupancyMap() {
    reset();
}

void OccupancyMap::reset() {
    // Initialize all cells as unknown
    for (int x = 0; x < MAP_WIDTH; x++) {
        for (int y = 0; y < MAP_HEIGHT; y++) {
            grid[x][y] = CELL_UNKNOWN;
        }
    }
    
    // Robot starts at center
    robotX = MAP_CENTER_X;
    robotY = MAP_CENTER_Y;
    robotHeading = 0;
    totalDistanceCm = 0;
    lastUpdateTime = millis();
    
    // World origin tracking
    worldOriginX = 0;
    worldOriginY = 0;
    mapShiftCount = 0;
    
    Serial.println("[MAP] Reset - robot at center (infinite exploration enabled)");
}

bool OccupancyMap::inBounds(int x, int y) {
    return x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT;
}

uint8_t OccupancyMap::getCell(int x, int y) {
    if (!inBounds(x, y)) return CELL_OCCUPIED;  // Out of bounds = wall
    return grid[x][y];
}

void OccupancyMap::setCell(int x, int y, uint8_t value) {
    if (inBounds(x, y)) {
        grid[x][y] = value;
    }
}

void OccupancyMap::updatePosition(float heading, float speedPercent, float deltaTime) {
    robotHeading = heading;
    
    // Estimate distance traveled based on speed and time
    // speedPercent is -100 to +100, deltaTime is in seconds
    float speedCmPerSec = (speedPercent / 100.0f) * MAX_SPEED_CM_S;
    float distanceCm = speedCmPerSec * deltaTime;
    
    // Convert heading to radians (0° = +Y direction in our grid)
    float headingRad = robotHeading * PI / 180.0f;
    
    // Update position
    // In our coordinate system: +Y is forward (heading 0°), +X is right
    float dx = sin(headingRad) * distanceCm / CELL_SIZE_CM;
    float dy = cos(headingRad) * distanceCm / CELL_SIZE_CM;
    
    robotX += dx;
    robotY += dy;
    
    totalDistanceCm += abs(distanceCm);
    
    // Check if we need to shift the map (robot near edge)
    checkAndShiftMap();
    
    // Mark current position as free
    int gx = (int)robotX;
    int gy = (int)robotY;
    if (inBounds(gx, gy)) {
        grid[gx][gy] = CELL_FREE;
    }
}

void OccupancyMap::checkAndShiftMap() {
    int shiftX = 0;
    int shiftY = 0;
    
    // Check if robot is too close to any edge
    if (robotX < EDGE_MARGIN) {
        shiftX = MAP_CENTER_X - (int)robotX;  // Shift right (positive)
    } else if (robotX > MAP_WIDTH - EDGE_MARGIN) {
        shiftX = MAP_CENTER_X - (int)robotX;  // Shift left (negative)
    }
    
    if (robotY < EDGE_MARGIN) {
        shiftY = MAP_CENTER_Y - (int)robotY;  // Shift up (positive)
    } else if (robotY > MAP_HEIGHT - EDGE_MARGIN) {
        shiftY = MAP_CENTER_Y - (int)robotY;  // Shift down (negative)
    }
    
    if (shiftX != 0 || shiftY != 0) {
        shiftMap(shiftX, shiftY);
    }
}

void OccupancyMap::shiftMap(int shiftX, int shiftY) {
    mapShiftCount++;
    
    // Update world origin to track absolute position
    worldOriginX -= shiftX * CELL_SIZE_CM;
    worldOriginY -= shiftY * CELL_SIZE_CM;
    
    Serial.printf("[MAP] Shifting map by (%d, %d) cells - shift #%d\n", 
                  shiftX, shiftY, mapShiftCount);
    Serial.printf("[MAP] World origin now at (%.0f, %.0f) cm\n", 
                  worldOriginX, worldOriginY);
    
    // Create a temporary buffer for the shift
    // We'll do this in-place by careful copying
    
    if (shiftX > 0) {
        // Shift right: copy from left to right
        for (int x = MAP_WIDTH - 1; x >= 0; x--) {
            for (int y = 0; y < MAP_HEIGHT; y++) {
                int srcX = x - shiftX;
                if (srcX >= 0 && srcX < MAP_WIDTH) {
                    grid[x][y] = grid[srcX][y];
                } else {
                    grid[x][y] = CELL_UNKNOWN;  // New unexplored area
                }
            }
        }
    } else if (shiftX < 0) {
        // Shift left: copy from right to left
        for (int x = 0; x < MAP_WIDTH; x++) {
            for (int y = 0; y < MAP_HEIGHT; y++) {
                int srcX = x - shiftX;
                if (srcX >= 0 && srcX < MAP_WIDTH) {
                    grid[x][y] = grid[srcX][y];
                } else {
                    grid[x][y] = CELL_UNKNOWN;
                }
            }
        }
    }
    
    if (shiftY > 0) {
        // Shift up: copy from bottom to top
        for (int y = MAP_HEIGHT - 1; y >= 0; y--) {
            for (int x = 0; x < MAP_WIDTH; x++) {
                int srcY = y - shiftY;
                if (srcY >= 0 && srcY < MAP_HEIGHT) {
                    grid[x][y] = grid[x][srcY];
                } else {
                    grid[x][y] = CELL_UNKNOWN;
                }
            }
        }
    } else if (shiftY < 0) {
        // Shift down: copy from top to bottom
        for (int y = 0; y < MAP_HEIGHT; y++) {
            for (int x = 0; x < MAP_WIDTH; x++) {
                int srcY = y - shiftY;
                if (srcY >= 0 && srcY < MAP_HEIGHT) {
                    grid[x][y] = grid[x][srcY];
                } else {
                    grid[x][y] = CELL_UNKNOWN;
                }
            }
        }
    }
    
    // Update robot position to new grid coordinates
    robotX += shiftX;
    robotY += shiftY;
    
    Serial.printf("[MAP] Robot now at grid (%d, %d)\n", (int)robotX, (int)robotY);
}

void OccupancyMap::addReading(float distanceCm, float heading) {
    // Clamp distance to reasonable range
    if (distanceCm < 2 || distanceCm > 400) return;
    
    // Trace ray from robot position in the given heading direction
    bool hitObstacle = (distanceCm < 395);  // If < max range, we hit something
    
    traceRay(robotX, robotY, heading, distanceCm, hitObstacle);
}

void OccupancyMap::traceRay(float startX, float startY, float angle, float distanceCm, bool hitObstacle) {
    float angleRad = angle * PI / 180.0f;
    
    // Direction vector
    float dirX = sin(angleRad);
    float dirY = cos(angleRad);
    
    // Number of steps (one per cell approximately)
    float distanceCells = distanceCm / CELL_SIZE_CM;
    int numSteps = (int)distanceCells;
    
    // Trace ray, marking cells as free
    for (int i = 1; i < numSteps; i++) {
        int cellX = (int)(startX + dirX * i);
        int cellY = (int)(startY + dirY * i);
        
        if (!inBounds(cellX, cellY)) break;
        
        // Decrease occupancy (mark as more likely free)
        uint8_t current = grid[cellX][cellY];
        if (current > FREE_DECREMENT) {
            grid[cellX][cellY] = current - FREE_DECREMENT;
        } else {
            grid[cellX][cellY] = CELL_FREE;
        }
    }
    
    // If we hit an obstacle, mark that cell as occupied
    if (hitObstacle) {
        int obstacleX = (int)(startX + dirX * distanceCells);
        int obstacleY = (int)(startY + dirY * distanceCells);
        
        if (inBounds(obstacleX, obstacleY)) {
            // Increase occupancy
            uint8_t current = grid[obstacleX][obstacleY];
            if (current < 255 - OCCUPIED_INCREMENT) {
                grid[obstacleX][obstacleY] = current + OCCUPIED_INCREMENT;
            } else {
                grid[obstacleX][obstacleY] = CELL_OCCUPIED;
            }
            
            // Also mark adjacent cells (ultrasonic has ~15° cone)
            // This accounts for the sensor's beam width
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    int nx = obstacleX + dx;
                    int ny = obstacleY + dy;
                    if (inBounds(nx, ny) && (dx != 0 || dy != 0)) {
                        uint8_t adj = grid[nx][ny];
                        if (adj < 255 - OCCUPIED_INCREMENT/2) {
                            grid[nx][ny] = adj + OCCUPIED_INCREMENT/2;
                        }
                    }
                }
            }
        }
    }
}

float OccupancyMap::getEstimatedClearance(float heading, float maxRange) {
    float angleRad = heading * PI / 180.0f;
    float dirX = sin(angleRad);
    float dirY = cos(angleRad);
    
    float maxCells = maxRange / CELL_SIZE_CM;
    
    for (int i = 1; i <= (int)maxCells; i++) {
        int cellX = (int)(robotX + dirX * i);
        int cellY = (int)(robotY + dirY * i);
        
        if (!inBounds(cellX, cellY)) {
            return (i - 1) * CELL_SIZE_CM;  // Hit map edge
        }
        
        // Consider occupied if confidence > 180
        if (grid[cellX][cellY] > 180) {
            return (i - 1) * CELL_SIZE_CM;
        }
    }
    
    return maxRange;  // Clear to max range
}

float OccupancyMap::getBestTurnDirection() {
    // Check clearance in 8 directions (every 45°)
    float bestHeading = robotHeading;
    float bestClearance = 0;
    
    for (int i = 0; i < 8; i++) {
        float checkHeading = robotHeading + (i * 45) - 180;
        float clearance = getEstimatedClearance(checkHeading, 100);
        
        // Slight bias toward forward directions
        if (abs(i * 45 - 180) < 90) {
            clearance *= 1.1f;  // 10% bonus for forward-ish directions
        }
        
        if (clearance > bestClearance) {
            bestClearance = clearance;
            bestHeading = checkHeading;
        }
    }
    
    // Return offset from current heading
    float offset = bestHeading - robotHeading;
    while (offset > 180) offset -= 360;
    while (offset < -180) offset += 360;
    
    return offset;
}

int OccupancyMap::getExploredCells() {
    int count = 0;
    for (int x = 0; x < MAP_WIDTH; x++) {
        for (int y = 0; y < MAP_HEIGHT; y++) {
            if (grid[x][y] != CELL_UNKNOWN) count++;
        }
    }
    return count;
}

int OccupancyMap::getOccupiedCells() {
    int count = 0;
    for (int x = 0; x < MAP_WIDTH; x++) {
        for (int y = 0; y < MAP_HEIGHT; y++) {
            if (grid[x][y] > 180) count++;  // High confidence = occupied
        }
    }
    return count;
}

float OccupancyMap::getExplorationPercent() {
    return (getExploredCells() * 100.0f) / (MAP_WIDTH * MAP_HEIGHT);
}

void OccupancyMap::printMap() {
    // Downsample factor - 20:1 for 500x500 map = 25x25 output
    const int DS = 20;
    
    Serial.printf("\n[MAP] Top-down view (%dx%d downsampled, 25m x 25m PERSISTENT):\n", DS, DS);
    Serial.println("    Legend: . = free, # = obstacle, ? = unknown, @ = robot");
    
    // Print header with X coordinates
    Serial.print("   ");
    for (int x = 0; x < MAP_WIDTH; x += DS) {
        Serial.printf("%d", (x / DS) % 10);
    }
    Serial.println();
    
    // Print map (downsampled)
    for (int y = MAP_HEIGHT - 1; y >= 0; y -= DS) {
        Serial.printf("%2d ", (y / DS) % 100);
        
        for (int x = 0; x < MAP_WIDTH; x += DS) {
            // Check if robot is in this block
            int rx = (int)robotX;
            int ry = (int)robotY;
            if (rx >= x && rx < x + DS && ry >= y - DS + 1 && ry <= y) {
                Serial.print("@");
                continue;
            }
            
            // Sample the block
            int freeCount = 0;
            int occCount = 0;
            int total = 0;
            
            for (int dx = 0; dx < DS && x + dx < MAP_WIDTH; dx++) {
                for (int dy = 0; dy < DS && y - dy >= 0; dy++) {
                    uint8_t val = grid[x + dx][y - dy];
                    if (val < 60) freeCount++;
                    else if (val > 180) occCount++;
                    total++;
                }
            }
            
            // Pick dominant cell type
            if (occCount > total / 8) {
                Serial.print("#");
            } else if (freeCount > total / 2) {
                Serial.print(".");
            } else {
                Serial.print("?");
            }
        }
        Serial.println();
    }
    
    // Print stats
    Serial.printf("[MAP] Robot at grid(%d,%d) heading %.0f° | Full course: 25m x 25m\n", 
                  (int)robotX, (int)robotY, robotHeading);
    Serial.printf("[MAP] World pos: (%.1f, %.1f) m from start | Distance: %.1f m\n",
                  getWorldX() / 100.0f, getWorldY() / 100.0f, totalDistanceCm / 100.0f);
    Serial.printf("[MAP] Explored: %.1f%% (%d cells), Obstacles: %d cells\n",
                  getExplorationPercent(), getExploredCells(), getOccupiedCells());
}
