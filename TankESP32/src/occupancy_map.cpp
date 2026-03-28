#include "occupancy_map.h"
#include "config.h"

#include <math.h>
#include <stdlib.h>

// Helper macro for 1D indexing: index = x * MAP_HEIGHT + y
#define IDX(x,y) ((x) * MAP_HEIGHT + (y))

uint8_t OccupancyMap::getVisitCount(int x, int y) {
    if (inBounds(x, y) && visitCount) return visitCount[IDX(x,y)];
    return 255;
}

CellType OccupancyMap::getCellType(int x, int y) {
    uint8_t val = getCell(x, y);
    if (val == CELL_HILL) return CTYPE_HILL;
    if (val == CELL_PIT) return CTYPE_PIT;
    if (val == CELL_HAZARD) return CTYPE_HAZARD;
    if (val < THRESH_FREE) return CTYPE_FREE;
    if (val < THRESH_LIKELY_FREE) return CTYPE_LIKELY_FREE;
    if (val > THRESH_OCCUPIED) return CTYPE_OCCUPIED;
    if (val > THRESH_LIKELY_OCC) return CTYPE_LIKELY_OCCUPIED;
    return CTYPE_UNKNOWN;
}

void OccupancyMap::markHazardCell() {
    int x = (int)robotX;
    int y = (int)robotY;
    if (inBounds(x, y) && grid) {
        grid[IDX(x,y)] = CELL_HAZARD;
    }
}

void OccupancyMap::markPitAhead(float heading, float distanceCm) {
    if (!grid) return;
    float angleRad = heading * PI / 180.0f;
    float dirX = sin(angleRad);
    float dirY = cos(angleRad);
    float startCells = distanceCm / CELL_SIZE_CM;
    
    // Mark a strip of PIT_MARK_DEPTH_CELLS deep starting at the detection distance
    for (int i = 0; i < PIT_MARK_DEPTH_CELLS; i++) {
        float dist = startCells + i;
        int cx = (int)(robotX + dirX * dist);
        int cy = (int)(robotY + dirY * dist);
        if (!inBounds(cx, cy)) break;
        grid[IDX(cx, cy)] = CELL_PIT;
        // Also mark ±1 cell perpendicular to heading for width
        int px = (int)(-dirY);  // perpendicular
        int py = (int)(dirX);
        if (inBounds(cx + px, cy + py)) grid[IDX(cx + px, cy + py)] = CELL_PIT;
        if (inBounds(cx - px, cy - py)) grid[IDX(cx - px, cy - py)] = CELL_PIT;
    }
    Serial.printf("[MAP] Pit marked at heading %.0f, dist %.0fcm (%d cells deep)\n",
                  heading, distanceCm, PIT_MARK_DEPTH_CELLS);
}

void OccupancyMap::markHillAhead(float heading, float distanceCm, float estimatedHeightCm) {
    if (!grid) return;
    float angleRad = heading * PI / 180.0f;
    float dirX = sin(angleRad);
    float dirY = cos(angleRad);
    float startCells = distanceCm / CELL_SIZE_CM;
    
    // Mark a strip of HILL_MARK_DEPTH_CELLS deep starting at the detection distance
    for (int i = 0; i < HILL_MARK_DEPTH_CELLS; i++) {
        float dist = startCells + i;
        int cx = (int)(robotX + dirX * dist);
        int cy = (int)(robotY + dirY * dist);
        if (!inBounds(cx, cy)) break;
        grid[IDX(cx, cy)] = CELL_HILL;
        // Also mark \u00b11 cell perpendicular for width
        int px = (int)(-dirY);
        int py = (int)(dirX);
        if (inBounds(cx + px, cy + py)) grid[IDX(cx + px, cy + py)] = CELL_HILL;
        if (inBounds(cx - px, cy - py)) grid[IDX(cx - px, cy - py)] = CELL_HILL;
    }
    
    // Store terrain height reading at the center of the marked area
    int midDist = (int)(startCells + HILL_MARK_DEPTH_CELLS / 2);
    int midX = (int)(robotX + dirX * midDist);
    int midY = (int)(robotY + dirY * midDist);
    addTerrainReading(midX, midY, (int16_t)estimatedHeightCm);
    
    Serial.printf("[MAP] Hill marked at heading %.0f, dist %.0fcm, est height %.0fcm\n",
                  heading, distanceCm, estimatedHeightCm);
}

void OccupancyMap::addTerrainReading(int gx, int gy, int16_t heightCm) {
    terrainRing[terrainRingIdx].gridX = (int16_t)gx;
    terrainRing[terrainRingIdx].gridY = (int16_t)gy;
    terrainRing[terrainRingIdx].heightCm = heightCm;
    terrainRingIdx = (terrainRingIdx + 1) % TERRAIN_RING_SIZE;
    if (terrainRingCount < TERRAIN_RING_SIZE) terrainRingCount++;
}

void OccupancyMap::setOdometryPosition(float x_cm, float y_cm, float theta_rad) {
    // Convert world cm to grid coordinates
    robotX = (x_cm / CELL_SIZE_CM) + MAP_CENTER_X;
    robotY = (y_cm / CELL_SIZE_CM) + MAP_CENTER_Y;
    robotHeading = theta_rad * 180.0f / PI; // store as degrees for consistency
    // Mark cell as visited
    int gx = (int)robotX;
    int gy = (int)robotY;
    if (inBounds(gx, gy) && visitCount) {
        uint8_t &vc = visitCount[IDX(gx,gy)];
        if (vc < 255) vc++;
    }
}

OccupancyMap::OccupancyMap() {
    // Don't allocate here — global constructors run before setup()
    // when the heap may not be fully available. Call begin() from setup().
    grid = nullptr;
    visitCount = nullptr;
    robotX = MAP_CENTER_X;
    robotY = MAP_CENTER_Y;
    robotHeading = 0;
    totalDistanceCm = 0;
    lastUpdateTime = 0;
    worldOriginX = 0;
    worldOriginY = 0;
    mapShiftCount = 0;
    terrainRingIdx = 0;
    terrainRingCount = 0;
}

void OccupancyMap::begin() {
    Serial.printf("[MAP] Allocating %d bytes for grid + visit (heap free: %u)\n",
                  MAP_WIDTH * MAP_HEIGHT * 2, ESP.getFreeHeap());
    grid = (uint8_t*)malloc(MAP_WIDTH * MAP_HEIGHT);
    visitCount = (uint8_t*)malloc(MAP_WIDTH * MAP_HEIGHT);
    if (!grid || !visitCount) {
        Serial.printf("[MAP] ERROR: Failed to allocate map buffers! (heap free: %u)\n", ESP.getFreeHeap());
        if (grid) free(grid);
        if (visitCount) free(visitCount);
        grid = nullptr;
        visitCount = nullptr;
        return;
    }
    Serial.printf("[MAP] Allocated OK (heap free: %u)\n", ESP.getFreeHeap());
    reset();
}

void OccupancyMap::reset() {
    busy = true;
    if (!grid || !visitCount) { busy = false; return; }
    // Initialize all cells as unknown and visits to 0
    for (int x = 0; x < MAP_WIDTH; x++) {
        for (int y = 0; y < MAP_HEIGHT; y++) {
            grid[IDX(x,y)] = CELL_UNKNOWN;
            visitCount[IDX(x,y)] = 0;
        }
    }
    
    // Robot starts at center
    robotX = MAP_CENTER_X;
    robotY = MAP_CENTER_Y;
    robotHeading = 0;
    totalDistanceCm = 0;
    lastUpdateTime = millis();
    
    // Seed a small area of FREE cells around the robot so frontier-based
    // exploration can immediately find frontiers at the edges of this zone.
    // Without this, all cells are UNKNOWN and no frontiers exist at startup.
    const int SEED_RADIUS = 1;  // 1 cell = 50cm around robot
    for (int dx = -SEED_RADIUS; dx <= SEED_RADIUS; dx++) {
        for (int dy = -SEED_RADIUS; dy <= SEED_RADIUS; dy++) {
            int sx = MAP_CENTER_X + dx;
            int sy = MAP_CENTER_Y + dy;
            if (sx >= 0 && sx < MAP_WIDTH && sy >= 0 && sy < MAP_HEIGHT) {
                grid[IDX(sx, sy)] = CELL_FREE;
            }
        }
    }
    
    // World origin tracking
    worldOriginX = 0;
    worldOriginY = 0;
    mapShiftCount = 0;
    
    // Reset terrain ring buffer
    terrainRingIdx = 0;
    terrainRingCount = 0;
    memset(terrainRing, 0, sizeof(terrainRing));
    
    Serial.println("[MAP] Reset - robot at center, seeded free zone (infinite exploration enabled)");
    busy = false;
}

bool OccupancyMap::inBounds(int x, int y) {
    return x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT;
}

uint8_t OccupancyMap::getCell(int x, int y) {
    if (!inBounds(x, y) || !grid) return CELL_OCCUPIED;  // Out of bounds = wall
    return grid[IDX(x,y)];
}

void OccupancyMap::setCell(int x, int y, uint8_t value) {
    if (inBounds(x, y) && grid) {
        grid[IDX(x,y)] = value;
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
    if (!grid) return;
    busy = true;
    mapShiftCount++;
    
    // Update world origin to track absolute position
    worldOriginX -= shiftX * CELL_SIZE_CM;
    worldOriginY -= shiftY * CELL_SIZE_CM;
    
    Serial.printf("[MAP] Shifting map by (%d, %d) cells - shift #%d\n", 
                  shiftX, shiftY, mapShiftCount);
    Serial.printf("[MAP] World origin now at (%.0f, %.0f) cm\n", 
                  worldOriginX, worldOriginY);
    
    // Shift in X
    if (shiftX > 0) {
        // Shift right: copy from left to right
        for (int x = MAP_WIDTH - 1; x >= 0; x--) {
            for (int y = 0; y < MAP_HEIGHT; y++) {
                int srcX = x - shiftX;
                if (srcX >= 0 && srcX < MAP_WIDTH) {
                    grid[IDX(x,y)] = grid[IDX(srcX,y)];
                } else {
                    grid[IDX(x,y)] = CELL_UNKNOWN;  // New unexplored area
                }
            }
        }
    } else if (shiftX < 0) {
        // Shift left: copy from right to left
        for (int x = 0; x < MAP_WIDTH; x++) {
            for (int y = 0; y < MAP_HEIGHT; y++) {
                int srcX = x - shiftX;
                if (srcX >= 0 && srcX < MAP_WIDTH) {
                    grid[IDX(x,y)] = grid[IDX(srcX,y)];
                } else {
                    grid[IDX(x,y)] = CELL_UNKNOWN;
                }
            }
        }
    }
    
    // Shift in Y
    if (shiftY > 0) {
        // Shift up: copy from bottom to top
        for (int y = MAP_HEIGHT - 1; y >= 0; y--) {
            for (int x = 0; x < MAP_WIDTH; x++) {
                int srcY = y - shiftY;
                if (srcY >= 0 && srcY < MAP_HEIGHT) {
                    grid[IDX(x,y)] = grid[IDX(x,srcY)];
                } else {
                    grid[IDX(x,y)] = CELL_UNKNOWN;
                }
            }
        }
    } else if (shiftY < 0) {
        // Shift down: copy from top to bottom
        for (int y = 0; y < MAP_HEIGHT; y++) {
            for (int x = 0; x < MAP_WIDTH; x++) {
                int srcY = y - shiftY;
                if (srcY >= 0 && srcY < MAP_HEIGHT) {
                    grid[IDX(x,y)] = grid[IDX(x,srcY)];
                } else {
                    grid[IDX(x,y)] = CELL_UNKNOWN;
                }
            }
        }
    }
    
    // Update robot position to new grid coordinates
    robotX += shiftX;
    robotY += shiftY;
    
    Serial.printf("[MAP] Robot now at grid (%d, %d)\n", (int)robotX, (int)robotY);
    busy = false;
}

void OccupancyMap::addReading(float distanceCm, float heading) {
    if (!grid) return;
    // Clamp distance to reasonable range
    if (distanceCm < 2 || distanceCm > 400) return;
    
    // Trace ray from robot position in the given heading direction
    bool hitObstacle = (distanceCm < 395);  // If < max range, we hit something
    
    busy = true;
    traceRay(robotX, robotY, heading, distanceCm, hitObstacle);
    busy = false;
}

void OccupancyMap::traceRay(float startX, float startY, float angle, float distanceCm, bool hitObstacle) {
    if (!grid) return;
    float angleRad = angle * PI / 180.0f;
    
    // Direction vector
    float dirX = sin(angleRad);
    float dirY = cos(angleRad);
    
    // Number of steps (one per cell approximately)
    float distanceCells = distanceCm / CELL_SIZE_CM;
    int numSteps = (int)distanceCells;
    
    // Distance-dependent confidence scaling:
    // Closer readings are more reliable. Scale factor: 1.0 at 0cm, 0.3 at 400cm.
    float distFactor = 1.0f - 0.7f * (distanceCm / 400.0f);
    if (distFactor < 0.3f) distFactor = 0.3f;
    
    // Scaled decrements for ray-cleared cells
    int freeDecScaled = (int)(FREE_DECREMENT * distFactor);
    if (freeDecScaled < 3) freeDecScaled = 3;
    
    // Trace ray, marking cells as free
    for (int i = 1; i < numSteps; i++) {
        int cellX = (int)(startX + dirX * i);
        int cellY = (int)(startY + dirY * i);
        
        if (!inBounds(cellX, cellY)) break;
        
        uint8_t current = grid[IDX(cellX,cellY)];
        // Don't clear hazard, pit, or hill cells \u2014 they were set by terrain detection
        if (current == CELL_HAZARD || current == CELL_PIT || current == CELL_HILL) continue;
        
        // Decrease occupancy (mark as more likely free)
        if (current > freeDecScaled) {
            grid[IDX(cellX,cellY)] = current - freeDecScaled;
        } else {
            grid[IDX(cellX,cellY)] = CELL_FREE;
        }
    }
    
    // If we hit an obstacle, mark that cell and beam cone neighbors
    if (hitObstacle) {
        int obstacleX = (int)(startX + dirX * distanceCells);
        int obstacleY = (int)(startY + dirY * distanceCells);
        
        if (inBounds(obstacleX, obstacleY)) {
            // Scaled increment — closer hits get higher confidence
            int occIncScaled = (int)(OCCUPIED_INCREMENT * distFactor);
            if (occIncScaled < 10) occIncScaled = 10;
            
            // Mark the primary obstacle cell
            uint8_t current = grid[IDX(obstacleX,obstacleY)];
            if (current != CELL_HAZARD && current != CELL_PIT && current != CELL_HILL) {
                if (current < 255 - occIncScaled) {
                    grid[IDX(obstacleX,obstacleY)] = current + occIncScaled;
                } else {
                    grid[IDX(obstacleX,obstacleY)] = CELL_OCCUPIED;
                }
            }
            
            // Beam cone spread: ultrasonic ~15° half-angle
            // At distance d cells, beam width ≈ d * tan(15°) ≈ d * 0.27
            // With 50cm cells, clamp to 0-1 to avoid painting huge areas
            int spread = (int)(distanceCells * 0.27f);
            if (spread > 1) spread = 1;
            
            // Half-confidence for adjacent cells in the beam cone
            int adjInc = occIncScaled / 2;
            if (adjInc < 5) adjInc = 5;
            
            for (int dx = -spread; dx <= spread; dx++) {
                for (int dy = -spread; dy <= spread; dy++) {
                    if (dx == 0 && dy == 0) continue;
                    // Only within circular spread radius
                    if (dx * dx + dy * dy > spread * spread) continue;
                    int nx = obstacleX + dx;
                    int ny = obstacleY + dy;
                    if (inBounds(nx, ny)) {
                        uint8_t adj = grid[IDX(nx,ny)];
                        if (adj != CELL_HAZARD && adj != CELL_PIT && adj != CELL_HILL && adj < 255 - adjInc) {
                            grid[IDX(nx,ny)] = adj + adjInc;
                        }
                    }
                }
            }
        }
    }
}

float OccupancyMap::getEstimatedClearance(float heading, float maxRange) {
    if (!grid) return maxRange;
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
        // Use CellType classification for consistent obstacle detection
        CellType ct = getCellType(cellX, cellY);
        if (ct == CTYPE_OCCUPIED || ct == CTYPE_HAZARD || ct == CTYPE_PIT || ct == CTYPE_HILL) {
            return (i - 1) * CELL_SIZE_CM;
        }
        if (ct == CTYPE_LIKELY_OCCUPIED) {
            // Likely occupied — reduce clearance by 50% as a penalty
            return (i - 1) * CELL_SIZE_CM * 0.5f;
        }
        // Penalize high-visit cells (avoid backtracking)
        uint8_t visits = getVisitCount(cellX, cellY);
        if (visits > 2) {
            float penalty = 1.0f - 0.2f * (visits - 2);
            if (penalty < 0.2f) penalty = 0.2f;
            return (i - 1) * CELL_SIZE_CM * penalty;
        }
    }
    return maxRange;  // Clear to max range
}

int OccupancyMap::getExploredCells() {
    if (!grid) return 0;
    int count = 0;
    for (int x = 0; x < MAP_WIDTH; x++) {
        for (int y = 0; y < MAP_HEIGHT; y++) {
            if (grid[IDX(x,y)] != CELL_UNKNOWN) count++;
        }
    }
    return count;
}

int OccupancyMap::getOccupiedCells() {
    int count = 0;
    if (!grid) return 0;
    for (int x = 0; x < MAP_WIDTH; x++) {
        for (int y = 0; y < MAP_HEIGHT; y++) {
            uint8_t val = grid[IDX(x,y)];
            if (val > THRESH_LIKELY_OCC && val != CELL_HAZARD && val != CELL_PIT && val != CELL_HILL) count++;
        }
    }
    return count;
}

int OccupancyMap::getHazardCells() {
    int count = 0;
    if (!grid) return 0;
    for (int x = 0; x < MAP_WIDTH; x++) {
        for (int y = 0; y < MAP_HEIGHT; y++) {
            if (grid[IDX(x,y)] == CELL_HAZARD) count++;
        }
    }
    return count;
}

int OccupancyMap::getPitCells() {
    int count = 0;
    if (!grid) return 0;
    for (int x = 0; x < MAP_WIDTH; x++) {
        for (int y = 0; y < MAP_HEIGHT; y++) {
            if (grid[IDX(x,y)] == CELL_PIT) count++;
        }
    }
    return count;
}

int OccupancyMap::getHillCells() {
    int count = 0;
    if (!grid) return 0;
    for (int x = 0; x < MAP_WIDTH; x++) {
        for (int y = 0; y < MAP_HEIGHT; y++) {
            if (grid[IDX(x,y)] == CELL_HILL) count++;
        }
    }
    return count;
}

float OccupancyMap::getExplorationPercent() {
    return (getExploredCells() * 100.0f) / (MAP_WIDTH * MAP_HEIGHT);
}

void OccupancyMap::printMap() {
    if (!grid) { Serial.println("[MAP] Grid not allocated"); return; }
    const int DS = 20;
    
    Serial.printf("\n[MAP] Top-down view (%dx%d downsampled, 10m x 10m):\n", DS, DS);
    Serial.println("    Legend: . = free, ~ = likely free, # = obstacle, ! = hazard, v = pit, ^ = hill, ? = unknown, @ = robot");
    
    Serial.print("   ");
    for (int x = 0; x < MAP_WIDTH; x += DS) {
        Serial.printf("%d", (x / DS) % 10);
    }
    Serial.println();
    
    for (int y = MAP_HEIGHT - 1; y >= 0; y -= DS) {
        Serial.printf("%2d ", (y / DS) % 100);
        
        for (int x = 0; x < MAP_WIDTH; x += DS) {
            int rx = (int)robotX;
            int ry = (int)robotY;
            if (rx >= x && rx < x + DS && ry >= y - DS + 1 && ry <= y) {
                Serial.print("@");
                continue;
            }
            
            int freeCount = 0;
            int occCount = 0;
            int hazCount = 0;
            int pitCount = 0;
            int hillCount = 0;
            int total = 0;
            
            for (int dx = 0; dx < DS && x + dx < MAP_WIDTH; dx++) {
                for (int dy = 0; dy < DS && y - dy >= 0; dy++) {
                    uint8_t val = grid[IDX(x + dx, y - dy)];
                    if (val == CELL_PIT) pitCount++;
                    else if (val == CELL_HILL) hillCount++;
                    else if (val == CELL_HAZARD) hazCount++;
                    else if (val < THRESH_LIKELY_FREE) freeCount++;
                    else if (val > THRESH_LIKELY_OCC) occCount++;
                    total++;
                }
            }
            
            if (pitCount > 0) {
                Serial.print("v");  // v = pit/drop-off
            } else if (hillCount > 0) {
                Serial.print("^");  // ^ = hill/slope
            } else if (hazCount > 0) {
                Serial.print("#");
            } else if (freeCount > total / 2) {
                Serial.print(".");
            } else {
                Serial.print("?");
            }
        }
        Serial.println();
    }
    
    Serial.printf("[MAP] Robot at grid(%d,%d) heading %.0f° | 10m x 10m\n", 
                  (int)robotX, (int)robotY, robotHeading);
    Serial.printf("[MAP] World pos: (%.1f, %.1f) m from start | Distance: %.1f m\n",
                  getWorldX() / 100.0f, getWorldY() / 100.0f, totalDistanceCm / 100.0f);
    Serial.printf("[MAP] Explored: %.1f%% (%d cells), Obstacles: %d, Hazards: %d, Pits: %d, Hills: %d\n",
                  getExplorationPercent(), getExploredCells(), getOccupiedCells(), getHazardCells(), getPitCells(), getHillCells());
}

