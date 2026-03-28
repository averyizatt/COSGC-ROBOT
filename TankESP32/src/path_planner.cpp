#include "path_planner.h"
#include <math.h>

// Temporary wavefront grid - smaller working area for WROOM DRAM limits
// to save memory and compute paths in the robot's vicinity
#define WAVE_SIZE 10   // 10x10 = matches map size (5m x 5m at 50cm)
#define WAVE_OFFSET 5  // Center offset

static uint16_t waveGrid[WAVE_SIZE][WAVE_SIZE];
#define WAVE_OBSTACLE 65535
#define WAVE_UNKNOWN  65534
#define WAVE_UNVISITED 65533

PathPlanner::PathPlanner() {
    occupancyMap = nullptr;
    mode = PLANNER_IDLE;
    goalX = 0;
    goalY = 0;
    goalReached = false;
    pathLength = 0;
    currentWaypointIndex = 0;
    frontiersFound = 0;
    currentFrontierIndex = 0;
    homeX = MAP_CENTER_X;
    homeY = MAP_CENTER_Y;
    exploreStartTime = 0;
    lastFrontierRetry = 0;
}

void PathPlanner::begin(OccupancyMap* map) {
    occupancyMap = map;
    homeX = map->getRobotX();
    homeY = map->getRobotY();
    Serial.println("[PLANNER] Initialized - home position set");
}

void PathPlanner::setGoal(float worldX, float worldY) {
    // Convert world coordinates to grid
    int gx = MAP_CENTER_X + (int)(worldX / CELL_SIZE_CM);
    int gy = MAP_CENTER_Y + (int)(worldY / CELL_SIZE_CM);
    setGoalGrid(gx, gy);
}

void PathPlanner::setGoalGrid(int gridX, int gridY) {
    goalX = constrain(gridX, 0, MAP_WIDTH - 1);
    goalY = constrain(gridY, 0, MAP_HEIGHT - 1);
    goalReached = false;
    mode = PLANNER_GOTO_GOAL;
    pathLength = 0;
    
    Serial.printf("[PLANNER] Goal set: grid(%d, %d)\n", goalX, goalY);
}

void PathPlanner::returnHome() {
    goalX = homeX;
    goalY = homeY;
    goalReached = false;
    mode = PLANNER_RETURN_HOME;
    pathLength = 0;
    
    Serial.printf("[PLANNER] Returning home: grid(%d, %d)\n", homeX, homeY);
}

void PathPlanner::startExploration() {
    mode = PLANNER_EXPLORE;
    goalReached = false;
    pathLength = 0;
    frontiersFound = 0;
    exploreStartTime = millis();
    lastFrontierRetry = 0;
    
    Serial.println("[PLANNER] Starting frontier exploration");
}

void PathPlanner::stop() {
    mode = PLANNER_IDLE;
    pathLength = 0;
    Serial.println("[PLANNER] Stopped");
}

const char* PathPlanner::getModeString() {
    switch (mode) {
        case PLANNER_IDLE: return "IDLE";
        case PLANNER_EXPLORE: return "EXPLORE";
        case PLANNER_GOTO_GOAL: return "GOTO_GOAL";
        case PLANNER_RETURN_HOME: return "RETURN_HOME";
        case PLANNER_FOLLOW_PATH: return "FOLLOW_PATH";
        default: return "UNKNOWN";
    }
}

bool PathPlanner::isTraversable(int x, int y) {
    if (x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT) return false;
    CellType ct = occupancyMap->getCellType(x, y);
    // Fog of war: only navigate through cells KNOWN to be free.
    // UNKNOWN cells act as walls — the robot must physically discover an area
    // before the planner can route through it.
    return ct == CTYPE_FREE || ct == CTYPE_LIKELY_FREE;
}

// BFS flood-fill from (startX, startY) through traversable cells.
// Builds reachable[][] map — used to pre-filter frontiers so the planner
// never jumps to a disconnected region of the map.
void PathPlanner::computeReachability(int startX, int startY, bool reachable[MAP_WIDTH][MAP_HEIGHT]) {
    memset(reachable, 0, MAP_WIDTH * MAP_HEIGHT * sizeof(bool));
    if (!occupancyMap) return;

    // Clamp start to grid bounds (odometry drift can push robot off slightly)
    int sx = constrain(startX, 0, MAP_WIDTH - 1);
    int sy = constrain(startY, 0, MAP_HEIGHT - 1);

    // Ring-buffer BFS queue — grid is 10x10 = 100 cells max
    uint8_t qx[MAP_WIDTH * MAP_HEIGHT];
    uint8_t qy[MAP_WIDTH * MAP_HEIGHT];
    int head = 0, tail = 0;

    // Force start cell reachable regardless of type (robot might sit on an
    // uncertain cell due to odometry drift — don't lock ourselves out)
    reachable[sx][sy] = true;
    qx[tail] = sx; qy[tail] = sy; tail++;

    const int dx[] = {0, 1, 0, -1, 1, 1, -1, -1};
    const int dy[] = {1, 0, -1, 0, 1, -1, 1, -1};

    while (head < tail) {
        int cx = qx[head], cy = qy[head]; head++;
        for (int d = 0; d < 8; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (nx >= 0 && nx < MAP_WIDTH && ny >= 0 && ny < MAP_HEIGHT) {
                if (!reachable[nx][ny] && isTraversable(nx, ny)) {
                    reachable[nx][ny] = true;
                    qx[tail] = nx; qy[tail] = ny; tail++;
                }
            }
        }
    }
}

bool PathPlanner::isFrontier(int x, int y) {
    if (x < 1 || x >= MAP_WIDTH - 1 || y < 1 || y >= MAP_HEIGHT - 1) return false;
    
    CellType ct = occupancyMap->getCellType(x, y);
    // Must be free space (confident or likely)
    if (ct != CTYPE_FREE && ct != CTYPE_LIKELY_FREE) return false;
    
    // Check if adjacent to unknown
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            if (dx == 0 && dy == 0) continue;
            CellType nct = occupancyMap->getCellType(x + dx, y + dy);
            if (nct == CTYPE_UNKNOWN) {
                return true;
            }
        }
    }
    return false;
}

float PathPlanner::headingToWaypoint(int fromX, int fromY, int toX, int toY) {
    float dx = toX - fromX;
    float dy = toY - fromY;
    float heading = atan2(dx, dy) * 180.0f / PI;
    if (heading < 0) heading += 360;
    return heading;
}

float PathPlanner::getDistanceToGoal() {
    if (!occupancyMap) return -1;
    int rx = occupancyMap->getRobotX();
    int ry = occupancyMap->getRobotY();
    float dx = (goalX - rx) * CELL_SIZE_CM;
    float dy = (goalY - ry) * CELL_SIZE_CM;
    return sqrt(dx * dx + dy * dy);
}

bool PathPlanner::getCurrentWaypoint(int& x, int& y) {
    if (pathLength == 0 || currentWaypointIndex >= pathLength) return false;
    x = path[currentWaypointIndex].x;
    y = path[currentWaypointIndex].y;
    return true;
}

float PathPlanner::update(int robotX, int robotY, float currentHeading) {
    if (!occupancyMap || mode == PLANNER_IDLE) {
        return -999;  // No recommendation
    }
    
    // Check if current waypoint reached
    if (pathLength > 0 && currentWaypointIndex < pathLength) {
        int wpX = path[currentWaypointIndex].x;
        int wpY = path[currentWaypointIndex].y;
        int dx = robotX - wpX;
        int dy = robotY - wpY;
        float dist = sqrt(dx * dx + dy * dy);
        
        if (dist < WAYPOINT_THRESHOLD) {
            path[currentWaypointIndex].reached = true;
            currentWaypointIndex++;
            
            if (currentWaypointIndex >= pathLength) {
                // Path complete!
                if (mode == PLANNER_GOTO_GOAL || mode == PLANNER_RETURN_HOME) {
                    goalReached = true;
                    Serial.println("[PLANNER] Goal reached!");
                    mode = PLANNER_IDLE;
                    return -999;
                } else if (mode == PLANNER_EXPLORE) {
                    // Find next frontier
                    pathLength = 0;
                }
            }
        }
    }
    
    // Compute path if needed
    if (pathLength == 0) {
        if (mode == PLANNER_EXPLORE) {
            // Find frontiers and path to nearest
            frontiersFound = findFrontiers(robotX, robotY);
            if (frontiersFound > 0) {
                int bestFrontier = selectBestFrontier(robotX, robotY);
                if (bestFrontier >= 0) {
                    goalX = frontiers[bestFrontier].x;
                    goalY = frontiers[bestFrontier].y;
                    computePath(robotX, robotY, goalX, goalY);
                }
            } else {
                // No frontiers found — but don't give up if exploration is young
                // or if the map is still mostly unknown (sensors haven't populated it yet).
                // Retry periodically instead of going permanently IDLE.
                unsigned long elapsed = millis() - exploreStartTime;
                float explored = occupancyMap->getExplorationPercent();
                if (elapsed < 30000 || explored < 5.0f) {
                    // Early exploration or barely mapped — let reactive nav drive
                    // while sensors populate free cells, then retry
                    if (millis() - lastFrontierRetry > 3000) {
                        Serial.printf("[PLANNER] No frontiers yet (%.1f%% explored, %lus) — retrying...\n",
                                      explored, elapsed / 1000);
                        lastFrontierRetry = millis();
                    }
                    return -999;  // Let reactive nav handle movement
                }
                Serial.printf("[PLANNER] No more frontiers - exploration complete! (%.1f%% explored)\n", explored);
                mode = PLANNER_IDLE;
                return -999;
            }
        } else {
            // Path to specific goal
            computePath(robotX, robotY, goalX, goalY);
        }
        
        if (pathLength == 0) {
            // No path found - let reactive navigation handle it
            return -999;
        }
        
        currentWaypointIndex = 0;
    }
    
    // Return heading to current waypoint
    if (currentWaypointIndex < pathLength) {
        int wpX = path[currentWaypointIndex].x;
        int wpY = path[currentWaypointIndex].y;
        return headingToWaypoint(robotX, robotY, wpX, wpY);
    }
    
    return -999;
}

bool PathPlanner::computePath(int startX, int startY, int endX, int endY) {
    pathLength = 0;
    
    // Compute offset to center wavefront grid on goal
    int offsetX = endX - WAVE_OFFSET;
    int offsetY = endY - WAVE_OFFSET;
    
    // Check if start and end are within working area
    int localStartX = startX - offsetX;
    int localStartY = startY - offsetY;
    int localEndX = endX - offsetX;
    int localEndY = endY - offsetY;
    
    if (localStartX < 0 || localStartX >= WAVE_SIZE ||
        localStartY < 0 || localStartY >= WAVE_SIZE) {
        // Start too far from goal - move toward goal first
        // Create simple path toward goal
        float heading = headingToWaypoint(startX, startY, endX, endY);
        int stepX = (int)(sin(heading * PI / 180.0f) * 20);
        int stepY = (int)(cos(heading * PI / 180.0f) * 20);
        
        path[0].x = startX + stepX;
        path[0].y = startY + stepY;
        path[0].reached = false;
        pathLength = 1;
        return true;
    }
    
    // Initialize wavefront grid
    for (int x = 0; x < WAVE_SIZE; x++) {
        for (int y = 0; y < WAVE_SIZE; y++) {
            int mapX = x + offsetX;
            int mapY = y + offsetY;
            
            if (mapX < 0 || mapX >= MAP_WIDTH || mapY < 0 || mapY >= MAP_HEIGHT) {
                waveGrid[x][y] = WAVE_OBSTACLE;
            } else if (!isTraversable(mapX, mapY)) {
                waveGrid[x][y] = WAVE_OBSTACLE;
            } else {
                waveGrid[x][y] = WAVE_UNVISITED;
            }
        }
    }
    
    // Wavefront propagation from goal
    waveGrid[localEndX][localEndY] = 0;
    
    bool changed = true;
    uint16_t currentWave = 0;
    
    while (changed && currentWave < WAVE_UNVISITED) {
        changed = false;
        
        for (int x = 1; x < WAVE_SIZE - 1; x++) {
            for (int y = 1; y < WAVE_SIZE - 1; y++) {
                if (waveGrid[x][y] == currentWave) {
                    // Propagate to neighbors
                    const int dx[] = {0, 1, 0, -1, 1, 1, -1, -1};
                    const int dy[] = {1, 0, -1, 0, 1, -1, 1, -1};
                    
                    for (int d = 0; d < 8; d++) {
                        int nx = x + dx[d];
                        int ny = y + dy[d];
                        
                        if (waveGrid[nx][ny] == WAVE_UNVISITED) {
                            // Base cost: 10 for cardinal, 14 for diagonal
                            uint16_t cost = (d < 4) ? 10 : 14;
                            
                            // Visit-count penalty: strongly prefer unvisited cells
                            // Each prior visit adds 10 to cost (up to +100)
                            int mapNX = nx + offsetX;
                            int mapNY = ny + offsetY;
                            if (mapNX >= 0 && mapNX < MAP_WIDTH && mapNY >= 0 && mapNY < MAP_HEIGHT) {
                                uint8_t vc = occupancyMap->getVisitCount(mapNX, mapNY);
                                cost += min((uint16_t)(vc * 10), (uint16_t)100);
                            }
                            
                            waveGrid[nx][ny] = currentWave + cost;
                            changed = true;
                        }
                    }
                }
            }
        }
        currentWave += 10;
        
        // Early exit if we reached the start
        if (waveGrid[localStartX][localStartY] < WAVE_UNVISITED) {
            break;
        }
    }
    
    // Check if path exists
    if (waveGrid[localStartX][localStartY] >= WAVE_UNVISITED) {
        Serial.println("[PLANNER] No path found to goal");
        return false;
    }
    
    // Trace path from start to goal (following gradient descent)
    int currentX = localStartX;
    int currentY = localStartY;
    
    while (pathLength < MAX_PATH_LENGTH) {
        // Store waypoint (convert back to map coordinates)
        path[pathLength].x = currentX + offsetX;
        path[pathLength].y = currentY + offsetY;
        path[pathLength].reached = false;
        pathLength++;
        
        // Check if reached goal
        if (currentX == localEndX && currentY == localEndY) {
            break;
        }
        
        // Find neighbor with lowest cost
        int bestX = currentX;
        int bestY = currentY;
        uint16_t bestCost = waveGrid[currentX][currentY];
        
        const int dx[] = {0, 1, 0, -1, 1, 1, -1, -1};
        const int dy[] = {1, 0, -1, 0, 1, -1, 1, -1};
        
        for (int d = 0; d < 8; d++) {
            int nx = currentX + dx[d];
            int ny = currentY + dy[d];
            
            if (nx >= 0 && nx < WAVE_SIZE && ny >= 0 && ny < WAVE_SIZE) {
                if (waveGrid[nx][ny] < bestCost) {
                    bestCost = waveGrid[nx][ny];
                    bestX = nx;
                    bestY = ny;
                }
            }
        }
        
        if (bestX == currentX && bestY == currentY) {
            // Stuck - shouldn't happen
            break;
        }
        
        currentX = bestX;
        currentY = bestY;
    }
    
    // Simplify path
    simplifyPath();
    
    Serial.printf("[PLANNER] Path computed: %d waypoints\n", pathLength);
    return pathLength > 0;
}

void PathPlanner::simplifyPath() {
    if (pathLength < 3) return;
    
    // Remove collinear waypoints
    Waypoint simplified[MAX_PATH_LENGTH];
    int simplifiedLength = 0;
    
    simplified[simplifiedLength++] = path[0];
    
    for (int i = 1; i < pathLength - 1; i++) {
        // Check if point i is collinear with i-1 and i+1
        int dx1 = path[i].x - path[i-1].x;
        int dy1 = path[i].y - path[i-1].y;
        int dx2 = path[i+1].x - path[i].x;
        int dy2 = path[i+1].y - path[i].y;
        
        // Cross product to check collinearity
        int cross = dx1 * dy2 - dy1 * dx2;
        
        if (abs(cross) > 2) {  // Not collinear - keep this waypoint
            simplified[simplifiedLength++] = path[i];
        }
    }
    
    simplified[simplifiedLength++] = path[pathLength - 1];
    
    // Copy back
    for (int i = 0; i < simplifiedLength; i++) {
        path[i] = simplified[i];
    }
    pathLength = simplifiedLength;
}

int PathPlanner::findFrontiers(int robotX, int robotY) {
    frontiersFound = 0;

    // --- Phase 1: BFS flood-fill to find ALL cells reachable through known free space ---
    // This is the fog-of-war gate: a frontier must be reachable through explored territory.
    // Without this, the planner could jump to a disconnected region on the other side of
    // unknown cells, causing the apparent "teleport" the user observed.
    bool reachable[MAP_WIDTH][MAP_HEIGHT];
    computeReachability(robotX, robotY, reachable);

    // --- Phase 2: Scan ALL cells for frontiers that are reachable ---
    for (int x = 1; x < MAP_WIDTH - 1 && frontiersFound < MAX_FRONTIERS; x++) {
        for (int y = 1; y < MAP_HEIGHT - 1 && frontiersFound < MAX_FRONTIERS; y++) {
            if (!reachable[x][y]) continue;  // Fog-of-war gate — skip unreachable cells
            if (!isFrontier(x, y)) continue;

            // Deduplicate: skip if too close to an already-added frontier
            bool tooClose = false;
            for (int f = 0; f < frontiersFound; f++) {
                int dx = frontiers[f].x - x;
                int dy = frontiers[f].y - y;
                if (dx * dx + dy * dy < 4) {  // within 2 cells (~1m)
                    tooClose = true;
                    break;
                }
            }

            if (!tooClose) {
                frontiers[frontiersFound].x = x;
                frontiers[frontiersFound].y = y;
                frontiers[frontiersFound].reached = false;
                frontiersFound++;
            }
        }
    }

    Serial.printf("[PLANNER] Frontiers: %d reachable found\n", frontiersFound);
    return frontiersFound;
}

int PathPlanner::selectBestFrontier(int robotX, int robotY) {
    if (frontiersFound == 0) return -1;
    
    int bestIndex = 0;
    float bestScore = -99999;
    
    for (int i = 0; i < frontiersFound; i++) {
        int dx = frontiers[i].x - robotX;
        int dy = frontiers[i].y - robotY;
        float distance = sqrt(dx * dx + dy * dy);
        
        // Score: prefer closer frontiers, but not too close
        float score = 0;
        
        if (distance > 5) {  // At least 25cm away
            score = 1000 - distance;  // Prefer closer
            
            // Exploration bonus: prefer frontiers in less-visited areas
            // Sample visit count in a 5x5 area around the frontier
            int visitSum = 0;
            for (int ox = -2; ox <= 2; ox++) {
                for (int oy = -2; oy <= 2; oy++) {
                    int gx = frontiers[i].x + ox;
                    int gy = frontiers[i].y + oy;
                    if (gx >= 0 && gx < MAP_WIDTH && gy >= 0 && gy < MAP_HEIGHT) {
                        visitSum += occupancyMap->getVisitCount(gx, gy);
                    }
                }
            }
            // Less visited = higher bonus (up to +500)
            score += max(0, 500 - visitSum * 20);
        }
        
        if (score > bestScore) {
            bestScore = score;
            bestIndex = i;
        }
    }
    
    return bestIndex;
}

void PathPlanner::printPath() {
    Serial.printf("[PLANNER] Path (%d waypoints):\n", pathLength);
    for (int i = 0; i < min(pathLength, 10); i++) {
        Serial.printf("  %d: (%d, %d) %s\n", i, path[i].x, path[i].y,
                      path[i].reached ? "[DONE]" : (i == currentWaypointIndex ? "[CURRENT]" : ""));
    }
    if (pathLength > 10) {
        Serial.printf("  ... and %d more\n", pathLength - 10);
    }
}
