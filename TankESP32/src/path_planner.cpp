#include "path_planner.h"
#include <math.h>

// Temporary wavefront grid - we'll use a smaller working area
// to save memory and compute paths in the robot's vicinity
#define WAVE_SIZE 100  // 100x100 = 10KB working area (5m x 5m)
#define WAVE_OFFSET 50 // Center offset

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
    uint8_t cell = occupancyMap->getCell(x, y);
    // Traversable if free or unknown (we'll explore unknown)
    return cell < 180;  // Not definitely an obstacle
}

bool PathPlanner::isFrontier(int x, int y) {
    if (x < 1 || x >= MAP_WIDTH - 1 || y < 1 || y >= MAP_HEIGHT - 1) return false;
    
    uint8_t cell = occupancyMap->getCell(x, y);
    // Must be free space
    if (cell > 60) return false;
    
    // Check if adjacent to unknown
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            if (dx == 0 && dy == 0) continue;
            uint8_t neighbor = occupancyMap->getCell(x + dx, y + dy);
            if (neighbor >= 120 && neighbor <= 135) {  // Unknown (around 127)
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
                Serial.println("[PLANNER] No more frontiers - exploration complete!");
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
            } else {
                uint8_t cell = occupancyMap->getCell(mapX, mapY);
                if (cell > 180) {
                    waveGrid[x][y] = WAVE_OBSTACLE;
                } else {
                    waveGrid[x][y] = WAVE_UNVISITED;
                }
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
                            // Cost: 10 for cardinal, 14 for diagonal
                            uint16_t cost = (d < 4) ? 10 : 14;
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
    
    // Search in expanding squares from robot position
    for (int radius = 10; radius < 200 && frontiersFound < MAX_FRONTIERS; radius += 10) {
        int startX = max(0, robotX - radius);
        int endX = min(MAP_WIDTH - 1, robotX + radius);
        int startY = max(0, robotY - radius);
        int endY = min(MAP_HEIGHT - 1, robotY + radius);
        
        // Sample every 5 cells to speed up search
        for (int x = startX; x <= endX && frontiersFound < MAX_FRONTIERS; x += 5) {
            for (int y = startY; y <= endY && frontiersFound < MAX_FRONTIERS; y += 5) {
                if (isFrontier(x, y)) {
                    // Check if too close to existing frontier
                    bool tooClose = false;
                    for (int f = 0; f < frontiersFound; f++) {
                        int dx = frontiers[f].x - x;
                        int dy = frontiers[f].y - y;
                        if (dx * dx + dy * dy < 400) {  // 20 cells = 1m
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
        }
    }
    
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
            
            // Bonus for frontiers roughly ahead of robot
            // (would need heading info for this - skip for now)
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
