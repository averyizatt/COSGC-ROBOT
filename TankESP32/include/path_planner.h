#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <Arduino.h>
#include "occupancy_map.h"

// Planner modes
enum PlannerMode {
    PLANNER_IDLE = 0,
    PLANNER_EXPLORE,      // Frontier-based exploration
    PLANNER_GOTO_GOAL,    // Navigate to specific goal
    PLANNER_RETURN_HOME,  // Return to start position
    PLANNER_FOLLOW_PATH   // Following computed path
};

// Waypoint structure
struct Waypoint {
    int x;
    int y;
    bool reached;
};

// Maximum path length (in waypoints)
#define MAX_PATH_LENGTH 200
#define MAX_FRONTIERS 50

class PathPlanner {
public:
    PathPlanner();
    
    // Initialize with map reference
    void begin(OccupancyMap* map);
    
    // Set goal position (world coordinates in cm)
    void setGoal(float worldX, float worldY);
    
    // Command to return to start
    void returnHome();
    
    // Command to explore (frontier-based)
    void startExploration();
    
    // Stop current plan
    void stop();
    
    // Update planner - call frequently
    // Returns recommended heading to follow (degrees)
    // Returns -999 if no recommendation (use reactive navigation)
    float update(int robotX, int robotY, float currentHeading);
    
    // Get current mode
    PlannerMode getMode() { return mode; }
    const char* getModeString();
    
    // Check if goal reached
    bool isGoalReached() { return goalReached; }
    
    // Check if path exists to goal
    bool hasValidPath() { return pathLength > 0; }
    
    // Get distance to goal (cm)
    float getDistanceToGoal();
    
    // Get current target waypoint
    bool getCurrentWaypoint(int& x, int& y);
    
    // Debug
    void printPath();

private:
    OccupancyMap* occupancyMap;
    PlannerMode mode;
    
    // Internal: set goal in grid coordinates
    void setGoalGrid(int gridX, int gridY);
    
    // Goal
    int goalX, goalY;
    bool goalReached;
    
    // Path storage
    Waypoint path[MAX_PATH_LENGTH];
    int pathLength;
    int currentWaypointIndex;
    
    // Frontier exploration
    Waypoint frontiers[MAX_FRONTIERS];
    int frontiersFound;
    int currentFrontierIndex;
    
    // Wavefront buffer - reuses part of stack temporarily
    // We'll compute in chunks to save memory
    static const int WAVE_CHUNK_SIZE = 50;
    
    // Home position
    int homeX, homeY;
    
    // Exploration timing — prevent premature IDLE when map is mostly unknown
    unsigned long exploreStartTime;
    unsigned long lastFrontierRetry;
    
    // Waypoint reached threshold (grid cells)
    static const int WAYPOINT_THRESHOLD = 1;  // 50cm (1 cell at 50cm resolution)
    
    // Compute path using wavefront algorithm
    bool computePath(int startX, int startY, int endX, int endY);
    
    // Find frontiers (boundaries between known and unknown)
    int findFrontiers(int robotX, int robotY);
    
    // Select best frontier to explore
    int selectBestFrontier(int robotX, int robotY);
    
    // Check if cell is traversable (only FREE/LIKELY_FREE — fog of war enforced)
    bool isTraversable(int x, int y);
    
    // Check if cell is a frontier
    bool isFrontier(int x, int y);

    // BFS flood-fill through free cells to find all cells reachable from (startX, startY)
    void computeReachability(int startX, int startY, bool reachable[MAP_WIDTH][MAP_HEIGHT]);
    
    // Get heading to waypoint
    float headingToWaypoint(int fromX, int fromY, int toX, int toY);
    
    // Simplify path (remove unnecessary waypoints)
    void simplifyPath();
};

#endif // PATH_PLANNER_H
