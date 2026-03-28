import cv2
import time
import os
import csv
import numpy as np
from glob import glob
from datetime import datetime
import heapq

# Grid setup
GRID_W = 100
GRID_H = 100
grid = np.zeros((GRID_H, GRID_W), dtype=np.uint8)

# Fixed goal position (top-center)
start_pos = (GRID_H - 1, GRID_W // 2)
goal_pos = (0, GRID_W // 2)

# CSV logger
log_file = open("log.csv", "w", newline='')
log_writer = csv.writer(log_file)
log_writer.writerow(["Timestamp", "Edges", "Lines", "Path Length"])

def get_next_debug_filename():
    files = sorted(glob("debug_*.jpg"))
    if not files:
        return "debug_001.jpg"
    last_num = int(files[-1].split("_")[-1].split(".")[0])
    return f"debug_{last_num + 1:03}.jpg"

def capture_image():
    os.system(
        "libcamera-jpeg -o frame.jpg --mode 1640:1232 "
        "--width 1280 --height 960 --shutter 10000 --gain 8 "
        "--brightness 0.1 --timeout 1 --quality 90"
    )

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid, start, goal):
    neighbors = [(-1,0), (1,0), (0,-1), (0,1)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))

    while oheap:
        _, current = heapq.heappop(oheap)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path
        close_set.add(current)
        for i, j in neighbors:
            neighbor = (current[0]+i, current[1]+j)
            tentative_g_score = gscore[current] + 1
            if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]:
                if grid[neighbor[0]][neighbor[1]] == 1:
                    continue
            else:
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if tentative_g_score < gscore.get(neighbor, float('inf')) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return []

def process_image():
    img = cv2.imread("frame.jpg")
    if img is None:
        print("❌ Failed to load image")
        return 0, 0, 0

    height, width = img.shape[:2]

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 75, 150)
    edge_count = np.count_nonzero(edges)

    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength=50, maxLineGap=10)
    line_count = len(lines) if lines is not None else 0

    # Mark obstacles in SLAM grid
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            gx = int(GRID_W * x1 / width)
            gy = int(GRID_H * (height - y1) / height)
            gx = min(max(gx, 0), GRID_W - 1)
            gy = min(max(gy, 0), GRID_H - 1)
            grid[gy, gx] = 1  # mark obstacle

    # Compute path with A*
    path = astar(grid, start_pos, goal_pos)
    path_len = len(path)

    # Create color map image
    map_img = np.zeros((GRID_H, GRID_W, 3), dtype=np.uint8)
    map_img[grid == 1] = (0, 255, 0)        # obstacles green
    map_img[start_pos] = (255, 255, 255)    # robot white
    map_img[goal_pos] = (0, 0, 255)         # goal red
    for (r, c) in path:
        map_img[r, c] = (255, 0, 0)         # path blue

    # Save files
    debug_filename = get_next_debug_filename()
    cv2.putText(img, f"Path len: {path_len}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 0, 0), 2)
    cv2.imwrite(debug_filename, img)
    print(f"🖼️ Saved {debug_filename}")
    cv2.imwrite("map.png", cv2.resize(map_img, (500, 500), interpolation=cv2.INTER_NEAREST))

    # Log info
    log_writer.writerow([datetime.now().isoformat(), edge_count, line_count, path_len])
    log_file.flush()
    return edge_count, line_count, path_len

# === Main Loop ===
try:
    while True:
        capture_image()
        process_image()
        time.sleep(2)
except KeyboardInterrupt:
    print("🛑 Stopped")
    log_file.close()
