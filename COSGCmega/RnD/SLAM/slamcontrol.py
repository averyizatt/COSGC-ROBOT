import cv2
import numpy as np
import os
import csv
import time
import RPi.GPIO as GPIO
import serial
from datetime import datetime
import heapq

# === GPIO SETUP ===
TRIG = 23
ECHO = 24
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# === Arduino Serial ===
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)

# === Grid Setup ===
GRID_W, GRID_H = 100, 100
grid = np.zeros((GRID_H, GRID_W), dtype=np.uint8)
robot_pos = (GRID_H - 1, GRID_W // 2)
goal_pos = (0, GRID_W // 2)

# === Logger ===
log_file = open("log.csv", "w", newline='')
log_writer = csv.writer(log_file)
log_writer.writerow(["Time", "Edges", "Lines", "PathLen", "Distance", "Command"])

# === Distance ===
def get_distance_cm():
    GPIO.output(TRIG, False)
    time.sleep(0.05)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(ECHO) == 0:
        start_time = time.time()
    while GPIO.input(ECHO) == 1:
        stop_time = time.time()

    duration = stop_time - start_time
    return round((duration * 34300) / 2, 1)

# === Pathfinding ===
def heuristic(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def astar(grid, start, goal):
    neighbors = [(-1,0),(1,0),(0,-1),(0,1)]
    close_set = set()
    came_from = {}
    g = {start: 0}
    f = {start: heuristic(start, goal)}
    oheap = [(f[start], start)]

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
        for dx, dy in neighbors:
            neighbor = (current[0]+dx, current[1]+dy)
            if 0 <= neighbor[0] < GRID_H and 0 <= neighbor[1] < GRID_W:
                if grid[neighbor[0]][neighbor[1]] == 1:
                    continue
                tentative_g = g[current] + 1
                if neighbor in close_set and tentative_g >= g.get(neighbor, 0):
                    continue
                if tentative_g < g.get(neighbor, float('inf')) or neighbor not in [n[1] for n in oheap]:
                    came_from[neighbor] = current
                    g[neighbor] = tentative_g
                    f[neighbor] = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(oheap, (f[neighbor], neighbor))
    return []

# === Movement Command Conversion ===
def get_direction(prev, curr):
    dy = curr[1] - prev[1]
    dx = prev[0] - curr[0]
    if dx == 1 and dy == 0:
        return "F"
    elif dx == -1 and dy == 0:
        return "B"
    elif dx == 0 and dy == 1:
        return "R"
    elif dx == 0 and dy == -1:
        return "L"
    return "S"

# === MAIN LOOP ===
try:
    while True:
        os.system("libcamera-jpeg -o frame.jpg --mode 1640:1232 --width 1280 --height 960 "
                  "--shutter 6000 --gain 2 --brightness 0.2 --timeout 1 --quality 90")
        img = cv2.imread("frame.jpg")
        if img is None:
            print("❌ Failed to capture image.")
            continue
        height, width = img.shape[:2]

        blurred = cv2.GaussianBlur(img, (5, 5), 0)
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 100)
        cv2.imwrite("edges.jpg", edges)
        edge_count = np.count_nonzero(edges)

        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=50, maxLineGap=10)
        line_count = len(lines) if lines is not None else 0
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                gx = int(GRID_W * x1 / width)
                gy = int(GRID_H * (height - y1) / height)
                gx = min(max(gx, 0), GRID_W - 1)
                gy = min(max(gy, 0), GRID_H - 1)
                grid[gy, gx] = 1
                cv2.line(img, (x1, y1), (x2, y2), (255, 255, 0), 2)

        distance = get_distance_cm()
        print(f"📏 Distance: {distance} cm")
        if 0 < distance < 30:
            fx, fy = robot_pos[0] - 1, robot_pos[1]
            if 0 <= fx < GRID_H:
                grid[fx, fy] = 1

        path = astar(grid, robot_pos, goal_pos)
        path_len = len(path)

        map_img = np.zeros((GRID_H, GRID_W, 3), dtype=np.uint8)
        map_img[grid == 1] = (0, 255, 0)
        map_img[robot_pos] = (255, 255, 255)
        map_img[goal_pos] = (0, 0, 255)
        for (r, c) in path:
            map_img[r, c] = (255, 0, 0)

        cv2.imwrite("debug.jpg", img)
        cv2.imwrite("map.png", cv2.resize(map_img, (500, 500), interpolation=cv2.INTER_NEAREST))

        # Get and send movement command
        if len(path) > 1:
            move_cmd = get_direction(path[0], path[1])
        else:
            move_cmd = "S"
        print(f"🚀 Command: {move_cmd}")
        ser.write((move_cmd + "\n").encode())
        time.sleep(0.2)  # short delay before sending center
        ser.write(b"C\n")  # center steering after each move

        # Log
        log_writer.writerow([datetime.now().isoformat(), edge_count, line_count, path_len, distance, move_cmd])
        log_file.flush()
        time.sleep(2)

except KeyboardInterrupt:
    print("🛑 Ending...")
    ser.close()
    GPIO.cleanup()
    log_file.close()
