from sensors import capture_image_and_edges, get_distance_cm
from pathfinding import astar, get_direction, build_grid_from_lines
from movement import send_command, read_arduino_response
from utils import init_logger, log_entry

import numpy as np
import time
from datetime import datetime

# === Grid Setup ===
GRID_W, GRID_H = 100, 100
robot_pos = (GRID_H - 1, GRID_W // 2)
goal_pos = (0, GRID_W // 2)
grid = np.zeros((GRID_H, GRID_W), dtype=np.uint8)

# === Logging ===
log_writer, log_file = init_logger()

try:
    while True:
        img, edges, edge_count, lines, line_count = capture_image_and_edges()
        grid = build_grid_from_lines(lines, GRID_W, GRID_H, grid.shape, img.shape)

        distance = get_distance_cm()
        print(f"📏 Ultrasonic: {distance} cm")

        if 0 < distance < 30:
            fx, fy = robot_pos[0] - 1, robot_pos[1]
            if 0 <= fx < GRID_H:
                grid[fx, fy] = 1

        path = astar(grid, robot_pos, goal_pos)
        path_len = len(path)

        if len(path) > 1:
            move_cmd = get_direction(path[0], path[1], distance)
        else:
            move_cmd = "S"

        print(f"\U0001F680 Sending: {move_cmd}")
        send_command(move_cmd)
        time.sleep(0.2)
        send_command("C")

        arduino_response = read_arduino_response()

        log_entry(log_writer, datetime.now().isoformat(), edge_count, line_count, path_len, distance, move_cmd, arduino_response)
        log_file.flush()
        time.sleep(2)

except KeyboardInterrupt:
    print("\U0001F6D1 Terminated.")
    import movement
    movement.cleanup_serial()
    import sensors
    sensors.cleanup_gpio()
    log_file.close()
