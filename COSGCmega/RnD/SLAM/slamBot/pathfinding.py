import heapq

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
            if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]:
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

def get_direction(prev, curr, distance):
    dy = curr[1] - prev[1]
    dx = prev[0] - curr[0]
    if dx == 1 and dy == 0:
        return "F"
    elif dx == -1 and dy == 0:
        return "B"
    elif dx == 0 and dy == 1:
        return "Y" if distance < 10 else "R"
    elif dx == 0 and dy == -1:
        return "X" if distance < 10 else "L"
    return "S"

def build_grid_from_lines(lines, width, height, grid_shape, img_shape):
    grid = np.zeros(grid_shape, dtype=np.uint8)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            gx = int(width * x1 / img_shape[1])
            gy = int(height * (img_shape[0] - y1) / img_shape[0])
            gx = min(max(gx, 0), width - 1)
            gy = min(max(gy, 0), height - 1)
            grid[gy, gx] = 1
    return grid
