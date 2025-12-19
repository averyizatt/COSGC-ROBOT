"""Navigator: builds a coarse occupancy grid from observations and runs A*.

This navigator is intentionally simple: it maintains a local occupancy grid
centered on the robot's latest SLAM pose, marks cells occupied when obstacles
are observed (using an estimated distance), and runs A* to a goal in world
coordinates. It exposes `set_goal(x,y)` and `update(perception)` which returns
`next_target` as a (x,y) waypoint in world coordinates. It also exposes a
convenience `get_navigation_command(current_pose, target)` to convert target
into steering advice.
"""

import math
import numpy as np
import heapq

# Allow this module to work both as a package import and as a script import.
try:
    from . import dynamic  # type: ignore
except Exception:
    import dynamic  # type: ignore


class Navigator:
    def __init__(self, grid_size=20, resolution=0.2):
        """grid_size: number of cells across (square grid), resolution: meters/cell"""
        self.grid_size = int(grid_size)
        self.resolution = float(resolution)
        self.half = self.grid_size // 2
        # occupancy: 0 free, 1 occupied
        self.occupancy = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        self.trajectory = []  # list of (x,y) poses
        self.goal = None
        # dynamic obstacle tracker
        self.dynamic = dynamic.Manager(max_age=1.2, match_dist=0.8)

        # Global map (world-frame) occupancy for maze-like planning.
        # This is a coarse grid that accumulates obstacles over time.
        self.global_enabled = True
        self.global_size = 200  # cells across
        self.global_res = self.resolution  # meters per cell
        self.global_half = self.global_size // 2
        self.global_occupancy = np.zeros((self.global_size, self.global_size), dtype=np.uint8)
        # World coordinate that maps to the center cell of global map (origin anchor)
        self.global_origin = None  # (x0, y0)

    def _global_world_to_cell(self, x, y):
        if self.global_origin is None:
            return None
        dx = x - self.global_origin[0]
        dy = y - self.global_origin[1]
        cx = int(round(dx / self.global_res)) + self.global_half
        cy = int(round(dy / self.global_res)) + self.global_half
        if 0 <= cx < self.global_size and 0 <= cy < self.global_size:
            return (cx, cy)
        return None

    def _global_cell_to_world(self, cx, cy):
        if self.global_origin is None:
            return None
        dx = (cx - self.global_half) * self.global_res
        dy = (cy - self.global_half) * self.global_res
        return (self.global_origin[0] + dx, self.global_origin[1] + dy)

    def _global_mark_occupied(self, x, y, radius_cells=1):
        c = self._global_world_to_cell(x, y)
        if c is None:
            return
        cx, cy = c
        r = int(max(0, radius_cells))
        x0 = max(0, cx - r)
        x1 = min(self.global_size - 1, cx + r)
        y0 = max(0, cy - r)
        y1 = min(self.global_size - 1, cy + r)
        self.global_occupancy[y0:y1+1, x0:x1+1] = 1

    def world_to_cell(self, x, y, origin):
        """Convert world coords to grid cell indices. origin is the center world (x0,y0)."""
        dx = x - origin[0]
        dy = y - origin[1]
        cx = int(round(dx / self.resolution)) + self.half
        cy = int(round(dy / self.resolution)) + self.half
        return cx, cy

    def cell_to_world(self, cx, cy, origin):
        dx = (cx - self.half) * self.resolution
        dy = (cy - self.half) * self.resolution
        return origin[0] + dx, origin[1] + dy

    def clear_grid(self):
        self.occupancy.fill(0)

    def set_goal(self, x, y):
        self.goal = (float(x), float(y))

    def add_observation(self, obs_world):
        """Mark a single obstacle at world coords (x,y)."""
        if obs_world is None:
            return
        x, y = obs_world
        # origin will be set during update; store raw obstacles as trajectory-like
        # but here we simply mark in current occupancy by caller
        return (x, y)

    def update(self, perception):
        """Update occupancy using perception and the current SLAM pose.

        perception is expected to have: 'slam_pose' (dict with x,y) and
        optionally 'obstacles' with normalized boxes. We estimate obstacle
        world positions heuristically and mark occupancy.
        """
        pose = perception.get('slam_pose')
        if pose is None:
            return None
        # extract x,y
        if isinstance(pose, dict) and 'x' in pose and 'y' in pose:
            x0, y0 = float(pose['x']), float(pose['y'])
            yaw = float(pose.get('yaw', 0.0))
        elif isinstance(pose, list) or isinstance(pose, (tuple,)):
            # assume 3-vector
            x0, y0 = float(pose[0]), float(pose[1])
            yaw = 0.0
        else:
            return None

        origin = (x0, y0)
        self.clear_grid()

        # Initialize global map anchor at first valid pose.
        if self.global_enabled and self.global_origin is None:
            self.global_origin = (x0, y0)

        # record trajectory point
        self.trajectory.append((x0, y0))
        if len(self.trajectory) > 500:
            self.trajectory.pop(0)

        # mark obstacles from perception.obstacles heuristically
        obstacles = perception.get('obstacles', [])
        for obj in obstacles:
            box = obj.get('box', [0,0,0,0])
            # box is normalized [ymin, xmin, ymax, xmax]; use ymin to estimate distance
            ymin = float(box[0])
            # map ymin in [0,1] -> distance 0.3..3.5 meters (heuristic)
            est_dist = 0.3 + (1.0 - min(max(ymin, 0.0), 1.0)) * 3.2
            # compute object angle relative to robot: use box center x coordinate
            center_x = (box[1] + box[3]) / 2.0
            angle_image = (center_x - 0.5) * (math.radians(70))  # assume 70deg hfov
            # convert to world coords
            ox = x0 + est_dist * math.cos(yaw + angle_image)
            oy = y0 + est_dist * math.sin(yaw + angle_image)
            cx, cy = self.world_to_cell(ox, oy, origin)
            if 0 <= cx < self.grid_size and 0 <= cy < self.grid_size:
                self.occupancy[cy, cx] = 1
            if self.global_enabled:
                self._global_mark_occupied(ox, oy, radius_cells=1)

        # dynamic obstacle detections (world coords) - update trackers and mark predicted positions
        dyn = perception.get('dynamic_detections')
        if dyn:
            # expect list of (x,y)
            try:
                self.dynamic.update([(float(x), float(y)) for (x, y) in dyn])
            except Exception:
                pass
        # predict tracked positions and mark them as occupied
        for (_id, px, py) in self.dynamic.get_tracks():
            cx, cy = self.world_to_cell(px, py, origin)
            if 0 <= cx < self.grid_size and 0 <= cy < self.grid_size:
                self.occupancy[cy, cx] = 1
            if self.global_enabled:
                self._global_mark_occupied(px, py, radius_cells=1)

        # also mark ultrasonic obstacle in front if provided
        d = perception.get('distance_cm')
        if d is not None:
            # place an obstacle at measured distance along robot forward
            dist_m = float(d) / 100.0
            ox = x0 + dist_m * math.cos(yaw)
            oy = y0 + dist_m * math.sin(yaw)
            cx, cy = self.world_to_cell(ox, oy, origin)
            if 0 <= cx < self.grid_size and 0 <= cy < self.grid_size:
                self.occupancy[cy, cx] = 1
            if self.global_enabled:
                self._global_mark_occupied(ox, oy, radius_cells=1)

        # Plan path if we have a goal
        if self.goal is None:
            return None
        # Choose planning grid: global when goal is set, local otherwise.
        use_global = self.global_enabled and (self.goal is not None)
        planner = (perception.get('settings', {}) or {}).get('nav_planner', 'astar')
        if use_global and self.global_origin is not None:
            start = self._global_world_to_cell(x0, y0)
            goal = self._global_world_to_cell(self.goal[0], self.goal[1])
            if start is None or goal is None:
                return None
            if str(planner).lower() == 'bfs':
                path = self._bfs_global(start, goal)
            else:
                path = self._astar_global(start, goal)
        else:
            start = self.world_to_cell(x0, y0, origin)
            goal = self.world_to_cell(self.goal[0], self.goal[1], origin)
            if str(planner).lower() == 'bfs':
                path = self._bfs(start, goal)
            else:
                path = self._astar(start, goal)
        if not path:
            return None
        # apply optional smoothing (simple moving average on the path)
        smooth = perception.get('settings', {}).get('smoothing', False)
        if smooth:
            path = self.smooth_path(path)
        # convert first path cell to world target
        next_cell = path[0]
        if use_global:
            w = self._global_cell_to_world(next_cell[0], next_cell[1])
            if w is None:
                return None
            return w
        tx, ty = self.cell_to_world(next_cell[0], next_cell[1], origin)
        return (tx, ty)

    def _astar_global(self, start, goal):
        # A* on global occupancy grid
        sx, sy = start
        gx, gy = goal
        if self.global_occupancy[sy, sx] == 1 or self.global_occupancy[gy, gx] == 1:
            return None
        closed = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: math.hypot(start[0]-goal[0], start[1]-goal[1])}
        heap = [(fscore[start], start)]
        while heap:
            _, current = heapq.heappop(heap)
            if current == goal:
                path = []
                node = current
                while node != start:
                    path.append(node)
                    node = came_from[node]
                path.reverse()
                return path
            closed.add(current)
            x, y = current
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(1,-1),(-1,1),(1,1)]:
                nb = (x+dx, y+dy)
                nx, ny = nb
                if not (0 <= nx < self.global_size and 0 <= ny < self.global_size):
                    continue
                if self.global_occupancy[ny, nx] == 1:
                    continue
                tentative = gscore[current] + math.hypot(dx, dy)
                if nb in closed and tentative >= gscore.get(nb, 1e9):
                    continue
                if tentative < gscore.get(nb, 1e9):
                    came_from[nb] = current
                    gscore[nb] = tentative
                    fscore[nb] = tentative + math.hypot(nx-goal[0], ny-goal[1])
                    heapq.heappush(heap, (fscore[nb], nb))
        return None

    def _bfs_global(self, start, goal):
        # BFS on global occupancy grid (4-neighborhood)
        sx, sy = start
        gx, gy = goal
        if self.global_occupancy[sy, sx] == 1 or self.global_occupancy[gy, gx] == 1:
            return None
        from collections import deque
        q = deque([start])
        came_from = {start: None}
        while q:
            cur = q.popleft()
            if cur == goal:
                break
            x, y = cur
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nb = (x+dx, y+dy)
                nx, ny = nb
                if not (0 <= nx < self.global_size and 0 <= ny < self.global_size):
                    continue
                if nb in came_from:
                    continue
                if self.global_occupancy[ny, nx] == 1:
                    continue
                came_from[nb] = cur
                q.append(nb)
        if goal not in came_from:
            return None
        path = []
        node = goal
        while node is not None and node != start:
            path.append(node)
            node = came_from[node]
        path.reverse()
        return path

    def smooth_path(self, path, window=3):
        """Simple smoothing: apply a sliding window average to the path cell list.

        Input: path is list of (cx,cy) cells. Returns new list of smoothed cells.
        """
        if len(path) <= 2 or window <= 1:
            return path
        coords = [(float(x), float(y)) for (x, y) in path]
        sm = []
        n = len(coords)
        for i in range(n):
            start = max(0, i - window//2)
            end = min(n, i + window//2 + 1)
            sx = sum(coords[j][0] for j in range(start, end)) / (end - start)
            sy = sum(coords[j][1] for j in range(start, end)) / (end - start)
            sm.append((int(round(sx)), int(round(sy))))
        # remove duplicates while preserving order
        out = []
        last = None
        for c in sm:
            if c != last:
                out.append(c)
            last = c
        return out

    # ---------- A* implementation ----------
    def _neighbors(self, node):
        x, y = node
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(1,-1),(-1,1),(1,1)]:
            nx, ny = x+dx, y+dy
            if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                yield (nx, ny)

    def _heuristic(self, a, b):
        return math.hypot(a[0]-b[0], a[1]-b[1])

    def _astar(self, start, goal):
        sx, sy = start
        gx, gy = goal
        if not (0 <= sx < self.grid_size and 0 <= sy < self.grid_size):
            return None
        if not (0 <= gx < self.grid_size and 0 <= gy < self.grid_size):
            return None
        closed = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: self._heuristic(start, goal)}
        heap = [(fscore[start], start)]
        while heap:
            _, current = heapq.heappop(heap)
            if current == goal:
                # reconstruct path (excluding start)
                path = []
                node = current
                while node != start:
                    path.append(node)
                    node = came_from[node]
                path.reverse()
                return path
            closed.add(current)
            for nb in self._neighbors(current):
                if self.occupancy[nb[1], nb[0]] == 1:
                    continue
                tentative = gscore[current] + self._heuristic(current, nb)
                if nb in closed and tentative >= gscore.get(nb, 1e9):
                    continue
                if tentative < gscore.get(nb, 1e9):
                    came_from[nb] = current
                    gscore[nb] = tentative
                    fscore[nb] = tentative + self._heuristic(nb, goal)
                    heapq.heappush(heap, (fscore[nb], nb))
        return None

    # ---------- BFS implementation ----------
    def _bfs(self, start, goal):
        """Breadth-first search on the occupancy grid.

        Note: BFS finds the shortest path in number of steps (not weighted distance).
        We use 4-neighborhood for more stable maze behavior.
        """
        sx, sy = start
        gx, gy = goal
        if not (0 <= sx < self.grid_size and 0 <= sy < self.grid_size):
            return None
        if not (0 <= gx < self.grid_size and 0 <= gy < self.grid_size):
            return None
        if self.occupancy[sy, sx] == 1 or self.occupancy[gy, gx] == 1:
            return None

        from collections import deque

        q = deque([start])
        came_from = {start: None}
        while q:
            cur = q.popleft()
            if cur == goal:
                break
            x, y = cur
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nb = (x + dx, y + dy)
                nx, ny = nb
                if not (0 <= nx < self.grid_size and 0 <= ny < self.grid_size):
                    continue
                if nb in came_from:
                    continue
                if self.occupancy[ny, nx] == 1:
                    continue
                came_from[nb] = cur
                q.append(nb)

        if goal not in came_from:
            return None

        # reconstruct path (excluding start)
        path = []
        node = goal
        while node is not None and node != start:
            path.append(node)
            node = came_from[node]
        path.reverse()
        return path
