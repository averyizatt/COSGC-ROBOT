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
import time
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
        # Dynamic occupancy (decays over time). Stored as float32 intensity [0..1].
        self.global_dyn = np.zeros((self.global_size, self.global_size), dtype=np.float32)
        self._global_dyn_last_decay_ts = time.time()
        # Observed mask: 0 = unknown/unobserved, 1 = observed.
        # NOTE: Occupancy is independent: a cell can be observed and occupied.
        self.global_seen = np.zeros((self.global_size, self.global_size), dtype=np.uint8)
        # World coordinate that maps to the center cell of global map (origin anchor)
        self.global_origin = None  # (x0, y0)

        # Exploration / planning runtime state
        self._explore_goal_cell = None
        self._explore_goal_ts = 0.0
        self._last_plan_stats = {}
        self._last_frontier_stats = {}

    def _decay_global_dynamic(self, settings):
        """Decay dynamic occupancy layer over time."""
        now = time.time()
        dt = max(0.0, now - float(getattr(self, '_global_dyn_last_decay_ts', now)))
        self._global_dyn_last_decay_ts = now

        try:
            half_life_s = float((settings or {}).get('nav_dyn_half_life_s', 6.0))
        except Exception:
            half_life_s = 6.0
        if half_life_s <= 0.0:
            # disabled decay; treat as persistent
            return
        # exponential decay factor
        factor = 0.5 ** (dt / half_life_s) if dt > 0 else 1.0
        if factor < 1.0:
            self.global_dyn *= float(factor)

        # prune tiny noise
        try:
            thresh = float((settings or {}).get('nav_dyn_prune_thresh', 0.05))
        except Exception:
            thresh = 0.05
        if thresh > 0:
            self.global_dyn[self.global_dyn < thresh] = 0.0

    def _global_set_dynamic(self, cx, cy, radius_cells=1, strength=1.0):
        if not self._global_in_bounds(cx, cy):
            return
        r = int(max(0, radius_cells))
        x0 = max(0, cx - r)
        x1 = min(self.global_size - 1, cx + r)
        y0 = max(0, cy - r)
        y1 = min(self.global_size - 1, cy + r)
        s = float(max(0.0, min(1.0, strength)))
        patch = self.global_dyn[y0:y1+1, x0:x1+1]
        # take max so repeated observations reinforce
        np.maximum(patch, s, out=patch)

    def _global_mark_dynamic_occupied(self, x, y, radius_cells=1, strength=1.0):
        c = self._global_world_to_cell(x, y)
        if c is None:
            return
        cx, cy = c
        self._global_mark_seen_cell(cx, cy, radius_cells=radius_cells)
        self._global_set_dynamic(cx, cy, radius_cells=radius_cells, strength=strength)

    def _global_combined_occupancy(self, settings):
        """Combine static occupancy and dynamic layer into a uint8 occupancy grid."""
        try:
            dyn_thresh = float((settings or {}).get('nav_dyn_thresh', 0.40))
        except Exception:
            dyn_thresh = 0.40
        dyn_block = (self.global_dyn >= dyn_thresh).astype(np.uint8)
        # static occupied OR dynamic occupied
        return np.maximum(self.global_occupancy, dyn_block)

    def _global_in_bounds(self, cx, cy):
        return 0 <= cx < self.global_size and 0 <= cy < self.global_size

    def _global_mark_seen_cell(self, cx, cy, radius_cells=0):
        if not self._global_in_bounds(cx, cy):
            return
        r = int(max(0, radius_cells))
        x0 = max(0, cx - r)
        x1 = min(self.global_size - 1, cx + r)
        y0 = max(0, cy - r)
        y1 = min(self.global_size - 1, cy + r)
        self.global_seen[y0:y1+1, x0:x1+1] = 1

    def _global_mark_seen_world(self, x, y, radius_cells=0):
        c = self._global_world_to_cell(x, y)
        if c is None:
            return
        self._global_mark_seen_cell(c[0], c[1], radius_cells=radius_cells)

    def _global_mark_seen_disc_around_pose(self, x0, y0, radius_m):
        """Mark cells around pose as observed.

        This is not a true sensor raycast, but it gives the exploration logic a
        notion of explored vs unknown without requiring depth.
        """
        c0 = self._global_world_to_cell(x0, y0)
        if c0 is None:
            return
        r_cells = int(max(0, round(float(radius_m) / self.global_res)))
        cx0, cy0 = c0
        x_min = max(0, cx0 - r_cells)
        x_max = min(self.global_size - 1, cx0 + r_cells)
        y_min = max(0, cy0 - r_cells)
        y_max = min(self.global_size - 1, cy0 + r_cells)
        rr2 = r_cells * r_cells
        for cy in range(y_min, y_max + 1):
            dy = cy - cy0
            for cx in range(x_min, x_max + 1):
                dx = cx - cx0
                if dx * dx + dy * dy <= rr2:
                    self.global_seen[cy, cx] = 1

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
        # If we saw an obstacle here, the area is observed as well.
        self._global_mark_seen_cell(cx, cy, radius_cells=radius_cells)
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

        settings = (perception.get('settings', {}) or {})

        # Decay dynamic layer and mark local area around robot as observed for exploration.
        if self.global_enabled:
            self._decay_global_dynamic(settings)
            try:
                observe_m = float(settings.get('nav_observe_radius_m', 1.0))
            except Exception:
                observe_m = 1.0
            self._global_mark_seen_disc_around_pose(x0, y0, observe_m)

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
                # Camera-derived obstacles are noisy -> dynamic by default.
                self._global_mark_dynamic_occupied(ox, oy, radius_cells=1, strength=1.0)

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
                # Tracked dynamics should decay.
                self._global_mark_dynamic_occupied(px, py, radius_cells=1, strength=1.0)

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
                # Ultrasonic obstacle is transient.
                self._global_mark_dynamic_occupied(ox, oy, radius_cells=1, strength=1.0)
        # Optional exploration: auto-pick a goal from the global map.
        explore_enabled = bool(settings.get('nav_explore_enabled', False))
        user_goal_present = bool(perception.get('nav_goal'))
        if self.global_enabled and explore_enabled and (not user_goal_present):
            self._maybe_set_explore_goal((x0, y0), settings)
            if self._explore_goal_cell is not None:
                g = self._global_cell_to_world(self._explore_goal_cell[0], self._explore_goal_cell[1])
                if g is not None:
                    self.goal = (float(g[0]), float(g[1]))

        # Always attach navigator-side debug payload for visibility.
        try:
            nav_dbg = perception.get('nav_debug')
            if not isinstance(nav_dbg, dict):
                nav_dbg = {}
            nav_dbg.update({
                'planner': (settings or {}).get('nav_planner', 'astar'),
                'explore_enabled': bool(explore_enabled),
                'user_goal_present': bool(user_goal_present),
                'explore_goal_cell': None if self._explore_goal_cell is None else [int(self._explore_goal_cell[0]), int(self._explore_goal_cell[1])],
                'explore_goal_world': None,
                'frontier': self._last_frontier_stats,
                'plan': self._last_plan_stats,
            })
            if self._explore_goal_cell is not None:
                gw = self._global_cell_to_world(self._explore_goal_cell[0], self._explore_goal_cell[1])
                if gw is not None:
                    nav_dbg['explore_goal_world'] = [float(gw[0]), float(gw[1])]
            perception['nav_debug'] = nav_dbg
        except Exception:
            pass

        # Plan path if we have a goal
        if self.goal is None:
            return None
        # Choose planning grid: global when goal is set, local otherwise.
        use_global = self.global_enabled and (self.goal is not None)
        planner = (settings or {}).get('nav_planner', 'astar')
        plan_max_exp = None
        plan_max_s = None
        try:
            plan_max_exp = int((settings or {}).get('nav_plan_max_expansions', 12000))
        except Exception:
            plan_max_exp = 12000
        try:
            plan_max_s = float((settings or {}).get('nav_plan_max_time_s', 0.06))
        except Exception:
            plan_max_s = 0.06
        if use_global and self.global_origin is not None:
            combined = self._global_combined_occupancy(settings)
            start = self._global_world_to_cell(x0, y0)
            goal = self._global_world_to_cell(self.goal[0], self.goal[1])
            if start is None or goal is None:
                return None
            if str(planner).lower() == 'bfs':
                path = self._bfs_global(start, goal, occ=combined, max_expansions=plan_max_exp, max_time_s=plan_max_s, allow_unknown=False)
            else:
                path = self._astar_global(start, goal, occ=combined, max_expansions=plan_max_exp, max_time_s=plan_max_s, allow_unknown=False)
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

    def _maybe_set_explore_goal(self, pose_xy, settings):
        """Update exploration goal cell when needed."""
        now = time.time()
        try:
            replan_s = float(settings.get('nav_explore_replan_s', 2.0))
        except Exception:
            replan_s = 2.0
        # Keep current explore goal for a bit to avoid thrashing.
        if self._explore_goal_cell is not None and (now - self._explore_goal_ts) < replan_s:
            return

        start = self._global_world_to_cell(float(pose_xy[0]), float(pose_xy[1]))
        if start is None:
            self._explore_goal_cell = None
            return

        goal = self._select_frontier_goal(start, settings)
        self._explore_goal_cell = goal
        self._explore_goal_ts = now

    def _select_frontier_goal(self, start_cell, settings):
        """Pick a frontier cell as an exploration goal.

        Frontier = observed free cell adjacent (4-neighborhood) to unknown.
        We search outward from start over observed-free cells until we find
        enough frontier candidates, then pick the nearest.
        """
        from collections import deque

        max_scan = None
        try:
            max_scan = int(settings.get('nav_explore_frontier_max_scan', 8000))
        except Exception:
            max_scan = 8000
        try:
            max_candidates = int(settings.get('nav_explore_frontier_candidates', 50))
        except Exception:
            max_candidates = 50

        sx, sy = start_cell
        if not self._global_in_bounds(sx, sy):
            return None

        # Only traverse observed and free space.
        if self.global_seen[sy, sx] != 1 or self.global_occupancy[sy, sx] == 1:
            return None

        q = deque([start_cell])
        visited = {start_cell}
        candidates = []
        scanned = 0
        while q and scanned < max_scan:
            cur = q.popleft()
            scanned += 1
            x, y = cur

            if self._is_frontier_cell(x, y):
                candidates.append(cur)
                if len(candidates) >= max_candidates:
                    break

            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = x + dx, y + dy
                nb = (nx, ny)
                if nb in visited:
                    continue
                if not self._global_in_bounds(nx, ny):
                    continue
                if self.global_seen[ny, nx] != 1:
                    continue
                if self.global_occupancy[ny, nx] == 1:
                    continue
                visited.add(nb)
                q.append(nb)

        if not candidates:
            self._last_frontier_stats = {'scanned': scanned, 'candidates': 0}
            return None

        # Nearest in BFS order is already approximately nearest; still compute min distance.
        best = None
        best_d2 = 1e18
        for (cx, cy) in candidates:
            d2 = (cx - sx) * (cx - sx) + (cy - sy) * (cy - sy)
            if d2 < best_d2:
                best_d2 = d2
                best = (cx, cy)
        self._last_frontier_stats = {'scanned': scanned, 'candidates': len(candidates)}
        return best

    def _is_frontier_cell(self, cx, cy):
        if not self._global_in_bounds(cx, cy):
            return False
        if self.global_seen[cy, cx] != 1:
            return False
        if self.global_occupancy[cy, cx] == 1:
            return False
        # adjacent unknown
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = cx + dx, cy + dy
            if not self._global_in_bounds(nx, ny):
                continue
            if self.global_seen[ny, nx] == 0:
                return True
        return False

    def _astar_global(self, start, goal, occ, max_expansions=12000, max_time_s=0.06, allow_unknown=False):
        # A* on global occupancy grid
        sx, sy = start
        gx, gy = goal
        if occ[sy, sx] == 1 or occ[gy, gx] == 1:
            return None
        # Do not plan through unknown space unless explicitly allowed.
        if not allow_unknown:
            if self.global_seen[sy, sx] != 1 or self.global_seen[gy, gx] != 1:
                return None
        closed = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: math.hypot(start[0]-goal[0], start[1]-goal[1])}
        heap = [(fscore[start], start)]
        t0 = time.time()
        expansions = 0
        while heap:
            expansions += 1
            if expansions > int(max_expansions):
                self._last_plan_stats = {'planner': 'astar_global', 'aborted': 'max_expansions', 'expansions': expansions}
                return None
            if max_time_s is not None and (time.time() - t0) > float(max_time_s):
                self._last_plan_stats = {'planner': 'astar_global', 'aborted': 'max_time', 'expansions': expansions}
                return None
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
                if occ[ny, nx] == 1:
                    continue
                if not allow_unknown and self.global_seen[ny, nx] != 1:
                    continue
                tentative = gscore[current] + math.hypot(dx, dy)
                if nb in closed and tentative >= gscore.get(nb, 1e9):
                    continue
                if tentative < gscore.get(nb, 1e9):
                    came_from[nb] = current
                    gscore[nb] = tentative
                    fscore[nb] = tentative + math.hypot(nx-goal[0], ny-goal[1])
                    heapq.heappush(heap, (fscore[nb], nb))
        self._last_plan_stats = {'planner': 'astar_global', 'aborted': 'no_path', 'expansions': expansions}
        return None

    def _bfs_global(self, start, goal, occ, max_expansions=12000, max_time_s=0.06, allow_unknown=False):
        # BFS on global occupancy grid (4-neighborhood)
        sx, sy = start
        gx, gy = goal
        if occ[sy, sx] == 1 or occ[gy, gx] == 1:
            return None
        if not allow_unknown:
            if self.global_seen[sy, sx] != 1 or self.global_seen[gy, gx] != 1:
                return None
        from collections import deque
        q = deque([start])
        came_from = {start: None}
        t0 = time.time()
        expansions = 0
        while q:
            expansions += 1
            if expansions > int(max_expansions):
                self._last_plan_stats = {'planner': 'bfs_global', 'aborted': 'max_expansions', 'expansions': expansions}
                return None
            if max_time_s is not None and (time.time() - t0) > float(max_time_s):
                self._last_plan_stats = {'planner': 'bfs_global', 'aborted': 'max_time', 'expansions': expansions}
                return None
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
                if occ[ny, nx] == 1:
                    continue
                if not allow_unknown and self.global_seen[ny, nx] != 1:
                    continue
                came_from[nb] = cur
                q.append(nb)
        if goal not in came_from:
            self._last_plan_stats = {'planner': 'bfs_global', 'aborted': 'no_path', 'expansions': expansions}
            return None
        path = []
        node = goal
        while node is not None and node != start:
            path.append(node)
            node = came_from[node]
        path.reverse()
        self._last_plan_stats = {'planner': 'bfs_global', 'aborted': None, 'expansions': expansions}
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
