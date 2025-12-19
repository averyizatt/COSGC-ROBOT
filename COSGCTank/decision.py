class DecisionMaker:
    def __init__(self, obstacle_stop_threshold=0.35, boundary_min_lines=1, hysteresis_frames=3):
        # tuning parameters
        self.obstacle_stop_threshold = float(obstacle_stop_threshold)
        self.boundary_min_lines = int(boundary_min_lines)
        self.hysteresis_frames = int(hysteresis_frames)

        # internal state for smoothing
        self.prev_command = None
        self.command_counter = 0
        # stuck detection state
        self.prev_pose = None
        self.no_progress_count = 0
        self.stuck_threshold = 3  # number of consecutive frames with no progress

    def _choose_immediate(self, perception):
        """Highest priority immediate checks that should bypass hysteresis."""
        terrain = perception.get("terrain", {})
        # if a deep dip detected, do immediate stop+reverse
        if terrain.get("dip_detected"):
            return {"command": "STOP_REVERSE", "reason": "Dip detected"}

        # terrain roughness: if very rough, slow and prefer careful steering
        roughness = terrain.get('variance', 0.0)
        if roughness > 120.0:
            return {"command": "SLOW", "reason": f"High roughness ({roughness:.1f})"}

        # Ultrasonic distance check (safety)
        distance = perception.get("distance_cm")
        if distance is not None:
            # If extremely close, emergency stop and reverse
            if distance < 12.0:
                return {"command": "STOP_REVERSE", "reason": f"Ultrasonic too close ({distance:.1f}cm)"}
            # if within a near range prefer turning rather than driving forward
            if distance < 30.0:
                # choose turn direction based on boundary suggestion if present
                b = perception.get('boundary', {})
                left = len(b.get('left_boundary', []))
                right = len(b.get('right_boundary', []))
                dir_cmd = 'TURN_RIGHT' if left >= right else 'TURN_LEFT'
                return {"command": dir_cmd, "reason": f"Obstacle by distance ({distance:.1f}cm)"}

        # Obstacle close - immediate steering or stop depending on vertical position
        obstacles = perception.get("obstacles", [])
        for obj in obstacles:
            ymin = obj["box"][0]
            score = obj.get("score", 0.0)
            cls = obj.get('class')
            # treat 'contour' detections with less priority but still react
            label = obj.get('label','')
            if score > 0.5 and ymin > self.obstacle_stop_threshold:
                # if dynamic detection (moving) prioritize avoidance
                dyn = perception.get('dynamic_detections', [])
                if dyn:
                    return {"command": "STOP_REVERSE", "reason": f"Moving obstacle close: {label}"}
                # prefer turning away from the object's horizontal center using box center
                center_x = (obj['box'][1] + obj['box'][3]) / 2.0
                if center_x < 0.5:
                    return {"command": "TURN_RIGHT", "reason": f"Obstacle left-close: {label}"}
                else:
                    return {"command": "TURN_LEFT", "reason": f"Obstacle right-close: {label}"}

        return None

    def _detect_stuck(self, perception, command):
        # Use SLAM pose to detect lack of forward progress when commanded to move
        pose = perception.get('slam_pose')
        if pose is None:
            # can't determine; reset counters
            self.prev_pose = None
            self.no_progress_count = 0
            return False
        x, y = None, None
        if isinstance(pose, dict) and 'x' in pose and 'y' in pose:
            x = float(pose['x']); y = float(pose['y'])
        elif isinstance(pose, (list, tuple)) and len(pose) >= 2:
            x = float(pose[0]); y = float(pose[1])
        else:
            return False

        if self.prev_pose is None:
            self.prev_pose = (x, y)
            self.no_progress_count = 0
            return False

        dx = ((x - self.prev_pose[0])**2 + (y - self.prev_pose[1])**2)**0.5
        self.prev_pose = (x, y)
        # if commanded forward but very small movement, increment
        if command in ('FORWARD', 'SLOW') and dx < 0.02:
            self.no_progress_count += 1
            if self.no_progress_count >= self.stuck_threshold:
                self.no_progress_count = 0
                return True
        else:
            self.no_progress_count = 0
        return False

    def _choose_by_boundary(self, perception):
        boundary = perception.get("boundary", {})
        left = boundary.get("left_boundary", [])
        right = boundary.get("right_boundary", [])

        if len(left) < self.boundary_min_lines and len(right) < self.boundary_min_lines:
            return {"command": "SLOW", "reason": "Both boundaries weak"}
        if len(left) < self.boundary_min_lines:
            return {"command": "ADJUST_LEFT", "reason": "Left boundary lost, drifting right"}
        if len(right) < self.boundary_min_lines:
            return {"command": "ADJUST_RIGHT", "reason": "Right boundary lost, drifting left"}

        return None

    def decide(self, perception):
        """Return a decision dict with smoothing/hysteresis.

        Keeps interface: returns {'command': str, 'reason': str}
        """
        # Apply live settings if present
        s = perception.get('settings', {})
        try:
            self.obstacle_stop_threshold = float(s.get('dec_obstacle_stop_ymin', self.obstacle_stop_threshold))
            self.stuck_threshold = int(s.get('dec_stuck_frames', self.stuck_threshold))
        except Exception:
            pass

        # 1) Immediate priority checks
        immediate = self._choose_immediate(perception)
        if immediate is not None:
            # Emergency commands bypass hysteresis
            self.prev_command = immediate["command"]
            self.command_counter = 0
            return immediate

        # 2) Boundary-based corrections
        boundary_decision = self._choose_by_boundary(perception)
        if boundary_decision is not None:
            candidate = boundary_decision["command"]
        else:
            # 3) Terrain incline check
            terrain = perception.get("terrain", {})
            if terrain.get("incline_detected"):
                candidate = "SLOW"
            else:
                candidate = "FORWARD"

        # Apply hysteresis: require the candidate to appear for N frames before switching
        # stuck detection: if we are commanding forward but not moving, attempt escape
        if self._detect_stuck(perception, self.prev_command or candidate):
            # try a small reverse and turn
            self.prev_command = 'STOP_REVERSE'
            self.command_counter = 0
            return {"command": "STOP_REVERSE", "reason": "Stuck detected - escape", "stuck": True}

        if candidate == self.prev_command:
            self.command_counter = 0
            return {"command": candidate, "reason": "Continuation"}
        else:
            self.command_counter += 1
            if self.command_counter >= self.hysteresis_frames:
                self.prev_command = candidate
                self.command_counter = 0
                return {"command": candidate, "reason": "Hysteresis switch"}
            else:
                # keep previous command while waiting for confirmation
                return {"command": self.prev_command or "FORWARD", "reason": "Hysteresis hold"}

    def map_to_motor(self, command):
        """Map decision command to motor control action names.

        This returns a small tuple (action, params) for use by motor_control.
        """
        if command == "FORWARD":
            return ("forward", {"speed": 0.6})
        if command == "SLOW":
            return ("forward", {"speed": 0.35})
        if command == "TURN_LEFT":
            return ("turn_left", {"speed": 0.5})
        if command == "TURN_RIGHT":
            return ("turn_right", {"speed": 0.5})
        if command == "STOP_REVERSE":
            return ("stop_and_reverse", {"duration": 0.5})
        if command == "ADJUST_LEFT":
            return ("adjust_left", {"speed": 0.45})
        if command == "ADJUST_RIGHT":
            return ("adjust_right", {"speed": 0.45})
        return ("stop", {})
