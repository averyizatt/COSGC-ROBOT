class DecisionMaker:
    def __init__(self, obstacle_stop_threshold=0.35, boundary_min_lines=1, hysteresis_frames=3):
        # tuning parameters
        self.obstacle_stop_threshold = float(obstacle_stop_threshold)
        self.boundary_min_lines = int(boundary_min_lines)
        self.hysteresis_frames = int(hysteresis_frames)

        # internal state for smoothing
        self.prev_command = None
        self.command_counter = 0

    def _choose_immediate(self, perception):
        """Highest priority immediate checks that should bypass hysteresis."""
        terrain = perception.get("terrain", {})
        if terrain.get("dip_detected"):
            return {"command": "STOP_REVERSE", "reason": "Dip detected"}

        # Ultrasonic distance check (safety)
        distance = perception.get("distance_cm")
        if distance is not None:
            # If extremely close, emergency stop and reverse
            if distance < 12.0:
                return {"command": "STOP_REVERSE", "reason": f"Ultrasonic too close ({distance:.1f}cm)"}
            # if within a near range prefer turning rather than driving forward
            if distance < 30.0:
                return {"command": "TURN_RIGHT", "reason": f"Obstacle detected by distance ({distance:.1f}cm)"}

        # Obstacle close - immediate steering or stop depending on vertical position
        obstacles = perception.get("obstacles", [])
        for obj in obstacles:
            ymin = obj["box"][0]
            score = obj.get("score", 0.0)
            if score > 0.5 and ymin > self.obstacle_stop_threshold:
                # prefer turning away from the object's horizontal center
                return {"command": "TURN_LEFT", "reason": f"Obstacle close: class {obj.get('class')}"}

        return None

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
