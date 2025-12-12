class DecisionMaker:
    def __init__(self):
        # tuning parameters
        self.obstacle_stop_threshold = 0.35    # normalized distance from bottom of frame
        self.boundary_min_lines = 1            # if one side missing -> drift adjustment

    def decide(self, perception):
        obstacles = perception["obstacles"]
        boundary = perception["boundary"]
        terrain = perception["terrain"]

        # -------- 1. DIP DETECTION (highest priority) -------- #
        if terrain["dip_detected"]:
            return {
                "command": "STOP_REVERSE",
                "reason": "Dip detected"
            }

        # -------- 2. OBSTACLE DETECTION -------- #
        for obj in obstacles:
            ymin = obj["box"][0]   # top of bounding box (TFLite gives normalized coords)
            score = obj["score"]

            # object is near bottom of frame (close)
            if ymin > self.obstacle_stop_threshold and score > 0.5:
                return {
                    "command": "TURN_LEFT",
                    "reason": f"Obstacle close: class {obj['class']}"
                }

        # -------- 3. BOUNDARY DETECTION -------- #
        left_lines = boundary["left_boundary"]
        right_lines = boundary["right_boundary"]

        if len(left_lines) < self.boundary_min_lines:
            return {
                "command": "ADJUST_LEFT",
                "reason": "Left boundary lost, drifting right"
            }
        if len(right_lines) < self.boundary_min_lines:
            return {
                "command": "ADJUST_RIGHT",
                "reason": "Right boundary lost, drifting left"
            }

        # -------- 4. TERRAIN INCLINE -------- #
        if terrain["incline_detected"]:
            return {
                "command": "SLOW",
                "reason": "Incline detected, reducing speed"
            }

        # -------- 5. DEFAULT ACTION -------- #
        return {
            "command": "FORWARD",
            "reason": "Path clear"
        }
