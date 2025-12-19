import math
import time


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

        # turn commitment state (helps avoid getting turned around in maze-like corridors)
        self._turn_lock_cmd = None
        self._turn_lock_until = 0.0

        # Recovery (stuck/escape) state machine
        self._recovery_stage = None
        self._recovery_until = 0.0
        self._recovery_dir = 1  # +1 => prefer left, -1 => prefer right (alternates)
        self._recovery_attempts = 0
        self._recovery_cooldown_until = 0.0

    def _choose_immediate(self, perception):
        """Highest priority immediate checks that should bypass hysteresis."""
        # IMU safety: tilt risk (slow/stop) + jolt/roughness (gyro spike) cooldown
        s = perception.get('settings', {}) if isinstance(perception, dict) else {}
        imu = perception.get('imu', {}) if isinstance(perception, dict) else {}
        if imu and imu.get('connected'):
            try:
                roll_slow = float(s.get('dec_imu_roll_slow_deg', 18.0))
                pitch_slow = float(s.get('dec_imu_pitch_slow_deg', 18.0))
                roll_stop = float(s.get('dec_imu_roll_stop_deg', 28.0))
                pitch_stop = float(s.get('dec_imu_pitch_stop_deg', 28.0))
                gyro_jolt = float(s.get('dec_imu_gyro_jolt_rps', 3.5))
                jolt_cooldown = float(s.get('dec_imu_jolt_cooldown_s', 1.0))
            except Exception:
                roll_slow, pitch_slow, roll_stop, pitch_stop = 18.0, 18.0, 28.0, 28.0
                gyro_jolt, jolt_cooldown = 3.5, 1.0

            # stateful: remember last detected jolt timestamp
            if not hasattr(self, '_imu_last_jolt_ts'):
                self._imu_last_jolt_ts = 0.0

            roll = imu.get('roll')
            pitch = imu.get('pitch')
            if isinstance(roll, (int, float)) and isinstance(pitch, (int, float)):
                roll_deg = abs(math.degrees(float(roll)))
                pitch_deg = abs(math.degrees(float(pitch)))
                if roll_deg >= roll_stop or pitch_deg >= pitch_stop:
                    return {"command": "STOP_REVERSE", "reason": f"IMU tilt stop (r={roll_deg:.1f} p={pitch_deg:.1f})"}
                if roll_deg >= roll_slow or pitch_deg >= pitch_slow:
                    return {"command": "SLOW", "reason": f"IMU tilt slow (r={roll_deg:.1f} p={pitch_deg:.1f})"}

            gx, gy, gz = imu.get('gx'), imu.get('gy'), imu.get('gz')
            if all(isinstance(v, (int, float)) for v in (gx, gy, gz)):
                gmag = math.sqrt(float(gx) ** 2 + float(gy) ** 2 + float(gz) ** 2)
                if gmag >= gyro_jolt:
                    self._imu_last_jolt_ts = time.time()
                    return {"command": "SLOW", "reason": f"IMU jolt (|g|={gmag:.2f})"}

            if (time.time() - float(getattr(self, '_imu_last_jolt_ts', 0.0))) <= jolt_cooldown:
                return {"command": "SLOW", "reason": "IMU jolt cooldown"}

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

        now = None
        try:
            now = float(perception.get('timestamp'))
        except Exception:
            now = time.time()
        try:
            self.obstacle_stop_threshold = float(s.get('dec_obstacle_stop_ymin', self.obstacle_stop_threshold))
            self.stuck_threshold = int(s.get('dec_stuck_frames', self.stuck_threshold))
        except Exception:
            pass

        # Speed policy
        # - `auto_speed` is a 0..1 throttle knob (mapped to PWM by MotorController)
        # - when stuck, boost to `auto_speed_stuck`
        try:
            auto_speed = float(s.get('auto_speed', 0.6))
        except Exception:
            auto_speed = 0.6
        try:
            auto_speed_slow = float(s.get('auto_speed_slow', 0.35))
        except Exception:
            auto_speed_slow = 0.35
        try:
            auto_speed_turn = float(s.get('auto_speed_turn', 1.0))
        except Exception:
            auto_speed_turn = 1.0
        try:
            auto_speed_stuck = float(s.get('auto_speed_stuck', 1.0))
        except Exception:
            auto_speed_stuck = 1.0

        # Heading awareness (uses SLAM yaw + navigator waypoint)
        # If enabled, this can override the normal boundary/terrain candidate with TURN/ADJUST/FORWARD
        # to point toward a short-horizon target.
        nav_cmd = None
        try:
            nav_enabled = bool(s.get('nav_heading_enabled', True))
        except Exception:
            nav_enabled = True
        if nav_enabled:
            try:
                pose = perception.get('slam_pose')
                nt = perception.get('nav_target')
                if isinstance(pose, dict) and isinstance(nt, dict):
                    if all(k in pose for k in ('x', 'y')) and all(k in nt for k in ('x', 'y')):
                        x0, y0 = float(pose['x']), float(pose['y'])
                        yaw = float(pose.get('yaw', 0.0))
                        tx, ty = float(nt['x']), float(nt['y'])
                        goal_ang = math.atan2(ty - y0, tx - x0)
                        # normalize angle error to [-pi, pi]
                        err = (goal_ang - yaw + math.pi) % (2 * math.pi) - math.pi

                        try:
                            turn_stop = float(s.get('nav_turn_stop_deg', 30.0))
                            turn_adjust = float(s.get('nav_turn_adjust_deg', 10.0))
                        except Exception:
                            turn_stop, turn_adjust = 30.0, 10.0
                        err_deg = abs(math.degrees(err))

                        if err_deg >= turn_stop:
                            nav_cmd = 'TURN_LEFT' if err > 0 else 'TURN_RIGHT'
                        elif err_deg >= turn_adjust:
                            nav_cmd = 'ADJUST_LEFT' if err > 0 else 'ADJUST_RIGHT'
                        else:
                            nav_cmd = 'FORWARD'

                        # add some debug fields into perception for logging/overlay
                        try:
                            perception['nav_debug'] = {
                                'goal_ang': goal_ang,
                                'yaw': yaw,
                                'err': err,
                                'err_deg': err_deg,
                            }
                        except Exception:
                            pass
            except Exception:
                nav_cmd = None

        # 1) Immediate priority checks
        immediate = self._choose_immediate(perception)
        if immediate is not None:
            # Safety overrides cancel any turn lock.
            self._turn_lock_cmd = None
            self._turn_lock_until = 0.0
            # Also cancel recovery state.
            self._recovery_stage = None
            self._recovery_until = 0.0
            self._recovery_attempts = 0
            # Emergency commands bypass hysteresis
            self.prev_command = immediate["command"]
            self.command_counter = 0
            # annotate speed hints
            cmd = immediate.get('command')
            if cmd == 'SLOW':
                immediate['speed'] = auto_speed_slow
            elif cmd in ('TURN_LEFT', 'TURN_RIGHT', 'ADJUST_LEFT', 'ADJUST_RIGHT'):
                immediate['speed'] = auto_speed_turn
            elif cmd in ('STOP_REVERSE',):
                immediate['speed'] = auto_speed_stuck
            else:
                immediate['speed'] = auto_speed
            return immediate

        # Recovery state machine (highest priority after immediate safety)
        # Uses staged actions to avoid repeating the same reverse loop.
        try:
            rec_enabled = bool(s.get('rec_enabled', True))
        except Exception:
            rec_enabled = True
        try:
            rec_reverse_s = float(s.get('rec_reverse_s', 0.55))
            rec_turn_s = float(s.get('rec_turn_s', 0.65))
            rec_forward_s = float(s.get('rec_forward_s', 0.70))
            rec_cooldown_s = float(s.get('rec_cooldown_s', 1.0))
            rec_max_attempts = int(s.get('rec_max_attempts', 4))
        except Exception:
            rec_reverse_s, rec_turn_s, rec_forward_s, rec_cooldown_s, rec_max_attempts = 0.55, 0.65, 0.70, 1.0, 4

        if now < float(self._recovery_cooldown_until):
            # during cooldown, avoid re-entering recovery repeatedly
            pass
        elif rec_enabled and self._recovery_stage in ('reverse', 'turn', 'forward') and now < float(self._recovery_until):
            if self._recovery_stage == 'reverse':
                return {'command': 'STOP_REVERSE', 'reason': 'Recovery: reverse', 'speed': auto_speed_stuck, 'recovery': True}
            if self._recovery_stage == 'turn':
                cmd = 'TURN_LEFT' if self._recovery_dir > 0 else 'TURN_RIGHT'
                return {'command': cmd, 'reason': 'Recovery: pivot', 'speed': auto_speed_turn, 'recovery': True}
            if self._recovery_stage == 'forward':
                return {'command': 'SLOW', 'reason': 'Recovery: forward', 'speed': auto_speed_slow, 'recovery': True}

        # If recovery stage elapsed, advance it (or end).
        if self._recovery_stage in ('reverse', 'turn', 'forward') and now >= float(self._recovery_until):
            if self._recovery_stage == 'reverse':
                self._recovery_stage = 'turn'
                self._recovery_until = now + max(0.05, rec_turn_s)
                cmd = 'TURN_LEFT' if self._recovery_dir > 0 else 'TURN_RIGHT'
                return {'command': cmd, 'reason': 'Recovery: pivot', 'speed': auto_speed_turn, 'recovery': True}
            if self._recovery_stage == 'turn':
                self._recovery_stage = 'forward'
                self._recovery_until = now + max(0.05, rec_forward_s)
                return {'command': 'SLOW', 'reason': 'Recovery: forward', 'speed': auto_speed_slow, 'recovery': True}
            if self._recovery_stage == 'forward':
                # end recovery and set cooldown
                self._recovery_stage = None
                self._recovery_until = 0.0
                self._recovery_cooldown_until = now + max(0.0, rec_cooldown_s)
                return {'command': 'SLOW', 'reason': 'Recovery: done', 'speed': auto_speed_slow, 'recovery': False}

        # If we're currently committed to a turn, keep turning until timer expires.
        try:
            turn_commit_s = float(s.get('nav_turn_commit_s', 0.55))
        except Exception:
            turn_commit_s = 0.55
        if turn_commit_s < 0.0:
            turn_commit_s = 0.0
        if self._turn_lock_cmd in ('TURN_LEFT', 'TURN_RIGHT') and now < float(self._turn_lock_until):
            cmd = self._turn_lock_cmd
            return {
                'command': cmd,
                'reason': 'Turn commit',
                'speed': auto_speed_turn,
                'nav_turn_commit_remaining_s': max(0.0, float(self._turn_lock_until) - now),
            }

        # 2) Navigation/heading-aware command (when available)
        # Use this as the candidate *before* boundary heuristics.
        if nav_cmd is not None:
            candidate = nav_cmd
        else:
            # 3) Boundary-based corrections
            boundary_decision = self._choose_by_boundary(perception)
            if boundary_decision is not None:
                candidate = boundary_decision["command"]
            else:
                # 4) Terrain incline check
                terrain = perception.get("terrain", {})
                if terrain.get("incline_detected"):
                    candidate = "SLOW"
                else:
                    candidate = "FORWARD"

        # Apply hysteresis: require the candidate to appear for N frames before switching
        # stuck detection: if we are commanding forward but not moving, attempt escape
        if self._detect_stuck(perception, self.prev_command or candidate):
            # Start staged recovery unless disabled.
            self.prev_command = 'STOP_REVERSE'
            self.command_counter = 0
            if rec_enabled and self._recovery_attempts < max(1, rec_max_attempts) and now >= float(self._recovery_cooldown_until):
                self._recovery_attempts += 1
                # alternate direction each attempt to avoid repeated bad turn
                self._recovery_dir *= -1
                self._recovery_stage = 'reverse'
                self._recovery_until = now + max(0.05, rec_reverse_s)
                return {"command": "STOP_REVERSE", "reason": "Stuck detected - recovery", "stuck": True, "speed": auto_speed_stuck, 'recovery': True, 'recovery_attempt': self._recovery_attempts}
            return {"command": "STOP_REVERSE", "reason": "Stuck detected - escape", "stuck": True, "speed": auto_speed_stuck}

        if candidate == self.prev_command:
            self.command_counter = 0
            speed = auto_speed
            if candidate == 'SLOW':
                speed = auto_speed_slow
            return {"command": candidate, "reason": "Continuation", "speed": speed}
        else:
            self.command_counter += 1
            if self.command_counter >= self.hysteresis_frames:
                self.prev_command = candidate
                self.command_counter = 0
                speed = auto_speed
                if candidate == 'SLOW':
                    speed = auto_speed_slow
                elif candidate in ('TURN_LEFT', 'TURN_RIGHT', 'ADJUST_LEFT', 'ADJUST_RIGHT'):
                    speed = auto_speed_turn

                # Start turn commitment when we switch into a turn.
                if candidate in ('TURN_LEFT', 'TURN_RIGHT') and turn_commit_s > 0.0:
                    self._turn_lock_cmd = candidate
                    self._turn_lock_until = now + turn_commit_s
                return {"command": candidate, "reason": "Hysteresis switch", "speed": speed}
            else:
                # keep previous command while waiting for confirmation
                hold_cmd = self.prev_command or "FORWARD"
                speed = auto_speed
                if hold_cmd == 'SLOW':
                    speed = auto_speed_slow
                elif hold_cmd in ('TURN_LEFT', 'TURN_RIGHT', 'ADJUST_LEFT', 'ADJUST_RIGHT'):
                    speed = auto_speed_turn
                return {"command": hold_cmd, "reason": "Hysteresis hold", "speed": speed}

    def map_to_motor(self, command, speed=None):
        """Map decision command to motor control action names.

        This returns a small tuple (action, params) for use by motor_control.
        """
        sp = 0.6 if speed is None else speed
        if command == "FORWARD":
            return ("forward", {"speed": sp})
        if command == "SLOW":
            return ("forward", {"speed": sp})
        if command == "TURN_LEFT":
            return ("turn_left", {"speed": sp})
        if command == "TURN_RIGHT":
            return ("turn_right", {"speed": sp})
        if command == "STOP_REVERSE":
            # speed is handled inside motor logic via duty mapping; use longer reverse when stuck
            dur = 0.6 if (speed is not None and speed >= 0.95) else 0.5
            return ("stop_and_reverse", {"duration": dur})
        if command == "ADJUST_LEFT":
            return ("adjust_left", {"speed": sp})
        if command == "ADJUST_RIGHT":
            return ("adjust_right", {"speed": sp})
        return ("stop", {})
