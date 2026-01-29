from flask import Flask, render_template, request, jsonify, Response
import os
import cv2
import numpy as np
import re
import shutil
import subprocess
import time
import threading
import json
import traceback
from pathlib import Path
import io
import math
import requests
from collections import deque

from hardware.motor_control import MotorController
from hardware.camera import FrameProvider

TEMPLATES_DIR = Path(__file__).resolve().parent.parent / 'templates'
app = Flask(__name__, template_folder=str(TEMPLATES_DIR))


def _parse_camera_device(device):
    """Parse camera selector.

    Accepts: 0, '0', '/dev/video0'. Returns an int index when possible.
    """
    if device is None:
        return 0
    if isinstance(device, int):
        return device
    s = str(device).strip()
    if s.startswith('/dev/video'):
        s = s.replace('/dev/video', '')
    try:
        return int(s)
    except Exception:
        return 0


_video_provider = None
_video_provider_key = None


_autonomy_thread = None
_autonomy_stop = threading.Event()
_autonomy_lock = threading.Lock()
_autonomy_last_error = None
_autonomy_last_traceback = None
_autonomy_detector_init_error = None
_autonomy_detector_init_traceback = None
_autonomy_heartbeat_ts = 0.0
_autonomy_last_slam_ts = 0.0
_autonomy_restart_attempts = 0

_navigator = None
_navigator_lock = threading.Lock()

_power_monitor = None
_power_lock = threading.Lock()
_power_latest = None
_power_latest_ts = None


def _get_video_provider(device_idx, width, height, fps):
    global _video_provider, _video_provider_key
    key = (int(device_idx), int(width), int(height), int(fps))
    if _video_provider is None or _video_provider_key != key:
        try:
            if _video_provider is not None:
                _video_provider.release()
        except Exception:
            pass
        _video_provider = FrameProvider(width=int(width), height=int(height), fps=int(fps), camera_index=int(device_idx))
        _video_provider_key = key
        try:
            print(f"[video] opened /dev/video{device_idx} @ {width}x{height} {fps}fps")
        except Exception:
            pass
    return _video_provider


def _contour_obstacles(frame_rgb, settings=None):
    """Contour-only obstacle detector (no TFLite dependency).

    Returns normalized boxes compatible with DecisionMaker.
    """
    if frame_rgb is None:
        return []
    try:
        s = settings or {}
        gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
        blur = cv2.bilateralFilter(gray, d=5, sigmaColor=75, sigmaSpace=75)
        th = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY_INV, 11, 2)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        h, w = frame_rgb.shape[:2]
        dets = []
        min_area = float(s.get('det_contour_min_area', 400))
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue
            x, y, cw, ch = cv2.boundingRect(cnt)
            ymin = y / h
            xmin = x / w
            ymax = (y + ch) / h
            xmax = (x + cw) / w
            score = min(1.0, (area / (w * h)) * 10.0)
            dets.append({'box': [float(ymin), float(xmin), float(ymax), float(xmax)], 'class': 0, 'score': float(max(0.35, score)), 'label': 'contour'})
        dets.sort(key=lambda x: x.get('score', 0.0), reverse=True)
        return dets
    except Exception:
        return []


def _autonomy_loop():
    """Background autonomy loop driven by MODE.

    Runs when MODE != 'rc'. Uses the shared camera provider + SETTINGS and
    commands the server's global MotorController.
    """
    global latest_log, latest_log_ts, _autonomy_last_error, _autonomy_last_traceback
    global _autonomy_detector_init_error, _autonomy_detector_init_traceback
    global _autonomy_heartbeat_ts, _autonomy_last_slam_ts
    _autonomy_last_error = None
    _autonomy_last_traceback = None
    _autonomy_detector_init_error = None
    _autonomy_detector_init_traceback = None

    # Lazy imports: keep server start fast and allow partial installs.
    try:
        from objectDetection.boundaries import BoundaryDetector
        from objectDetection.terrain import TerrainAnalyzer
        from slam.decision import DecisionMaker
        from objectDetection.overlay import OverlayDrawer
        from slam.navigator import Navigator
    except Exception as e:
        _autonomy_last_error = f"import_failed:{type(e).__name__}:{e}"
        try:
            _autonomy_last_traceback = traceback.format_exc()
        except Exception:
            _autonomy_last_traceback = None
        return

    # Optional: full model detector (tflite). Try once; stick to contour if unavailable.
    det = None
    det_available = False
    if bool(SETTINGS.get('det_use_tflite', False)):
        try:
            from slam.detector import ObstacleDetector
            det = ObstacleDetector(settings=SETTINGS)
            det_available = True
        except Exception as e:
            _autonomy_detector_init_error = f"{type(e).__name__}:{e}"
            try:
                _autonomy_detector_init_traceback = traceback.format_exc()
            except Exception:
                _autonomy_detector_init_traceback = None
            det = None
            det_available = False

    bounds = BoundaryDetector()
    terrain = TerrainAnalyzer()
    decider = DecisionMaker()
    overlay = OverlayDrawer()

    nav = None
    try:
        nav = Navigator(grid_size=40, resolution=0.2)
    except Exception:
        nav = None
    with _navigator_lock:
        globals()['_navigator'] = nav

    # Optional ultrasonic
    us = None
    try:
        from hardware.ultrasonic import Ultrasonic
        us = Ultrasonic(trig_pin=24, echo_pin=25)
    except Exception:
        us = None

    last_motor_cmd = None
    dist_history = deque(maxlen=5)
    dist_last_ts = 0.0
    power_history = deque(maxlen=5)

    def _median_current(hist, key):
        vals = []
        for p in hist:
            try:
                v = p.get(key, {}).get('current_a')
                if isinstance(v, (int, float)):
                    vals.append(float(v))
            except Exception:
                continue
        if not vals:
            return None
        vals.sort()
        mid = len(vals) // 2
        if len(vals) % 2 == 1:
            return vals[mid]
        return (vals[mid - 1] + vals[mid]) / 2.0

    def _sleep_interruptible(total_s: float) -> bool:
        """Sleep up to total_s seconds, returning False if stop/mode/estop aborts."""
        try:
            total = float(total_s)
        except Exception:
            total = 0.0
        end = time.time() + max(0.0, total)
        while time.time() < end:
            if _autonomy_stop.is_set():
                return False
            try:
                with _keepalive_lock:
                    if MODE == 'rc' or bool(_e_stop):
                        return False
            except Exception:
                pass
            time.sleep(min(0.05, max(0.0, end - time.time())))
        return True

    while not _autonomy_stop.is_set():
        try:
            _autonomy_heartbeat_ts = time.time()
            with _keepalive_lock:
                mode_now = MODE
                estop_now = bool(_e_stop)

            if mode_now == 'rc' or estop_now:
                # Ensure we aren't moving while disabled.
                try:
                    if last_motor_cmd is not None:
                        motor.stop()
                        last_motor_cmd = None
                except Exception:
                    pass
                time.sleep(0.1)
                continue

            # Snapshot settings for consistency in this iteration.
            try:
                settings_snapshot = dict(SETTINGS)
            except Exception:
                settings_snapshot = SETTINGS

            tick_enabled = bool(settings_snapshot.get('auto_tick_enabled', True))

            dev_idx = _parse_camera_device(settings_snapshot.get('cam_device', '/dev/video0'))
            width = int(settings_snapshot.get('cam_width', 640))
            height = int(settings_snapshot.get('cam_height', 480))
            fps = int(settings_snapshot.get('cam_fps', 20))

            provider = _get_video_provider(dev_idx, width, height, fps)

            # Tick mode: stop first to reduce motion blur, then capture.
            if tick_enabled:
                try:
                    motor.stop()
                    last_motor_cmd = None
                except Exception:
                    pass
                _sleep_interruptible(float(settings_snapshot.get('auto_tick_settle_s', 0.06)))

            data = provider.get_frame()
            if data is None:
                time.sleep(0.05)
                continue
            frame = data.get('frame')
            ts = float(data.get('timestamp', time.time()))

            detector_info = {
                'kind': 'tflite' if det is not None else 'contour',
                'init_error': _autonomy_detector_init_error,
            }

            if det is not None:
                try:
                    det.settings = settings_snapshot
                except Exception:
                    pass

            try:
                if det is not None:
                    obstacles = det.detect(frame)
                else:
                    obstacles = _contour_obstacles(frame, settings_snapshot)
            except Exception as e:
                detector_info['kind'] = 'contour_fallback'
                detector_info['detect_error'] = f"{type(e).__name__}:{e}"
                obstacles = _contour_obstacles(frame, settings_snapshot)

            boundary_info = bounds.detect(frame)
            terrain_info = terrain.analyze(frame)

            dist_cm = None
            if us is not None:
                try:
                    dist_cm = us.read_distance_cm(samples=3)
                except Exception:
                    dist_cm = None

            power = None
            try:
                with _power_lock:
                    if _power_latest_ts is not None:
                        if (time.time() - float(_power_latest_ts)) <= 1.0:
                            power = _power_latest
                    else:
                        power = None
            except Exception:
                power = None

            if power is not None:
                power_history.append(power)
            else:
                power_history.clear()
            power_filtered = None
            if power_history:
                try:
                    ml = _median_current(power_history, 'motor_left')
                    mr = _median_current(power_history, 'motor_right')
                    sys_i = _median_current(power_history, 'system')
                    total_i = None
                    try:
                        vals = [v for v in (ml, mr) if isinstance(v, (int, float))]
                        total_i = sum(vals) if vals else None
                    except Exception:
                        total_i = None
                    power_filtered = {}
                    if ml is not None:
                        power_filtered['motor_left'] = {'current_a': ml}
                    if mr is not None:
                        power_filtered['motor_right'] = {'current_a': mr}
                    if sys_i is not None:
                        power_filtered['system'] = {'current_a': sys_i}
                    if total_i is not None:
                        power_filtered['total_motor_current_a'] = total_i
                except Exception:
                    power_filtered = None
            if power_filtered is not None:
                power = power_filtered

            slam_pose = None
            try:
                resp = requests.get('http://127.0.0.1:5001/pose', timeout=0.05)
                if resp.status_code == 200:
                    slam_pose = (resp.json() or {}).get('pose')
                    if isinstance(slam_pose, dict) and 'x' in slam_pose and 'y' in slam_pose:
                        _autonomy_last_slam_ts = ts
            except Exception:
                slam_pose = None

            try:
                nav_goal_copy = dict(NAV_GOAL) if isinstance(NAV_GOAL, dict) else None
            except Exception:
                nav_goal_copy = None

            if dist_cm is not None and isinstance(dist_cm, (int, float)) and 1.0 <= float(dist_cm) <= 500.0:
                dist_history.append(float(dist_cm))
                dist_last_ts = ts
            dist_filtered = None
            if dist_history and (ts - dist_last_ts) <= 1.0:
                try:
                    dist_filtered = float(np.median(np.array(dist_history, dtype=np.float32)))
                except Exception:
                    try:
                        dist_filtered = sum(dist_history) / len(dist_history)
                    except Exception:
                        dist_filtered = None

            dynamic_detections = []
            try:
                pose = slam_pose
                if isinstance(pose, dict) and 'x' in pose and 'y' in pose:
                    x0 = float(pose['x']); y0 = float(pose.get('y', 0.0))
                    yaw = float(pose.get('yaw', 0.0))
                    for obj in obstacles:
                        box = obj.get('box', [0, 0, 0, 0])
                        ymin = float(box[0])
                        est_dist = 0.3 + (1.0 - min(max(ymin, 0.0), 1.0)) * 3.2
                        center_x = (box[1] + box[3]) / 2.0
                        angle_image = (center_x - 0.5) * (math.radians(70))
                        ox = x0 + est_dist * math.cos(yaw + angle_image)
                        oy = y0 + est_dist * math.sin(yaw + angle_image)
                        dynamic_detections.append((ox, oy))
            except Exception:
                dynamic_detections = []

            perception = {
                'timestamp': ts,
                'obstacles': obstacles,
                'detector': detector_info,
                'boundary': boundary_info,
                'terrain': terrain_info,
                'distance_cm': dist_filtered,
                'power': power,
                'settings': settings_snapshot,
                'dynamic_detections': dynamic_detections,
                'slam_pose': slam_pose,
                'nav_goal': None,
                'nav_target': None,
                'nav_debug': None,
                'imu': None,
            }

            nav_target = None
            nav_goal_payload = nav_goal_copy if nav_goal_copy is not None else None
            if nav is not None and isinstance(slam_pose, dict) and 'x' in slam_pose and 'y' in slam_pose:
                try:
                    x0 = float(slam_pose['x']); y0 = float(slam_pose.get('y', 0.0))
                    yaw = float(slam_pose.get('yaw', 0.0))
                    explore_enabled = bool(settings_snapshot.get('nav_explore_enabled', False))
                    if isinstance(nav_goal_copy, dict) and 'x' in nav_goal_copy and 'y' in nav_goal_copy:
                        nav_goal_payload = {'x': float(nav_goal_copy['x']), 'y': float(nav_goal_copy['y'])}
                        with _navigator_lock:
                            nav.set_goal(nav_goal_payload['x'], nav_goal_payload['y'])
                    elif not explore_enabled:
                        nav_goal_dist_m = float(settings_snapshot.get('nav_goal_dist_m', 1.2))
                        gx = x0 + nav_goal_dist_m * math.cos(yaw)
                        gy = y0 + nav_goal_dist_m * math.sin(yaw)
                        nav_goal_payload = {'x': gx, 'y': gy}
                        with _navigator_lock:
                            nav.set_goal(gx, gy)
                    else:
                        nav_goal_payload = None
                except Exception:
                    pass

                try:
                    with _navigator_lock:
                        nav_target_raw = nav.update(perception)
                    if nav_target_raw is not None:
                        nav_target = {'x': float(nav_target_raw[0]), 'y': float(nav_target_raw[1])}
                except Exception:
                    nav_target = None
            perception['nav_goal'] = nav_goal_payload
            perception['nav_target'] = nav_target

            decision = decider.decide(perception)

            # Execute motor action
            if tick_enabled:
                cmd = decision.get('command')
                try:
                    sp = float(decision.get('speed')) if decision.get('speed') is not None else None
                except Exception:
                    sp = None
                if sp is None:
                    sp = float(settings_snapshot.get('auto_speed', 0.6))

                fwd_s = float(settings_snapshot.get('auto_tick_forward_s', 0.18))
                turn_s = float(settings_snapshot.get('auto_tick_turn_s', 0.16))
                rev_s = float(settings_snapshot.get('auto_tick_reverse_s', 0.22))
                idle_s = float(settings_snapshot.get('auto_tick_idle_s', 0.02))

                try:
                    if cmd in ('FORWARD', 'SLOW'):
                        motor.forward(speed=sp)
                        last_motor_cmd = 'forward'
                        _sleep_interruptible(fwd_s)
                    elif cmd == 'TURN_LEFT':
                        motor.turn_left(speed=sp)
                        last_motor_cmd = 'turn_left'
                        _sleep_interruptible(turn_s)
                    elif cmd == 'TURN_RIGHT':
                        motor.turn_right(speed=sp)
                        last_motor_cmd = 'turn_right'
                        _sleep_interruptible(turn_s)
                    elif cmd == 'ADJUST_LEFT':
                        # Avoid MotorController.adjust_left() because it sleeps internally.
                        motor.turn_left(speed=min(1.0, max(0.0, sp)))
                        last_motor_cmd = 'turn_left'
                        _sleep_interruptible(max(0.02, turn_s * 0.6))
                    elif cmd == 'ADJUST_RIGHT':
                        motor.turn_right(speed=min(1.0, max(0.0, sp)))
                        last_motor_cmd = 'turn_right'
                        _sleep_interruptible(max(0.02, turn_s * 0.6))
                    elif cmd == 'STOP_REVERSE':
                        # Interruptible reverse maneuver (avoid MotorController.stop_and_reverse() internal sleeps)
                        motor.stop()
                        _sleep_interruptible(0.04)
                        motor.reverse(speed=min(1.0, max(0.0, sp)))
                        last_motor_cmd = 'reverse'
                        _sleep_interruptible(rev_s)
                    else:
                        motor.stop()
                        last_motor_cmd = None
                except Exception:
                    try:
                        motor.stop()
                    except Exception:
                        pass
                    last_motor_cmd = None

                try:
                    motor.stop()
                    last_motor_cmd = None
                except Exception:
                    pass
                _sleep_interruptible(idle_s)

            else:
                action, params = decider.map_to_motor(decision.get('command'), speed=decision.get('speed'))
                try:
                    if hasattr(motor, action):
                        try:
                            getattr(motor, action)(**(params or {}))
                        except TypeError:
                            getattr(motor, action)()
                        last_motor_cmd = action
                except Exception:
                    # fail safe
                    try:
                        motor.stop()
                    except Exception:
                        pass
                    last_motor_cmd = None

            # Update UI/debug log buffer (same structure as /log expects)
            try:
                latest_log = overlay.export_log(perception, decision)
                latest_log_ts = time.time()
            except Exception:
                pass

            # Lightweight status marker
            try:
                with _keepalive_lock:
                    _active_actions.clear()
                    # Represent autonomy command with same letters used by RC display.
                    cmd = decision.get('command')
                    if cmd in ('FORWARD', 'SLOW'):
                        _active_actions.add('f')
                    elif cmd in ('TURN_LEFT', 'ADJUST_LEFT'):
                        _active_actions.add('l')
                    elif cmd in ('TURN_RIGHT', 'ADJUST_RIGHT'):
                        _active_actions.add('r')
                    elif cmd == 'STOP_REVERSE':
                        _active_actions.add('b')
            except Exception:
                pass

            # Pace loop roughly; tick mode already sleeps during actions.
            if not tick_enabled:
                time.sleep(0.01)

        except Exception as e:
            _autonomy_last_error = f"loop_error:{type(e).__name__}:{e}"
            try:
                _autonomy_last_traceback = traceback.format_exc()
            except Exception:
                _autonomy_last_traceback = None
            try:
                motor.stop()
            except Exception:
                pass
            time.sleep(0.2)

    with _navigator_lock:
        globals()['_navigator'] = None


def _start_autonomy_if_needed():
    global _autonomy_thread
    with _autonomy_lock:
        if _autonomy_thread is not None and _autonomy_thread.is_alive():
            return
        _autonomy_stop.clear()
        _autonomy_thread = threading.Thread(target=_autonomy_loop, daemon=True)
        _autonomy_thread.start()


def _stop_autonomy():
    global _autonomy_thread
    with _autonomy_lock:
        _autonomy_stop.set()
        try:
            motor.stop()
        except Exception:
            pass
        try:
            with _keepalive_lock:
                _active_actions.clear()
        except Exception:
            pass


def _camera_caps_v4l2(device: str):
    """Best-effort camera capability probe.

    Uses `v4l2-ctl --list-formats-ext` if installed (v4l-utils).
    Returns a dict:
      {
        'device': '/dev/video0',
        'sizes': [ {'w': 640, 'h': 480, 'fps': [30.0, 15.0]} , ... ],
        'formats': ['MJPG', 'YUYV']
      }
    """
    dev = str(device or '/dev/video0').strip() or '/dev/video0'
    if not dev.startswith('/dev/video'):
        # accept numeric
        try:
            dev = f"/dev/video{int(dev)}"
        except Exception:
            dev = '/dev/video0'

    exe = shutil.which('v4l2-ctl')
    if not exe:
        return {'device': dev, 'sizes': [], 'formats': [], 'source': 'v4l2-ctl-missing'}

    try:
        proc = subprocess.run(
            [exe, '-d', dev, '--list-formats-ext'],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=2.0,
            check=False,
        )
        out = proc.stdout or ''
    except Exception as e:
        return {'device': dev, 'sizes': [], 'formats': [], 'source': f'v4l2-ctl-error:{type(e).__name__}'}

    formats = []
    # Example: "Pixel Format: 'MJPG'" or "Pixel Format: 'YUYV'"
    for m in re.finditer(r"Pixel\s+Format:\s*'([A-Z0-9]{3,8})'", out):
        fmt = m.group(1)
        if fmt and fmt not in formats:
            formats.append(fmt)

    sizes_map = {}  # (w,h) -> set(fps)
    current_w = None
    current_h = None

    # Lines typically contain:
    #   Size: Discrete 640x480
    #   Interval: Discrete 0.033s (30.000 fps)
    for line in out.splitlines():
        line = line.strip()
        m = re.search(r"Size:\s*Discrete\s*(\d+)x(\d+)", line)
        if m:
            current_w = int(m.group(1))
            current_h = int(m.group(2))
            sizes_map.setdefault((current_w, current_h), set())
            continue

        m = re.search(r"\((\d+(?:\.\d+)?)\s*fps\)", line)
        if m and current_w is not None and current_h is not None:
            try:
                fps = float(m.group(1))
                sizes_map.setdefault((current_w, current_h), set()).add(fps)
            except Exception:
                pass

    sizes = []
    for (w, h), fps_set in sorted(sizes_map.items(), key=lambda x: (x[0][0], x[0][1])):
        fps_list = sorted(fps_set, reverse=True)
        sizes.append({'w': int(w), 'h': int(h), 'fps': fps_list})

    return {'device': dev, 'sizes': sizes, 'formats': formats, 'source': 'v4l2-ctl'}


@app.route('/camera_caps')
def camera_caps():
    """Return supported camera sizes/FPS for the selected device."""
    device = request.args.get('device', SETTINGS.get('cam_device', '/dev/video0'))
    caps = _camera_caps_v4l2(device)
    return jsonify(caps)

# Motor controller (abstracted)
# DRV8833 pinout (BCM):
#   BIN1 (BN1)  -> GPIO 23
#   BIN2 (BN2)  -> GPIO 18
#   STBY/SLEEP  -> GPIO 4
#   AIN2 (AN2)  -> GPIO 27
#   AIN1 (AN1)  -> GPIO 17
#
# If forward/back are swapped on your wiring, toggle invert_left/invert_right.
motor = MotorController(
    pins={"AIN1": 17, "AIN2": 27, "BIN1": 23, "BIN2": 18, "STBY": 4},
    invert_left=True,
    invert_right=True,
    pwm_mode='in_pins',
)

# Keepalive / watchdog state
_last_keepalive = {}
_keepalive_lock = threading.Lock()
_active_actions = set()
_e_stop = False
# Mode: 'rc' (remote control) or 'autonomous'
MODE = 'rc'

# Optional navigation goal (world coordinates). When set, autonomy can plan toward it.
NAV_GOAL = None  # {'x': float, 'y': float}
# runtime settings (tunable from web UI)
SETTINGS = {
    'smoothing': True,
    # detector tuning
    'det_gamma': 0.9,
    'det_use_tflite': False,
    'det_contour_min_area': 400,
    'det_clahe_clip': 3.0,
    'det_rock_score_threshold': 0.45,
    # decision tuning
    'dec_obstacle_stop_ymin': 0.35,
    'dec_rough_slow_thresh': 200.0,
    'dec_rough_slow_frames': 3,
    'dec_ultra_stop_cm': 12.0,
    'dec_ultra_turn_cm': 30.0,
    'dec_stuck_dx_m': 0.02,
    'dec_stuck_frames': 3,

    # Current-aware decision making (requires power_enabled + working I2C reads)
    # These defaults are conservative; tune to your drivetrain.
    'dec_current_enabled': True,
    # If total motor current stays above this while trying to move, assume stall/stuck.
    'dec_current_motor_stall_a': 4.0,
    'dec_current_stall_frames': 3,
    # Hard limits: immediately stop+reverse when exceeded.
    'dec_current_motor_over_a': 7.0,
    'dec_current_system_over_a': 10.0,
    # Cooldown after an overcurrent trip.
    'dec_current_over_cooldown_s': 1.0,
    # RC tuning
    'rc_speed': 0.6,
    'rc_turn_gain': 1.0,

    # Autonomy speed policy (0..1 throttle knobs; MotorController maps to PWM duty)
    'auto_speed': 0.6,
    'auto_speed_slow': 0.35,
    'auto_speed_turn': 1.0,
    # When stuck is detected (via SLAM pose delta), boost to full
    'auto_speed_stuck': 1.0,

    # Tick-based autonomy (stop → detect → act → stop)
    # This reduces motion blur and can lower CPU/camera load.
    'auto_tick_enabled': True,
    # Time to remain stopped before grabbing a frame (seconds)
    'auto_tick_settle_s': 0.06,
    # Drive durations per tick (seconds)
    'auto_tick_forward_s': 0.18,
    'auto_tick_turn_s': 0.16,
    'auto_tick_reverse_s': 0.22,
    # Pause between ticks (seconds)
    'auto_tick_idle_s': 0.02,

    # IMU (MPU-6050 / GY-521) tuning
    'imu_enabled': True,
    'imu_i2c_bus': 1,
    'imu_i2c_addr': 0x68,
    'imu_alpha': 0.98,
    'imu_dlpf_cfg': 3,
    'imu_sample_div': 4,
    'imu_gyro_calib_seconds': 1.0,

    # Navigation exploration (global world-frame)
    # When enabled, Navigator will auto-pick frontier goals when no /nav_goal is set.
    'nav_explore_enabled': False,
    'nav_explore_replan_s': 2.0,
    'nav_explore_frontier_max_scan': 8000,
    'nav_explore_frontier_candidates': 50,
    # Mark observed radius around robot for "known" vs "unknown" (meters)
    'nav_observe_radius_m': 1.0,

    # Exploration stability / loop avoidance
    'nav_explore_goal_reached_m': 0.5,
    'nav_explore_progress_timeout_s': 4.0,
    'nav_explore_progress_min_delta_m': 0.15,
    'nav_explore_visit_weight': 0.25,
    'nav_explore_blacklist_s': 12.0,
    # Planning budgets to prevent stalls
    'nav_plan_max_expansions': 12000,
    'nav_plan_max_time_s': 0.06,

    # Dynamic obstacle layer (decays over time)
    'nav_dyn_half_life_s': 6.0,
    'nav_dyn_thresh': 0.40,
    'nav_dyn_prune_thresh': 0.05,

    # Navigation global map persistence
    'nav_persist_enabled': False,
    # Where to save the global map (.npz). Relative paths are relative to COSGCTank/.
    'nav_persist_path': 'nav_map_global.npz',
    # Periodic save interval (seconds)
    'nav_persist_period_s': 5.0,

    # Yaw PID (autonomous steering smoothing)
    'pid_yaw_enabled': False,
    'pid_yaw_kp': 1.2,
    'pid_yaw_ki': 0.0,
    'pid_yaw_kd': 0.08,
    'pid_yaw_out_limit': 1.0,
    'pid_yaw_i_limit': 0.6,

    # Web video stream (MJPEG via /video_feed)
    # Accepts '/dev/videoN' or 'N'
    'cam_device': '/dev/video0',
    'cam_width': 640,
    'cam_height': 480,
    'cam_fps': 20,
    'cam_jpeg_quality': 80,

    # Power / current sensing (I2C)
    # Two motor current channels plus an overall power channel.
    # Defaults assume INA219-style sensors at 0x40/0x43 and a system channel at 0x41.
    'power_enabled': False,
    'power_i2c_bus': 1,
    'power_motor_left_addr': 0x40,
    'power_motor_right_addr': 0x43,
    'power_system_addr': 0x41,
    # Shunt resistors in ohms (set these to match your hardware)
    'power_motor_left_shunt_ohm': 0.1,
    'power_motor_right_shunt_ohm': 0.1,
    'power_system_shunt_ohm': 0.1,
    # Poll interval (seconds)
    'power_poll_s': 0.25,
}

SETTINGS_PATH = Path(__file__).with_name('settings.json')


def _load_settings_from_disk():
    global SETTINGS
    try:
        if not SETTINGS_PATH.exists():
            return
        data = json.loads(SETTINGS_PATH.read_text())
        if not isinstance(data, dict):
            return
        # Only accept known keys; coerce to the existing type
        for k, v in data.items():
            if k not in SETTINGS:
                continue
            if isinstance(SETTINGS[k], bool):
                SETTINGS[k] = bool(v)
            elif isinstance(SETTINGS[k], int):
                try:
                    SETTINGS[k] = int(v)
                except Exception:
                    pass
            elif isinstance(SETTINGS[k], float):
                try:
                    SETTINGS[k] = float(v)
                except Exception:
                    pass
            else:
                SETTINGS[k] = v
    except Exception:
        # best-effort; don't prevent server start
        pass


def _save_settings_to_disk():
    try:
        SETTINGS_PATH.write_text(json.dumps(SETTINGS, indent=2, sort_keys=True))
    except Exception:
        # best-effort
        pass


def _clamp(v, lo, hi):
    try:
        x = float(v)
    except Exception:
        return lo
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def _clamp_int(v, lo, hi):
    try:
        x = int(v)
    except Exception:
        return lo
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def _apply_settings_ranges():
    """Enforce safe min/max ranges on known tuning knobs."""
    # RC
    SETTINGS['rc_speed'] = _clamp(SETTINGS.get('rc_speed', 0.6), 0.2, 1.0)
    SETTINGS['rc_turn_gain'] = _clamp(SETTINGS.get('rc_turn_gain', 1.0), 0.2, 2.5)

    # Detector
    SETTINGS['det_gamma'] = _clamp(SETTINGS.get('det_gamma', 0.9), 0.5, 1.5)
    try:
        SETTINGS['det_use_tflite'] = bool(SETTINGS.get('det_use_tflite', False))
    except Exception:
        SETTINGS['det_use_tflite'] = False
    SETTINGS['det_contour_min_area'] = _clamp_int(SETTINGS.get('det_contour_min_area', 400), 50, 20000)
    SETTINGS['det_clahe_clip'] = _clamp(SETTINGS.get('det_clahe_clip', 3.0), 0.5, 10.0)
    SETTINGS['det_rock_score_threshold'] = _clamp(SETTINGS.get('det_rock_score_threshold', 0.45), 0.0, 1.0)

    # Decision
    SETTINGS['dec_obstacle_stop_ymin'] = _clamp(SETTINGS.get('dec_obstacle_stop_ymin', 0.35), 0.0, 1.0)
    SETTINGS['dec_rough_slow_thresh'] = _clamp(SETTINGS.get('dec_rough_slow_thresh', 200.0), 0.0, 2000.0)
    SETTINGS['dec_rough_slow_frames'] = _clamp_int(SETTINGS.get('dec_rough_slow_frames', 3), 1, 50)
    # Ultrasonic thresholds (HC-SR04 typical operating range)
    SETTINGS['dec_ultra_stop_cm'] = _clamp(SETTINGS.get('dec_ultra_stop_cm', 12.0), 2.0, 200.0)
    SETTINGS['dec_ultra_turn_cm'] = _clamp(SETTINGS.get('dec_ultra_turn_cm', 30.0), 5.0, 300.0)
    SETTINGS['dec_stuck_dx_m'] = _clamp(SETTINGS.get('dec_stuck_dx_m', 0.02), 0.0, 1.0)
    SETTINGS['dec_stuck_frames'] = _clamp_int(SETTINGS.get('dec_stuck_frames', 3), 1, 50)

    # Current-aware decisions
    SETTINGS['dec_current_enabled'] = bool(SETTINGS.get('dec_current_enabled', True))
    SETTINGS['dec_current_motor_stall_a'] = _clamp(SETTINGS.get('dec_current_motor_stall_a', 4.0), 0.0, 200.0)
    SETTINGS['dec_current_stall_frames'] = _clamp_int(SETTINGS.get('dec_current_stall_frames', 3), 1, 200)
    SETTINGS['dec_current_motor_over_a'] = _clamp(SETTINGS.get('dec_current_motor_over_a', 7.0), 0.0, 400.0)
    SETTINGS['dec_current_system_over_a'] = _clamp(SETTINGS.get('dec_current_system_over_a', 10.0), 0.0, 400.0)
    SETTINGS['dec_current_over_cooldown_s'] = _clamp(SETTINGS.get('dec_current_over_cooldown_s', 1.0), 0.0, 10.0)

    # IMU decision tunables
    SETTINGS['dec_imu_roll_slow_deg'] = _clamp(SETTINGS.get('dec_imu_roll_slow_deg', 18.0), 0.0, 60.0)
    SETTINGS['dec_imu_pitch_slow_deg'] = _clamp(SETTINGS.get('dec_imu_pitch_slow_deg', 18.0), 0.0, 60.0)
    SETTINGS['dec_imu_roll_stop_deg'] = _clamp(SETTINGS.get('dec_imu_roll_stop_deg', 28.0), 0.0, 80.0)
    SETTINGS['dec_imu_pitch_stop_deg'] = _clamp(SETTINGS.get('dec_imu_pitch_stop_deg', 28.0), 0.0, 80.0)
    SETTINGS['dec_imu_gyro_jolt_rps'] = _clamp(SETTINGS.get('dec_imu_gyro_jolt_rps', 3.5), 0.1, 30.0)
    SETTINGS['dec_imu_jolt_cooldown_s'] = _clamp(SETTINGS.get('dec_imu_jolt_cooldown_s', 1.0), 0.0, 10.0)

    # Recovery / escape state machine
    SETTINGS['rec_enabled'] = bool(SETTINGS.get('rec_enabled', True))
    SETTINGS['rec_reverse_s'] = _clamp(SETTINGS.get('rec_reverse_s', 0.55), 0.05, 2.0)
    SETTINGS['rec_turn_s'] = _clamp(SETTINGS.get('rec_turn_s', 0.65), 0.05, 3.0)
    SETTINGS['rec_forward_s'] = _clamp(SETTINGS.get('rec_forward_s', 0.70), 0.05, 3.0)
    SETTINGS['rec_cooldown_s'] = _clamp(SETTINGS.get('rec_cooldown_s', 1.0), 0.0, 10.0)
    SETTINGS['rec_max_attempts'] = _clamp_int(SETTINGS.get('rec_max_attempts', 4), 0, 20)

    # Autonomy speed policy
    SETTINGS['auto_speed'] = _clamp(SETTINGS.get('auto_speed', 0.6), 0.0, 1.0)
    SETTINGS['auto_speed_slow'] = _clamp(SETTINGS.get('auto_speed_slow', 0.35), 0.0, 1.0)
    SETTINGS['auto_speed_turn'] = _clamp(SETTINGS.get('auto_speed_turn', 1.0), 0.0, 1.0)
    SETTINGS['auto_speed_stuck'] = _clamp(SETTINGS.get('auto_speed_stuck', 1.0), 0.0, 1.0)

    # Tick-based autonomy
    SETTINGS['auto_tick_enabled'] = bool(SETTINGS.get('auto_tick_enabled', True))
    SETTINGS['auto_tick_settle_s'] = _clamp(SETTINGS.get('auto_tick_settle_s', 0.06), 0.0, 0.5)
    SETTINGS['auto_tick_forward_s'] = _clamp(SETTINGS.get('auto_tick_forward_s', 0.18), 0.02, 2.0)
    SETTINGS['auto_tick_turn_s'] = _clamp(SETTINGS.get('auto_tick_turn_s', 0.16), 0.02, 2.0)
    SETTINGS['auto_tick_reverse_s'] = _clamp(SETTINGS.get('auto_tick_reverse_s', 0.22), 0.02, 2.0)
    SETTINGS['auto_tick_idle_s'] = _clamp(SETTINGS.get('auto_tick_idle_s', 0.02), 0.0, 1.0)

    # IMU (MPU-6050)
    SETTINGS['imu_enabled'] = bool(SETTINGS.get('imu_enabled', True))
    SETTINGS['imu_i2c_bus'] = _clamp_int(SETTINGS.get('imu_i2c_bus', 1), 0, 10)
    # 0x68 or 0x69 are common; allow 0x08..0x77 (7-bit I2C address range)
    SETTINGS['imu_i2c_addr'] = _clamp_int(SETTINGS.get('imu_i2c_addr', 0x68), 0x08, 0x77)
    SETTINGS['imu_alpha'] = _clamp(SETTINGS.get('imu_alpha', 0.98), 0.0, 0.999)
    SETTINGS['imu_dlpf_cfg'] = _clamp_int(SETTINGS.get('imu_dlpf_cfg', 3), 0, 6)
    SETTINGS['imu_sample_div'] = _clamp_int(SETTINGS.get('imu_sample_div', 4), 0, 255)
    SETTINGS['imu_gyro_calib_seconds'] = _clamp(SETTINGS.get('imu_gyro_calib_seconds', 1.0), 0.2, 10.0)

    # Navigation / planning
    # planner: 'astar' (default) or 'bfs'
    try:
        SETTINGS['nav_planner'] = str(SETTINGS.get('nav_planner', 'astar')).lower()
    except Exception:
        SETTINGS['nav_planner'] = 'astar'
    if SETTINGS['nav_planner'] not in ('astar', 'bfs'):
        SETTINGS['nav_planner'] = 'astar'
    SETTINGS['nav_heading_enabled'] = bool(SETTINGS.get('nav_heading_enabled', True))
    SETTINGS['nav_turn_adjust_deg'] = _clamp(SETTINGS.get('nav_turn_adjust_deg', 10.0), 0.0, 90.0)
    SETTINGS['nav_turn_stop_deg'] = _clamp(SETTINGS.get('nav_turn_stop_deg', 30.0), 0.0, 180.0)
    SETTINGS['nav_goal_dist_m'] = _clamp(SETTINGS.get('nav_goal_dist_m', 1.2), 0.2, 5.0)
    SETTINGS['nav_turn_commit_s'] = _clamp(SETTINGS.get('nav_turn_commit_s', 0.55), 0.0, 3.0)

    # Exploration
    SETTINGS['nav_explore_enabled'] = bool(SETTINGS.get('nav_explore_enabled', False))
    SETTINGS['nav_explore_replan_s'] = _clamp(SETTINGS.get('nav_explore_replan_s', 2.0), 0.2, 30.0)
    SETTINGS['nav_explore_frontier_max_scan'] = _clamp_int(SETTINGS.get('nav_explore_frontier_max_scan', 8000), 500, 200000)
    SETTINGS['nav_explore_frontier_candidates'] = _clamp_int(SETTINGS.get('nav_explore_frontier_candidates', 50), 1, 500)
    SETTINGS['nav_observe_radius_m'] = _clamp(SETTINGS.get('nav_observe_radius_m', 1.0), 0.2, 10.0)
    SETTINGS['nav_explore_goal_reached_m'] = _clamp(SETTINGS.get('nav_explore_goal_reached_m', 0.5), 0.1, 3.0)
    SETTINGS['nav_explore_progress_timeout_s'] = _clamp(SETTINGS.get('nav_explore_progress_timeout_s', 4.0), 0.2, 30.0)
    SETTINGS['nav_explore_progress_min_delta_m'] = _clamp(SETTINGS.get('nav_explore_progress_min_delta_m', 0.15), 0.0, 5.0)
    SETTINGS['nav_explore_visit_weight'] = _clamp(SETTINGS.get('nav_explore_visit_weight', 0.25), 0.0, 10.0)
    SETTINGS['nav_explore_blacklist_s'] = _clamp(SETTINGS.get('nav_explore_blacklist_s', 12.0), 0.0, 120.0)

    # Planning budgets
    SETTINGS['nav_plan_max_expansions'] = _clamp_int(SETTINGS.get('nav_plan_max_expansions', 12000), 200, 500000)
    SETTINGS['nav_plan_max_time_s'] = _clamp(SETTINGS.get('nav_plan_max_time_s', 0.06), 0.0, 1.0)

    # Dynamic occupancy decay
    SETTINGS['nav_dyn_half_life_s'] = _clamp(SETTINGS.get('nav_dyn_half_life_s', 6.0), 0.0, 120.0)
    SETTINGS['nav_dyn_thresh'] = _clamp(SETTINGS.get('nav_dyn_thresh', 0.40), 0.05, 1.0)
    SETTINGS['nav_dyn_prune_thresh'] = _clamp(SETTINGS.get('nav_dyn_prune_thresh', 0.05), 0.0, 0.5)

    # Persistence
    SETTINGS['nav_persist_enabled'] = bool(SETTINGS.get('nav_persist_enabled', False))
    try:
        SETTINGS['nav_persist_path'] = str(SETTINGS.get('nav_persist_path', 'nav_map_global.npz'))
    except Exception:
        SETTINGS['nav_persist_path'] = 'nav_map_global.npz'
    SETTINGS['nav_persist_period_s'] = _clamp(SETTINGS.get('nav_persist_period_s', 5.0), 0.0, 300.0)

    # PID yaw
    SETTINGS['pid_yaw_enabled'] = bool(SETTINGS.get('pid_yaw_enabled', False))
    SETTINGS['pid_yaw_kp'] = _clamp(SETTINGS.get('pid_yaw_kp', 1.2), 0.0, 10.0)
    SETTINGS['pid_yaw_ki'] = _clamp(SETTINGS.get('pid_yaw_ki', 0.0), 0.0, 10.0)
    SETTINGS['pid_yaw_kd'] = _clamp(SETTINGS.get('pid_yaw_kd', 0.08), 0.0, 10.0)
    SETTINGS['pid_yaw_out_limit'] = _clamp(SETTINGS.get('pid_yaw_out_limit', 1.0), 0.05, 1.0)
    SETTINGS['pid_yaw_i_limit'] = _clamp(SETTINGS.get('pid_yaw_i_limit', 0.6), 0.0, 5.0)

    # Camera stream
    try:
        SETTINGS['cam_device'] = str(SETTINGS.get('cam_device', '/dev/video0')).strip() or '/dev/video0'
    except Exception:
        SETTINGS['cam_device'] = '/dev/video0'
    SETTINGS['cam_width'] = _clamp_int(SETTINGS.get('cam_width', 640), 160, 1920)
    SETTINGS['cam_height'] = _clamp_int(SETTINGS.get('cam_height', 480), 120, 1080)
    SETTINGS['cam_fps'] = _clamp_int(SETTINGS.get('cam_fps', 20), 1, 60)
    SETTINGS['cam_jpeg_quality'] = _clamp_int(SETTINGS.get('cam_jpeg_quality', 80), 20, 95)

    # Power/current sensing
    SETTINGS['power_enabled'] = bool(SETTINGS.get('power_enabled', False))
    SETTINGS['power_i2c_bus'] = _clamp_int(SETTINGS.get('power_i2c_bus', 1), 0, 10)
    SETTINGS['power_motor_left_addr'] = _clamp_int(SETTINGS.get('power_motor_left_addr', 0x40), 0x08, 0x77)
    SETTINGS['power_motor_right_addr'] = _clamp_int(SETTINGS.get('power_motor_right_addr', 0x43), 0x08, 0x77)
    SETTINGS['power_system_addr'] = _clamp_int(SETTINGS.get('power_system_addr', 0x41), 0x08, 0x77)
    SETTINGS['power_motor_left_shunt_ohm'] = _clamp(SETTINGS.get('power_motor_left_shunt_ohm', 0.1), 0.001, 5.0)
    SETTINGS['power_motor_right_shunt_ohm'] = _clamp(SETTINGS.get('power_motor_right_shunt_ohm', 0.1), 0.001, 5.0)
    SETTINGS['power_system_shunt_ohm'] = _clamp(SETTINGS.get('power_system_shunt_ohm', 0.1), 0.001, 5.0)
    SETTINGS['power_poll_s'] = _clamp(SETTINGS.get('power_poll_s', 0.25), 0.05, 5.0)


_load_settings_from_disk()
_apply_settings_ranges()

# How long without a keepalive before server forces stop (seconds)
KEEPALIVE_TIMEOUT = 0.6

def _watchdog_loop():
    while True:
        now = time.time()
        to_stop = []
        with _keepalive_lock:
            for action, ts in list(_last_keepalive.items()):
                if now - ts > KEEPALIVE_TIMEOUT and action in _active_actions:
                    to_stop.append(action)
        for a in to_stop:
            try:
                motor.stop()
            except Exception:
                pass
            with _keepalive_lock:
                _active_actions.discard(a)
                _last_keepalive.pop(a, None)
        time.sleep(0.1)

# start watchdog thread
_watchdog_thread = threading.Thread(target=_watchdog_loop, daemon=True)
_watchdog_thread.start()


def _autonomy_watchdog_loop():
    """Restart autonomy if the loop stalls for too long."""
    while True:
        try:
            now = time.time()
            stale = None
            try:
                if _autonomy_heartbeat_ts > 0:
                    stale = now - float(_autonomy_heartbeat_ts)
            except Exception:
                stale = None

            if MODE != 'rc' and _autonomy_thread is not None and _autonomy_thread.is_alive():
                try:
                    if stale is not None and stale <= 4.0:
                        globals()['_autonomy_restart_attempts'] = 0
                except Exception:
                    pass
                if stale is not None and stale > 4.0:
                    try:
                        globals()['_autonomy_last_error'] = 'heartbeat_timeout'
                    except Exception:
                        pass
                    try:
                        attempts = globals().get('_autonomy_restart_attempts', 0)
                    except Exception:
                        attempts = 0
                    if attempts >= 3:
                        try:
                            globals()['_autonomy_last_error'] = 'heartbeat_timeout_max'
                        except Exception:
                            pass
                        time.sleep(1.0)
                    else:
                        try:
                            globals()['_autonomy_restart_attempts'] = attempts + 1
                        except Exception:
                            pass
                        backoff = 0.5 * (attempts + 1)
                        time.sleep(backoff)
                        try:
                            _stop_autonomy()
                        except Exception:
                            pass
                        time.sleep(0.25)
                        try:
                            _start_autonomy_if_needed()
                        except Exception:
                            pass
        except Exception:
            pass
        time.sleep(0.4)


_autonomy_watchdog_thread = threading.Thread(target=_autonomy_watchdog_loop, daemon=True)
_autonomy_watchdog_thread.start()


def _power_poll_loop():
    """Poll I2C power/current sensors best-effort.

    Runs continuously; when disabled it just sleeps.
    """
    global _power_monitor, _power_latest, _power_latest_ts
    # Lazy import so dev machines don't require anything beyond requirements.txt
    try:
        from power_monitor import PowerMonitor, PowerMonitorSettings
    except Exception:
        PowerMonitor = None  # type: ignore
        PowerMonitorSettings = None  # type: ignore

    last_cfg = None
    while True:
        try:
            poll_s = float(SETTINGS.get('power_poll_s', 0.25))
        except Exception:
            poll_s = 0.25
        poll_s = max(0.05, min(5.0, poll_s))

        try:
            enabled = bool(SETTINGS.get('power_enabled', False))
        except Exception:
            enabled = False

        if not enabled or PowerMonitor is None:
            time.sleep(max(0.2, poll_s))
            continue

        try:
            cfg = {
                'enabled': True,
                'i2c_bus': int(SETTINGS.get('power_i2c_bus', 1)),
                'motor_left_addr': int(SETTINGS.get('power_motor_left_addr', 0x40)),
                'motor_right_addr': int(SETTINGS.get('power_motor_right_addr', 0x43)),
                'system_addr': int(SETTINGS.get('power_system_addr', 0x41)),
                'motor_left_shunt_ohm': float(SETTINGS.get('power_motor_left_shunt_ohm', 0.1)),
                'motor_right_shunt_ohm': float(SETTINGS.get('power_motor_right_shunt_ohm', 0.1)),
                'system_shunt_ohm': float(SETTINGS.get('power_system_shunt_ohm', 0.1)),
            }
        except Exception:
            cfg = None

        with _power_lock:
            if _power_monitor is None:
                try:
                    s = PowerMonitorSettings(
                        enabled=True,
                        i2c_bus=cfg['i2c_bus'],
                        motor_left_addr=cfg['motor_left_addr'],
                        motor_right_addr=cfg['motor_right_addr'],
                        system_addr=cfg['system_addr'],
                        motor_left_shunt_ohm=cfg['motor_left_shunt_ohm'],
                        motor_right_shunt_ohm=cfg['motor_right_shunt_ohm'],
                        system_shunt_ohm=cfg['system_shunt_ohm'],
                    )
                    _power_monitor = PowerMonitor(s)
                    last_cfg = dict(cfg)
                except Exception:
                    _power_monitor = None
                    last_cfg = None

            elif cfg is not None and last_cfg is not None and cfg != last_cfg:
                try:
                    s = PowerMonitorSettings(
                        enabled=True,
                        i2c_bus=cfg['i2c_bus'],
                        motor_left_addr=cfg['motor_left_addr'],
                        motor_right_addr=cfg['motor_right_addr'],
                        system_addr=cfg['system_addr'],
                        motor_left_shunt_ohm=cfg['motor_left_shunt_ohm'],
                        motor_right_shunt_ohm=cfg['motor_right_shunt_ohm'],
                        system_shunt_ohm=cfg['system_shunt_ohm'],
                    )
                    _power_monitor.update_settings(s)
                    last_cfg = dict(cfg)
                except Exception:
                    pass

            mon = _power_monitor

        # Read outside lock
        try:
            reading = mon.read() if mon is not None else None
        except Exception:
            reading = None

        if reading is not None:
            with _power_lock:
                _power_latest = reading
                _power_latest_ts = time.time()

        time.sleep(poll_s)


_power_thread = threading.Thread(target=_power_poll_loop, daemon=True)
_power_thread.start()

# ---------------- MOTOR ROUTES ---------------- #

@app.route("/")
def home():
    return render_template("control.html")


@app.route('/video_feed')
def video_feed():
    """MJPEG stream for the control page.

    Defaults to /dev/video0. Override via:
      - query params: ?device=0&width=640&height=480&fps=20
      - env vars: COSGC_CAMERA_DEVICE, COSGC_CAMERA_WIDTH, COSGC_CAMERA_HEIGHT, COSGC_CAMERA_FPS
    """
    device = request.args.get('device', SETTINGS.get('cam_device', os.environ.get('COSGC_CAMERA_DEVICE', '0')))
    width = request.args.get('width', SETTINGS.get('cam_width', os.environ.get('COSGC_CAMERA_WIDTH', '640')))
    height = request.args.get('height', SETTINGS.get('cam_height', os.environ.get('COSGC_CAMERA_HEIGHT', '480')))
    fps = request.args.get('fps', SETTINGS.get('cam_fps', os.environ.get('COSGC_CAMERA_FPS', '20')))
    quality = request.args.get('q', SETTINGS.get('cam_jpeg_quality', os.environ.get('COSGC_CAMERA_JPEG_QUALITY', '80')))

    dev_idx = _parse_camera_device(device)
    try:
        width = int(width)
        height = int(height)
        fps = int(float(fps))
        quality = int(quality)
    except Exception:
        width, height, fps, quality = 640, 480, 20, 80

    # Clamp to safe-ish ranges
    width = max(160, min(1920, width))
    height = max(120, min(1080, height))
    fps = max(1, min(60, fps))
    quality = max(20, min(95, quality))

    provider = _get_video_provider(dev_idx, width, height, fps)

    def gen():
        while True:
            data = provider.get_frame()
            if data is None:
                time.sleep(0.05)
                continue

            # FrameProvider returns RGB; encode as JPEG (BGR expected by OpenCV conventions)
            frame_rgb = data.get('frame')
            if frame_rgb is None:
                time.sleep(0.01)
                continue
            try:
                frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            except Exception:
                frame_bgr = frame_rgb

            ok, buf = cv2.imencode('.jpg', frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
            if not ok:
                time.sleep(0.01)
                continue

            yield (
                b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n'
            )

    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/cmd/<action>")
def command(action):
    # Backwards-compatible single-shot command
    ok = False
    with _keepalive_lock:
        # Respect e-stop
        if _e_stop:
            return "E-STOP ACTIVE", 423
        if action == "f":
            motor.forward(); ok = True
            _active_actions.add('f'); _last_keepalive['f'] = time.time()
        elif action == "b":
            motor.reverse(); ok = True
            _active_actions.add('b'); _last_keepalive['b'] = time.time()
        elif action == "l":
            motor.turn_left(); ok = True
            _active_actions.add('l'); _last_keepalive['l'] = time.time()
        elif action == "r":
            motor.turn_right(); ok = True
            _active_actions.add('r'); _last_keepalive['r'] = time.time()
        elif action == "s":
            # Soft stop should work in any mode: stop motion and return to RC.
            motor.stop(); ok = True
            _active_actions.clear(); _last_keepalive.clear()
            if MODE != 'rc':
                # Force RC mode so the UI can recover manual control.
                # Autonomy loop will be stopped outside the lock.
                globals()['MODE'] = 'rc'
    # If STOP forced a mode change, stop autonomy loop now.
    try:
        if action == 's' and MODE == 'rc':
            _stop_autonomy()
    except Exception:
        pass
    return ("OK", 200) if ok else ("Unknown action", 400)


@app.route("/cmd/<action>/<state>")
def command_state(action, state):
    """Stateful command endpoint. `state` is 'start' or 'stop'.

    Example: GET /cmd/f/start  -> begin forward
             GET /cmd/f/stop   -> stop
    """
    state = state.lower()
    if state not in ("start", "stop"):
        return "Invalid state", 400

    if state == "stop":
        motor.stop()
        with _keepalive_lock:
            _active_actions.discard(action)
            _last_keepalive.pop(action, None)
        return "OK"

    # start state: map action to motor method
    with _keepalive_lock:
        if _e_stop:
            return "E-STOP ACTIVE", 423
        if action == "f":
            motor.forward()
            _active_actions.add('f'); _last_keepalive['f'] = time.time()
        elif action == "b":
            motor.reverse()
            _active_actions.add('b'); _last_keepalive['b'] = time.time()
        elif action == "l":
            motor.turn_left()
            _active_actions.add('l'); _last_keepalive['l'] = time.time()
        elif action == "r":
            motor.turn_right()
            _active_actions.add('r'); _last_keepalive['r'] = time.time()
        else:
            return "Unknown action", 400
    return "OK"


@app.route('/keepalive', methods=['POST'])
def keepalive():
    """Client should POST JSON {action: 'f'|'b'|'l'|'r', state: 'start'|'stop'} repeatedly while holding."""
    data = request.get_json() or {}
    action = data.get('action')
    state = data.get('state')
    if not action or state not in ('start', 'stop'):
        return jsonify({'error': 'bad payload'}), 400

    with _keepalive_lock:
        if state == 'stop':
            motor.stop()
            _active_actions.discard(action)
            _last_keepalive.pop(action, None)
            return jsonify({'status': 'stopped'})
        # start
        # only allow keepalives in RC mode
        if MODE != 'rc':
            return jsonify({'error': 'not_in_rc_mode'}), 409
        if _e_stop:
            return jsonify({'error': 'e_stop_active'}), 423
        sp = float(SETTINGS.get('rc_speed', 0.6))
        # map action letters to motor calls
        if action == 'f':
            motor.forward(speed=sp)
        elif action == 'b':
            motor.reverse(speed=sp)
        elif action == 'l':
            motor.turn_left(speed=1.0)
        elif action == 'r':
            motor.turn_right(speed=1.0)
        else:
            return jsonify({'error': 'unknown action'}), 400
        _active_actions.add(action)
        _last_keepalive[action] = time.time()
    return jsonify({'status': 'ok'})


@app.route('/status')
def status():
    with _keepalive_lock:
        power_copy = None
        power_age = None
        try:
            with _power_lock:
                power_copy = _power_latest
                if _power_latest_ts is not None:
                    power_age = max(0.0, time.time() - float(_power_latest_ts))
        except Exception:
            power_copy = None
        # best-effort IMU summary from the last autonomy log (if running)
        imu_summary = None
        try:
            p = (latest_log or {}).get('perception', {}) if isinstance(latest_log, dict) else {}
            imu = p.get('imu') if isinstance(p, dict) else None
            if isinstance(imu, dict):
                orient = imu.get('orientation')
                if not isinstance(orient, dict):
                    orient = {}
                roll = orient.get('roll_rad')
                pitch = orient.get('pitch_rad')
                temp_c = imu.get('temperature_c')
                imu_summary = {
                    'connected': True,
                    'roll_deg': None if roll is None else (float(roll) * 57.29577951308232),
                    'pitch_deg': None if pitch is None else (float(pitch) * 57.29577951308232),
                    'temp_c': None if temp_c is None else float(temp_c),
                    'age_s': max(0.0, time.time() - latest_log_ts) if latest_log_ts else None,
                }
            else:
                imu_summary = {'connected': False, 'age_s': max(0.0, time.time() - latest_log_ts) if latest_log_ts else None}
        except Exception:
            imu_summary = None

        slam_summary = None
        try:
            slam_age = None if _autonomy_last_slam_ts == 0 else max(0.0, time.time() - _autonomy_last_slam_ts)
            p = (latest_log or {}).get('perception', {}) if isinstance(latest_log, dict) else {}
            slam_pose = p.get('slam_pose') if isinstance(p, dict) else None
            slam_summary = {
                'connected': slam_pose is not None,
                'age_s': slam_age,
                'pose': slam_pose,
            }
        except Exception:
            slam_summary = None

        nav_summary = None
        try:
            nav_summary = {
                'planner': SETTINGS.get('nav_planner', 'astar'),
                'heading_enabled': bool(SETTINGS.get('nav_heading_enabled', True)),
                'goal_dist_m': float(SETTINGS.get('nav_goal_dist_m', 1.2)),
                'turn_adjust_deg': float(SETTINGS.get('nav_turn_adjust_deg', 10.0)),
                'turn_stop_deg': float(SETTINGS.get('nav_turn_stop_deg', 30.0)),
            }
        except Exception:
            nav_summary = None

        nav_debug = None
        try:
            p = (latest_log or {}).get('perception', {}) if isinstance(latest_log, dict) else {}
            if isinstance(p, dict):
                nav_debug = p.get('nav_debug')
        except Exception:
            nav_debug = None

        decision_debug = None
        try:
            if isinstance(latest_log, dict):
                d = latest_log.get('decision')
                if isinstance(d, dict):
                    # keep it small; include common recovery/commit fields
                    decision_debug = {
                        'command': d.get('command'),
                        'reason': d.get('reason'),
                        'stuck': d.get('stuck'),
                        'recovery': d.get('recovery'),
                        'recovery_attempt': d.get('recovery_attempt'),
                        'nav_turn_commit_remaining_s': d.get('nav_turn_commit_remaining_s'),
                    }
        except Exception:
            decision_debug = None
        nav_target = None
        try:
            p = (latest_log or {}).get('perception', {}) if isinstance(latest_log, dict) else {}
            if isinstance(p, dict):
                nt = p.get('nav_target')
                nav_target = nt if isinstance(nt, dict) else None
        except Exception:
            nav_target = None
        return jsonify({
            'active_actions': list(_active_actions),
            'e_stop': bool(_e_stop),
            'mode': MODE,
            'autonomy_running': bool(_autonomy_thread is not None and _autonomy_thread.is_alive()),
            'autonomy_error': _autonomy_last_error,
            'autonomy_traceback': _autonomy_last_traceback,
            'detector_init_error': _autonomy_detector_init_error,
            'log_age_s': (max(0.0, time.time() - latest_log_ts) if latest_log_ts else None),
            'smoothing': bool(SETTINGS.get('smoothing', False)),
            'settings': SETTINGS,
            'imu': imu_summary,
            'slam': slam_summary,
            'nav': nav_summary,
            'nav_debug': nav_debug,
            'nav_goal': NAV_GOAL,
            'nav_target': nav_target,
            'decision': decision_debug,
            'power': power_copy,
            'power_age_s': power_age,
        })


@app.route('/debug/global_map.png')
def debug_global_map_png():
    """Render a lightweight global map PNG (seen vs occupied).

    This reads the persisted nav_map_global.npz (Navigator save) and visualizes:
      - red: occupied
      - gray: observed free
      - dark: unknown/unobserved
      - yellow: dynamic occupancy over threshold
    """
    try:
        import numpy as np
        import cv2
        from flask import Response

        occ = seen = dyn = origin = None
        map_label = 'live'
        try:
            with _navigator_lock:
                nav = _navigator
                if nav is not None:
                    occ = np.array(nav.global_occupancy, dtype=np.uint8, copy=True)
                    seen = np.array(nav.global_seen, dtype=np.uint8, copy=True)
                    dyn = np.array(nav.global_dyn, dtype=np.float32, copy=True)
                    if nav.global_origin is not None:
                        origin = np.array([nav.global_origin[0], nav.global_origin[1]], dtype=np.float32)
        except Exception:
            occ = seen = dyn = origin = None

        if occ is None:
            try:
                nav_path = SETTINGS.get('nav_persist_path', 'nav_map_global.npz')
            except Exception:
                nav_path = 'nav_map_global.npz'
            p = Path(nav_path)
            if not p.is_absolute():
                p = Path(__file__).parent / p

            if not p.exists():
                img = np.zeros((320, 320, 3), dtype=np.uint8)
                img[:] = (22, 24, 32)
                cv2.putText(img, 'No map file', (12, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (220, 220, 220), 2)
                cv2.putText(img, str(p), (12, 205), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
                ok, buf = cv2.imencode('.png', img)
                return Response(buf.tobytes(), mimetype='image/png') if ok else ('encode_failed', 500)

            data = np.load(p, allow_pickle=False)
            occ = np.array(data.get('global_occupancy'), dtype=np.uint8) if 'global_occupancy' in data else None
            seen = np.array(data.get('global_seen'), dtype=np.uint8) if 'global_seen' in data else None
            dyn = np.array(data.get('global_dyn'), dtype=np.float32) if 'global_dyn' in data else None
            origin = np.array(data.get('global_origin')).reshape(-1) if 'global_origin' in data else None
            map_label = p.name
        else:
            map_label = 'live'

        if occ is None or occ.ndim != 2:
            return ('no_occupancy', 500)

        h, w = occ.shape
        canvas = np.zeros((h, w, 3), dtype=np.uint8)
        canvas[:] = (25, 25, 30)  # unknown

        if seen is not None and seen.shape == occ.shape:
            canvas[seen == 1] = (80, 80, 80)

        dyn_mask = None
        if dyn is not None and dyn.shape == occ.shape:
            try:
                dyn_thresh = float(SETTINGS.get('nav_dyn_thresh', 0.40))
            except Exception:
                dyn_thresh = 0.40
            dyn_mask = dyn >= dyn_thresh
            canvas[dyn_mask] = (40, 200, 200)

        canvas[occ == 1] = (0, 0, 220)

        try:
            if origin is not None and origin.size >= 2 and np.isfinite(origin[0]) and np.isfinite(origin[1]):
                cx = w // 2
                cy = h // 2
                cv2.drawMarker(canvas, (cx, cy), (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=12, thickness=2)
        except Exception:
            pass

        scale = 2
        if max(h, w) < 300:
            scale = int(max(2, round(320 / max(h, w))))
        img = cv2.resize(canvas, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)
        cv2.putText(img, f"map: {map_label}", (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1)
        ok, buf = cv2.imencode('.png', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        if not ok:
            return ('encode_failed', 500)
        return Response(buf.tobytes(), mimetype='image/png')
    except Exception:
        return ('error', 500)


@app.route('/nav_goal', methods=['GET', 'POST', 'DELETE'])
def nav_goal():
    """Get/set/clear navigation goal.

    POST body: {"x": <float>, "y": <float>}
    DELETE clears.
    """
    global NAV_GOAL
    if request.method == 'GET':
        return jsonify({'nav_goal': NAV_GOAL})
    if request.method == 'DELETE':
        NAV_GOAL = None
        return jsonify({'nav_goal': NAV_GOAL})

    data = request.get_json(silent=True) or {}
    try:
        x = float(data.get('x'))
        y = float(data.get('y'))
    except Exception:
        return jsonify({'error': 'bad_goal'}), 400
    NAV_GOAL = {'x': x, 'y': y}
    return jsonify({'nav_goal': NAV_GOAL})


def _reset_nav_state(delete_file=True):
    """Clear live navigator state and optionally delete persisted map."""
    try:
        with _navigator_lock:
            nav = _navigator
            if nav is not None:
                nav.global_occupancy[:, :] = 0
                nav.global_seen[:, :] = 0
                nav.global_dyn[:, :] = 0.0
                nav.global_visit_count[:, :] = 0
                nav.global_origin = None
                nav.goal = None
                nav._explore_goal_cell = None
    except Exception:
        pass

    if not delete_file:
        return
    try:
        nav_path = SETTINGS.get('nav_persist_path', 'nav_map_global.npz')
    except Exception:
        nav_path = 'nav_map_global.npz'
    p = Path(nav_path)
    if not p.is_absolute():
        p = Path(__file__).parent / p
    try:
        if p.exists():
            p.unlink()
    except Exception:
        pass


@app.route('/nav_reset', methods=['POST'])
def nav_reset():
    _reset_nav_state(delete_file=True)
    return jsonify({'status': 'reset'})


@app.route('/emergency_stop', methods=['POST'])
def emergency_stop():
    global _e_stop
    # E-stop should always be recoverable from the UI without restarting.
    # We force RC mode so manual controls can be re-enabled after clearing.
    with _keepalive_lock:
        _e_stop = True
        try:
            globals()['MODE'] = 'rc'
        except Exception:
            pass
        try:
            # prefer safe_stop if available
            if hasattr(motor, 'safe_stop'):
                motor.safe_stop()
            else:
                motor.stop()
        except Exception:
            pass
        _active_actions.clear()
        _last_keepalive.clear()

    try:
        _stop_autonomy()
    except Exception:
        pass

    return jsonify({'status': 'e_stop_set', 'mode': MODE})


@app.route('/clear_emergency', methods=['POST'])
def clear_emergency():
    global _e_stop
    with _keepalive_lock:
        _e_stop = False
        # Clear any stale RC keepalive state.
        _active_actions.clear()
        _last_keepalive.clear()
        try:
            motor.stop()
        except Exception:
            pass
    return jsonify({'status': 'e_stop_cleared', 'mode': MODE})


@app.route('/mode', methods=['GET', 'POST'])
def mode():
    """GET returns current mode. POST JSON {mode: 'rc'|'autonomous'} to switch.

    When switching to a non-RC mode, the server starts a background autonomy loop.
    Switching back to RC stops the loop and stops the motors.
    """
    global MODE
    if request.method == 'GET':
        return jsonify({'mode': MODE})
    data = request.get_json() or {}
    m = data.get('mode')
    if m not in ('rc', 'autonomous', 'single_cam', 'stereo_cam'):
        return jsonify({'error': 'invalid mode'}), 400
    with _keepalive_lock:
        MODE = m
        # Always clear RC keepalive state when changing modes.
        _active_actions.clear(); _last_keepalive.clear()

        # Ensure motors are stopped on any mode change.
        try:
            motor.stop()
        except Exception:
            pass

    # Start/stop autonomy loop outside lock.
    if MODE == 'rc':
        _stop_autonomy()
    else:
        _start_autonomy_if_needed()
    return jsonify({'mode': MODE})


@app.route('/settings', methods=['GET', 'POST'])
def settings():
    global SETTINGS
    if request.method == 'GET':
        return jsonify(SETTINGS)
    data = request.get_json() or {}
    # only accept recognized keys; coerce types
    for k, v in data.items():
        if k not in SETTINGS:
            continue
        if isinstance(SETTINGS[k], bool):
            SETTINGS[k] = bool(v)
        elif isinstance(SETTINGS[k], int):
            try:
                SETTINGS[k] = int(v)
            except Exception:
                pass
        elif isinstance(SETTINGS[k], float):
            try:
                SETTINGS[k] = float(v)
            except Exception:
                pass
        else:
            SETTINGS[k] = v
    _apply_settings_ranges()
    _save_settings_to_disk()
    return jsonify(SETTINGS)


@app.route('/joystick', methods=['POST'])
def joystick():
    """Accept analog joystick input. Payload: {lx, ly, rx, ry} in range [-1,1].

    Mapping (simple):
      - if MODE != 'rc' => 409
      - if e_stop set => 423
      - use ly for throttle (forward/back), rx for steering/pivot.
    """
    data = request.get_json() or {}
    with _keepalive_lock:
        if MODE != 'rc':
            return jsonify({'error': 'not_in_rc_mode'}), 409
        if _e_stop:
            return jsonify({'error': 'e_stop_active'}), 423
    lx = float(data.get('lx', 0.0))
    ly = float(data.get('ly', 0.0))
    rx = float(data.get('rx', 0.0))
    ry = float(data.get('ry', 0.0))

    # deadzone
    DZ = 0.15
    if abs(ly) < DZ and abs(rx) < DZ:
        motor.stop()
        return jsonify({'status': 'stopped'})

    sp = float(SETTINGS.get('rc_speed', 0.6))
    turn_gain = float(SETTINGS.get('rc_turn_gain', 1.0))

    # steering vs throttle dominance
    if abs(ly) >= abs(rx):
        # throttle-dominant: forward/back
        if ly < -DZ:
            motor.forward(speed=sp)
        elif ly > DZ:
            motor.reverse(speed=sp)
        else:
            motor.stop()
    else:
        # steering-dominant: pivot
        if rx < -DZ:
            motor.turn_left(speed=1.0)
        elif rx > DZ:
            motor.turn_right(speed=1.0)
        else:
            motor.stop()

    return jsonify({'status': 'ok'})

@app.route("/led/<state>")
def led(state):
    GPIO.output(LED, GPIO.HIGH if state == "on" else GPIO.LOW)
    return "OK"

# ---------------- PERCEPTION LOGGING ---------------- #

latest_log = {}  # store latest perception + decision
latest_log_ts = 0.0

@app.route("/log", methods=["POST"])
def log_update():
    global latest_log, latest_log_ts
    latest_log = request.json
    latest_log_ts = time.time()
    return jsonify({"status": "received"})

@app.route("/console")
def show_console():
    return render_template("console.html")

@app.route("/console_data")
def console_data():
    return jsonify(latest_log)

# ---------------- MAIN ---------------- #

if __name__ == "__main__":
    try:
        motor.stop()
        app.run(host="0.0.0.0", port=5000)
    finally:
        motor.cleanup()
