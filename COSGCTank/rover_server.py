from flask import Flask, render_template, request, jsonify
import time
import threading
import json
from pathlib import Path

from motor_control import MotorController

app = Flask(__name__, template_folder="templates")

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
    'det_contour_min_area': 400,
    'det_clahe_clip': 3.0,
    'det_rock_score_threshold': 0.45,
    # decision tuning
    'dec_obstacle_stop_ymin': 0.35,
    'dec_ultra_stop_cm': 12.0,
    'dec_ultra_turn_cm': 30.0,
    'dec_stuck_dx_m': 0.02,
    'dec_stuck_frames': 3,
    # RC tuning
    'rc_speed': 0.6,
    'rc_turn_gain': 1.0,

    # Autonomy speed policy (0..1 throttle knobs; MotorController maps to PWM duty)
    'auto_speed': 0.6,
    'auto_speed_slow': 0.35,
    'auto_speed_turn': 1.0,
    # When stuck is detected (via SLAM pose delta), boost to full
    'auto_speed_stuck': 1.0,

    # IMU (MPU-6050 / GY-521) tuning
    'imu_enabled': True,
    'imu_i2c_bus': 1,
    'imu_i2c_addr': 0x68,
    'imu_alpha': 0.98,
    'imu_dlpf_cfg': 3,
    'imu_sample_div': 4,
    'imu_gyro_calib_seconds': 1.0,
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
    SETTINGS['det_contour_min_area'] = _clamp_int(SETTINGS.get('det_contour_min_area', 400), 50, 20000)
    SETTINGS['det_clahe_clip'] = _clamp(SETTINGS.get('det_clahe_clip', 3.0), 0.5, 10.0)
    SETTINGS['det_rock_score_threshold'] = _clamp(SETTINGS.get('det_rock_score_threshold', 0.45), 0.0, 1.0)

    # Decision
    SETTINGS['dec_obstacle_stop_ymin'] = _clamp(SETTINGS.get('dec_obstacle_stop_ymin', 0.35), 0.0, 1.0)
    # Ultrasonic thresholds (HC-SR04 typical operating range)
    SETTINGS['dec_ultra_stop_cm'] = _clamp(SETTINGS.get('dec_ultra_stop_cm', 12.0), 2.0, 200.0)
    SETTINGS['dec_ultra_turn_cm'] = _clamp(SETTINGS.get('dec_ultra_turn_cm', 30.0), 5.0, 300.0)
    SETTINGS['dec_stuck_dx_m'] = _clamp(SETTINGS.get('dec_stuck_dx_m', 0.02), 0.0, 1.0)
    SETTINGS['dec_stuck_frames'] = _clamp_int(SETTINGS.get('dec_stuck_frames', 3), 1, 50)

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

# ---------------- MOTOR ROUTES ---------------- #

@app.route("/")
def home():
    return render_template("control.html")

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
            motor.stop(); ok = True
            _active_actions.clear(); _last_keepalive.clear()
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
        return jsonify({
            'active_actions': list(_active_actions),
            'e_stop': bool(_e_stop),
            'mode': MODE,
            'smoothing': bool(SETTINGS.get('smoothing', False)),
            'settings': SETTINGS,
            'imu': imu_summary,
            'nav': nav_summary,
            'nav_debug': nav_debug,
            'nav_goal': NAV_GOAL,
            'decision': decision_debug,
        })


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


@app.route('/emergency_stop', methods=['POST'])
def emergency_stop():
    global _e_stop
    with _keepalive_lock:
        _e_stop = True
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
    return jsonify({'status': 'e_stop_set'})


@app.route('/clear_emergency', methods=['POST'])
def clear_emergency():
    global _e_stop
    with _keepalive_lock:
        _e_stop = False
    return jsonify({'status': 'e_stop_cleared'})


@app.route('/mode', methods=['GET', 'POST'])
def mode():
    """GET returns current mode. POST JSON {mode: 'rc'|'autonomous'} to switch.

    The server only tracks the desired mode. The autonomous stack (main loop / SLAM)
    should poll this endpoint or otherwise be notified to start/stop its pipelines.
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
        # if leaving RC, ensure motors are stopped
        if MODE != 'rc':
            try:
                motor.stop()
            except Exception:
                pass
            _active_actions.clear(); _last_keepalive.clear()
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
