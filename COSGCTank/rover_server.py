from flask import Flask, render_template, request, jsonify
import time
import threading

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
}

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
        return jsonify({
            'active_actions': list(_active_actions),
            'e_stop': bool(_e_stop),
            'mode': MODE,
            'smoothing': bool(SETTINGS.get('smoothing', False))
        })


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

@app.route("/log", methods=["POST"])
def log_update():
    global latest_log
    latest_log = request.json
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
