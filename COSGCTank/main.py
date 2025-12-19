from camera import FrameProvider
from detector import ObstacleDetector
from boundaries import BoundaryDetector
from terrain import TerrainAnalyzer
from decision import DecisionMaker
from overlay import OverlayDrawer
from motor_control import MotorController

import cv2
import time
import requests
import math

try:
    from power_monitor import PowerMonitor, PowerMonitorSettings
except Exception:
    PowerMonitor = None  # type: ignore
    PowerMonitorSettings = None  # type: ignore

SERVER_URL = "http://127.0.0.1:5000/log"   # default to local Flask server


def send_log_to_server(data):
    try:
        requests.post(SERVER_URL, json=data, timeout=0.25)
    except Exception:
        # Don't block the main loop on logging failures
        pass


def main():
    print("Starting rover perception system...")

    provider = FrameProvider(width=640, height=480, fps=20)
    det = ObstacleDetector(settings={})
    bounds = BoundaryDetector()
    terrain = TerrainAnalyzer()
    decider = DecisionMaker()
    overlay = OverlayDrawer()
    motor = MotorController()
    try:
        from ultrasonic import Ultrasonic
        # HC-SR04 wiring (BCM): TRIG=24, ECHO=25 (ECHO must be level-shifted to 3.3V)
        us = Ultrasonic(trig_pin=24, echo_pin=25)
    except Exception:
        us = None
    from navigator import Navigator
    from scale_estimator import ScaleEstimator
    nav = Navigator(grid_size=40, resolution=0.2)
    # Default local navigation behavior: keep a short-horizon goal ahead.
    # This makes the planner generate a corridor-following waypoint even without a global map.
    nav_goal_dist_m = 1.2
    scale_est = ScaleEstimator(window=80)

    frame_iter = provider.frames()
    last_time = time.time()
    fps_count = 0
    # settings cache (poll /settings periodically)
    settings_cache = {'smoothing': True}
    settings_last = 0.0

    nav_goal_cache = None
    nav_goal_last = 0.0

    # IMU (MPU-6050 / GY-521)
    imu = None
    imu_last_calib = 0.0

    # Power/current monitor (INA219-style; optional)
    pmon = None
    pmon_last_cfg = None
    pmon_last_read = 0.0

    try:
        for data in frame_iter:
            frame = data["frame"]
            ts = data["timestamp"]

            obstacles = det.detect(frame)
            boundary_info = bounds.detect(frame)
            terrain_info = terrain.analyze(frame)

            perception = {
                "timestamp": ts,
                "obstacles": obstacles,
                "boundary": boundary_info,
                "terrain": terrain_info
            }

            # read ultrasonic when available (non-blocking)
            if us is not None:
                try:
                    d = us.read_distance_cm(samples=3)
                except Exception:
                    d = None
                perception['distance_cm'] = d
            else:
                perception['distance_cm'] = None

            # fetch settings from server at most once per second
            try:
                if time.time() - settings_last > 1.0:
                    r = requests.get('http://127.0.0.1:5000/settings', timeout=0.25)
                    if r.status_code == 200:
                        settings_cache = r.json()
                    settings_last = time.time()
            except Exception:
                pass
            perception['settings'] = settings_cache

            # Power/current sensing (best-effort, optional)
            power = None
            try:
                enabled = bool(settings_cache.get('power_enabled', False))
            except Exception:
                enabled = False
            if enabled and PowerMonitor is not None:
                try:
                    cfg = {
                        'enabled': True,
                        'i2c_bus': int(settings_cache.get('power_i2c_bus', 1)),
                        'motor_left_addr': int(settings_cache.get('power_motor_left_addr', 0x40)),
                        'motor_right_addr': int(settings_cache.get('power_motor_right_addr', 0x43)),
                        'system_addr': int(settings_cache.get('power_system_addr', 0x41)),
                        'motor_left_shunt_ohm': float(settings_cache.get('power_motor_left_shunt_ohm', 0.1)),
                        'motor_right_shunt_ohm': float(settings_cache.get('power_motor_right_shunt_ohm', 0.1)),
                        'system_shunt_ohm': float(settings_cache.get('power_system_shunt_ohm', 0.1)),
                    }
                except Exception:
                    cfg = None

                # (Re)create if needed
                if pmon is None and cfg is not None:
                    try:
                        pmon = PowerMonitor(PowerMonitorSettings(**cfg))
                        pmon_last_cfg = dict(cfg)
                    except Exception:
                        pmon = None
                        pmon_last_cfg = None
                elif pmon is not None and cfg is not None and pmon_last_cfg is not None and cfg != pmon_last_cfg:
                    try:
                        pmon.update_settings(PowerMonitorSettings(**cfg))
                        pmon_last_cfg = dict(cfg)
                    except Exception:
                        pass

                # Read at most at the configured poll rate
                try:
                    poll_s = float(settings_cache.get('power_poll_s', 0.25))
                except Exception:
                    poll_s = 0.25
                poll_s = max(0.05, min(5.0, poll_s))
                if pmon is not None and (time.time() - pmon_last_read) >= poll_s:
                    try:
                        power = pmon.read()
                    except Exception:
                        power = None
                    pmon_last_read = time.time()
            else:
                pmon = None
                pmon_last_cfg = None

            perception['power'] = power

            # fetch navigation goal (if set) at most twice per second
            try:
                if time.time() - nav_goal_last > 0.5:
                    r = requests.get('http://127.0.0.1:5000/nav_goal', timeout=0.25)
                    if r.status_code == 200:
                        nav_goal_cache = (r.json() or {}).get('nav_goal')
                    nav_goal_last = time.time()
            except Exception:
                pass

            # IMU setup/use (best-effort)
            try:
                imu_enabled = bool(settings_cache.get('imu_enabled', True))
            except Exception:
                imu_enabled = True
            if not imu_enabled:
                imu = None
                perception['imu'] = None
            else:
                if imu is None:
                    try:
                        from imu import MPU6050, ImuSettings
                        imu = MPU6050(ImuSettings(
                            i2c_bus=int(settings_cache.get('imu_i2c_bus', 1)),
                            i2c_addr=int(settings_cache.get('imu_i2c_addr', 0x68)),
                            alpha=float(settings_cache.get('imu_alpha', 0.98)),
                            dlpf_cfg=int(settings_cache.get('imu_dlpf_cfg', 3)),
                            sample_div=int(settings_cache.get('imu_sample_div', 4)),
                            gyro_calib_seconds=float(settings_cache.get('imu_gyro_calib_seconds', 1.0)),
                        ))
                        # Calibrate once near startup if available
                        if getattr(imu, 'available', False):
                            imu.calibrate_gyro_bias(seconds=float(settings_cache.get('imu_gyro_calib_seconds', 1.0)))
                            imu_last_calib = time.time()
                        else:
                            imu = None
                    except Exception:
                        imu = None

                if imu is not None:
                    try:
                        # If settings changed significantly, refresh filter alpha
                        try:
                            imu.settings.alpha = float(settings_cache.get('imu_alpha', imu.settings.alpha))
                        except Exception:
                            pass
                        # Optionally re-calibrate gyro bias rarely (e.g. when rover is stationary)
                        if time.time() - imu_last_calib > 60.0:
                            try:
                                imu.calibrate_gyro_bias(seconds=float(settings_cache.get('imu_gyro_calib_seconds', 1.0)))
                            except Exception:
                                pass
                            imu_last_calib = time.time()

                        perception['imu'] = imu.read()
                    except Exception:
                        perception['imu'] = None
                else:
                    perception['imu'] = None
            # update detector tuning live
            try:
                det.settings = settings_cache
            except Exception:
                pass

            # poll SLAM bridge for current pose (if running)
            try:
                import requests as _req
                resp = _req.get('http://127.0.0.1:5001/pose', timeout=0.05)
                if resp.status_code == 200:
                    perception['slam_pose'] = resp.json().get('pose')
                else:
                    perception['slam_pose'] = None
            except Exception:
                perception['slam_pose'] = None

            # Set a short-horizon goal ahead of the rover when SLAM pose is available.
            # This is a local planner use-case (not global maze solving), but enables BFS/A* to
            # route around locally marked obstacles consistently across all autonomous modes.
            try:
                pose = perception.get('slam_pose')
                if isinstance(pose, dict) and 'x' in pose and 'y' in pose:
                    x0 = float(pose['x']); y0 = float(pose.get('y', 0.0))
                    yaw = float(pose.get('yaw', 0.0))
                    explore_enabled = bool(settings_cache.get('nav_explore_enabled', False))
                    # Prefer a user-provided goal if present.
                    if isinstance(nav_goal_cache, dict) and 'x' in nav_goal_cache and 'y' in nav_goal_cache:
                        nav.set_goal(float(nav_goal_cache['x']), float(nav_goal_cache['y']))
                        perception['nav_goal'] = {'x': float(nav_goal_cache['x']), 'y': float(nav_goal_cache['y'])}
                    elif not explore_enabled:
                        nav_goal_dist_m = float(settings_cache.get('nav_goal_dist_m', nav_goal_dist_m))
                        gx = x0 + nav_goal_dist_m * math.cos(yaw)
                        gy = y0 + nav_goal_dist_m * math.sin(yaw)
                        nav.set_goal(gx, gy)
                        perception['nav_goal'] = {'x': gx, 'y': gy}
                    else:
                        # Exploration mode: let Navigator auto-pick a frontier goal.
                        perception['nav_goal'] = None
            except Exception:
                pass

            # build dynamic detections list (estimate world positions from obstacle boxes)
            dyn_dets = []
            try:
                pose = perception.get('slam_pose')
                if pose is not None and 'x' in pose and 'y' in pose:
                    x0 = float(pose['x']); y0 = float(pose.get('y', 0.0))
                    yaw = float(pose.get('yaw', 0.0))
                    for obj in obstacles:
                        box = obj.get('box', [0,0,0,0])
                        ymin = float(box[0])
                        est_dist = 0.3 + (1.0 - min(max(ymin, 0.0), 1.0)) * 3.2
                        center_x = (box[1] + box[3]) / 2.0
                        angle_image = (center_x - 0.5) * (math.radians(70))
                        ox = x0 + est_dist * math.cos(yaw + angle_image)
                        oy = y0 + est_dist * math.sin(yaw + angle_image)
                        dyn_dets.append((ox, oy))
            except Exception:
                dyn_dets = []
            perception['dynamic_detections'] = dyn_dets

            # Provide perception to navigator and get navigation target
            try:
                nav_target = nav.update(perception)
            except Exception:
                nav_target = None

            if nav_target is not None:
                perception['nav_target'] = {'x': nav_target[0], 'y': nav_target[1]}
            else:
                perception['nav_target'] = None

            decision = decider.decide(perception)

            # Recovery-aware exploration: if we trigger recovery while exploring,
            # temporarily blacklist the current explore frontier goal.
            try:
                if bool(settings_cache.get('nav_explore_enabled', False)) and decision.get('recovery'):
                    cd = float(settings_cache.get('nav_explore_blacklist_s', 12.0))
                    # If navigator has an active explore goal cell, blacklist it.
                    eg = getattr(nav, '_explore_goal_cell', None)
                    if eg is not None and hasattr(nav, 'blacklist_explore_goal'):
                        nav.blacklist_explore_goal(eg, cooldown_s=cd)
            except Exception:
                pass

            # execute mapped motor action
            action, params = decider.map_to_motor(decision["command"], speed=decision.get('speed'))
            if hasattr(motor, action):
                try:
                    getattr(motor, action)(**params)
                except TypeError:
                    # method signature mismatch, call without params
                    getattr(motor, action)()

            # overlays and display
            frame_overlay = overlay.apply(frame.copy(), obstacles, boundary_info, terrain_info, decision)

            # compute simple FPS
            fps_count += 1
            if time.time() - last_time >= 1.0:
                fps = fps_count / (time.time() - last_time)
                fps_count = 0
                last_time = time.time()
            else:
                fps = 0.0

            frame_overlay = overlay.draw_fps(frame_overlay, fps)
            frame_overlay = overlay.draw_center_line(frame_overlay)

            cv2.imshow("Rover Vision", cv2.cvtColor(frame_overlay, cv2.COLOR_RGB2BGR))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # send log asynchronously (best-effort)
            log = overlay.export_log(perception, decision)
            send_log_to_server(log)

            # Try to compute a scale sample if slam pose delta and measured movement exists
            try:
                if 'prev_slam_pose' in locals() and perception.get('slam_pose') is not None:
                    prev = locals().get('prev_slam_pose')
                    cur = perception.get('slam_pose')
                    # compute vo delta if both have x,y
                    if isinstance(prev, dict) and isinstance(cur, dict) and 'x' in prev and 'x' in cur:
                        vo_delta = ((cur['x'] - prev['x'])**2 + (cur['y'] - prev['y'])**2)**0.5
                        # get measured delta from ultrasonic if we have a previous reading and motion was forward
                        measured = None
                        if us is not None:
                            # can't measure traveled distance directly without encoder; skip
                            measured = None
                        # only add sample when measured is available (user can extend this using encoders)
                        if measured is not None:
                            scale_est.add_sample(vo_delta, measured)
                    prev_slam_pose = perception.get('slam_pose')
            except Exception:
                pass

    finally:
        provider.release()
        motor.cleanup()
        try:
            if imu is not None:
                imu.close()
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
