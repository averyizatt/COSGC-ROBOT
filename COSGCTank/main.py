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
        us = Ultrasonic()
    except Exception:
        us = None
    from navigator import Navigator
    from scale_estimator import ScaleEstimator
    nav = Navigator(grid_size=40, resolution=0.2)
    scale_est = ScaleEstimator(window=80)

    frame_iter = provider.frames()
    last_time = time.time()
    fps_count = 0
    # settings cache (poll /settings periodically)
    settings_cache = {'smoothing': True}
    settings_last = 0.0

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
                    d = us.read_distance()
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
            # update detector tuning live
            try:
                det.settings = settings_cache
            except Exception:
                pass

            decision = decider.decide(perception)

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

            # If the navigation module returned a target, expose in decision dict
            if nav_target is not None:
                # include navigation suggestion for downstream use
                decision['nav_target'] = {'x': nav_target[0], 'y': nav_target[1]}

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
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
