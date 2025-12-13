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
    det = ObstacleDetector()
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

    frame_iter = provider.frames()
    last_time = time.time()
    fps_count = 0

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

            decision = decider.decide(perception)

            # execute mapped motor action
            action, params = decider.map_to_motor(decision["command"])
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

    finally:
        provider.release()
        motor.cleanup()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
