from camera import Camera
from detector import ObstacleDetector
from boundaries import BoundaryDetector
from terrain import TerrainAnalyzer
from decision import DecisionMaker
from overlay import OverlayDrawer

import cv2
import time
import requests
import json

SERVER_URL = "http://192.168.0.41:5000/log"   # <-- your Flask server

def send_log_to_server(perception, decision):
    data = {
        "obstacles": perception["obstacles"],
        "boundary": perception["boundary"],
        "terrain": perception["terrain"],
        "decision": decision
    }

    try:
        requests.post(SERVER_URL, json=data, timeout=0.2)
    except Exception as e:
        print("Could not send log:", e)

def main():
    print("Starting rover perception system...")

    cam = Camera()
    det = ObstacleDetector()
    bounds = BoundaryDetector()
    terrain = TerrainAnalyzer()
    decider = DecisionMaker()
    overlay = OverlayDrawer()

    time.sleep(1)

    while True:
        frame = cam.get_frame()

        obstacles = det.detect(frame)
        boundary_info = bounds.detect(frame)
        terrain_info = terrain.analyze(frame)

        perception = {
            "obstacles": obstacles,
            "boundary": boundary_info,
            "terrain": terrain_info
        }

        decision = decider.decide(perception)

        # Apply overlays
        frame_overlay = overlay.apply(frame.copy(), obstacles, boundary_info, terrain_info, decision)

        # Show in window
        cv2.imshow("Rover Vision", frame_overlay)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Send logs to your Flask server
        send_log_to_server(perception, decision)

        time.sleep(0.02)

if __name__ == '__main__':
    main()
