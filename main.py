from camera import Camera
from detector import ObstacleDetector
from boundaries import BoundaryDetector
from terrain import TerrainAnalyzer
from decision import DecisionMaker

import cv2
import time

def main():
    print("Starting rover perception system...")
    
    # Initialize modules
    cam = Camera()
    det = ObstacleDetector()
    bounds = BoundaryDetector()
    terrain = TerrainAnalyzer()
    decider = DecisionMaker()

    time.sleep(1)   # give camera time to warm up

    while True:
        frame = cam.get_frame()

        # -------- PERCEPTION -------- #
        obstacles = det.detect(frame)
        boundary_info = bounds.detect(frame)
        terrain_info = terrain.analyze(frame)

        perception = {
            "obstacles": obstacles,
            "boundary": boundary_info,
            "terrain": terrain_info
        }

        # -------- DECISION MAKING -------- #
        decision = decider.decide(perception)

        # -------- OUTPUT / DEBUG -------- #
        print("\n=== Frame Perception ===")
        print(f"Obstacles: {len(obstacles)} detected")
        print(f"Left boundary lines: {len(boundary_info['left_boundary'])}")
        print(f"Right boundary lines: {len(boundary_info['right_boundary'])}")
        print(f"Dip: {terrain_info['dip_detected']}  Incline: {terrain_info['incline_detected']}")

        print("\n=== Decision ===")
        print(f"Command: {decision['command']} | Reason: {decision['reason']}")

        # Here is where you'd actually SEND the command to the rover:
        # send_to_rover(decision["command"])

        # Control loop speed (20 Hz-ish)
        time.sleep(0.05)

if __name__ == '__main__':
    main()
