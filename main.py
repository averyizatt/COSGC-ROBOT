from camera import Camera
from detector import ObstacleDetector
from boundaries import BoundaryDetector
from terrain import TerrainAnalyzer
import cv2
import time

def main():
    cam = Camera()
    det = ObstacleDetector()
    bounds = BoundaryDetector()
    terrain = TerrainAnalyzer()

    while True:
        frame = cam.get_frame()

        obstacles = det.detect(frame)
        boundary = bounds.detect(frame)
        terrain_info = terrain.analyze(frame)

        print("\n=== Frame Results ===")
        print("Obstacles:", obstacles)
        print("Boundaries:", boundary)
        print("Terrain:", terrain_info)

        time.sleep(0.05)

if __name__ == '__main__':
    main()
