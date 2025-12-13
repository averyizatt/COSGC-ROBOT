import cv2
import numpy as np

class OverlayDrawer:
    def __init__(self):
        pass

    def draw_obstacles(self, frame, obstacles):
        h, w, _ = frame.shape
        for obj in obstacles:
            ymin, xmin, ymax, xmax = obj["box"]
            ymin, xmin, ymax, xmax = int(ymin*h), int(xmin*w), int(ymax*h), int(xmax*w)

            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            cv2.putText(frame, f"{obj['class']} {obj['score']:.2f}",
                        (xmin, ymin - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
        return frame

    def draw_boundaries(self, frame, boundary):
        for line in boundary["left_boundary"]:
            x1, y1, x2, y2 = line
            cv2.line(frame, (x1,y1), (x2,y2), (255,0,0), 2)
        
        for line in boundary["right_boundary"]:
            x1, y1, x2, y2 = line
            cv2.line(frame, (x1,y1), (x2,y2), (0,0,255), 2)

        return frame

    def draw_terrain(self, frame, terrain):
        text = f"Dip: {terrain['dip_detected']}  Incline: {terrain['incline_detected']}"
        cv2.putText(frame, text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)
        return frame

    def draw_decision(self, frame, decision):
        text = f"Decision: {decision['command']}"
        cv2.putText(frame, text, (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
        return frame

    def draw_center_line(self, frame, color=(200,200,200)):
        h, w, _ = frame.shape
        cv2.line(frame, (w//2, 0), (w//2, h), color, 1)
        return frame

    def draw_fps(self, frame, fps):
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
        return frame

    def export_log(self, perception, decision):
        """Return a JSON-serializable dict combining perception and decision for logging."""
        log = {
            'timestamp': float(perception.get('timestamp', 0.0)),
            'decision': decision,
            'obstacles': perception.get('obstacles', []),
            'boundary': perception.get('boundary', {}),
            'terrain': perception.get('terrain', {})
        }
        return log

    def apply(self, frame, obstacles, boundary, terrain, decision):
        frame = self.draw_obstacles(frame, obstacles)
        frame = self.draw_boundaries(frame, boundary)
        frame = self.draw_terrain(frame, terrain)
        frame = self.draw_decision(frame, decision)
        return frame
