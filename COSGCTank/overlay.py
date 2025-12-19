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

            label = obj.get('label', str(obj.get('class')))
            score = obj.get('score', 0.0)
            rock = obj.get('rock_score', None)
            # contour detections in orange; model detections in green
            color = (0, 200, 255) if label == 'contour' else (0, 255, 0)
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
            if rock is None:
                txt = f"{label} {score:.2f}"
            else:
                txt = f"{label} {score:.2f} rock={float(rock):.2f}"
            cv2.putText(frame, txt,
                        (xmin, ymin - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
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
        dip = terrain.get('dip_detected')
        inc = terrain.get('incline_detected')
        rough = terrain.get('variance', None)
        if rough is None:
            text = f"Dip: {dip}  Incline: {inc}"
        else:
            text = f"Dip: {dip}  Incline: {inc}  Rough: {rough:.1f}"
        cv2.putText(frame, text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)
        return frame

    def draw_decision(self, frame, decision):
        text = f"Decision: {decision.get('command')}"
        cv2.putText(frame, text, (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
        # stuck indicator
        if decision.get('stuck'):
            cv2.putText(frame, "STUCK: escape", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
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
            'perception': {
                'slam_pose': perception.get('slam_pose'),
                'nav_goal': perception.get('nav_goal'),
                'nav_target': perception.get('nav_target'),
                'nav_debug': perception.get('nav_debug'),
                'imu': perception.get('imu'),
                'distance_cm': perception.get('distance_cm'),
            },
            'obstacles': perception.get('obstacles', []),
            'boundary': perception.get('boundary', {}),
            'terrain': perception.get('terrain', {})
        }
        return log

    def draw_slam_map(self, frame, trajectory, occupancy, origin, resolution=0.2, map_size=200):
        """Draw a small top-down map in the top-right corner of `frame`.

        Args:
            frame: RGB image (H,W,3)
            trajectory: list of (x,y) tuples
            occupancy: 2D numpy array (rows,cols) with 0 free, 1 occupied
            origin: world (x0,y0) center for occupancy grid
            resolution: meters per grid cell
            map_size: pixel size of the overlay square
        """
        import cv2
        import numpy as np

        h, w, _ = frame.shape
        overlay = np.zeros((map_size, map_size, 3), dtype=np.uint8)
        overlay[:] = (30, 30, 30)

        rows, cols = occupancy.shape
        cell_h = map_size / rows
        cell_w = map_size / cols

        # draw occupancy
        for r in range(rows):
            for c in range(cols):
                if occupancy[r, c] == 1:
                    y0 = int(r * cell_h)
                    x0 = int(c * cell_w)
                    cv2.rectangle(overlay, (x0, y0), (int((c+1)*cell_w), int((r+1)*cell_h)), (0,0,255), -1)

        # draw trajectory (map coords -> cell indices)
        if trajectory:
            # compute transform from world to grid center (origin is center)
            cx = cols // 2
            cy = rows // 2
            pts = []
            for (x,y) in trajectory:
                dx = (x - origin[0]) / resolution
                dy = (y - origin[1]) / resolution
                cell_x = int(round(cx + dx))
                cell_y = int(round(cy + dy))
                px = int(cell_x * cell_w)
                py = int(cell_y * cell_h)
                pts.append((px, py))
            for i in range(1, len(pts)):
                cv2.line(overlay, pts[i-1], pts[i], (0,255,0), 1)

        # place overlay in top-right
        x_off = w - map_size - 10
        y_off = 10
        frame[y_off:y_off+map_size, x_off:x_off+map_size] = cv2.addWeighted(frame[y_off:y_off+map_size, x_off:x_off+map_size], 0.2, cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR), 0.8, 0)
        return frame

    def apply(self, frame, obstacles, boundary, terrain, decision):
        frame = self.draw_obstacles(frame, obstacles)
        frame = self.draw_boundaries(frame, boundary)
        frame = self.draw_terrain(frame, terrain)
        frame = self.draw_decision(frame, decision)
        return frame
