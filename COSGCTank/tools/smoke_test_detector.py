import json
import os
import sys
import numpy as np
import cv2

# Ensure COSGCTank root is on sys.path for package imports
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from slam.detector import ObstacleDetector


def synthetic_frame(w=640, h=360):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    # sky (top): brighter blue to ensure ROI filter keeps lower region
    img[:h//2] = (200, 200, 255)
    # ground (bottom): sandy-ish
    img[h//2:] = (180, 180, 160)
    # add a few dark "rocks" in lower band
    cv2.circle(img, (int(w*0.3), int(h*0.75)), 35, (30, 30, 30), -1)
    cv2.rectangle(img, (int(w*0.65)-30, int(h*0.7)-25), (int(w*0.65)+30, int(h*0.7)+25), (40,40,40), -1)
    # convert to RGB for detector
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


def main():
    settings = {
        'det_lower_roi_fraction': 0.55,
        'det_nms_iou': 0.4,
        'det_temporal_smoothing': True,
        'det_min_persistence': 1,  # one-shot for smoke
        'det_contour_min_area': 200,
        'det_rock_score_threshold': 0.4
    }
    det = ObstacleDetector(settings=settings)
    frame = synthetic_frame()
    dets = det.detect(frame)
    print(json.dumps({
        'count': len(dets),
        'detections': dets[:5]
    }, indent=2))


if __name__ == '__main__':
    main()
