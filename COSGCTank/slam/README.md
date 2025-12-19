# SLAM Integration — COSGCTank

This folder contains scaffolding and instructions for integrating a full SLAM
solution (OpenVSLAM / ORB-SLAM3 / RTAB-Map) with the COSGCTank codebase.

Goal

Run a SLAM engine as a separate process (native C++ recommended), expose a
simple HTTP/JSON bridge that publishes pose and map info, and consume the
pose from `main.py` for navigation and mapping.

Why separate process?
- SLAM engines (ORB-SLAM3, OpenVSLAM, RTAB-Map) are heavy and often C++.
- Running them as separate processes keeps the Python control loop responsive
  and lets you run the SLAM engine on a companion computer if necessary.

Overview of files
- `bridge.py` — small Flask service that accepts POSTed pose updates from a
  SLAM engine or proxies an engine's HTTP API. Exposes `GET /pose` and
  `POST /pose` for simple integration.
- `camera_calibration.py` — OpenCV chessboard calibrator to produce camera
  intrinsics YAML used by SLAM engines.
- `launch_openvslam.sh` — example script showing how to launch the OpenVSLAM
  binary with a config file and the bridge endpoint. (You must install
  OpenVSLAM separately.)

Recommended SLAM engines
- OpenVSLAM (https://openvslam.readthedocs.io/)
  - Good cross-platform project, supports various vocabularies and cameras.
  - Build instructions provided upstream; requires dependencies (Eigen, OpenCV).
- ORB-SLAM3 (https://github.com/UZ-SLAMLab/ORB_SLAM3)
  - Full-featured SLAM with monocular, stereo, RGB-D, IMU support.
  - More complex to build, but offers loop closure, relocalization.
- RTAB-Map
  - Best if you have an RGB-D sensor; includes mapping/occupancy grid features.

Integration pattern (recommended)
1. Calibrate camera to obtain intrinsics (use `camera_calibration.py`).
2. Build and run SLAM engine on the Pi or companion computer with your
   camera intrinsics and model.
3. Configure the SLAM engine to publish pose updates (OpenVSLAM can run map
   server or custom publisher; ORB-SLAM3 can be extended similarly).
4. Run `bridge.py` (or have the SLAM engine POST directly to the bridge) so
   the rest of the Python stack can `GET /pose`.
5. Update `main.py` to poll `/pose` each loop and add `perception['slam_pose']`.

Notes on pose format
- Pose should be provided as a 4x4 transform or as `{'x':..., 'y':..., 'z':..., 'qx':..., 'qy':..., 'qz':..., 'qw':...}` (position + quaternion). For 2D navigation you can use `(x, y, yaw)` in meters and radians.

Camera calibration
- SLAM engines require accurate intrinsics. Use `camera_calibration.py` to acquire `camera.yaml` (intrinsics + distortion). Place this file in the SLAM config and in the `models/` folder as needed.

Example workflow (OpenVSLAM)
1. Calibrate camera:
   ```bash
   python3 slam/camera_calibration.py --output slam/camera.yaml
   ```
2. Build OpenVSLAM according to its docs and prepare a config file that
   references `slam/camera.yaml`.
3. Launch OpenVSLAM pointing at camera device and map saving path.
4. Either have OpenVSLAM POST pose to `http://localhost:5001/pose` or run
   `bridge.py` and configure the engine to use its web API and the bridge.
5. Run your control loop in `main.py` — it will poll `http://localhost:5001/pose`.
