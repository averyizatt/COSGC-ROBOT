"""Simple SLAM bridge server

This Flask service accepts POSTed pose updates from an external SLAM engine
(or proxies them) and exposes a simple GET `/pose` endpoint other components
can poll.

Usage:
    python3 slam/bridge.py

Endpoints:
- POST /pose  { 'pose': {...}, 'timestamp': 123.4 }
- GET  /pose  -> { 'pose': {...}, 'timestamp': 123.4 }

The SLAM engine should POST pose updates here (or you can configure the
engine to expose a similar API and set SLAM_ENGINE_URL to proxy).
"""

from flask import Flask, request, jsonify
import os
import threading

app = Flask(__name__)

_lock = threading.Lock()
_last_pose = None

SLAM_ENGINE_URL = os.environ.get('SLAM_ENGINE_URL')

@app.route('/pose', methods=['GET', 'POST'])
def pose():
    global _last_pose
    if request.method == 'POST':
        data = request.get_json()
        with _lock:
            _last_pose = data
        return jsonify({'status': 'ok'})
    else:
        with _lock:
            if _last_pose is None:
                return jsonify({}), 204
            return jsonify(_last_pose)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001)
