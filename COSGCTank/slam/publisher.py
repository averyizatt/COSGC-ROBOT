"""Generic SLAM publisher that reads pose lines and POSTs to the bridge.

This script can be used to translate SLAM engine console output into HTTP
POSTs to the local bridge (`/pose`). It is intentionally generic and uses a
regular expression to extract pose numbers from each line.

Usage examples:
  # pipe ORB-SLAM3 stdout to the publisher
  ./orb_slam3 ... | python3 slam/publisher.py --pattern "POSE: ([0-9eE+\-. ]+)"

  # read from a file
  python3 slam/publisher.py --input slam/poses.txt --pattern "POSE: ([0-9eE+\-. ]+)"

The captured group should contain space-separated values that represent either
(x y z qw qx qy qz) or (tx ty tz rx ry rz qw) depending on your engine. The
publisher will attempt to parse floats and post a JSON payload
{"pose": {x,y,z,qx,qy,qz,qw}, "timestamp": ...} to the bridge at
http://127.0.0.1:5001/pose
"""

import re
import sys
import time
import json
import argparse
import requests

parser = argparse.ArgumentParser()
parser.add_argument('--input', default=None, help='Input file (default: stdin)')
parser.add_argument('--pattern', required=True, help='Regex pattern with one capturing group containing space-separated numbers')
parser.add_argument('--url', default='http://127.0.0.1:5001/pose', help='bridge URL')
parser.add_argument('--interval', type=float, default=0.0, help='sleep between POSTs')
args = parser.parse_args()

PAT = re.compile(args.pattern)

def parse_numbers(s):
    parts = s.strip().split()
    nums = []
    for p in parts:
        try:
            nums.append(float(p))
        except Exception:
            pass
    return nums


def build_pose_from_numbers(nums):
    # Try to infer format. If 7 numbers assume x y z qx qy qz qw or x y z qw qx qy qz
    if len(nums) >= 7:
        # Heuristic: if last number's abs>1 assume quaternion last? fallback mapping below
        # We'll assume format x y z qx qy qz qw
        x, y, z, qx, qy, qz, qw = nums[:7]
        return {'x': x, 'y': y, 'z': z, 'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw}
    # If 3 numbers, return x,y,z only
    if len(nums) >= 3:
        x, y, z = nums[:3]
        return {'x': x, 'y': y, 'z': z}
    return None


def post_pose(pose):
    payload = {'pose': pose, 'timestamp': time.time()}
    try:
        resp = requests.post(args.url, json=payload, timeout=0.5)
        if resp.status_code != 200:
            print('Bridge returned', resp.status_code)
    except Exception as e:
        print('Error posting pose:', e)


def run():
    if args.input:
        fh = open(args.input, 'r')
    else:
        fh = sys.stdin

    while True:
        line = fh.readline()
        if not line:
            if args.input:
                break
            time.sleep(0.01)
            continue
        m = PAT.search(line)
        if not m:
            continue
        group = m.group(1)
        nums = parse_numbers(group)
        pose = build_pose_from_numbers(nums)
        if pose is not None:
            post_pose(pose)
            if args.interval > 0:
                time.sleep(args.interval)

if __name__ == '__main__':
    run()
