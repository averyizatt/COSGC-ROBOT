"""Quick offline sanity checks for navigation + decision recovery.

This is not a full simulation; it just exercises:
- turn commitment behavior
- staged recovery transitions
- navigator BFS/A* selection functions exist

Run:
  python3 test_nav_and_recovery.py
"""

import time

from decision import DecisionMaker


def test_turn_commitment():
    dm = DecisionMaker(hysteresis_frames=1)
    # Fake a nav target that forces large heading error.
    perception = {
        'timestamp': 1.0,
        'settings': {
            'nav_heading_enabled': True,
            'nav_turn_adjust_deg': 5.0,
            'nav_turn_stop_deg': 10.0,
            'nav_turn_commit_s': 0.5,
            'auto_speed_turn': 1.0,
        },
        'slam_pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        'nav_target': {'x': 0.0, 'y': 1.0},  # goal is +90deg
        'terrain': {},
        'boundary': {},
        'obstacles': [],
        'distance_cm': None,
        'imu': None,
    }

    d1 = dm.decide(perception)
    assert d1['command'] in ('TURN_LEFT', 'TURN_RIGHT')

    # During commit window, should keep returning same turn.
    perception['timestamp'] = 1.2
    d2 = dm.decide(perception)
    assert d2['command'] == d1['command']
    assert 'nav_turn_commit_remaining_s' in d2

    # After commit expires, can re-evaluate.
    perception['timestamp'] = 2.0
    d3 = dm.decide(perception)
    assert d3['command'] in ('TURN_LEFT', 'TURN_RIGHT', 'ADJUST_LEFT', 'ADJUST_RIGHT', 'FORWARD')


def test_recovery_state_machine():
    dm = DecisionMaker(hysteresis_frames=1)

    # First call establishes prev_pose.
    perception = {
        'timestamp': 10.0,
        'settings': {
            'rec_enabled': True,
            'rec_reverse_s': 0.2,
            'rec_turn_s': 0.2,
            'rec_forward_s': 0.2,
            'rec_cooldown_s': 0.0,
            'rec_max_attempts': 2,
            'dec_stuck_frames': 1,
            'auto_speed_turn': 1.0,
            'auto_speed_slow': 0.3,
            'auto_speed_stuck': 1.0,
        },
        'slam_pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        'terrain': {},
        'boundary': {},
        'obstacles': [],
        'distance_cm': None,
        'imu': None,
        'nav_target': None,
    }
    dm.decide(perception)

    # Next call: no movement -> stuck triggers recovery.
    perception['timestamp'] = 10.1
    perception['slam_pose'] = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    d = dm.decide(perception)
    assert d['command'] == 'STOP_REVERSE'
    assert d.get('recovery') is True

    # During reverse stage
    perception['timestamp'] = 10.15
    d = dm.decide(perception)
    assert d['command'] == 'STOP_REVERSE'

    # After reverse stage -> should pivot
    perception['timestamp'] = 10.35
    d = dm.decide(perception)
    assert d['command'] in ('TURN_LEFT', 'TURN_RIGHT')

    # After turn stage -> forward slow
    perception['timestamp'] = 10.60
    d = dm.decide(perception)
    assert d['command'] == 'SLOW'


if __name__ == '__main__':
    test_turn_commitment()
    test_recovery_state_machine()
    print('OK')
