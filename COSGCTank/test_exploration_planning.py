"""Offline sanity tests for global exploration + planning budgets.

Run:
  python3 COSGC-ROBOT/COSGCTank/test_exploration_planning.py

These tests avoid any Pi hardware and only exercise Navigator logic.
"""

from navigator import Navigator


def _make_nav():
    nav = Navigator(grid_size=20, resolution=0.2)
    nav.global_enabled = True
    nav.global_size = 60
    nav.global_half = nav.global_size // 2

    # re-init arrays to match new size
    import numpy as np

    nav.global_occupancy = np.zeros((nav.global_size, nav.global_size), dtype=np.uint8)
    nav.global_seen = np.zeros((nav.global_size, nav.global_size), dtype=np.uint8)
    nav.global_origin = (0.0, 0.0)
    return nav


def test_frontier_selected_when_no_goal():
    nav = _make_nav()

    settings = {
        'nav_explore_enabled': True,
        'nav_explore_replan_s': 0.0,
        'nav_observe_radius_m': 1.0,
        'nav_explore_frontier_max_scan': 5000,
        'nav_explore_frontier_candidates': 50,
        'nav_plan_max_expansions': 2000,
        'nav_plan_max_time_s': 1.0,
        'nav_planner': 'bfs',
    }

    # Put robot at origin; mark a small observed disc.
    perception = {
        'slam_pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        'settings': settings,
        'obstacles': [],
        'dynamic_detections': [],
        'distance_cm': None,
        'nav_goal': None,
    }

    tgt = nav.update(perception)
    # We may or may not get an immediate waypoint (depends on whether frontier is within observed
    # and path exists), but we should have selected an explore goal cell in most cases.
    assert nav._explore_goal_cell is not None, "Expected an exploration frontier goal cell"


def test_planner_budget_aborts_fast():
    nav = _make_nav()

    # Mark start/goal as seen, but make a huge open seen area and very low max expansions.
    start = nav._global_world_to_cell(0.0, 0.0)
    goal = nav._global_world_to_cell(4.0, 4.0)
    assert start is not None and goal is not None

    # Mark everything as seen-free to encourage large expansions
    nav.global_seen[:, :] = 1

    path = nav._bfs_global(start, goal, max_expansions=10, max_time_s=10.0, allow_unknown=False)
    assert path is None, "Expected BFS to abort due to max_expansions"
    assert nav._last_plan_stats.get('aborted') == 'max_expansions'


if __name__ == '__main__':
    test_frontier_selected_when_no_goal()
    test_planner_budget_aborts_fast()
    print('OK')
