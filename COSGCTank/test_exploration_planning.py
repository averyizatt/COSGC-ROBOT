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
    nav.global_dyn = np.zeros((nav.global_size, nav.global_size), dtype=np.float32)
    nav.global_origin = (0.0, 0.0)
    nav.global_visit_count = np.zeros((nav.global_size, nav.global_size), dtype=np.uint16)
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

    occ = nav._global_combined_occupancy({'nav_dyn_thresh': 0.4})
    path = nav._bfs_global(start, goal, occ=occ, max_expansions=10, max_time_s=10.0, allow_unknown=False)
    assert path is None, "Expected BFS to abort due to max_expansions"
    assert nav._last_plan_stats.get('aborted') == 'max_expansions'


def test_dynamic_decay_clears_cells():
    nav = _make_nav()
    # mark one dynamic occupied cell
    c = nav._global_world_to_cell(1.0, 0.0)
    assert c is not None
    nav._global_set_dynamic(c[0], c[1], radius_cells=0, strength=1.0)
    # force decay by backdating last timestamp
    nav._global_dyn_last_decay_ts -= 100.0
    nav._decay_global_dynamic({'nav_dyn_half_life_s': 1.0, 'nav_dyn_prune_thresh': 0.2})
    assert float(nav.global_dyn[c[1], c[0]]) == 0.0


def test_goal_lock_holds_until_timeout():
    nav = _make_nav()
    settings = {
        'nav_explore_enabled': True,
        'nav_explore_replan_s': 10.0,
        'nav_observe_radius_m': 1.0,
        'nav_explore_frontier_max_scan': 5000,
        'nav_explore_frontier_candidates': 50,
        'nav_explore_goal_reached_m': 0.1,
        'nav_explore_progress_timeout_s': 10.0,
        'nav_explore_progress_min_delta_m': 999.0,
        'nav_explore_visit_weight': 0.0,
        'nav_plan_max_expansions': 2000,
        'nav_plan_max_time_s': 1.0,
        'nav_planner': 'bfs',
        'nav_dyn_half_life_s': 6.0,
        'nav_dyn_thresh': 0.4,
        'nav_dyn_prune_thresh': 0.05,
    }
    perception = {
        'slam_pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        'settings': settings,
        'obstacles': [],
        'dynamic_detections': [],
        'distance_cm': None,
        'nav_goal': None,
    }
    nav.update(perception)
    g1 = nav._explore_goal_cell
    assert g1 is not None
    # Immediately call again; should keep goal due to long replan_s
    nav.update(perception)
    assert nav._explore_goal_cell == g1


if __name__ == '__main__':
    test_frontier_selected_when_no_goal()
    test_planner_budget_aborts_fast()
    test_dynamic_decay_clears_cells()
    test_goal_lock_holds_until_timeout()
    print('OK')
