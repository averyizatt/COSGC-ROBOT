"""Offline test for Navigator global map persistence.

Run:
  python3 COSGC-ROBOT/COSGCTank/test_nav_persistence.py

This test avoids any Pi hardware; it only exercises Navigator save/load.
"""

import os
import tempfile


def test_save_and_load_roundtrip():
    from navigator import Navigator
    import numpy as np

    nav = Navigator(grid_size=20, resolution=0.2)
    nav.global_enabled = True
    nav.global_size = 40
    nav.global_half = nav.global_size // 2

    # Re-init arrays to match new size
    nav.global_occupancy = np.zeros((nav.global_size, nav.global_size), dtype=np.uint8)
    nav.global_seen = np.zeros((nav.global_size, nav.global_size), dtype=np.uint8)
    nav.global_dyn = np.zeros((nav.global_size, nav.global_size), dtype=np.float32)
    nav.global_visit_count = np.zeros((nav.global_size, nav.global_size), dtype=np.uint16)
    nav.global_origin = (1.25, -0.75)

    # Seed distinctive data
    nav.global_occupancy[3, 7] = 1
    nav.global_seen[0:5, 0:6] = 1
    nav.global_dyn[10, 11] = 0.9
    nav.global_visit_count[2, 2] = 42

    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, 'map.npz')
        nav.save_global_map(path)

        nav2 = Navigator(grid_size=20, resolution=0.2)
        nav2.global_enabled = True
        nav2.global_size = 40
        nav2.global_half = nav2.global_size // 2
        nav2.global_res = nav2.resolution
        nav2.global_occupancy = np.zeros((nav2.global_size, nav2.global_size), dtype=np.uint8)
        nav2.global_seen = np.zeros((nav2.global_size, nav2.global_size), dtype=np.uint8)
        nav2.global_dyn = np.zeros((nav2.global_size, nav2.global_size), dtype=np.float32)
        nav2.global_visit_count = np.zeros((nav2.global_size, nav2.global_size), dtype=np.uint16)

        ok = nav2.load_global_map(path)
        assert ok is True

        assert nav2.global_origin == nav.global_origin
        assert np.array_equal(nav2.global_occupancy, nav.global_occupancy)
        assert np.array_equal(nav2.global_seen, nav.global_seen)
        assert np.allclose(nav2.global_dyn, nav.global_dyn)
        assert np.array_equal(nav2.global_visit_count, nav.global_visit_count)


def test_atomic_write_no_tmp_left():
    from navigator import Navigator

    nav = Navigator(grid_size=20, resolution=0.2)

    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, 'map.npz')
        nav.save_global_map(path)
        assert os.path.exists(path)
        assert not os.path.exists(path + '.tmp')


if __name__ == '__main__':
    test_save_and_load_roundtrip()
    test_atomic_write_no_tmp_left()
    print('OK')
