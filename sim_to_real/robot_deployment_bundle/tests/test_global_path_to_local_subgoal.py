from pathlib import Path
import sys
import numpy as np
import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / 'ros2_ws/src/semantic_nav_runtime/scripts'
sys.path.insert(0, str(SCRIPTS))
from path_subgoal_core import choose_lookahead, map_to_base


def test_straight_path():
    index, point = choose_lookahead([[0, 0], [1, 0], [2, 0]], [0.2, 0], 1.2)
    assert index == 0 and np.allclose(point, [1.2, 0])


def test_corner_path_accumulates_arc_length():
    _, point = choose_lookahead([[0, 0], [1, 0], [1, 2]], [0, 0], 1.5)
    assert np.allclose(point, [1, 0.5])


def test_short_path_uses_final_point():
    _, point = choose_lookahead([[0, 0], [0.4, 0]], [0, 0], 1.2)
    assert np.allclose(point, [0.4, 0])


def test_robot_in_middle_restarts_nearest_search():
    index, point = choose_lookahead([[0, 0], [1, 0], [2, 0], [3, 0]], [2.1, 0], 0.5)
    assert index == 2 and np.allclose(point, [2.5, 0])


def test_empty_and_invalid_paths_rejected():
    with pytest.raises(ValueError): choose_lookahead([], [0, 0], 1.2)
    with pytest.raises(ValueError): choose_lookahead([[0, float('nan')]], [0, 0], 1.2)


def test_rotation_map_to_base():
    assert np.allclose(map_to_base([1, 0], [0, 0, np.pi / 2]), [0, -1], atol=1e-8)


def test_path_refresh_has_no_stale_index():
    _, old = choose_lookahead([[0, 0], [10, 0], [11, 0]], [9, 0], 0.5)
    _, new = choose_lookahead([[0, 0], [1, 0]], [0, 0], 0.5)
    assert np.allclose(old, [10.5, 0]) and np.allclose(new, [0.5, 0])


def test_node_declares_path_timeout():
    text = (SCRIPTS / 'global_path_to_local_subgoal.py').read_text()
    assert "'path_timeout'" in text and 'path timeout' in text
