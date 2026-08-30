import importlib.util
import math
from pathlib import Path

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "calf_policy_node.py"
SPEC = importlib.util.spec_from_file_location("calf_policy_node", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_dual_native_points_project_directly_to_uniform_calf_ring():
    identity = (0.0, 0.0, 0.0, 1.0)
    first = MODULE.scan_points_in_base(
        [2.0], 0.0, 1.0, 0.1, 50.0, (0.5, 0.0, 0.0), identity,
        self_filter=(10.0, 11.0, 10.0, 11.0),
    )
    second = MODULE.scan_points_in_base(
        [1.0], 0.0, 1.0, 0.1, 50.0, (0.5, 0.0, 0.0), identity,
        self_filter=(10.0, 11.0, 10.0, 11.0),
    )
    scan = MODULE.uniform_calf_scan(np.concatenate((first, second), axis=0))
    forward = int(round(math.pi / MODULE.TARGET_INCREMENT))
    assert scan.shape == (216,)
    assert np.isclose(scan[forward], 1.5)
    assert np.count_nonzero(scan < MODULE.MAX_LIDAR_DIST) == 1


def test_no_return_and_normalization_match_training_contract():
    scan = MODULE.uniform_calf_scan(np.empty((0, 3), dtype=np.float32))
    normalized = MODULE.normalize_calf_scan(scan)
    assert np.all(scan == MODULE.MAX_LIDAR_DIST)
    assert np.all(normalized == 0.0)
    near = MODULE.normalize_calf_scan(np.asarray([MODULE.ROBOT_RADIUS]))
    assert np.isclose(near[0], 1.0)


def test_observation_uses_oldest_to_newest_strided_history():
    goal, kin = MODULE.goal_and_kinematics((2.0, 1.0), 0.2, -0.1, 0.8)
    lidar_buffer = [
        np.full(MODULE.NUM_RAYS, index / 10.0, dtype=np.float32)
        for index in range(MODULE.BUFFER_LEN)
    ]
    goal_buffer = [goal + index for index in range(MODULE.BUFFER_LEN)]
    pose_buffer = [
        np.asarray([float(index), 0.0, 0.0], dtype=np.float32)
        for index in range(MODULE.BUFFER_LEN)
    ]
    lidar, goals, ego, observation = MODULE.observation_components(
        kin, goal_buffer, pose_buffer, lidar_buffer
    )
    assert lidar.shape == (3, 216)
    assert goals.shape == (3, 2)
    assert ego.shape == (3, 3)
    assert observation.shape == (668,)
    assert np.allclose(lidar[:, 0], [0.0, 0.2, 0.4])
    assert np.allclose(ego[:, 0], [-4.0, -2.0, 0.0])
    assert np.isfinite(observation).all()


def test_current_ppo_checkpoint_is_shape_exact_and_finite():
    checkpoint = (
        Path(__file__).resolve().parents[5]
        / "github_src/drl_vo_nav-drl_vo/LegNav-Sim-master/checkpoints/ppo/ppo_legs_best.msgpack"
    )
    infer, count = MODULE.load_calf_ppo(checkpoint, 0.8)
    action = np.asarray(infer(np.zeros(MODULE.OBS_DIM, dtype=np.float32)))
    assert count > 0
    assert action.shape == (2,)
    assert np.isfinite(action).all()
