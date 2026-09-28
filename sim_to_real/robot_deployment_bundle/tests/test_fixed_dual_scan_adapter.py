from pathlib import Path
import importlib.util
import numpy as np
import pytest


pytest.importorskip('rclpy', reason='ROS 2 Python runtime required')
PATH = Path(__file__).resolve().parents[1] / 'ros2_ws/src/semantic_nav_runtime/scripts/fixed_dual_scan_adapter.py'
SPEC = importlib.util.spec_from_file_location('fixed_dual_scan_adapter', PATH)
MOD = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(MOD)


def test_resampling_keeps_2000_geometrically_aligned_beams():
    ranges, intensity = MOD.adapt_scan_arrays(np.full(2000, 2.0), [], input_angle_min=-np.pi,
        input_angle_increment=2*np.pi/2000, input_range_min=0.02, input_range_max=30.0)
    assert ranges.shape == (2000,) and intensity.shape == (2000,) and np.allclose(ranges, 2.0)


def test_invalid_range_is_inf_not_metadata_only():
    ranges, _ = MOD.adapt_scan_arrays([0.01] * 2000, [], input_angle_min=-np.pi,
        input_angle_increment=2*np.pi/2000, input_range_min=0.01, input_range_max=30.0)
    assert np.isinf(ranges).all()


def test_non_panoramic_input_is_rejected():
    with pytest.raises(ValueError):
        MOD.circular_nearest_indices(20, -1.0, 0.01, 2000, -np.pi, np.pi)
