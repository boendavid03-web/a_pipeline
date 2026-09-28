import importlib.util
import math
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).parents[1] / "scripts" / "fixed_dual_scan_adapter.py"
SPEC = importlib.util.spec_from_file_location("fixed_dual_scan_adapter", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_real_2000_beam_grid_is_mapped_to_fixed_contract():
    ranges = np.arange(2000, dtype=np.float32) / 100.0
    intensities = np.arange(2000, dtype=np.float32) % 256
    output_ranges, output_intensities = MODULE.adapt_scan_arrays(
        ranges,
        intensities,
        input_angle_min=-math.pi,
        input_angle_increment=2.0 * math.pi / 2000.0,
        input_range_min=0.1,
        input_range_max=50.0,
    )
    assert output_ranges.shape == (2000,)
    assert output_intensities.shape == (2000,)
    assert output_ranges[-1] == output_ranges[0]
    assert output_intensities[-1] == output_intensities[0]
    assert np.isinf(output_ranges[ranges[:1999] > 8.0]).all()


def test_invalid_and_out_of_contract_ranges_become_infinite():
    ranges = np.full(2000, 2.0, dtype=np.float32)
    ranges[0] = np.nan
    ranges[1] = 0.05
    ranges[2] = 9.0
    output_ranges, _ = MODULE.adapt_scan_arrays(
        ranges,
        [],
        input_angle_min=-math.pi,
        input_angle_increment=2.0 * math.pi / 2000.0,
        input_range_min=0.1,
        input_range_max=50.0,
    )
    assert np.isinf(output_ranges[0])
    assert np.isinf(output_ranges[1])
    assert np.isinf(output_ranges[2])
    assert output_ranges[3] == 2.0


def test_non_panoramic_input_is_rejected():
    with np.testing.assert_raises_regex(ValueError, "not panoramic"):
        MODULE.adapt_scan_arrays(
            np.ones(1000, dtype=np.float32),
            [],
            input_angle_min=-math.pi / 2.0,
            input_angle_increment=math.pi / 999.0,
            input_range_min=0.1,
            input_range_max=30.0,
        )
