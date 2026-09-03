import struct
import sys
from pathlib import Path

import pytest
from sensor_msgs.msg import PointCloud2, PointField


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from pedestrian_perception_visualizer import read_scored_points  # noqa: E402


def scored_cloud(values):
    message = PointCloud2()
    message.header.frame_id = "base_link"
    message.header.stamp.sec = 3
    message.header.stamp.nanosec = 40
    message.height = 1
    message.width = len(values)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="confidence", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.point_step = 12
    message.row_step = 12 * message.width
    message.is_dense = True
    message.data = b"".join(struct.pack("<fff", *item) for item in values)
    return message


def test_read_scored_points_preserves_xy_and_confidence():
    message = scored_cloud([(1.25, -0.5, 0.97), (2.0, 0.25, 0.81)])
    actual = read_scored_points(message)
    expected = [(1.25, -0.5, 0.97), (2.0, 0.25, 0.81)]
    assert len(actual) == len(expected)
    for actual_point, expected_point in zip(actual, expected):
        assert actual_point == pytest.approx(expected_point)


def test_read_scored_points_ignores_nonfinite_detection():
    message = scored_cloud([(1.0, 2.0, 0.95), (float("nan"), 0.0, 0.9)])
    assert read_scored_points(message)[0] == pytest.approx((1.0, 2.0, 0.95))


def test_read_scored_points_requires_project_contract():
    message = scored_cloud([(1.0, 2.0, 0.95)])
    message.fields = message.fields[:2]
    with pytest.raises(ValueError, match="x/y/confidence"):
        read_scored_points(message)


def test_visualizer_source_keeps_ground_truth_out_of_tracker_inputs():
    source = (SCRIPTS / "pedestrian_perception_visualizer.py").read_text(encoding="utf-8")
    assert "self.ground_truth_callback" in source
    assert "self._ground_truth_markers" in source
    assert "self.tracker" not in source
    assert "/pedestrian_visualization" in source
