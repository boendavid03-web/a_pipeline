import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = PROJECT_ROOT / "isaac_sim/scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from convert_gazebo_boxes_to_usda import load_static_boxes  # noqa: E402
from people_route_geometry import (  # noqa: E402
    edge_is_continuously_safe,
    point_within_clear_bounds,
    segment_intersects_expanded_box,
)


WORLD = (
    PROJECT_ROOT
    / "workspaces/ros2_ws/src/semantic_nav_gazebo/worlds/gazebo_eng_lobby.world"
)


class PeopleRouteGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        boxes, _ = load_static_boxes(WORLD)
        cls.boxes = {box.name: box for box in boxes}

    def test_regression_edges_touching_static_boxes_are_rejected(self):
        cases = {
            "grey_wall_46": ((3.925, 12.775), (3.775, 12.925)),
            "grey_wall_36": ((17.775, 19.775), (13.475, 18.325)),
            "grey_wall_79": ((21.175, 10.975), (21.025, 10.825)),
        }
        for name, (start, end) in cases.items():
            self.assertTrue(
                segment_intersects_expanded_box(start, end, self.boxes[name], 0.55),
                name,
            )
            self.assertFalse(
                edge_is_continuously_safe(
                    start, end, self.boxes.values(), (0.0, 0.0, 32.0, 24.0), 0.55
                ),
                name,
            )

    def test_clearance_boundary_contact_is_unsafe(self):
        self.assertFalse(point_within_clear_bounds((0.55, 12.0), (0.0, 0.0, 32.0, 24.0), 0.55))
        self.assertFalse(point_within_clear_bounds((31.45, 12.0), (0.0, 0.0, 32.0, 24.0), 0.55))


if __name__ == "__main__":
    unittest.main()
