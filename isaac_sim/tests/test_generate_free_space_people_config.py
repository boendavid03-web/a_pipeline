from __future__ import annotations

import math
import random
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = PROJECT_ROOT / "isaac_sim/scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from generate_free_space_people_config import (  # noqa: E402
    FreeSpaceMap,
    allocate_pedestrian_counts,
    gazebo_compatible_groups,
    load_gazebo_clusters,
)


SCENARIO = (
    PROJECT_ROOT
    / "workspaces/ros2_ws/src/semantic_nav_gazebo/scenarios/lobby/eng_hall_15.xml"
)


class FakeFreeSpace:
    def nearest(
        self, x: float, y: float, candidates=None
    ) -> tuple[float, float]:
        if candidates is not None:
            return min(
                candidates,
                key=lambda cell: ((cell[0] - x) ** 2 + (cell[1] - y) ** 2, cell),
            )
        return round(x, 4), round(y, 4)

    def route(self, start, goal):
        return [start] if start == goal else [start, goal]

    def segment_free(self, start, end):
        return True

    def nearest_separated(
        self, x: float, y: float, occupied, minimum_separation: float, candidates=None
    ):
        cells = (
            list(candidates)
            if candidates is not None
            else [(round(x + minimum_separation * index, 4), round(y, 4)) for index in range(20)]
        )
        eligible = [
            cell
            for cell in cells
            if all(
                (cell[0] - other[0]) ** 2 + (cell[1] - other[1]) ** 2
                >= minimum_separation**2 - 1.0e-9
                for other in occupied
            )
        ]
        if not eligible:
            raise ValueError("cannot place another pedestrian")
        return min(
            eligible,
            key=lambda cell: ((cell[0] - x) ** 2 + (cell[1] - y) ** 2, cell),
        )

    def simplify(self, path, minimum_segment=0.5, maximum_segment=1.0):
        return path

    def merge_short_visible(self, path, minimum_segment=0.5, maximum_segment=1.0):
        return path

    def world(self, cell):
        return [cell[0], cell[1], 0.0]


class SyntheticVisibilityGrid:
    """Small visibility graph exercising the real patrol simplifier."""

    simplify = FreeSpaceMap.simplify
    merge_short_visible = FreeSpaceMap.merge_short_visible
    resolution = 0.1

    def __init__(self, points, visible_edges):
        self.points = points
        self.visible_edges = {frozenset(edge) for edge in visible_edges}

    def world(self, cell):
        x, y = self.points[cell]
        return [x, y, 0.0]

    def segment_free(self, start, end):
        return start == end or frozenset((start, end)) in self.visible_edges


def template_groups(count: int = 6) -> dict[str, dict[str, object]]:
    return {
        f"cluster_{index}": {
            "num": 1,
            "asset_path": "Isaac/People/Characters/",
            "routines": [
                {
                    "patrol": {
                        "weight": 1,
                        "repeat": 1,
                        "speed_range": [0.9, 1.1],
                        "path_points": [[0.0, 0.0, 0.0]],
                    }
                }
            ],
        }
        for index in range(count)
    }


class GazeboPeopleConfigTest(unittest.TestCase):
    def test_static_box_clearance_is_inflated_exactly_once(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "map.pgm").write_bytes(
                b"P5\n9 9\n255\n" + bytes([255]) * 81
            )
            (root / "map.yaml").write_text(
                "image: map.pgm\nresolution: 1.0\n"
                "origin: [0.0, 0.0, 0.0]\nfree_thresh: 0.25\n",
                encoding="utf-8",
            )
            box = SimpleNamespace(
                pose=SimpleNamespace(x=4.5, y=4.5, yaw=0.0),
                size=(1.0, 1.0),
            )

            grid = FreeSpaceMap(root / "map.yaml", 1.0, [box])

            self.assertFalse(grid.contains_world(5.5, 4.5))
            self.assertTrue(grid.contains_world(6.5, 4.5))

    def test_simplifier_avoids_greedy_micro_segment(self) -> None:
        # Greedy visibility selects 0 -> 2 -> 3 -> 5 and leaves a 14 cm
        # segment at the corner.  The alternate 0 -> 1 -> 4 -> 5 route has
        # the same point count, remains visible, and keeps every edge >= 0.5 m.
        grid = SyntheticVisibilityGrid(
            {
                0: (0.0, 0.0),
                1: (0.5, 0.0),
                2: (1.0, 0.0),
                3: (1.1, 0.1),
                4: (1.2, 0.6),
                5: (1.7, 0.6),
            },
            {
                (0, 1),
                (0, 2),
                (1, 2),
                (1, 4),
                (2, 3),
                (3, 4),
                (3, 5),
                (4, 5),
            },
        )

        simplified = grid.simplify([0, 1, 2, 3, 4, 5], 0.5)

        self.assertEqual(simplified, [0, 1, 4, 5])
        self.assertEqual(simplified, grid.simplify([0, 1, 2, 3, 4, 5], 0.5))

    def test_simplifier_retains_required_short_obstacle_corner(self) -> None:
        grid = SyntheticVisibilityGrid(
            {
                0: (0.0, 0.0),
                1: (1.0, 0.0),
                2: (1.1, 0.1),
                3: (2.0, 1.0),
            },
            {(0, 1), (1, 2), (2, 3)},
        )

        # There is no collision-free shortcut over node 1 or 2, so the
        # sub-threshold turn is intentionally preserved instead of cutting the
        # obstacle corner.
        self.assertEqual(grid.simplify([0, 1, 2, 3], 0.5), [0, 1, 2, 3])

    def test_join_merger_preserves_spawn_and_required_corner(self) -> None:
        grid = SyntheticVisibilityGrid(
            {
                0: (0.0, 0.0),
                1: (0.4, 0.0),
                2: (1.0, 0.0),
                3: (1.1, 0.1),
                4: (2.0, 1.0),
                5: (0.0, 0.0),
            },
            {(0, 1), (0, 2), (1, 2), (2, 3), (3, 4), (4, 5)},
        )

        merged = grid.merge_short_visible([0, 1, 2, 3, 4, 5], 0.5)

        self.assertEqual(merged, [0, 2, 3, 4, 5])
        self.assertEqual(merged[0], 0)
        self.assertEqual(merged[-1], 5)

    def test_simplifier_retains_safe_intermediate_cells_for_maximum_edge(self) -> None:
        points = {index: (float(index), 0.0) for index in range(25)}
        visible = {(index, index + 1) for index in range(24)}
        grid = SyntheticVisibilityGrid(points, visible)

        first = grid.simplify(list(range(25)), 0.5, 1.0)
        second = grid.simplify(list(range(25)), 0.5, 1.0)

        self.assertEqual(first, second)
        self.assertEqual(first, list(range(25)))
        self.assertLessEqual(
            max(
                math.dist(grid.world(start)[:2], grid.world(end)[:2])
                for start, end in zip(first, first[1:])
            ),
            1.0,
        )

    def test_merger_does_not_create_overlong_short_join(self) -> None:
        grid = SyntheticVisibilityGrid(
            {0: (0.0, 0.0), 1: (0.4, 0.0), 2: (1.6, 0.0)},
            {(0, 1), (1, 2), (0, 2)},
        )
        self.assertEqual(grid.merge_short_visible([0, 1, 2], 0.5, 1.0), [0, 1, 2])

    def test_scenario_has_expected_non_robot_clusters(self) -> None:
        clusters = load_gazebo_clusters(SCENARIO)
        self.assertEqual([cluster["count"] for cluster in clusters], [5, 5, 2, 1, 1, 1])
        self.assertEqual(len(clusters), 6)
        self.assertEqual(clusters[0]["route"][0], (25.0, 9.0))
        self.assertEqual(clusters[-1]["route"], [(14.0, 22.0), (16.0, 6.0)])

    def test_largest_remainder_matches_gazebo(self) -> None:
        weights = [5, 5, 2, 1, 1, 1]
        self.assertEqual(allocate_pedestrian_counts(weights, -1), weights)
        self.assertEqual(allocate_pedestrian_counts(weights, 0), [0, 0, 0, 0, 0, 0])
        self.assertEqual(allocate_pedestrian_counts(weights, 1), [1, 0, 0, 0, 0, 0])
        self.assertEqual(allocate_pedestrian_counts(weights, 19), [7, 6, 3, 1, 1, 1])
        self.assertEqual(sum(allocate_pedestrian_counts(weights, 50)), 50)

    def test_generation_is_seeded_and_assigns_one_speed_per_person(self) -> None:
        clusters = load_gazebo_clusters(SCENARIO)
        allocation = allocate_pedestrian_counts(
            [int(cluster["count"]) for cluster in clusters], 19
        )
        first = gazebo_compatible_groups(
            template_groups(), clusters, allocation, FakeFreeSpace(), random.Random(7), 1.0,
            maximum_segment=100.0,
        )
        second = gazebo_compatible_groups(
            template_groups(), clusters, allocation, FakeFreeSpace(), random.Random(7), 1.0,
            maximum_segment=100.0,
        )
        different_seed = gazebo_compatible_groups(
            template_groups(), clusters, allocation, FakeFreeSpace(), random.Random(8), 1.0,
            maximum_segment=100.0,
        )

        self.assertEqual(first, second)
        self.assertNotEqual(first, different_seed)
        self.assertEqual(len(first), 19)
        self.assertEqual(
            [sum(name.startswith(f"gazebo_{letter}_") for name in first) for letter in "abcdef"],
            allocation,
        )
        for group in first.values():
            self.assertEqual(group["num"], 1)
            patrol = group["routines"][0]["patrol"]
            self.assertEqual(patrol["speed_range"][0], patrol["speed_range"][1])
            self.assertGreaterEqual(patrol["speed_range"][0], 0.1)
            self.assertEqual(patrol["path_points"][0], patrol["path_points"][-1])

    def test_template_cluster_count_must_match_scenario(self) -> None:
        clusters = load_gazebo_clusters(SCENARIO)
        with self.assertRaisesRegex(ValueError, "template group count"):
            gazebo_compatible_groups(
                template_groups(5),
                clusters,
                [1, 1, 1, 1, 1, 1],
                FakeFreeSpace(),
                random.Random(7),
                1.0,
            )

    def test_generation_constrains_projection_to_one_component(self) -> None:
        clusters = [
            {
                "count": 1,
                "center": (30.0, 10.0),
                "extent": (2.0, 2.0),
                "route": [(3.0, 10.0), (25.0, 21.0)],
            }
        ]
        component = {(3.0, 10.0), (25.0, 21.0), (27.0, 10.0)}

        groups = gazebo_compatible_groups(
            template_groups(1),
            clusters,
            [1],
            FakeFreeSpace(),
            random.Random(15),
            0.9,
            component,
            maximum_segment=100.0,
        )

        points = groups["gazebo_a_001"]["routines"][0]["patrol"]["path_points"]
        self.assertTrue(points)
        self.assertEqual(points[0], points[-1])
        self.assertTrue(all(tuple(point[:2]) in component for point in points))

    def test_generation_separates_collapsed_start_projections(self) -> None:
        clusters = [
            {
                "count": 2,
                "center": (0.0, 0.0),
                "extent": (0.0, 0.0),
                "route": [(2.0, 0.0), (3.0, 0.0)],
            }
        ]
        component = {(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)}

        groups = gazebo_compatible_groups(
            template_groups(1),
            clusters,
            [2],
            FakeFreeSpace(),
            random.Random(7),
            1.0,
            component,
            1.0,
            maximum_segment=100.0,
        )

        starts = [
            group["routines"][0]["patrol"]["path_points"][0]
            for group in groups.values()
        ]
        self.assertGreaterEqual(
            ((starts[0][0] - starts[1][0]) ** 2 + (starts[0][1] - starts[1][1]) ** 2) ** 0.5,
            1.0,
        )

    def test_cluster_members_enter_cyclic_route_at_distinct_phases(self) -> None:
        clusters = [
            {
                "count": 3,
                "center": (0.0, 0.0),
                "extent": (0.0, 0.0),
                "route": [(10.0, 0.0), (20.0, 0.0), (30.0, 0.0)],
            }
        ]
        component = {
            (0.0, 0.0),
            (1.0, 0.0),
            (2.0, 0.0),
            (10.0, 0.0),
            (20.0, 0.0),
            (30.0, 0.0),
        }

        groups = gazebo_compatible_groups(
            template_groups(1),
            clusters,
            [3],
            FakeFreeSpace(),
            random.Random(7),
            1.0,
            component,
            1.0,
            maximum_segment=100.0,
        )

        first_route_targets = [
            group["routines"][0]["patrol"]["path_points"][1][:2]
            for group in groups.values()
        ]
        self.assertEqual(
            first_route_targets,
            [[10.0, 0.0], [20.0, 0.0], [30.0, 0.0]],
        )


if __name__ == "__main__":
    unittest.main()
