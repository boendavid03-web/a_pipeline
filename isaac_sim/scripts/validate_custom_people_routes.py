#!/usr/bin/env python3
"""Verify that the custom Isaac pedestrian patrols remain in the Gazebo map.

IRA's patrol primitive visits consecutive path points.  The old configuration
used the centre of Gazebo *area* waypoints directly, which is not equivalent
to a traversable route: a few centres lie in wall geometry and a straight
link can cut through a room.  This preflight check protects the authored
routes against that regression using the source Gazebo collision boxes.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_CONFIG = SCRIPT_DIR / "ira_people_demo/custom_eng_lobby_people.yaml"
DEFAULT_WORLD = (
    PROJECT_ROOT
    / "workspaces/ros2_ws/src/semantic_nav_gazebo/worlds/gazebo_eng_lobby.world"
)
DEFAULT_CLEARANCE_M = 0.55
DEFAULT_MIN_START_SEPARATION_M = 0.0

# Share the exact SDF parsing used to create the USD, rather than maintaining
# a second, subtly different list of walls for the route safety gate.
sys.path.insert(0, str(SCRIPT_DIR))
from convert_gazebo_boxes_to_usda import Box, load_static_boxes  # noqa: E402
from people_route_geometry import (  # noqa: E402
    DEFAULT_LOBBY_BOUNDS,
    edge_is_continuously_safe,
    point_within_clear_bounds,
    segment_intersects_expanded_box,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--world", type=Path, default=DEFAULT_WORLD)
    parser.add_argument(
        "--bounds",
        nargs=4,
        type=float,
        metavar=("X_MIN", "Y_MIN", "X_MAX", "Y_MAX"),
        default=DEFAULT_LOBBY_BOUNDS,
    )
    parser.add_argument("--clearance", type=float, default=DEFAULT_CLEARANCE_M)
    parser.add_argument(
        "--min-start-separation",
        type=float,
        default=DEFAULT_MIN_START_SEPARATION_M,
        help="Minimum centre-to-centre distance between patrol start points.",
    )
    return parser.parse_args()


def validate_route(
    name: str,
    points: list[tuple[float, float]],
    boxes: list[Box],
    bounds: tuple[float, float, float, float],
    clearance: float,
) -> list[str]:
    errors: list[str] = []
    x_min, y_min, x_max, y_max = bounds
    for index, point in enumerate(points):
        x, y = point
        if not point_within_clear_bounds(point, bounds, clearance):
            errors.append(f"{name}: point {index} ({x:.2f}, {y:.2f}) leaves map bounds")
        for box in boxes:
            if segment_intersects_expanded_box(point, point, box, clearance):
                errors.append(f"{name}: point {index} ({x:.2f}, {y:.2f}) intersects {box.name}")
                break
    for index, (start, end) in enumerate(zip(points, points[1:] + points[:1])):
        if edge_is_continuously_safe(start, end, boxes, bounds, clearance):
            continue
        offending_box = next(
            (
                box
                for box in boxes
                if segment_intersects_expanded_box(start, end, box, clearance)
            ),
            None,
        )
        if offending_box is not None:
            errors.append(
                f"{name}: segment {index} ({start[0]:.2f}, {start[1]:.2f}) -> "
                f"({end[0]:.2f}, {end[1]:.2f}) intersects {offending_box.name}"
            )
        else:
            errors.append(
                f"{name}: segment {index} ({start[0]:.2f}, {start[1]:.2f}) -> "
                f"({end[0]:.2f}, {end[1]:.2f}) leaves map bounds"
            )
    return errors


def main() -> int:
    args = parse_args()
    if args.clearance <= 0.0 or not math.isfinite(args.clearance):
        raise SystemExit("ERROR: --clearance must be a positive finite number")
    if args.min_start_separation < 0.0 or not math.isfinite(
        args.min_start_separation
    ):
        raise SystemExit(
            "ERROR: --min-start-separation must be a non-negative finite number"
        )
    bounds = tuple(args.bounds)
    if not all(math.isfinite(value) for value in bounds) or bounds[2] <= bounds[0] or bounds[3] <= bounds[1]:
        raise SystemExit("ERROR: --bounds must be finite and ordered")
    with args.config.open(encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    boxes, _ = load_static_boxes(args.world)
    groups = config["isaacsim.replicator.agent"]["character"]["groups"]
    errors: list[str] = []
    route_count = 0
    route_starts: list[tuple[str, tuple[float, float]]] = []
    for group_name, group in groups.items():
        for routine in group.get("routines", []):
            patrol = routine.get("patrol")
            if patrol is None:
                continue
            points = patrol.get("path_points", [])
            planar_points: list[tuple[float, float]] = []
            for point in points:
                if not isinstance(point, list) or len(point) != 3:
                    errors.append(f"{group_name}: invalid patrol point {point!r}")
                    continue
                if abs(float(point[2])) > 1.0e-6:
                    errors.append(f"{group_name}: path point must remain on Z=0, got {point!r}")
                planar_points.append((float(point[0]), float(point[1])))
            if len(planar_points) < 2:
                errors.append(f"{group_name}: patrol needs at least two path points")
                continue
            route_count += 1
            route_starts.append((group_name, planar_points[0]))
            errors.extend(
                validate_route(group_name, planar_points, boxes, bounds, args.clearance)
            )
    minimum_start_distance = math.inf
    for index, (first_name, first) in enumerate(route_starts):
        for second_name, second in route_starts[index + 1 :]:
            distance = math.hypot(first[0] - second[0], first[1] - second[1])
            minimum_start_distance = min(minimum_start_distance, distance)
            if distance + 1.0e-9 < args.min_start_separation:
                errors.append(
                    f"{first_name}/{second_name}: starts are only {distance:.3f} m "
                    f"apart; require {args.min_start_separation:.3f} m"
                )
    if errors:
        print("CUSTOM_PEOPLE_ROUTE_CHECK=FAIL", file=sys.stderr)
        print("\n".join(errors), file=sys.stderr)
        return 1
    print(
        "CUSTOM_PEOPLE_ROUTE_CHECK=PASS "
        f"routes={route_count} boxes={len(boxes)} clearance_m={args.clearance:.2f} "
        f"min_start_separation_m="
        f"{minimum_start_distance if math.isfinite(minimum_start_distance) else 0.0:.3f} "
        f"bounds=({bounds[0]:.1f},{bounds[1]:.1f})..({bounds[2]:.1f},{bounds[3]:.1f})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
