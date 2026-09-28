#!/usr/bin/env python3
"""Generate the neutral Isaac 5/HuNav crowd scene from existing route logic."""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import sys
import tempfile
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[4]
SCRIPT_DIR = PROJECT_ROOT / "isaac_sim" / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from convert_gazebo_boxes_to_usda import load_static_boxes  # noqa: E402
from generate_free_space_people_config import (  # noqa: E402
    DEFAULT_MAX_PATROL_SEGMENT_M,
    DEFAULT_MIN_PATROL_SEGMENT_M,
    DEFAULT_SPAWN_CLEARANCE_M,
    SCENARIO_AB_MODES,
    FreeSpaceMap,
    allocate_pedestrian_counts,
    configure_opposed_pair_test,
    gazebo_compatible_groups,
    load_gazebo_clusters,
    validate_generated_routes,
)
from people_route_geometry import DEFAULT_LOBBY_BOUNDS  # noqa: E402


SCHEMA = "a_pipeline.isaac_hunav.scene/v1"
MAX_VALIDATED_PEDESTRIANS = 30


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-yaml", required=True, type=Path)
    parser.add_argument("--template", required=True, type=Path)
    parser.add_argument("--scenario", required=True, type=Path)
    parser.add_argument("--world", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--count", required=True, type=int)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--social-force-factor", type=float, default=5.0)
    parser.add_argument("--clearance", type=float, default=0.55)
    parser.add_argument("--spawn-clearance", type=float, default=DEFAULT_SPAWN_CLEARANCE_M)
    parser.add_argument("--min-patrol-segment", type=float, default=DEFAULT_MIN_PATROL_SEGMENT_M)
    parser.add_argument("--max-patrol-segment", type=float, default=DEFAULT_MAX_PATROL_SEGMENT_M)
    parser.add_argument("--ensure-all-clusters", action="store_true")
    parser.add_argument("--opposed-pair-test", action="store_true")
    parser.add_argument(
        "--scenario-ab-mode",
        choices=SCENARIO_AB_MODES,
        default="baseline",
        help=(
            "reuse the shared Isaac 6 scenario-topology modes; spread_radius "
            "distributes starts and per-person arrivals within authored extents/radii"
        ),
    )
    return parser.parse_args()


def _patrol(group: dict) -> tuple[list[list[float]], float]:
    patrols = [routine["patrol"] for routine in group.get("routines", []) if "patrol" in routine]
    if len(patrols) != 1:
        raise ValueError("each generated person must have exactly one patrol")
    patrol = patrols[0]
    points = [[float(value) for value in point] for point in patrol["path_points"]]
    speeds = patrol["speed_range"]
    if len(points) < 2 or len(speeds) != 2 or float(speeds[0]) != float(speeds[1]):
        raise ValueError("generated patrol or independent speed is malformed")
    return points, float(speeds[0])


def generate_document(args: argparse.Namespace) -> dict:
    if not 1 <= args.count <= MAX_VALIDATED_PEDESTRIANS:
        raise ValueError(
            f"count must be in [1, {MAX_VALIDATED_PEDESTRIANS}]; "
            "larger populations are not validated"
        )
    if not 0 <= args.seed <= 4_294_967_295:
        raise ValueError("seed must be between 0 and 4294967295")
    for label in (
        "speed", "social_force_factor", "clearance", "spawn_clearance",
        "min_patrol_segment", "max_patrol_segment",
    ):
        value = float(getattr(args, label))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{label} must be a positive finite number")

    import yaml

    static_boxes, _ = load_static_boxes(args.world)
    grid = FreeSpaceMap(args.map_yaml, args.clearance, static_boxes, bounds=DEFAULT_LOBBY_BOUNDS)
    clusters = load_gazebo_clusters(args.scenario)
    template = yaml.safe_load(args.template.read_text(encoding="utf-8"))
    template_groups = template["isaacsim.replicator.agent"]["character"]["groups"]
    allocation = allocate_pedestrian_counts(
        [int(cluster["count"]) for cluster in clusters],
        args.count,
        ensure_all_clusters=args.ensure_all_clusters,
    )
    component = max(grid.components(), key=len)
    groups = gazebo_compatible_groups(
        template_groups,
        clusters,
        allocation,
        grid,
        random.Random(args.seed),
        args.speed,
        component,
        args.spawn_clearance,
        args.min_patrol_segment,
        args.max_patrol_segment,
        False,
        args.scenario_ab_mode,
    )
    if args.opposed_pair_test:
        if args.count != 2:
            raise ValueError("--opposed-pair-test requires --count 2")
        configure_opposed_pair_test(groups)
    validate_generated_routes(groups, grid, args.max_patrol_segment)

    agents: list[dict] = []
    group_rows: list[tuple[int, int, dict]] = []
    for cluster_index, count in enumerate(allocation):
        for person_index in range(count):
            group_rows.append((cluster_index, person_index, clusters[cluster_index]))
    if len(group_rows) != len(groups) or len(groups) != args.count:
        raise ValueError("generator reduced or reordered the requested population")

    for track_id, ((group_name, group), (cluster_index, person_index, cluster)) in enumerate(
        zip(groups.items(), group_rows), start=1
    ):
        resolved_route, preferred_speed = _patrol(group)
        semantic = [[float(x), float(y), 0.0] for x, y in cluster["route"]]
        phase = person_index % len(semantic)
        semantic = semantic[phase:] + semantic[:phase]
        behavior_type = "scared" if args.count >= 2 and track_id == args.count else "regular"
        stable_id = f"arena_person_{track_id:03d}"
        agents.append(
            {
                "stable_id": stable_id,
                "track_id": track_id,
                "name": stable_id,
                "model": "actor1",
                "spawn": resolved_route[0],
                "semantic_route": semantic,
                "resolved_route": resolved_route,
                "route_cluster": cluster_index,
                "route_phase": phase,
                "preferred_speed": round(preferred_speed, 6),
                "radius": 0.25,
                "group_id": -1,
                "behavior": {
                    "type": behavior_type,
                    "duration": 40.0,
                    "once": False,
                    "velocity": round(preferred_speed, 6),
                    "interaction_distance": 2.0 if behavior_type == "scared" else 1.5,
                    "social_force_factor": float(args.social_force_factor),
                    "goal_force_factor": 2.0,
                    "obstacle_force_factor": 10.0,
                    "other_force_factor": 20.0,
                },
                "goal_radius": 0.30,
                "cyclic_goals": True,
                "waypoint_mode": 1,
                "generator_group": group_name,
            }
        )

    return {
        "schema": SCHEMA,
        "metadata": {
            "world": "a_pipeline_eng_lobby",
            "seed": args.seed,
            "requested_count": args.count,
            "actual_count": len(agents),
            "maximum_validated_count": MAX_VALIDATED_PEDESTRIANS,
            "allocation": allocation,
            "confirmed_free_map": str(args.map_yaml.resolve()),
            "gazebo_scenario": str(args.scenario.resolve()),
            "gazebo_world": str(args.world.resolve()),
            "clearance_m": args.clearance,
            "spawn_clearance_m": args.spawn_clearance,
            "social_force_factor": args.social_force_factor,
            "scenario_ab_mode": args.scenario_ab_mode,
            "unknown_is_blocked": True,
            "occupied_is_blocked": True,
        },
        "agents": agents,
    }


def write_atomic(path: Path, document: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent,
            prefix=f".{path.name}.", suffix=".tmp", delete=False,
        ) as handle:
            temporary = handle.name
            json.dump(document, handle, indent=2, sort_keys=True, allow_nan=False)
            handle.write("\n")
        os.replace(temporary, path)
        temporary = None
    finally:
        if temporary is not None:
            Path(temporary).unlink(missing_ok=True)


def main() -> int:
    args = parse_args()
    try:
        document = generate_document(args)
        write_atomic(args.output, document)
    except (KeyError, TypeError, ValueError) as error:
        raise SystemExit(f"ERROR: {error}") from error
    print(
        "ISAAC5_HUNAV_DYNAMIC_CONFIG=PASS "
        f"count={len(document['agents'])} seed={args.seed} "
        f"allocation={document['metadata']['allocation']} output={args.output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
