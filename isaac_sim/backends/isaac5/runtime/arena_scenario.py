#!/usr/bin/env python3
"""Load the Arena dynamic-obstacle subset used by the Isaac 5 crowd adapter."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path


Point3 = tuple[float, float, float]


@dataclass(frozen=True)
class ArenaPedestrian:
    name: str
    stable_id: str
    track_id: int
    model: str
    route: tuple[Point3, ...]
    semantic_route: tuple[Point3, ...]
    waypoint_mode: int
    behavior: str
    desired_velocity: float | None
    personal_space: float
    social_force_factor: float
    group_id: int
    interaction_distance: float
    radius: float
    goal_radius: float
    cyclic_goals: bool
    behavior_parameters: tuple[tuple[str, float | bool | str], ...]


BEHAVIORS = frozenset(
    {"regular", "impassive", "surprised", "scared", "curious", "threatening"}
)


def _point3(value: object, label: str) -> Point3:
    if not isinstance(value, list) or len(value) not in (2, 3):
        raise ValueError(f"{label} must be a two- or three-element list")
    if len(value) == 2:
        value = [*value, 0.0]
    point = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in point):
        raise ValueError(f"{label} must contain finite coordinates")
    return point


def load_arena_pedestrians(path: Path) -> tuple[ArenaPedestrian, ...]:
    """Read legacy Arena or neutral dynamic-crowd entries without ROS imports."""
    document = json.loads(path.read_text(encoding="utf-8"))
    neutral = document.get("schema") == "a_pipeline.isaac_hunav.scene/v1"
    if neutral:
        dynamic = document.get("agents", [])
    else:
        obstacles = document.get("obstacles", {})
        dynamic = obstacles.get("dynamic", []) if isinstance(obstacles, dict) else None
    if not isinstance(dynamic, list):
        raise ValueError("Arena scenario obstacles.dynamic/agents must be a list")
    # Some official Arena scenarios intentionally contain no dynamic agents
    # (for example ``empty.json`` and ``marl.json``).  Treat these as valid
    # zero-person scenes so the all-scene audit can cover them as well.
    if not dynamic:
        return ()

    pedestrians: list[ArenaPedestrian] = []
    names: set[str] = set()
    used_track_ids: set[int] = set()
    for index, entry in enumerate(dynamic):
        if not isinstance(entry, dict):
            raise ValueError(f"obstacles.dynamic[{index}] must be an object")
        name = str(entry.get("name", entry.get("stable_id", ""))).strip()
        if not name or name in names:
            raise ValueError(f"obstacles.dynamic[{index}] has a missing or duplicate name")
        names.add(name)
        waypoint_mode = int(entry.get("waypoint_mode", 1))
        if waypoint_mode not in (0, 1, 2):
            raise ValueError(f"{name}: unsupported Arena waypoint_mode={waypoint_mode}")
        stable_id = str(entry.get("stable_id", name)).strip()
        track_id = int(entry.get("track_id", entry.get("id", index + 1)))
        # Several legacy Arena files use ``id: 0`` for every dynamic
        # obstacle.  Keep explicit neutral track IDs, but make the legacy
        # fallback deterministic and unique for downstream tracking.
        if track_id in used_track_ids:
            track_id = index + 1
            while track_id in used_track_ids:
                track_id += 1
        used_track_ids.add(track_id)
        route_values = entry.get("resolved_route") if neutral else None
        if route_values is None:
            waypoints = entry.get("waypoints")
            if not isinstance(waypoints, list):
                raise ValueError(f"{name}: waypoints must be a list")
            # waypoint_mode=0 is used by Arena for stationary/blocked agents;
            # the native Isaac backend imports these with their spawn pose as
            # the goal.  Preserve that contract as a one-point route.
            route_values = [entry.get("pos"), *waypoints]
        if not isinstance(route_values, list) or not route_values:
            raise ValueError(f"{name}: resolved route must contain at least one point")
        route = tuple(
            _point3(point, f"{name}.resolved_route[{route_index}]")
            for route_index, point in enumerate(route_values)
        )
        semantic_values = entry.get("semantic_route", route_values)
        if not isinstance(semantic_values, list) or not semantic_values:
            raise ValueError(f"{name}: semantic route must be non-empty")
        semantic_route = tuple(
            _point3(point, f"{name}.semantic_route[{route_index}]")
            for route_index, point in enumerate(semantic_values)
        )
        if waypoint_mode != 0 and len(route) < 2:
            raise ValueError(f"{name}: moving route must contain at least two points")
        if waypoint_mode != 0 and all(
            math.dist(route[0][:2], point[:2]) <= 1.0e-9 for point in route[1:]
        ):
            raise ValueError(f"{name}: route must contain a non-zero planar leg")
        behavior_value = entry.get("behavior", "regular")
        behavior_object = behavior_value if isinstance(behavior_value, dict) else {"type": behavior_value}
        behavior = str(behavior_object.get("type", "regular")).strip().lower()
        if behavior not in BEHAVIORS:
            raise ValueError(f"{name}: unsupported HuNav behavior {behavior!r}")
        desired_velocity_value = entry.get("preferred_speed", entry.get("desired_velocity"))
        desired_velocity = (
            None if desired_velocity_value is None else float(desired_velocity_value)
        )
        personal_space = float(entry.get("personal_space", 0.70))
        social_force_factor = float(
            behavior_object.get("social_force_factor", entry.get("social_force_factor", 5.0))
        )
        group_id = int(entry.get("group_id", -1))
        interaction_distance = float(
            behavior_object.get("interaction_distance", entry.get("interaction_distance", 1.5))
        )
        radius = float(entry.get("radius", 0.25))
        goal_radius = float(entry.get("goal_radius", 0.30))
        numeric = (personal_space, social_force_factor, interaction_distance, radius, goal_radius)
        if not all(math.isfinite(value) and value >= 0.0 for value in numeric):
            raise ValueError(f"{name}: HuNav social parameters must be finite and non-negative")
        if desired_velocity is not None and (
            not math.isfinite(desired_velocity) or desired_velocity <= 0.0
        ):
            raise ValueError(f"{name}: desired_velocity must be finite and positive")
        pedestrians.append(
            ArenaPedestrian(
                name=name,
                stable_id=stable_id,
                track_id=track_id,
                model=str(entry.get("model", "actor1")),
                route=route,
                semantic_route=semantic_route,
                waypoint_mode=waypoint_mode,
                behavior=behavior,
                desired_velocity=desired_velocity,
                personal_space=personal_space,
                social_force_factor=social_force_factor,
                group_id=group_id,
                interaction_distance=interaction_distance,
                radius=radius,
                goal_radius=goal_radius,
                cyclic_goals=bool(entry.get("cyclic_goals", True)),
                behavior_parameters=tuple(sorted(behavior_object.items())),
            )
        )
    track_ids = [person.track_id for person in pedestrians]
    stable_ids = [person.stable_id for person in pedestrians]
    if len(set(track_ids)) != len(track_ids) or len(set(stable_ids)) != len(stable_ids):
        raise ValueError("pedestrian stable_id and track_id values must be unique")
    return tuple(pedestrians)


def select_arena_pedestrians(
    pedestrians: tuple[ArenaPedestrian, ...], count: int
) -> tuple[ArenaPedestrian, ...]:
    if not 1 <= count <= len(pedestrians):
        raise ValueError(
            f"pedestrian count must be in [1, {len(pedestrians)}] for this Arena scenario"
        )
    return pedestrians[:count]
