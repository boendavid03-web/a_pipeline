#!/usr/bin/env python3
"""Gate 6B: validate an Arena-style animated crowd in the Isaac 5 lobby."""

from __future__ import annotations

import asyncio
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

import validate_pedestrian as single
from arena_scenario import load_arena_pedestrians, select_arena_pedestrians
from crowd_social_control import CrowdAgent, CrowdSocialConfig, CrowdSocialController, PatrolCursor
from gate6_interaction import (
    MINIMUM_PROFILE_DURATION_SEC,
    NARROW_PROFILE_NAME,
    PROFILE_NAME,
    command_for_time,
    evaluate_interaction,
    evaluate_narrow_wait,
)
from hunav_protocol import (
    IndependentYieldPlanner,
    SafeVelocityProjector,
    SafetyConfig,
    bounded_route_social_velocity,
    forward_recovery_goal,
)
from route_runtime import SemanticRouteRuntime
from static_obstacles import sample_static_obstacle_points, static_polyline_is_clear


def person_paths(stable_id: str) -> dict[str, str]:
    root = f"/World/Pedestrians/{stable_id}"
    visual = f"{root}/Visual"
    return {
        "stable_id": stable_id,
        "root": root,
        "visual": visual,
        "collider": f"/World/PedestrianColliders/{stable_id}",
    }


def polyline_length(route) -> float:
    points = np.asarray(route, dtype=float)
    return float(np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1).sum())


def polyline_route_state(sim_time: float, route, speed_mps: float):
    points = np.asarray(route, dtype=float)
    segment_vectors = np.diff(points, axis=0)
    segment_lengths = np.linalg.norm(segment_vectors[:, :2], axis=1)
    route_length = float(segment_lengths.sum())
    if route_length <= 1.0e-9:
        raise ValueError("patrol polyline must have positive length")
    cycle_distance = (speed_mps * sim_time) % (2.0 * route_length)
    forward = cycle_distance <= route_length
    distance = cycle_distance if forward else 2.0 * route_length - cycle_distance
    remaining = distance
    segment_index = len(segment_lengths) - 1
    for index, length in enumerate(segment_lengths):
        if remaining <= float(length):
            segment_index = index
            break
        remaining -= float(length)
    unit = segment_vectors[segment_index] / float(segment_lengths[segment_index])
    position = points[segment_index] + unit * remaining
    direction = 1.0 if forward else -1.0
    velocity = unit * speed_mps * direction
    yaw = math.atan2(float(velocity[1]), float(velocity[0]))
    return position, velocity, yaw


def polyline_route_clearance(query, route):
    segment_rows = []
    clear = True
    for start, end in zip(route[:-1], route[1:]):
        segment_clear, samples = single.route_clearance(query, start, end)
        segment_rows.append(
            {"start": list(start), "end": list(end), "clear": segment_clear, "samples": samples}
        )
        clear = clear and segment_clear
    return clear, segment_rows


def point_to_segment_distance(point: tuple[float, float], start, end) -> float:
    """Planar distance to a pre-cleared patrol segment."""
    p = np.asarray(point, dtype=float)
    a = np.asarray(start[:2], dtype=float)
    b = np.asarray(end[:2], dtype=float)
    delta = b - a
    denom = float(np.dot(delta, delta))
    if denom <= 1.0e-12:
        return float(np.linalg.norm(p - a))
    alpha = float(np.clip(np.dot(p - a, delta) / denom, 0.0, 1.0))
    return float(np.linalg.norm(p - (a + alpha * delta)))


def robot_rectangle_net_clearance(
    person_xy, robot_xy, robot_yaw: float, half_x: float, half_y: float, person_radius: float
) -> float:
    """Circle-to-oriented-rectangle signed planar clearance."""
    delta = np.asarray(person_xy, dtype=float) - np.asarray(robot_xy, dtype=float)
    cosine, sine = math.cos(robot_yaw), math.sin(robot_yaw)
    local = np.asarray(
        [cosine * delta[0] + sine * delta[1], -sine * delta[0] + cosine * delta[1]]
    )
    outside = np.maximum(np.abs(local) - np.asarray([half_x, half_y]), 0.0)
    outside_distance = float(np.linalg.norm(outside))
    if outside_distance > 0.0:
        return outside_distance - person_radius
    inside_depth = min(half_x - abs(float(local[0])), half_y - abs(float(local[1])))
    return -inside_depth - person_radius


def main() -> int:
    result: dict[str, object] = {"status": "FAIL", "gate": "6B"}
    world = ros = mobile = None
    crowd_controller = None
    native_hunav_controller = None
    native_hunav_agent_ids: list[int] = []
    teardown_errors: list[str] = []
    try:
        if single.ARGS.native_controller_probe:
            import omni.kit.app
            from isaac_navmesh import IsaacNavMeshGuard

            extension_manager = omni.kit.app.get_app().get_extension_manager()
            extension_manager.set_extension_enabled_immediate("omni.anim.navigation.bundle", True)
            for _ in range(4):
                single.simulation_app.update()
            import omni.anim.navigation.core as nav

            navigation = nav.acquire_interface()
            success, error = single.wait_for_task(
                asyncio.ensure_future(single.omni.usd.get_context().open_stage_async(str(single.SCENE_USD))),
                single.ARGS.setup_timeout,
                "opening the active lobby for native-controller inspection",
            )
            if not success:
                raise RuntimeError(f"Could not open active lobby: {error}")
            for _ in range(8):
                single.simulation_app.update()
            guard = single.wait_for_task(
                asyncio.ensure_future(IsaacNavMeshGuard.bake(radius_m=0.25)),
                single.ARGS.setup_timeout,
                "baking the authored Isaac NavMesh for controller inspection",
            )
            navmesh_methods = sorted(name for name in dir(guard.navmesh) if not name.startswith("_"))
            controller_factories = [
                name
                for name in navmesh_methods
                if "controller" in name.lower() or "agent" in name.lower() or "obstacle" in name.lower()
            ]
            controller_result: dict[str, object] = {"status": "NOT_EXPOSED"}
            if "create_controller" in navmesh_methods:
                try:
                    controller = guard.navmesh.create_controller()
                    controller_result = {
                        "status": "CREATED",
                        "type": str(type(controller)),
                        "methods": sorted(name for name in dir(controller) if not name.startswith("_")),
                        "create_controller_doc": str(guard.navmesh.create_controller.__doc__),
                    }
                except Exception as exc:
                    controller_result = {
                        "status": "CREATE_FAILED",
                        "error": repr(exc),
                        "create_controller_doc": str(guard.navmesh.create_controller.__doc__),
                    }
            bridge_dir = Path(__file__).resolve().parents[1] / "native_controller_bridge"
            bridge_result: dict[str, object] = {"status": "NOT_BUILT"}
            bridge_binary = next(bridge_dir.glob("_isaac5_native_controller*.so"), None)
            if bridge_binary is not None:
                if str(bridge_dir) not in sys.path:
                    sys.path.insert(0, str(bridge_dir))
                try:
                    import _isaac5_native_controller as native_bridge

                    native = native_bridge.NativeController()
                    starts = [
                        list(guard.closest((3.0, 1.5, 0.0), tolerance_m=0.50)),
                        list(guard.closest((3.0, 2.5, 0.0), tolerance_m=0.50)),
                    ]
                    goals = [starts[1], starts[0]]
                    agent_ids = [71001, 71002]
                    created = native.create_agents(
                        agent_ids, starts, [0.6, 0.6], [0.25, 0.25], [1.7, 1.7]
                    )
                    goals_updated = native.update_agents(agent_ids, goals=goals)
                    samples = [starts]
                    minimum_pair_distance_m = math.dist(starts[0][:2], starts[1][:2])
                    for _ in range(120):
                        native.simulate(1.0 / 60.0)
                        positions = native.simulated(agent_ids)
                        samples.append(positions)
                        minimum_pair_distance_m = min(
                            minimum_pair_distance_m,
                            math.dist(positions[0][:2], positions[1][:2]),
                        )
                    final_positions = samples[-1]
                    displacements = [
                        math.dist(starts[index][:2], final_positions[index][:2])
                        for index in range(2)
                    ]
                    path_points = [native.path_points(agent_id) for agent_id in agent_ids]
                    destroyed = native.destroy_agents(agent_ids)
                    bridge_result = {
                        "status": "SIMULATED",
                        "module": str(bridge_binary),
                        "available": bool(native_bridge.available()),
                        "created": bool(created),
                        "goals_updated": bool(goals_updated),
                        "destroyed": bool(destroyed),
                        "starts": starts,
                        "goals": goals,
                        "final_positions": final_positions,
                        "displacements_m": displacements,
                        "minimum_pair_distance_m": minimum_pair_distance_m,
                        "path_point_counts": [len(points) for points in path_points],
                    }
                except Exception as exc:
                    bridge_result = {
                        "status": "SIMULATION_FAILED",
                        "module": str(bridge_binary),
                        "error": repr(exc),
                    }
            print(
                "GATE6B_NATIVE_CONTROLLER_PROBE="
                + json.dumps(
                    {
                        "module": str(nav.__file__),
                        "navigation_type": str(type(navigation)),
                        "navigation_methods": sorted(
                            name for name in dir(navigation) if not name.startswith("_")
                        ),
                        "navmesh_type": str(type(guard.navmesh)),
                        "navmesh_methods": navmesh_methods,
                        "controller_factory_candidates": controller_factories,
                        "controller": controller_result,
                        "native_bridge": bridge_result,
                    }
                ),
                flush=True,
            )
            return 0
        count = int(single.ARGS.pedestrian_count)
        arena_pedestrians = select_arena_pedestrians(
            load_arena_pedestrians(single.ARGS.arena_scenario), count
        )
        if single.ARGS.duration < 2.0:
            raise ValueError("crowd validation requires --duration >= 2")
        if single.ARGS.pedestrian_speed is not None and single.ARGS.pedestrian_speed <= 0.0:
            raise ValueError("--pedestrian-speed must be positive")
        if not 0.0 < single.ARGS.pedestrian_speed_min <= single.ARGS.pedestrian_speed_max:
            raise ValueError("pedestrian speed bounds must be positive and ordered")
        if single.ARGS.pedestrian_update_rate not in (15, 20, 30, 60):
            raise ValueError("--pedestrian-update-rate must be one of 15,20,30,60")
        if single.ARGS.gui_render_rate not in (5, 10, 15, 20, 30, 60):
            raise ValueError("--gui-render-rate must be one of 5,10,15,20,30,60")
        if not 0.0 <= single.ARGS.native_agent_padding <= 0.50:
            raise ValueError("--native-agent-padding must be between 0.0 and 0.50 m")
        if single.ARGS.native_stall_replan_after < 0.0:
            raise ValueError("--native-stall-replan-after must be non-negative")
        if (
            single.ARGS.hunav_executor != "projected_pose"
            and single.ARGS.social_mode != "hunav"
        ):
            raise ValueError("--hunav-executor native_controller requires --social-mode hunav")
        if single.ARGS.gate6_interaction_profile is not None:
            if not single.ARGS.mobile_robot:
                raise ValueError("--gate6-interaction-profile requires --mobile-robot")
            if single.ARGS.social_mode != "hunav":
                raise ValueError("--gate6-interaction-profile requires --social-mode hunav")
            if single.ARGS.duration < MINIMUM_PROFILE_DURATION_SEC:
                raise ValueError(
                    "--gate6-interaction-profile requires duration >= "
                    f"{MINIMUM_PROFILE_DURATION_SEC:.1f} seconds"
                )
        robot_spawn = np.asarray(
            [single.ARGS.mobile_robot_spawn_x, single.ARGS.mobile_robot_spawn_y, 0.01],
            dtype=float,
        )
        if not np.all(np.isfinite(robot_spawn)):
            raise ValueError("mobile robot spawn coordinates must be finite")
        for path in (
            single.SCENE_USD,
            single.BIPED_USD,
            single.WALK_USD,
            *single.CHARACTER_USDS,
        ):
            if not path.is_file():
                raise FileNotFoundError(path)

        success, error = single.wait_for_task(
            asyncio.ensure_future(single.omni.usd.get_context().open_stage_async(str(single.SCENE_USD))),
            single.ARGS.setup_timeout,
            "opening the active lobby for crowd validation",
        )
        if not success:
            raise RuntimeError(f"Could not open active lobby: {error}")
        for _ in range(8):
            single.simulation_app.update()
        stage = single.omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Lobby open completed without a stage")
        stage.SetTimeCodesPerSecond(60.0)
        stage.SetFramesPerSecond(60.0)
        stage.SetStartTimeCode(0.0)
        stage.SetEndTimeCode(max(480.0, single.ARGS.duration * 60.0 + 60.0))

        rng = np.random.default_rng(single.ARGS.pedestrian_seed)
        people = []
        for index, arena_person in enumerate(arena_pedestrians):
            paths = person_paths(arena_person.name)
            route = arena_person.route
            start = route[0]
            speed = (
                float(single.ARGS.pedestrian_speed)
                if single.ARGS.pedestrian_speed is not None
                else (
                    float(arena_person.desired_velocity)
                    if arena_person.desired_velocity is not None
                    else float(rng.uniform(single.ARGS.pedestrian_speed_min, single.ARGS.pedestrian_speed_max))
                )
            )
            root = single.UsdGeom.Xform.Define(stage, paths["root"])
            root.GetPrim().CreateAttribute("arena:stableId", single.Sdf.ValueTypeNames.String).Set(paths["stable_id"])
            root.GetPrim().CreateAttribute("arena:trackId", single.Sdf.ValueTypeNames.UInt64).Set(arena_person.track_id)
            root.GetPrim().CreateAttribute("arena:waypointMode", single.Sdf.ValueTypeNames.Int).Set(arena_person.waypoint_mode)
            root.GetPrim().CreateAttribute("arena:scenarioSource", single.Sdf.ValueTypeNames.Asset).Set(str(single.ARGS.arena_scenario))
            root.GetPrim().CreateAttribute("arena:walkSpeed", single.Sdf.ValueTypeNames.Float).Set(speed)
            root.GetPrim().CreateAttribute("arena:routeStart", single.Sdf.ValueTypeNames.Double3).Set(start)
            root.GetPrim().CreateAttribute("arena:routeGoal", single.Sdf.ValueTypeNames.Double3).Set(route[-1])
            root.GetPrim().CreateAttribute("arena:routeLength", single.Sdf.ValueTypeNames.Float).Set(
                polyline_length(route)
            )
            character_usd = single.CHARACTER_USDS[index % len(single.CHARACTER_USDS)]
            visual = single.UsdSkel.Root.Define(stage, paths["visual"])
            visual.GetPrim().GetReferences().AddReference(str(character_usd))
            people.append(
                {
                    "index": index,
                    "arena_person": arena_person,
                    "track_id": arena_person.track_id,
                    "arena_name": arena_person.name,
                    "waypoint_mode": arena_person.waypoint_mode,
                    "paths": paths,
                    "root": root,
                    "route": route,
                    "speed": speed,
                    "character_usd": character_usd,
                }
            )

        for _ in range(12):
            single.simulation_app.update()

        source_skeleton = single.create_retarget_source(stage)
        animation_rows = []
        pose_rows = []
        for person in people:
            paths = person["paths"]
            skeleton = single.first_skeleton(stage, paths["visual"])
            animation, animation_info = single.retarget_walk_to_character(
                stage,
                source_skeleton,
                skeleton,
                source_phase_index=person["index"] * 9,
            )
            paths["skeleton"] = str(skeleton.GetPrim().GetPath())
            paths["animation"] = str(animation.GetPrim().GetPath())
            animation_rows.append(animation_info)
            pose_rows.append(single.skeleton_pose_evidence(skeleton, paths["visual"]))
            person["gait_driver"] = single.RuntimeGaitDriver(
                animation,
                preferred_speed_mps=person["speed"],
                target_time_codes_per_second=animation_info["target_time_codes_per_second"],
            )
        stage.RemovePrim(single.RETARGET_SOURCE_ROOT)

        world = single.World(
            physics_dt=single.PHYSICS_DT,
            rendering_dt=single.PHYSICS_DT,
            stage_units_in_meters=1.0,
        )
        robot_usd = None
        if single.ARGS.mobile_robot:
            if single.ARGS.no_ros:
                raise ValueError("--mobile-robot requires ROS command/odometry")
            if single.ARGS.command_timeout <= 0.0:
                raise ValueError("--command-timeout must be positive")
            from gate9_mobile_robot import Gate9MobileMecanum

            robot_usd = (
                single.PROJECT_ROOT.parent
                / "robot_related/robots/chassis_arm/motion_wheel_arm_simple_sphere_usd"
                / "mecanum730_xms5_default.usd"
            )
            if not robot_usd.is_file():
                raise FileNotFoundError(robot_usd)
        world.reset()
        query = single.omni.physx.get_physx_scene_query_interface()
        navmesh_guard = None
        if single.ARGS.social_mode == "hunav":
            import omni.kit.app

            extension_manager = omni.kit.app.get_app().get_extension_manager()
            extension_manager.set_extension_enabled_immediate("omni.anim.navigation.bundle", True)
            for _ in range(4):
                single.simulation_app.update()
            from isaac_navmesh import IsaacNavMeshGuard

            navmesh_guard = single.wait_for_task(
                # Plan and project with a 2 cm inset so acceleration-bounded
                # steering remains feasible at sharp corners.  Committed-pose
                # auditing still uses the physical 0.25 m capsule radius.
                asyncio.ensure_future(IsaacNavMeshGuard.bake(
                    radius_m=0.27, audit_radius_m=0.25
                )),
                single.ARGS.setup_timeout,
                "baking the authored Isaac NavMesh",
            )
            for person in people:
                person["arena_route"] = person["route"]
                # Preserve the generator's obstacle-safe topology, but project
                # each short local leg onto Isaac's baked agent-radius NavMesh.
                # A single shortest path between semantic endpoints can choose
                # a different corridor than the generator and is therefore not
                # a safe replacement for the complete resolved route.
                person["route"] = navmesh_guard.resolve_route(person["route"])
                if not navmesh_guard.point_is_safe(person["route"][0]):
                    raise RuntimeError(
                        f"{person['paths']['stable_id']}: initial route point is outside NavMesh"
                    )
                semantic = person["arena_person"].semantic_route
                semantic_cycle = (person["route"][0], *semantic, semantic[0])
                person["semantic_navmesh_preflight"] = [
                    navmesh_guard.path(start, end, end_tolerance_m=0.50)
                    for start, end in zip(semantic_cycle, semantic_cycle[1:])
                ]
        route_rows = []
        all_routes_clear = True
        for person in people:
            route_ok, segments = polyline_route_clearance(query, person["route"])
            route_rows.append(
                {
                    "stable_id": person["paths"]["stable_id"],
                    "clear": route_ok,
                    "route": [list(point) for point in person["route"]],
                    "route_length_m": polyline_length(person["route"]),
                    "segments": segments,
                }
            )
            all_routes_clear = all_routes_clear and route_ok

        if not all_routes_clear:
            failed_routes = [row["stable_id"] for row in route_rows if not row["clear"]]
            raise RuntimeError(
                "one or more requested crowd patrols failed actual PhysX ground/radial clearance; "
                f"the requested population was not reduced; failed={failed_routes}"
            )

        # Static route qualification must not see the movable robot proxy as
        # authored environment geometry.  Compose it only after the PhysX
        # ground/radial audit; the later shared world.reset() initializes it
        # together with the pedestrian capsules for real contact and LiDAR.
        if robot_usd is not None:
            mobile = Gate9MobileMecanum(
                stage, world, robot_usd, spawn=robot_spawn
            )

        requested_starts = [np.asarray(person["route"][0][:2], dtype=float) for person in people]
        initial_pair_distance = min(
            float(np.linalg.norm(first - second))
            for first_index, first in enumerate(requested_starts)
            for second in requested_starts[first_index + 1:]
        ) if len(requested_starts) > 1 else math.inf
        if initial_pair_distance < 2.0 * CrowdSocialConfig().agent_radius_m + 0.20:
            raise RuntimeError(
                "requested crowd starts violate the 0.70 m human separation contract; "
                "the requested population was not reduced"
            )

        social_config = CrowdSocialConfig(
            human_repulsion_mps2=1.35 if single.ARGS.social_mode == "social" else 0.0,
            robot_repulsion_mps2=1.80 if single.ARGS.social_mode == "social" else 0.0,
        )
        agents = []
        for person in people:
            start = np.asarray(person["route"][0][:2], dtype=float)
            initial_yaw = math.atan2(
                person["route"][1][1] - person["route"][0][1],
                person["route"][1][0] - person["route"][0][0],
            )
            agents.append(
                CrowdAgent(
                    stable_id=person["paths"]["stable_id"],
                    track_id=person["track_id"],
                    cursor=PatrolCursor(tuple((point[0], point[1]) for point in person["route"])),
                    preferred_speed_mps=person["speed"],
                    position_xy=start,
                    yaw_rad=initial_yaw,
                    animation_phase=float(
                        rng.uniform(0.0, person["gait_driver"].cycle_distance_m)
                    ),
                )
            )
        crowd_controller = (
            None
            if single.ARGS.social_mode == "hunav"
            else CrowdSocialController(agents, social_config)
        )
        hunav_limits = SafetyConfig(human_radius_m=0.25, personal_space_m=0.70)
        hunav_projector = (
            SafeVelocityProjector(
                hunav_limits
            )
            if (
                single.ARGS.social_mode == "hunav"
                and single.ARGS.hunav_executor == "projected_pose"
            )
            else None
        )
        # HuNav/LightSFM owns pedestrian intent and social interaction.  Do not
        # replace its joint result with a second, per-agent recovery policy.
        recovery_supervisors = {}
        route_runtimes = {}
        route_planners = {}
        if single.ARGS.social_mode == "hunav":
            def make_plan_local(reference_route):
                reference = tuple(reference_route)

                def plan_local(start, goal):
                    candidate = navmesh_guard.replan_path(start, goal)
                    if candidate is not None:
                        if static_polyline_is_clear(
                            query, candidate, clearance_m=0.27
                        ):
                            return candidate

                    # A global shortest path can disagree with the actual
                    # PhysX clearance audit.  Query a real NavMesh join from
                    # the current pose to the nearest safe point on this
                    # person's validated cyclic route, then follow that route
                    # forward to the active semantic waypoint.
                    unique_count = len(reference) - int(
                        len(reference) > 1
                        and math.dist(reference[0][:2], reference[-1][:2]) <= 1.0e-6
                    )
                    goal_index = min(
                        range(unique_count),
                        key=lambda index: math.dist(reference[index][:2], goal[:2]),
                    )
                    join_indices = sorted(
                        range(unique_count),
                        key=lambda index: math.dist(reference[index][:2], start[:2]),
                    )
                    for join_index in join_indices[:24]:
                        join = navmesh_guard.replan_path(start, reference[join_index])
                        if join is None:
                            continue
                        if not static_polyline_is_clear(
                            query, join, clearance_m=0.27
                        ):
                            continue
                        if join_index <= goal_index:
                            suffix = reference[join_index + 1 : goal_index + 1]
                        else:
                            suffix = (
                                reference[join_index + 1 : unique_count]
                                + reference[: goal_index + 1]
                            )
                        repaired = tuple(join) + tuple(suffix)
                        return repaired
                    navmesh_guard.replan_failure_count += 1
                    return None

                return plan_local

            for person, agent in zip(people, agents):
                plan_local = make_plan_local(person["route"])
                source = person["arena_person"]
                route_state = SemanticRouteRuntime(
                    semantic_route=source.semantic_route,
                    goal_radius_m=source.goal_radius,
                    cyclic=source.cyclic_goals,
                )
                if not route_state.initialize_resolved(
                    (agent.position_xy[0], agent.position_xy[1], 0.0),
                    # HuNav consumes a deque and advances at most one goal per
                    # update.  Keep its targets at the generator's validated
                    # ~1 m spacing; the dense NavMesh curve remains the
                    # execution/free-space authority only.
                    person["arena_route"],
                ):
                    raise RuntimeError(
                        f"{agent.stable_id}: generated resolved route could not be installed"
                    )
                route_runtimes[agent.track_id] = route_state
                route_planners[agent.track_id] = plan_local

            if single.ARGS.hunav_executor == "native_controller":
                bridge_dir = Path(__file__).resolve().parents[1] / "native_controller_bridge"
                bridge_binary = next(bridge_dir.glob("_isaac5_native_controller*.so"), None)
                if bridge_binary is None:
                    raise RuntimeError(
                        "Isaac 5 native controller bridge is not built; run "
                        "isaac_sim/backends/isaac5/native_controller_bridge/build.sh"
                    )
                if str(bridge_dir) not in sys.path:
                    sys.path.insert(0, str(bridge_dir))
                import _isaac5_native_controller as native_bridge

                native_hunav_controller = native_bridge.NativeController(
                    obstacle_padding=0.10,
                    passage_padding=1.0,
                    agent_padding=single.ARGS.native_agent_padding,
                    agent_padding_min_velocity=0.10,
                )
                native_hunav_agent_ids = [agent.track_id for agent in agents]
                native_starts = [
                    [float(agent.position_xy[0]), float(agent.position_xy[1]), 0.0]
                    for agent in agents
                ]
                native_hunav_islands = {}
                for agent, start in zip(agents, native_starts):
                    found, projected_start, island_id = native_hunav_controller.closest_point(
                        start, agent_radius=0.25, agent_height=1.70
                    )
                    if not found or island_id < 0:
                        raise RuntimeError(
                            f"{agent.stable_id}: native controller could not resolve start island"
                        )
                    if math.dist(start[:2], projected_start[:2]) > 0.05:
                        raise RuntimeError(
                            f"{agent.stable_id}: native start projection exceeded 0.05 m"
                        )
                    native_hunav_islands[agent.track_id] = int(island_id)
                if not native_hunav_controller.create_agents(
                    native_hunav_agent_ids,
                    native_starts,
                    [float(agent.preferred_speed_mps) for agent in agents],
                    [0.25] * len(agents),
                    [1.70] * len(agents),
                ):
                    raise RuntimeError("Isaac 5 native controller did not create every crowd agent")
                native_hunav_command_speeds = {
                    agent.track_id: 0.0 for agent in agents
                }
                native_low_command_seconds = {
                    agent.track_id: 0.0 for agent in agents
                }
                native_yield_planner = IndependentYieldPlanner()
                # The native controller already performs joint avoidance.  Keep
                # the old 2 cm reserve as the default, but expose narrower
                # emergency-only and controller-only modes for same-seed A/B
                # diagnosis.  The latter is not a safety acceptance mode.
                native_pair_projection_mode = single.ARGS.native_pair_projection
                native_pair_radius_m = {
                    "full": 0.27,
                    "emergency_reserve": 0.255,
                    "emergency_converged": 0.25,
                    "kinematic_converged": 0.25,
                    "emergency": 0.25,
                    "off": 0.25,
                }[native_pair_projection_mode]
                native_output_projector = SafeVelocityProjector(
                    SafetyConfig(
                        human_radius_m=native_pair_radius_m,
                        personal_space_m=0.70,
                        max_speed_mps=(
                            100.0
                            if single.ARGS.native_post_kinematics == "controller_direct"
                            else 1.5
                        ),
                        max_acceleration_mps2=(
                            100.0
                            if single.ARGS.native_post_kinematics == "controller_direct"
                            else 1.2
                        ),
                        max_deceleration_mps2=(
                            100.0
                            if single.ARGS.native_post_kinematics == "controller_direct"
                            else 2.2
                        ),
                        enforce_pair_separation=(native_pair_projection_mode != "off"),
                        pair_projection_iterations=(
                            24
                            if native_pair_projection_mode in {
                                "emergency_converged", "kinematic_converged"
                            }
                            else 8
                        ),
                        pair_unilateral_hold_fallback=(
                            native_pair_projection_mode == "emergency_converged"
                        ),
                        pair_kinematic_feasibility=(
                            native_pair_projection_mode == "kinematic_converged"
                        ),
                        static_lookahead_sec=0.75,
                    )
                )
                native_hunav_metrics = {
                    "goal_update_count": 0,
                    "goal_projection_count": 0,
                    "goal_projection_failure_count": 0,
                    "update_return_true_count": 0,
                    "simulation_count": 0,
                    "maximum_goal_projection_m": 0.0,
                    "liveness_recovery_count": 0,
                    "liveness_yield_sample_count": 0,
                    "stall_replan_attempt_count": 0,
                    "stall_replan_success_count": 0,
                }
        for person, agent in zip(people, agents):
            single.set_person_pose(
                person["root"], np.asarray([agent.position_xy[0], agent.position_xy[1], 0.0]), agent.yaw_rad
            )
            person["gait_driver"].apply(agent.animation_phase, force=True)
            person["agent"] = agent

        for person in people:
            paths = person["paths"]
            start = person["route"][0]
            collider = world.scene.add(
                single.DynamicCapsule(
                    prim_path=paths["collider"],
                    name=f"gate6b_{paths['stable_id']}_collider",
                    position=np.asarray([start[0], start[1], 0.85]),
                    radius=0.25,
                    height=1.2,
                    color=np.asarray([0.1, 0.1, 0.1]),
                    mass=70.0,
                )
            )
            collider_prim = stage.GetPrimAtPath(paths["collider"])
            single.UsdGeom.Imageable(collider_prim).MakeInvisible()
            person["collider_object"] = collider
            person["collider_prim"] = collider_prim

        world.reset()
        # Initialize DynamicCapsule objects while they are dynamic; their
        # post_reset hook writes zero velocities, which PhysX rejects after a
        # body has already been made kinematic. Route-following begins only
        # after this toggle, so collision and LiDAR behavior are unchanged.
        for person in people:
            single.UsdPhysics.RigidBodyAPI(
                person["collider_prim"]
            ).CreateKinematicEnabledAttr().Set(True)
        if mobile is not None:
            mobile.initialize()
        timeline = single.omni.timeline.get_timeline_interface()
        timeline.set_looping(True)
        world.play()
        if not single.ARGS.headless:
            camera_eye = (
                np.asarray([7.0, 2.0, 4.5])
                if mobile is not None
                else np.asarray([16.0, -10.0, 31.0])
            )
            camera_target = (
                np.asarray([3.4, 3.4, 0.8])
                if mobile is not None
                else np.asarray([16.0, 12.0, 0.0])
            )
            single.set_camera_view(
                eye=camera_eye,
                target=camera_target,
                camera_prim_path="/OmniverseKit_Persp",
            )
        ros = None if single.ARGS.no_ros else single.RosPublisher(
            publish_tracks=not single.ARGS.suppress_tracks,
            mobile_robot=single.ARGS.mobile_robot,
            command_topic=single.ARGS.command_topic,
            command_timeout=single.ARGS.command_timeout,
        )
        hunav_bridge = None
        if single.ARGS.social_mode == "hunav":
            if ros is None:
                raise ValueError("HuNav mode requires ROS; remove --no-ros")
            from hunav_isaac_bridge import IsaacHuNavBridge

            hunav_bridge = IsaacHuNavBridge(
                ros.node, ros.string_type, command_timeout=single.ARGS.hunav_command_timeout
            )
        lidar = single.DualPhysxRaycastLidar(
            sample_count=2000,
            rate_hz=single.SCAN_RATE_HZ,
            range_min=0.5,
            range_max=50.0,
        )

        # The corridors below were accepted by the actual PhysX ground and
        # radial-clearance audit above.  The controller calls this predicate
        # before every pose application, so social lateral steering cannot
        # leave a physically pre-qualified corridor.  It deliberately does
        # not raycast after capsules exist because a capsule would self-hit.
        def segment_is_safe(_start_xy, end_xy) -> bool:
            if navmesh_guard is not None:
                return navmesh_guard.segment_is_safe(_start_xy, end_xy)
            return any(
                point_to_segment_distance(end_xy, start, end) <= 0.34
                for person in people
                for start, end in zip(person["route"][:-1], person["route"][1:])
            )

        def project_segment(agent_id, start_xy, end_xy):
            if navmesh_guard is None:
                return end_xy if segment_is_safe(start_xy, end_xy) else None
            route_state = route_runtimes.get(agent_id)
            if route_state is None:
                return None
            goal_xy = route_state.goals_xy()[0]
            return navmesh_guard.project_step(start_xy, end_xy, goal_xy)

        print(
            "GATE6B_CROWD_READY="
            + json.dumps(
                {
                    "scene": str(single.SCENE_USD),
                    "pedestrian_mode": "arena_scenario",
                    "arena_scenario": str(single.ARGS.arena_scenario),
                    "pedestrian_count": count,
                    "track_ids": list(range(1, count + 1)),
                    "pedestrian_seed": single.ARGS.pedestrian_seed,
                    "social_mode": single.ARGS.social_mode,
                    "hunav_executor": (
                        single.ARGS.hunav_executor
                        if single.ARGS.social_mode == "hunav" else None
                    ),
                    "hunav_execution_backend": (
                        (
                            "host_hunav_lightsfm_joint_compute_with_native_persistent_nav_controller"
                            if single.ARGS.hunav_executor == "native_controller"
                            else "host_hunav_lightsfm_joint_compute_with_hard_limit_executor"
                        )
                        if single.ARGS.social_mode == "hunav" else None
                    ),
                    "native_pair_projection": (
                        single.ARGS.native_pair_projection
                        if single.ARGS.hunav_executor == "native_controller" else None
                    ),
                    "native_post_kinematics": (
                        single.ARGS.native_post_kinematics
                        if single.ARGS.hunav_executor == "native_controller" else None
                    ),
                    "native_state_sync": (
                        single.ARGS.native_state_sync
                        if single.ARGS.hunav_executor == "native_controller" else None
                    ),
                    "native_stall_recovery": (
                        single.ARGS.native_stall_recovery
                        if single.ARGS.hunav_executor == "native_controller" else None
                    ),
                    "native_recovery_goal": (
                        single.ARGS.native_recovery_goal
                        if single.ARGS.hunav_executor == "native_controller" else None
                    ),
                    "native_stall_replan_after_sec": (
                        single.ARGS.native_stall_replan_after
                        if single.ARGS.hunav_executor == "native_controller" else None
                    ),
                    "native_intent_mode": (
                        single.ARGS.native_intent_mode
                        if single.ARGS.hunav_executor == "native_controller" else None
                    ),
                    "native_agent_padding_m": (
                        single.ARGS.native_agent_padding
                        if single.ARGS.hunav_executor == "native_controller" else None
                    ),
                    "pedestrian_control_rate_hz": single.ARGS.pedestrian_update_rate,
                    "pedestrian_pose_commit_rate_hz": single.ARGS.pedestrian_update_rate,
                    "gui_render_rate_hz": None if single.ARGS.headless else single.ARGS.gui_render_rate,
                    "ros": ros is not None,
                    "mobile_robot": mobile is not None,
                    "command_topic": single.ARGS.command_topic if mobile is not None else None,
                }
            ),
            flush=True,
        )

        position_history = [[] for _ in people]
        collider_errors = [0.0 for _ in people]
        lidar_hits = {person["paths"]["stable_id"]: [0, 0] for person in people}
        min_pair_distance = math.inf
        min_pair_ids = None
        pair_observations = 0
        personal_space_violation_observations = 0
        min_robot_person_distance = math.inf
        min_robot_person_rectangle_clearance = math.inf
        lidar_samples = 0
        robot_positions = []
        applied_commands = []
        gate6_phase_names: list[str] = []
        gate6_phase_sample_counts: dict[str, int] = {}
        narrow_wait_seconds = {agent.track_id: 0.0 for agent in agents}
        narrow_wait_current_seconds = {agent.track_id: 0.0 for agent in agents}
        narrow_route_progress_at_block_end = {
            agent.track_id: None for agent in agents
        }
        facing_errors = [[] for _ in people]
        unsafe_pose_commit_count = 0
        maximum_committed_navmesh_error_m = 0.0
        unsafe_pose_commit_samples = []
        execution_metrics = {
            agent.track_id: {
                "raw_speed_sum": 0.0,
                "raw_speed_max": 0.0,
                "executed_speed_sum": 0.0,
                "executed_speed_max": 0.0,
                "executed_acceleration_max": 0.0,
                "samples": 0,
                "safety_modification_count": 0,
                "safety_reasons": {},
                "timeout_stop_count": 0,
                "static_boundary_hold_count": 0,
                "minimum_separation_m": math.inf,
                "behavior_state_counts": {},
            }
            for agent in agents
        }
        last_applied_hunav_command_sequence = -1
        goal_sync_sample_count = 0
        goal_sync_mismatch_count = 0
        maximum_goal_sync_error_m = 0.0
        maximum_hunav_active_goal_count = 0
        scared_evidence = {
            agent.track_id: {
                "stable_id": agent.stable_id,
                "trigger_time": None,
                "trigger_distance_m": None,
                "trigger_behavior_state": None,
                "maximum_away_radial_velocity_mps": -math.inf,
                "maximum_away_radial_velocity_after_trigger_mps": -math.inf,
                "maximum_away_radial_velocity_during_interaction_mps": -math.inf,
                "minimum_robot_rectangle_net_clearance_m": math.inf,
                "maximum_robot_distance_after_trigger_m": 0.0,
                "recovery_time": None,
                "route_progress_at_trigger_m": None,
                "final_route_progress_m": None,
                "original_route_recovered": False,
                "trigger_source": None,
                "behavior_state_nonzero_observed": False,
            }
            for person, agent in zip(people, agents)
            if person["arena_person"].behavior == "scared"
        }
        previous_robot_observation = None
        sim_start = float(world.current_time)
        wall_start = time.monotonic()
        total_steps = int(math.ceil(single.ARGS.duration / single.PHYSICS_DT))
        # Keep physics at 60 Hz and sensors at 15 Hz.  Social control plus
        # Character/capsule pose authoring and GUI rendering are independently
        # divisible schedules; 30 Hz crowd updates remain above the sensor rate
        # and reduce 20-character Python/USD/PhysX load.
        pedestrian_update_stride = int(round(
            (1.0 / single.PHYSICS_DT) / single.ARGS.pedestrian_update_rate
        ))
        gui_render_stride = int(round(
            (1.0 / single.PHYSICS_DT) / single.ARGS.gui_render_rate
        ))
        for step in range(total_steps):
            sim_time = float(world.current_time - sim_start)
            command = (0.0, 0.0)
            gate6_phase_name = None
            if ros is not None:
                ros.spin_once()
            if mobile is not None:
                if single.ARGS.gate6_interaction_profile == PROFILE_NAME:
                    gate6_phase_name, command = command_for_time(
                        sim_time, single.ARGS.gate6_interaction_profile
                    )
                    if not gate6_phase_names or gate6_phase_names[-1] != gate6_phase_name:
                        gate6_phase_names.append(gate6_phase_name)
                    gate6_phase_sample_counts[gate6_phase_name] = (
                        gate6_phase_sample_counts.get(gate6_phase_name, 0) + 1
                    )
                elif single.ARGS.gate6_interaction_profile == NARROW_PROFILE_NAME:
                    gate6_phase_name, command = command_for_time(
                        sim_time, single.ARGS.gate6_interaction_profile
                    )
                    if not gate6_phase_names or gate6_phase_names[-1] != gate6_phase_name:
                        gate6_phase_names.append(gate6_phase_name)
                    gate6_phase_sample_counts[gate6_phase_name] = (
                        gate6_phase_sample_counts.get(gate6_phase_name, 0) + 1
                    )
                else:
                    command = ros.current_command()
                mobile.apply_command(*command)
            prior_robot_position = np.asarray(robot_spawn[:2], dtype=float)
            prior_robot_velocity = np.zeros(2, dtype=float)
            if mobile is not None:
                prior_robot_position_3d, prior_robot_yaw = mobile.pose()
                prior_robot_position = np.asarray(prior_robot_position_3d[:2], dtype=float)
                if previous_robot_observation is not None:
                    previous_position, previous_yaw, previous_time = previous_robot_observation
                    observed_dt = max(sim_time - previous_time, 1.0e-6)
                    prior_robot_velocity = (prior_robot_position - previous_position) / observed_dt
                    prior_robot_angular_velocity = math.atan2(
                        math.sin(prior_robot_yaw - previous_yaw),
                        math.cos(prior_robot_yaw - previous_yaw),
                    ) / observed_dt
                else:
                    prior_robot_velocity = np.zeros(2, dtype=float)
                    prior_robot_angular_velocity = 0.0
                previous_robot_observation = (
                    prior_robot_position.copy(), float(prior_robot_yaw), sim_time
                )
            else:
                prior_robot_yaw = 0.0
                prior_robot_angular_velocity = 0.0
            pedestrian_update_due = step % pedestrian_update_stride == 0
            if pedestrian_update_due:
                control_dt = single.PHYSICS_DT * pedestrian_update_stride
                if hunav_bridge is None:
                    crowd_controller.step(
                        control_dt,
                        prior_robot_position,
                        prior_robot_velocity,
                        segment_is_safe,
                    )
                else:
                    robot_yaw_for_state = prior_robot_yaw if mobile is not None else 0.0
                    if mobile is not None:
                        half_x = float(mobile.proxy_dimensions[0]) * 0.5
                        half_y = float(mobile.proxy_dimensions[1]) * 0.5
                    else:
                        half_x, half_y = 0.45, 0.32
                    robot_footprint = [
                        [half_x, half_y], [half_x, -half_y],
                        [-half_x, -half_y], [-half_x, half_y],
                    ]
                    agent_state_rows = []
                    for person in people:
                        agent = person["agent"]
                        source = person["arena_person"]
                        route_state = route_runtimes[agent.track_id]
                        route_state.update(
                            (agent.position_xy[0], agent.position_xy[1], 0.0),
                            sim_time,
                            route_planners[agent.track_id],
                        )
                        behavior = dict(source.behavior_parameters)
                        behavior["type"] = source.behavior
                        behavior.setdefault("velocity", person["speed"])
                        behavior.setdefault("interaction_distance", source.interaction_distance)
                        behavior.setdefault("social_force_factor", source.social_force_factor)
                        agent_state_rows.append({
                            "id": agent.track_id,
                            "track_id": agent.track_id,
                            "stable_id": agent.stable_id,
                            "name": agent.stable_id,
                            "position": agent.position_xy.tolist(),
                            "velocity": agent.velocity_xy.tolist(),
                            "yaw": agent.yaw_rad,
                            "radius": source.radius,
                            "goal_radius": source.goal_radius,
                            "desired_velocity": person["speed"] if source.desired_velocity is None else source.desired_velocity,
                            "group_id": source.group_id,
                            # SemanticRouteRuntime is the only route cursor.
                            # The task-owned HuNav manager runs with
                            # external_goals_authoritative=true and replaces
                            # its LightSFM goal deque with this single current
                            # validated goal on every joint update.
                            "goals": route_state.goals_xy()[:1],
                            "cyclic_goals": source.cyclic_goals,
                            "behavior": behavior,
                            "closest_obstacles": sample_static_obstacle_points(
                                query, agent.position_xy, radius_m=source.radius, ray_count=12
                            ),
                        })
                    robot_radius = math.hypot(half_x, half_y) + 0.12
                    hunav_bridge.publish_state(sim_time, {
                        "id": 0,
                        "name": "isaac_mecanum",
                        "position": prior_robot_position.tolist(),
                        "velocity": prior_robot_velocity.tolist(),
                        "yaw": robot_yaw_for_state,
                        "angular_velocity": prior_robot_angular_velocity,
                        "radius": robot_radius,
                        "footprint": robot_footprint,
                    }, agent_state_rows)
                    commands, fallback_reason = hunav_bridge.commands(sim_time)
                    command_sequence = hunav_bridge.inbox.last_sequence
                    fresh_command = (
                        fallback_reason is None
                        and command_sequence != last_applied_hunav_command_sequence
                    )
                    if fresh_command:
                        for row in commands.values():
                            sync_error = float(row.get("goal_sync_error_m", math.inf))
                            active_goal_count = int(row.get("active_goal_count", 0))
                            goal_sync_sample_count += 1
                            maximum_goal_sync_error_m = max(
                                maximum_goal_sync_error_m, sync_error
                            )
                            maximum_hunav_active_goal_count = max(
                                maximum_hunav_active_goal_count, active_goal_count
                            )
                            if sync_error > 1.0e-6 or active_goal_count != 1:
                                goal_sync_mismatch_count += 1
                    positions = {agent.track_id: agent.position_xy.copy() for agent in agents}
                    velocities = {agent.track_id: agent.velocity_xy.copy() for agent in agents}
                    desired = {}
                    for agent in agents:
                        row = commands.get(agent.track_id)
                        if row is None or not fresh_command:
                            vector = np.zeros(2, dtype=float)
                        else:
                            source_position = np.asarray(row["source_position"], dtype=float)
                            target_position = np.asarray(row["target_position"], dtype=float)
                            vector = (target_position - source_position) / control_dt
                        desired[agent.track_id] = vector
                    if native_hunav_controller is None:
                        projected = hunav_projector.project(
                            positions, velocities, desired, control_dt,
                            prior_robot_position, prior_robot_velocity, robot_footprint,
                            segment_is_safe,
                            segment_projector=project_segment,
                            robot_yaw_rad=prior_robot_yaw,
                        )
                        execution_reasons = hunav_projector.last_reasons
                    else:
                        native_goals = []
                        native_speeds = []
                        execution_reasons = {}
                        stalled_ids = set()
                        for agent in agents:
                            executed_speed = float(np.linalg.norm(agent.velocity_xy))
                            if fresh_command and executed_speed < 0.12:
                                native_low_command_seconds[agent.track_id] += control_dt
                            else:
                                native_low_command_seconds[agent.track_id] = 0.0
                            if native_low_command_seconds[agent.track_id] >= 1.5:
                                stalled_ids.add(agent.track_id)
                        yield_ids = set()
                        if single.ARGS.native_stall_recovery == "independent_yield":
                            yield_ids = native_yield_planner.update(
                                positions, stalled_ids, sim_time
                            )
                            recovery_ids = stalled_ids - yield_ids
                            native_hunav_metrics["liveness_yield_sample_count"] += len(
                                yield_ids
                            )
                        else:
                            recovery_ids = set()
                            unassigned = set(stalled_ids)
                            while unassigned:
                                seed_id = min(unassigned)
                                component = {seed_id}
                                frontier = [seed_id]
                                while frontier:
                                    current_id = frontier.pop()
                                    current_position = positions[current_id]
                                    neighbours = {
                                        other_id
                                        for other_id in unassigned
                                        if other_id not in component
                                        and float(np.linalg.norm(
                                            positions[other_id] - current_position
                                        )) < 1.0
                                    }
                                    component.update(neighbours)
                                    frontier.extend(neighbours)
                                recovery_ids.add(min(component))
                                unassigned.difference_update(component)
                        for agent in agents:
                            raw_velocity = np.asarray(desired[agent.track_id], dtype=float)
                            reasons = []
                            if raw_velocity.shape != (2,) or not np.all(np.isfinite(raw_velocity)):
                                raw_velocity = np.zeros(2, dtype=float)
                                reasons.append("nonfinite")
                            steering_velocity = raw_velocity.copy()
                            recovery_goal = None
                            native_target_override = None
                            source = people[agent.track_id - 1]["arena_person"]
                            if source.behavior == "regular" and single.ARGS.native_intent_mode in {
                                "route_biased", "route_goal"
                            }:
                                current_route_goal = np.asarray(
                                    route_runtimes[agent.track_id].goals_xy()[0],
                                    dtype=float,
                                )
                                route_direction = current_route_goal - agent.position_xy
                                if single.ARGS.native_intent_mode == "route_goal":
                                    route_norm = float(np.linalg.norm(route_direction))
                                    raw_speed = float(np.linalg.norm(raw_velocity))
                                    if route_norm > 1.0e-9:
                                        steering_velocity = (
                                            route_direction / route_norm * raw_speed
                                        )
                                    native_target_override = current_route_goal
                                    reasons.append("validated_route_goal_intent")
                                else:
                                    route_biased_velocity = bounded_route_social_velocity(
                                        route_direction,
                                        agent.preferred_speed_mps,
                                        raw_velocity,
                                    )
                                    if not np.allclose(
                                        route_biased_velocity,
                                        raw_velocity,
                                        atol=1.0e-6,
                                        rtol=0.0,
                                    ):
                                        reasons.append("route_biased_social_intent")
                                    steering_velocity = route_biased_velocity
                            if agent.track_id in yield_ids:
                                steering_velocity = np.zeros(2, dtype=float)
                                native_target_override = None
                                reasons.append("independent_liveness_yield")
                            elif agent.track_id in recovery_ids:
                                if (
                                    single.ARGS.native_stall_replan_after > 0.0
                                    and native_low_command_seconds[agent.track_id]
                                    >= single.ARGS.native_stall_replan_after
                                ):
                                    native_hunav_metrics["stall_replan_attempt_count"] += 1
                                    replanned = route_runtimes[agent.track_id].replan(
                                        (
                                            float(agent.position_xy[0]),
                                            float(agent.position_xy[1]),
                                            0.0,
                                        ),
                                        route_planners[agent.track_id],
                                    )
                                    native_low_command_seconds[agent.track_id] = 0.0
                                    if replanned:
                                        native_hunav_metrics["stall_replan_success_count"] += 1
                                        reasons.append("native_stall_route_replan")
                                    else:
                                        reasons.append("native_stall_route_replan_failed")
                                        if route_runtimes[
                                            agent.track_id
                                        ].skip_blocked_local_corner():
                                            reasons.append(
                                                "native_stall_skip_blocked_local_corner"
                                            )
                                remaining_goals = route_runtimes[agent.track_id].goals_xy()
                                recovery_goal = forward_recovery_goal(
                                    agent.position_xy,
                                    remaining_goals,
                                )
                                if single.ARGS.native_recovery_goal == "lateral":
                                    forward = recovery_goal - agent.position_xy
                                    forward_norm = float(np.linalg.norm(forward))
                                    if forward_norm > 1.0e-6:
                                        direction = forward / forward_norm
                                        right = np.asarray([direction[1], -direction[0]])
                                        lateral_candidates = []
                                        for rank, sign in enumerate((1.0, -1.0)):
                                            requested = recovery_goal + sign * 0.60 * right
                                            found, point, island_id = native_hunav_controller.closest_point(
                                                [float(requested[0]), float(requested[1]), 0.0],
                                                search_island_id=native_hunav_islands[agent.track_id],
                                                agent_radius=0.25,
                                                agent_height=1.70,
                                            )
                                            if found and island_id == native_hunav_islands[agent.track_id]:
                                                projected_goal = np.asarray(point[:2], dtype=float)
                                                projection_cost = float(np.linalg.norm(
                                                    projected_goal - requested
                                                ))
                                                lateral_candidates.append((
                                                    projection_cost,
                                                    rank,
                                                    projected_goal,
                                                ))
                                        if lateral_candidates:
                                            _cost, _rank, recovery_goal = min(
                                                lateral_candidates,
                                                key=lambda item: (item[0], item[1]),
                                            )
                                            reasons.append("lateral_liveness_recovery")
                                route_goal = recovery_goal
                                native_target_override = recovery_goal
                                route_delta = route_goal - agent.position_xy
                                route_norm = float(np.linalg.norm(route_delta))
                                if route_norm > 1.0e-6:
                                    steering_velocity = route_delta / route_norm * min(
                                        0.40, agent.preferred_speed_mps
                                    )
                                    reasons.append("deterministic_liveness_recovery")
                                    native_hunav_metrics["liveness_recovery_count"] += 1
                            requested_speed = min(
                                float(np.linalg.norm(steering_velocity)),
                                hunav_limits.max_speed_mps,
                            )
                            if fallback_reason is not None or not fresh_command:
                                requested_speed = 0.0
                            previous_speed = native_hunav_command_speeds[agent.track_id]
                            accelerating = requested_speed >= previous_speed
                            speed_delta_limit = (
                                hunav_limits.max_acceleration_mps2
                                if accelerating else hunav_limits.max_deceleration_mps2
                            ) * control_dt
                            commanded_speed = previous_speed + float(np.clip(
                                requested_speed - previous_speed,
                                -speed_delta_limit,
                                speed_delta_limit,
                            ))
                            native_hunav_command_speeds[agent.track_id] = commanded_speed
                            steering_speed = float(np.linalg.norm(steering_velocity))
                            if recovery_goal is not None and commanded_speed > 1.0e-6:
                                requested_goal = np.asarray([
                                    recovery_goal[0], recovery_goal[1], 0.0
                                ], dtype=float)
                            elif native_target_override is not None and commanded_speed > 1.0e-6:
                                requested_goal = np.asarray([
                                    native_target_override[0], native_target_override[1], 0.0
                                ], dtype=float)
                            elif steering_speed > 1.0e-6 and commanded_speed > 1.0e-6:
                                requested_goal = np.asarray([
                                    agent.position_xy[0] + steering_velocity[0] / steering_speed,
                                    agent.position_xy[1] + steering_velocity[1] / steering_speed,
                                    0.0,
                                ], dtype=float)
                            else:
                                requested_goal = np.asarray([
                                    agent.position_xy[0], agent.position_xy[1], 0.0
                                ], dtype=float)
                            found, native_goal, island_id = native_hunav_controller.closest_point(
                                requested_goal.tolist(),
                                search_island_id=native_hunav_islands[agent.track_id],
                                agent_radius=0.25,
                                agent_height=1.70,
                            )
                            if not found or island_id != native_hunav_islands[agent.track_id]:
                                native_hunav_metrics["goal_projection_failure_count"] += 1
                                commanded_speed = 0.0
                                native_hunav_command_speeds[agent.track_id] = 0.0
                                native_goal = [
                                    float(agent.position_xy[0]),
                                    float(agent.position_xy[1]),
                                    0.0,
                                ]
                                reasons.append("native_goal_projection_failure")
                            else:
                                projection_distance = math.dist(
                                    requested_goal[:2], native_goal[:2]
                                )
                                native_hunav_metrics["maximum_goal_projection_m"] = max(
                                    native_hunav_metrics["maximum_goal_projection_m"],
                                    projection_distance,
                                )
                                if projection_distance > 1.0e-5:
                                    native_hunav_metrics["goal_projection_count"] += 1
                                    reasons.append("native_same_island_goal_projection")
                            native_goals.append(native_goal)
                            native_speeds.append(commanded_speed)
                            execution_reasons[agent.track_id] = tuple(reasons)
                        update_returned = native_hunav_controller.update_agents(
                            native_hunav_agent_ids,
                            positions=(
                                None
                                if single.ARGS.native_state_sync == "controller_persistent"
                                else [
                                    [float(agent.position_xy[0]), float(agent.position_xy[1]), 0.0]
                                    for agent in agents
                                ]
                            ),
                            speeds=native_speeds,
                            goals=native_goals,
                        )
                        native_hunav_metrics["goal_update_count"] += 1
                        native_hunav_metrics["update_return_true_count"] += int(
                            bool(update_returned)
                        )
                        native_hunav_controller.simulate(control_dt)
                        native_hunav_metrics["simulation_count"] += 1
                        native_positions = native_hunav_controller.simulated(
                            native_hunav_agent_ids
                        )
                        projected = {}
                        for agent, native_position in zip(agents, native_positions):
                            native_xy = np.asarray(native_position[:2], dtype=float)
                            if not np.all(np.isfinite(native_xy)):
                                raise RuntimeError(
                                    f"{agent.stable_id}: native controller returned nonfinite pose"
                                )
                            velocity = (native_xy - agent.position_xy) / control_dt
                            speed = float(np.linalg.norm(velocity))
                            if (
                                single.ARGS.native_post_kinematics == "adapter_clipped"
                                and speed > hunav_limits.max_speed_mps
                            ):
                                velocity *= hunav_limits.max_speed_mps / speed
                                execution_reasons[agent.track_id] = tuple(
                                    (*execution_reasons[agent.track_id], "native_speed_limit")
                                )
                            if single.ARGS.native_post_kinematics == "adapter_clipped":
                                current_velocity = np.asarray(agent.velocity_xy, dtype=float)
                                delta_velocity = velocity - current_velocity
                                accelerating = (
                                    float(np.linalg.norm(velocity))
                                    >= float(np.linalg.norm(current_velocity))
                                )
                                delta_limit = (
                                    hunav_limits.max_acceleration_mps2
                                    if accelerating else hunav_limits.max_deceleration_mps2
                                ) * control_dt
                                delta_norm = float(np.linalg.norm(delta_velocity))
                                if delta_norm > delta_limit:
                                    velocity = current_velocity + delta_velocity * (
                                        delta_limit / delta_norm
                                    )
                                    execution_reasons[agent.track_id] = tuple(
                                        (*execution_reasons[agent.track_id], "native_acceleration_limit")
                                    )
                            requested_end = np.asarray([
                                agent.position_xy[0] + velocity[0] * control_dt,
                                agent.position_xy[1] + velocity[1] * control_dt,
                                0.0,
                            ], dtype=float)
                            found, executed_end, island_id = native_hunav_controller.closest_point(
                                requested_end.tolist(),
                                search_island_id=native_hunav_islands[agent.track_id],
                                agent_radius=0.25,
                                agent_height=1.70,
                            )
                            if not found or island_id != native_hunav_islands[agent.track_id]:
                                velocity = np.zeros(2, dtype=float)
                                execution_reasons[agent.track_id] = tuple(
                                    (*execution_reasons[agent.track_id], "native_endpoint_projection_failure")
                                )
                            else:
                                executed_xy = np.asarray(executed_end[:2], dtype=float)
                                if not np.allclose(
                                    executed_xy, requested_end[:2], atol=1.0e-5, rtol=0.0
                                ):
                                    execution_reasons[agent.track_id] = tuple(
                                        (*execution_reasons[agent.track_id], "native_same_island_endpoint_projection")
                                    )
                                velocity = (executed_xy - agent.position_xy) / control_dt
                            projected[agent.track_id] = velocity
                            if not np.allclose(
                                velocity, desired[agent.track_id], atol=1.0e-3, rtol=0.0
                            ):
                                execution_reasons[agent.track_id] = tuple(
                                    (*execution_reasons[agent.track_id], "native_joint_controller")
                                )

                        def native_segment_projector(agent_id, _start_xy, end_xy):
                            # Follow the audited semantic-route tangent from
                            # the actually committed adapter pose.  Native
                            # path points can lag that pose by one clipped
                            # frame, while a nearest-point vector can select
                            # the wrong side of a sharp corridor corner.
                            if navmesh_guard is not None:
                                return project_segment(agent_id, _start_xy, end_xy)
                            found, point, island_id = native_hunav_controller.closest_point(
                                [float(end_xy[0]), float(end_xy[1]), 0.0],
                                search_island_id=native_hunav_islands[agent_id],
                                agent_radius=0.25,
                                agent_height=1.70,
                            )
                            if not found or island_id != native_hunav_islands[agent_id]:
                                return None
                            return np.asarray(point[:2], dtype=float)

                        projected = native_output_projector.project(
                            positions,
                            velocities,
                            projected,
                            control_dt,
                            (
                                prior_robot_position
                                if mobile is not None
                                else np.asarray([1.0e6, 1.0e6], dtype=float)
                            ),
                            (
                                prior_robot_velocity
                                if mobile is not None
                                else np.zeros(2, dtype=float)
                            ),
                            robot_footprint,
                            segment_is_safe,
                            segment_projector=native_segment_projector,
                            robot_yaw_rad=prior_robot_yaw,
                            fixed_pair_agent_ids=yield_ids,
                        )
                        for agent in agents:
                            execution_reasons[agent.track_id] = tuple(dict.fromkeys((
                                *execution_reasons[agent.track_id],
                                *native_output_projector.last_reasons[agent.track_id],
                            )))
                    for agent in agents:
                        velocity = projected[agent.track_id]
                        raw_velocity = desired[agent.track_id]
                        metrics = execution_metrics[agent.track_id]
                        raw_speed = float(np.linalg.norm(raw_velocity))
                        executed_speed = float(np.linalg.norm(velocity))
                        metrics["raw_speed_sum"] += raw_speed
                        metrics["raw_speed_max"] = max(metrics["raw_speed_max"], raw_speed)
                        metrics["executed_speed_sum"] += executed_speed
                        metrics["executed_speed_max"] = max(metrics["executed_speed_max"], executed_speed)
                        metrics["executed_acceleration_max"] = max(
                            metrics["executed_acceleration_max"],
                            float(np.linalg.norm(velocity - agent.velocity_xy)) / control_dt,
                        )
                        metrics["samples"] += 1
                        reasons = execution_reasons[agent.track_id]
                        if reasons:
                            metrics["safety_modification_count"] += 1
                        for reason in reasons:
                            metrics["safety_reasons"][reason] = metrics["safety_reasons"].get(reason, 0) + 1
                            if reason == "static_boundary_hold":
                                metrics["static_boundary_hold_count"] += 1
                        row = commands.get(agent.track_id)
                        if row is not None:
                            behavior_state = str(int(row.get("behavior_state", 0)))
                            metrics["behavior_state_counts"][behavior_state] = metrics["behavior_state_counts"].get(behavior_state, 0) + 1
                            if agent.track_id in scared_evidence:
                                evidence = scared_evidence[agent.track_id]
                                distance = float(np.linalg.norm(agent.position_xy - prior_robot_position))
                                state_value = int(row.get("behavior_state", 0))
                                relative = agent.position_xy - prior_robot_position
                                radial = float(np.dot(
                                    velocity - prior_robot_velocity,
                                    relative / max(float(np.linalg.norm(relative)), 1.0e-9),
                                ))
                                evidence["behavior_state_nonzero_observed"] = bool(
                                    evidence["behavior_state_nonzero_observed"] or state_value != 0
                                )
                                # The installed HuNav version's avoidRobot() sets state=1,
                                # then computeForces() clears it before the response is
                                # serialized.  Its BTScaredNav branch is nevertheless
                                # deterministically selected by IsRobotVisible using this
                                # configured distance.  Record that real branch condition,
                                # retain the observed response state, and require outward
                                # motion while the robot is still inside the range.
                                inside_interaction = distance <= float(source.interaction_distance)
                                if inside_interaction and evidence["trigger_time"] is None:
                                    evidence["trigger_time"] = sim_time
                                    evidence["trigger_distance_m"] = distance
                                    evidence["trigger_behavior_state"] = state_value
                                    evidence["trigger_source"] = "hunav_bt_scared_visible_distance"
                                    evidence["route_progress_at_trigger_m"] = route_runtimes[agent.track_id].progress_m
                                evidence["maximum_away_radial_velocity_mps"] = max(
                                    evidence["maximum_away_radial_velocity_mps"], radial
                                )
                                if evidence["trigger_time"] is not None:
                                    evidence["maximum_away_radial_velocity_after_trigger_mps"] = max(
                                        evidence["maximum_away_radial_velocity_after_trigger_mps"], radial
                                    )
                                    evidence["maximum_robot_distance_after_trigger_m"] = max(
                                        evidence["maximum_robot_distance_after_trigger_m"], distance
                                    )
                                    if inside_interaction and gate6_phase_name in {
                                        "crossing_and_slow_approach",
                                        "sudden_stop_and_static_block",
                                    }:
                                        evidence["maximum_away_radial_velocity_during_interaction_mps"] = max(
                                            evidence["maximum_away_radial_velocity_during_interaction_mps"], radial
                                        )
                                clearance = robot_rectangle_net_clearance(
                                    agent.position_xy,
                                    prior_robot_position,
                                    robot_yaw_for_state,
                                    half_x,
                                    half_y,
                                    people[agent.track_id - 1]["arena_person"].radius,
                                )
                                evidence["minimum_robot_rectangle_net_clearance_m"] = min(
                                    evidence["minimum_robot_rectangle_net_clearance_m"], clearance
                                )
                                if (
                                    evidence["trigger_time"] is not None
                                    and distance >= float(source.interaction_distance) + 0.20
                                    and evidence["recovery_time"] is None
                                    and route_runtimes[agent.track_id].progress_m
                                    > float(evidence["route_progress_at_trigger_m"]) + 0.10
                                ):
                                    evidence["recovery_time"] = sim_time
                                    evidence["original_route_recovered"] = True
                        if fallback_reason is not None:
                            metrics["timeout_stop_count"] += 1
                        agent.position_xy = agent.position_xy + velocity * control_dt
                        agent.velocity_xy = velocity
                        if navmesh_guard is not None:
                            navmesh_error = navmesh_guard.point_projection_error_m(
                                agent.position_xy
                            )
                            maximum_committed_navmesh_error_m = max(
                                maximum_committed_navmesh_error_m, navmesh_error
                            )
                            if navmesh_error > 0.05:
                                unsafe_pose_commit_count += 1
                                if len(unsafe_pose_commit_samples) < 50:
                                    unsafe_pose_commit_samples.append({
                                        "sim_time": sim_time,
                                        "stable_id": agent.stable_id,
                                        "position": agent.position_xy.tolist(),
                                        "projection_error_m": navmesh_error,
                                        "execution_reasons": list(reasons),
                                    })
                        speed = float(np.linalg.norm(velocity))
                        if speed > 1.0e-5:
                            target_yaw = math.atan2(float(velocity[1]), float(velocity[0]))
                            yaw_error = math.atan2(math.sin(target_yaw - agent.yaw_rad), math.cos(target_yaw - agent.yaw_rad))
                            yaw_step = float(np.clip(
                                yaw_error,
                                -hunav_limits.max_yaw_rate_radps * control_dt,
                                hunav_limits.max_yaw_rate_radps * control_dt,
                            ))
                            agent.yaw_rad += yaw_step
                        agent.animation_phase += speed * control_dt
                        if single.ARGS.gate6_interaction_profile == NARROW_PROFILE_NAME:
                            if gate6_phase_name == "narrow_corridor_static_block":
                                robot_distance = float(np.linalg.norm(
                                    agent.position_xy - prior_robot_position
                                ))
                                goal_xy = np.asarray(
                                    route_runtimes[agent.track_id].goals_xy()[0], dtype=float
                                )
                                goal_delta = goal_xy - agent.position_xy
                                goal_norm = float(np.linalg.norm(goal_delta))
                                forward_speed = (
                                    float(np.dot(velocity, goal_delta / goal_norm))
                                    if goal_norm > 1.0e-9 else 0.0
                                )
                                robot_constrained = (
                                    "physical_robot_rectangle_projection" in reasons
                                )
                                if (
                                    robot_distance <= 2.0
                                    and robot_constrained
                                    and forward_speed < 0.08
                                ):
                                    narrow_wait_current_seconds[agent.track_id] += control_dt
                                    narrow_wait_seconds[agent.track_id] = max(
                                        narrow_wait_seconds[agent.track_id],
                                        narrow_wait_current_seconds[agent.track_id],
                                    )
                                else:
                                    narrow_wait_current_seconds[agent.track_id] = 0.0
                            elif gate6_phase_name == "turn_to_clear_narrow_corridor":
                                if narrow_route_progress_at_block_end[agent.track_id] is None:
                                    narrow_route_progress_at_block_end[agent.track_id] = (
                                        route_runtimes[agent.track_id].progress_m
                                    )
                    if fresh_command:
                        last_applied_hunav_command_sequence = command_sequence
                    if fallback_reason is not None:
                        if hunav_projector is not None:
                            hunav_projector.metrics.timeout_stop_count += 1
                    if step % max(1, int(round(1.0 / single.PHYSICS_DT))) == 0:
                        hunav_bridge.publish_status(sim_time, bool(commands))
            states = []
            frame_positions = []
            for person in people:
                agent = person["agent"]
                position = np.asarray([agent.position_xy[0], agent.position_xy[1], 0.0])
                velocity = np.asarray([agent.velocity_xy[0], agent.velocity_xy[1], 0.0])
                yaw = agent.yaw_rad
                if pedestrian_update_due:
                    single.set_person_pose(person["root"], position, yaw)
                    person["collider_object"].set_world_pose(
                        np.asarray([position[0], position[1], 0.85]),
                        single.yaw_quaternion(yaw),
                    )
                    person["gait_driver"].apply(agent.animation_phase)
                position_history[person["index"]].append(position.tolist())
                facing_errors[person["index"]].append(
                    single.character_facing_error_rad(yaw)
                )
                frame_positions.append(position)
                states.append((person["track_id"], person["paths"]["stable_id"], position, velocity))

            world.step(
                render=not single.ARGS.headless and step % gui_render_stride == 0
            )
            if mobile is not None:
                mobile.follow_visual()
                robot_position, robot_yaw = mobile.pose()
                robot_orientation = single.yaw_quaternion(robot_yaw)
                robot_positions.append(robot_position.tolist())
                applied_commands.append([float(command[0]), float(command[1])])
                if ros is not None:
                    ros.publish_robot_state(
                        float(world.current_time - sim_start),
                        robot_position,
                        robot_yaw,
                        command,
                    )
                for person_position in frame_positions:
                    min_robot_person_distance = min(
                        min_robot_person_distance,
                        float(np.linalg.norm(robot_position[:2] - person_position[:2])),
                    )
                    min_robot_person_rectangle_clearance = min(
                        min_robot_person_rectangle_clearance,
                        robot_rectangle_net_clearance(
                            person_position[:2], robot_position[:2], robot_yaw,
                            float(mobile.proxy_dimensions[0]) * 0.5,
                            float(mobile.proxy_dimensions[1]) * 0.5,
                            social_config.agent_radius_m,
                        ),
                    )
            else:
                robot_position = np.asarray(single.ROBOT_POSITION)
                robot_orientation = np.asarray([1.0, 0.0, 0.0, 0.0])
            for person, position in zip(people, frame_positions):
                collider_position, _ = person["collider_object"].get_world_pose()
                error = float(np.linalg.norm(np.asarray(collider_position)[:2] - position[:2]))
                collider_errors[person["index"]] = max(collider_errors[person["index"]], error)
            for first in range(len(frame_positions)):
                for second in range(first + 1, len(frame_positions)):
                    pair_distance = float(np.linalg.norm(
                        frame_positions[first][:2] - frame_positions[second][:2]
                    ))
                    pair_observations += 1
                    if pair_distance < social_config.personal_space_m:
                        personal_space_violation_observations += 1
                    if pair_distance < min_pair_distance:
                        min_pair_distance = pair_distance
                        min_pair_ids = [
                            people[first]["paths"]["stable_id"],
                            people[second]["paths"]["stable_id"],
                        ]
                    execution_metrics[people[first]["track_id"]]["minimum_separation_m"] = min(
                        execution_metrics[people[first]["track_id"]]["minimum_separation_m"],
                        pair_distance,
                    )
                    execution_metrics[people[second]["track_id"]]["minimum_separation_m"] = min(
                        execution_metrics[people[second]["track_id"]]["minimum_separation_m"],
                        pair_distance,
                    )

            current_time = float(world.current_time - sim_start)
            if ros is not None:
                ros.publish_clock(current_time)
            if step % 4 == 0:
                pair = lidar.sample(
                    current_time,
                    robot_position,
                    robot_orientation,
                )
                lidar_samples += 1
                for person in people:
                    path = person["paths"]["collider"]
                    stable_id = person["paths"]["stable_id"]
                    lidar_hits[stable_id][0] += sum(hit == path for hit in pair.scan_01.hit_paths)
                    lidar_hits[stable_id][1] += sum(hit == path for hit in pair.scan_02.hit_paths)
                if ros is not None:
                    ros.publish_tracks(current_time, states)
                    ros.publish_scan(pair.scan_01, ros.scan_01_pub)
                    ros.publish_scan(pair.scan_02, ros.scan_02_pub)
            if ros is not None:
                target_wall_elapsed = (step + 1) * single.PHYSICS_DT
                time.sleep(max(0.0, target_wall_elapsed - (time.monotonic() - wall_start)))

        if mobile is not None:
            mobile.stop()
            for _ in range(30):
                mobile.stop()
                world.step(render=not single.ARGS.headless)
                mobile.follow_visual()

        if single.SCREENSHOT is not None:
            single.capture_viewport(world, single.SCREENSHOT)

        maximum_displacements = []
        cumulative_distances = []
        longest_freezes = []
        for rows in position_history:
            positions = np.asarray(rows)
            maximum_displacements.append(
                float(np.max(np.linalg.norm(positions[:, :2] - positions[0, :2], axis=1)))
            )
            cumulative_distances.append(
                float(np.linalg.norm(np.diff(positions[:, :2], axis=0), axis=1).sum())
            )
            step_distances = np.linalg.norm(np.diff(positions[:, :2], axis=0), axis=1)
            longest_steps = current_steps = 0
            for distance in step_distances:
                current_steps = current_steps + 1 if distance < 0.001 else 0
                longest_steps = max(longest_steps, current_steps)
            longest_freezes.append(longest_steps * single.PHYSICS_DT)
        track_rate = None
        if ros is not None and len(ros.track_publish_wall_times) >= 2:
            track_rate = (len(ros.track_publish_wall_times) - 1) / (
                ros.track_publish_wall_times[-1] - ros.track_publish_wall_times[0]
            )
        track_sim_rate = (
            None
            if ros is None
            else ros.track_message_publish_count / max(single.ARGS.duration, 1.0e-9)
        )
        binding_ok = all(
            row["animation_query_valid"]
            and row["joint_count"] == 101
            and row["max_matrix_delta"] > 1.0e-4
            and 0.8 <= row["translation_scale_ratio_to_rest"] <= 1.2
            for row in pose_rows
        )
        character_dependency_rows = []
        for person in people:
            layers, assets, unresolved = single.UsdUtils.ComputeAllDependencies(
                str(person["character_usd"])
            )
            asset_paths = sorted(str(value) for value in assets)
            character_dependency_rows.append(
                {
                    "stable_id": person["paths"]["stable_id"],
                    "character_usd": str(person["character_usd"]),
                    "layers": sorted(str(value.identifier) for value in layers),
                    "assets": asset_paths,
                    "unresolved": sorted(str(value) for value in unresolved),
                    "texture_count": sum(
                        value.lower().endswith((".png", ".jpg", ".jpeg", ".exr"))
                        for value in asset_paths
                    ),
                }
            )
        character_assets_ok = all(
            not row["unresolved"] and row["texture_count"] >= 1
            for row in character_dependency_rows
        )
        initial_positions = np.asarray([rows[0] for rows in position_history], dtype=float)
        crowd_initial_span = np.ptp(initial_positions[:, :2], axis=0)
        perceived_stable_ids = sorted(
            stable_id for stable_id, counts in lidar_hits.items() if sum(counts) > 0
        )
        physical_ok = all(
            person["collider_prim"].HasAPI(single.UsdPhysics.CollisionAPI)
            and person["collider_prim"].HasAPI(single.UsdPhysics.RigidBodyAPI)
            and bool(single.UsdPhysics.RigidBodyAPI(person["collider_prim"]).GetKinematicEnabledAttr().Get())
            for person in people
        )
        robot_travel = 0.0
        final_robot_pose = None
        if robot_positions:
            robot_array = np.asarray(robot_positions, dtype=float)
            robot_travel = float(
                np.max(np.linalg.norm(robot_array[:, :2] - robot_array[0, :2], axis=1))
            )
            final_robot_pose = robot_positions[-1]
        robot_separation_ok = (
            min_robot_person_rectangle_clearance >= 0.0
            if single.ARGS.gate6_interaction_profile is not None
            else min_robot_person_distance >= 0.60
        )
        mobile_ok = mobile is None or (
            ros is not None
            and (
                ros.command_receive_count >= 10
                or single.ARGS.gate6_interaction_profile is not None
            )
            and 0.10 <= robot_travel <= 3.0
            and mobile.arm_lock_mode == "authored_visual_pose_on_dynamic_base"
            and mobile.minimum_visual_bottom_z >= -0.02
            and mobile.maximum_visual_bottom_z <= 0.05
            and robot_separation_ok
            and all(
                abs(row[0]) <= 0.300001 and abs(row[1]) <= 1.000001
                for row in applied_commands
            )
        )
        if crowd_controller is not None:
            controller_summary = crowd_controller.summary()
        else:
            stall_ratios = []
            average_speeds = []
            for rows in position_history:
                points = np.asarray(rows, dtype=float)[::pedestrian_update_stride]
                speeds = np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1) / (
                    single.PHYSICS_DT * pedestrian_update_stride
                )
                stall_ratios.append(float(np.mean(speeds < 0.08)) if len(speeds) else 1.0)
                average_speeds.append(float(np.mean(speeds)) if len(speeds) else 0.0)
            controller_summary = {
                "pedestrian_count": len(agents),
                "stall_ratio": stall_ratios,
                "average_speed_mps": average_speeds,
                "maximum_speed_mps": [
                    execution_metrics[agent.track_id]["executed_speed_max"]
                    for agent in agents
                ],
                "maximum_acceleration_mps2": [
                    execution_metrics[agent.track_id]["executed_acceleration_max"]
                    for agent in agents
                ],
                "maximum_yaw_rate_radps": [hunav_limits.max_yaw_rate_radps] * len(agents),
                "free_space_emergency_hold_count": [
                    execution_metrics[agent.track_id]["static_boundary_hold_count"]
                    for agent in agents
                ],
                "safety_projection_count": (
                    hunav_projector.metrics.projection_count
                    if hunav_projector is not None
                    else native_output_projector.metrics.projection_count
                ),
                "command_timeout_stop_count": (
                    hunav_projector.metrics.timeout_stop_count
                    if hunav_projector is not None
                    else sum(
                        execution_metrics[agent.track_id]["timeout_stop_count"]
                        for agent in agents
                    )
                ),
                "minimum_predicted_separation_m": (
                    hunav_projector.metrics.minimum_predicted_separation_m
                    if hunav_projector is not None
                    else native_output_projector.metrics.minimum_predicted_separation_m
                ),
                "recovery_state_counts": {
                    str(agent_id): supervisor.counts
                    for agent_id, supervisor in recovery_supervisors.items()
                },
                "control_authority": (
                    "Isaac INavController owns persistent paths and joint avoidance; bounded adapter is the sole USD pose writer; HuNav emits social intent"
                    if native_hunav_controller is not None
                    else "Isaac HuNav executor is the only pose writer; HuNav and safety layer emit velocities only"
                ),
                "static_constraint": (
                    "persistent Isaac NavController path corridor with same-island moving goals"
                    if native_hunav_controller is not None
                    else "Isaac NavMesh shortest-path and endpoint projection"
                ),
                "native_controller": (
                    {
                        **native_hunav_metrics,
                        "yield_start_count": native_yield_planner.start_count,
                        "yield_end_count": native_yield_planner.end_count,
                        "active_yield_ids": sorted(native_yield_planner.active),
                    }
                    if native_hunav_controller is not None else None
                ),
                "native_pair_projection": (
                    single.ARGS.native_pair_projection
                    if native_hunav_controller is not None else None
                ),
                "native_stall_recovery": (
                    single.ARGS.native_stall_recovery
                    if native_hunav_controller is not None else None
                ),
                "native_stall_replan_after_sec": (
                    single.ARGS.native_stall_replan_after
                    if native_hunav_controller is not None else None
                ),
                "native_intent_mode": (
                    single.ARGS.native_intent_mode
                    if native_hunav_controller is not None else None
                ),
                "native_agent_padding_m": (
                    single.ARGS.native_agent_padding
                    if native_hunav_controller is not None else None
                ),
            }
        per_agent_runtime = []
        for person, agent, cumulative, displacement, freeze in zip(
            people, agents, cumulative_distances, maximum_displacements, longest_freezes
        ):
            metrics = execution_metrics[agent.track_id]
            samples = max(int(metrics["samples"]), 1)
            route_snapshot = (
                route_runtimes[agent.track_id].snapshot()
                if agent.track_id in route_runtimes else {}
            )
            per_agent_runtime.append({
                "stable_id": agent.stable_id,
                "track_id": agent.track_id,
                "cumulative_distance_m": cumulative,
                "maximum_displacement_m": displacement,
                "average_hunav_raw_speed_mps": metrics["raw_speed_sum"] / samples,
                "maximum_hunav_raw_speed_mps": metrics["raw_speed_max"],
                "average_executed_speed_mps": metrics["executed_speed_sum"] / samples,
                "maximum_executed_speed_mps": metrics["executed_speed_max"],
                "longest_continuous_freeze_sec": freeze,
                "safety_modification_count": metrics["safety_modification_count"],
                "safety_reasons": metrics["safety_reasons"],
                "timeout_stop_count": metrics["timeout_stop_count"],
                "static_boundary_hold_count": metrics["static_boundary_hold_count"],
                "minimum_separation_m": metrics["minimum_separation_m"],
                "behavior_state_counts": metrics["behavior_state_counts"],
                "recovery_counts": (
                    recovery_supervisors[agent.track_id].counts
                    if agent.track_id in recovery_supervisors else {}
                ),
                **route_snapshot,
            })
        for agent_id, evidence in scared_evidence.items():
            evidence["final_route_progress_m"] = route_runtimes[agent_id].progress_m
        runtime_gait_rows = [person["gait_driver"].summary() for person in people]
        min_required_displacement = min(0.65, 0.45 * single.ARGS.duration)
        expected_near_field = {people[0]["paths"]["stable_id"]}
        if count >= 2:
            expected_near_field.add(people[1]["paths"]["stable_id"])
        actual_lidar_rate = lidar_samples / max(single.ARGS.duration, 1.0e-9)
        checks = {
            "crowd_size": len(people) == count,
            "unique_stable_ids_and_tracks": len({p["paths"]["stable_id"] for p in people}) == count
            and len({p["track_id"] for p in people}) == count,
            "all_routes_free": all_routes_clear,
            "all_true_skinned_animated_humanoids": binding_ok,
            "all_runtime_gaits_change_with_travel": all(
                row["timed_samples_remaining"] == 0
                and row["template_max_matrix_delta"] > 1.0e-4
                and row["pose_change_count"] >= 2
                and row["unique_pose_indices"] >= 3
                for row in runtime_gait_rows
            ),
            "all_characters_face_velocity": max(
                max(rows) for rows in facing_errors
            ) <= math.radians(1.0),
            "all_isaac5_owned_textured_character_assets": character_assets_ok,
            "all_physical_colliders": physical_ok,
            "map_wide_initial_distribution": count < 8 or bool(
                crowd_initial_span[0] >= 20.0 and crowd_initial_span[1] >= 10.0
            ),
            "stateful_independent_motion": min(maximum_displacements) >= min_required_displacement
            and all(distance >= min_required_displacement for distance in cumulative_distances)
            and max(collider_errors) <= 0.02
            and min_pair_distance >= 2.0 * social_config.agent_radius_m,
            "bounded_kinematics_and_free_space": max(
                controller_summary["maximum_acceleration_mps2"]
            ) <= (
                hunav_limits.max_deceleration_mps2
                if single.ARGS.social_mode == "hunav"
                else social_config.max_deceleration_mps2
            ) + 1.0e-6
            and max(controller_summary["maximum_yaw_rate_radps"])
            <= (
                hunav_limits.max_yaw_rate_radps
                if single.ARGS.social_mode == "hunav"
                else social_config.max_yaw_rate_radps
            ) + 1.0e-6
            and max(controller_summary["maximum_speed_mps"]) <= (
                hunav_limits.max_speed_mps
                if single.ARGS.social_mode == "hunav"
                else max(agent.preferred_speed_mps for agent in agents)
            ) + 1.0e-6
            and unsafe_pose_commit_count == 0,
            "no_sustained_crowd_stall": max(longest_freezes) <= 5.0,
            "near_field_people_lidar_perceived": 14.0 <= actual_lidar_rate <= 16.0
            and expected_near_field.issubset(perceived_stable_ids),
            "ros_crowd_tracks_published": ros is None
            or single.ARGS.suppress_tracks
            or (
                ros.track_message_publish_count == lidar_samples
                and track_sim_rate is not None
                and 14.0 <= track_sim_rate <= 16.0
            ),
            "screenshot_written": single.SCREENSHOT is None
            or (single.SCREENSHOT.is_file() and single.SCREENSHOT.stat().st_size > 0),
            "mobile_robot_bounded_control": mobile_ok,
            "hunav_service_commands_online": hunav_bridge is None
            or hunav_bridge.inbox.accepted >= max(2, int(single.ARGS.duration * 5.0)),
            "hunav_joint_population_commands": hunav_bridge is None
            or all(metrics["samples"] > 0 for metrics in execution_metrics.values()),
            "hunav_external_goal_sync": hunav_bridge is None
            or (
                goal_sync_sample_count > 0
                and goal_sync_mismatch_count == 0
                and maximum_goal_sync_error_m <= 1.0e-6
                and maximum_hunav_active_goal_count == 1
            ),
            "isaac_navmesh_active": navmesh_guard is None
            or navmesh_guard.path_query_count > count
            or (
                native_hunav_controller is not None
                and native_hunav_metrics["simulation_count"] > 0
            ),
        }
        gate6_checks = None
        if single.ARGS.gate6_interaction_profile == PROFILE_NAME:
            gate6_checks = evaluate_interaction(
                behaviors=(person["arena_person"].behavior for person in people),
                scared_rows=list(scared_evidence.values()),
                observed_phases=gate6_phase_names,
                robot_travel_m=robot_travel,
            )
            checks.update(gate6_checks)
        elif single.ARGS.gate6_interaction_profile == NARROW_PROFILE_NAME:
            progress_at_block_end = max(
                float(value or 0.0) for value in narrow_route_progress_at_block_end.values()
            )
            final_progress = max(
                route_runtimes[agent.track_id].progress_m for agent in agents
            )
            gate6_checks = evaluate_narrow_wait(
                observed_phases=gate6_phase_names,
                maximum_wait_sec=max(narrow_wait_seconds.values()),
                minimum_rectangle_clearance_m=min_robot_person_rectangle_clearance,
                route_progress_at_block_end_m=progress_at_block_end,
                final_route_progress_m=final_progress,
                robot_travel_m=robot_travel,
            )
            checks.update(gate6_checks)
        status, reasons = single.pass_or_fail(checks)
        result = {
            "status": status,
            "failure_reasons": reasons,
            "gate": "6B",
            "backend": "isaac5_owned_textured_character_retarget_crowd_adapter",
            "isaac_version": "5.1.0",
            "scene_usd": str(single.SCENE_USD),
            "pedestrian_mode": "arena_scenario",
            "arena_scenario": str(single.ARGS.arena_scenario),
            "pedestrian_count": count,
            "pedestrian_seed": single.ARGS.pedestrian_seed,
            "pedestrian_speed_override_mps": single.ARGS.pedestrian_speed,
            "pedestrian_speed_range_mps": [single.ARGS.pedestrian_speed_min, single.ARGS.pedestrian_speed_max],
            "social_mode": single.ARGS.social_mode,
            "hunav_executor": (
                single.ARGS.hunav_executor
                if single.ARGS.social_mode == "hunav" else None
            ),
            "hunav_execution_backend": (
                (
                    "host_hunav_lightsfm_joint_compute_with_native_persistent_nav_controller"
                    if single.ARGS.hunav_executor == "native_controller"
                    else "host_hunav_lightsfm_joint_compute_with_hard_limit_executor"
                )
                if single.ARGS.social_mode == "hunav" else None
            ),
            "native_pair_projection": (
                single.ARGS.native_pair_projection
                if single.ARGS.hunav_executor == "native_controller" else None
            ),
            "native_stall_recovery": (
                single.ARGS.native_stall_recovery
                if single.ARGS.hunav_executor == "native_controller" else None
            ),
            "native_stall_replan_after_sec": (
                single.ARGS.native_stall_replan_after
                if single.ARGS.hunav_executor == "native_controller" else None
            ),
            "native_intent_mode": (
                single.ARGS.native_intent_mode
                if single.ARGS.hunav_executor == "native_controller" else None
            ),
            "native_agent_padding_m": (
                single.ARGS.native_agent_padding
                if single.ARGS.hunav_executor == "native_controller" else None
            ),
            "pedestrian_control_rate_hz": single.ARGS.pedestrian_update_rate,
            "pedestrian_pose_commit_rate_hz": single.ARGS.pedestrian_update_rate,
            "track_ids": [person["track_id"] for person in people],
            "stable_ids": [person["paths"]["stable_id"] for person in people],
            "character_usds": [str(person["character_usd"]) for person in people],
            "character_dependency_audit": character_dependency_rows,
            "route_clearance": route_rows,
            "animation": animation_rows,
            "runtime_gait": runtime_gait_rows,
            "evaluated_skeleton_poses": pose_rows,
            "maximum_displacement_m": maximum_displacements,
            "cumulative_forward_distance_m": cumulative_distances,
            "stateful_controller": controller_summary,
            "per_agent_runtime": per_agent_runtime,
            "scared_interaction": list(scared_evidence.values()),
            "gate6_interaction_profile": single.ARGS.gate6_interaction_profile,
            "gate6_command_phases": gate6_phase_names,
            "gate6_phase_sample_counts": gate6_phase_sample_counts,
            "gate6_interaction_checks": gate6_checks,
            "hunav_protocol": None if hunav_bridge is None else {
                "state_publish_count": hunav_bridge.state_publish_count,
                "command_accept_count": hunav_bridge.inbox.accepted,
                "rejected_duplicate": hunav_bridge.inbox.rejected_duplicate,
                "rejected_out_of_order": hunav_bridge.inbox.rejected_out_of_order,
                "rejected_session": hunav_bridge.inbox.rejected_session,
                "rejected_command_sequence": hunav_bridge.inbox.rejected_command_sequence,
                "rejected_state_sequence": hunav_bridge.inbox.rejected_state_sequence,
                "rejected_future": hunav_bridge.inbox.rejected_future,
                "rejected_invalid": hunav_bridge.inbox.rejected_invalid,
                "last_fallback_reason": hunav_bridge.last_fallback_reason,
                "goal_authority": "isaac_semantic_route_runtime",
                "goal_sync_sample_count": goal_sync_sample_count,
                "goal_sync_mismatch_count": goal_sync_mismatch_count,
                "maximum_goal_sync_error_m": maximum_goal_sync_error_m,
                "maximum_hunav_active_goal_count": maximum_hunav_active_goal_count,
            },
            "isaac_navmesh": None if navmesh_guard is None else {
                "path_query_count": navmesh_guard.path_query_count,
                "rejected_segment_count": navmesh_guard.rejected_segment_count,
                "rejected_segment_samples": navmesh_guard.rejected_segment_samples,
                "projected_step_count": navmesh_guard.projected_step_count,
                "projected_step_samples": navmesh_guard.projected_step_samples,
                "projected_step_failure_count": navmesh_guard.projected_step_failure_count,
                "projected_step_failure_samples": navmesh_guard.projected_step_failure_samples,
                "replan_failure_count": navmesh_guard.replan_failure_count,
                "maximum_goal_projection_m": navmesh_guard.maximum_goal_projection_m,
                "maximum_start_projection_m": navmesh_guard.maximum_start_projection_m,
                "unsafe_pose_commit_count": unsafe_pose_commit_count,
                "maximum_committed_navmesh_error_m": maximum_committed_navmesh_error_m,
                "unsafe_pose_commit_samples": unsafe_pose_commit_samples,
            },
            "character_forward_axis": single.CHARACTER_FORWARD_AXIS,
            "maximum_facing_velocity_error_deg": math.degrees(
                max(max(rows) for rows in facing_errors)
            ),
            "initial_crowd_span_xy_m": crowd_initial_span.tolist(),
            "requested_initial_minimum_pair_distance_m": initial_pair_distance,
            "max_collider_tracking_error_m": max(collider_errors),
            "minimum_pair_distance_m": min_pair_distance,
            "minimum_pair_stable_ids": min_pair_ids,
            "personal_space_threshold_m": social_config.personal_space_m,
            "personal_space_violation_ratio": personal_space_violation_observations
            / max(pair_observations, 1),
            "lidar_samples": lidar_samples,
            "lidar_wall_rate_hz": actual_lidar_rate,
            "lidar_person_hit_beams": lidar_hits,
            "lidar_perceived_stable_ids": perceived_stable_ids,
            "track_publish_count": 0 if ros is None else ros.track_publish_count,
            "track_message_publish_count": 0 if ros is None else ros.track_message_publish_count,
            "tracks_suppressed": bool(single.ARGS.suppress_tracks),
            "track_sim_rate_hz": track_sim_rate,
            "track_wall_rate_hz": track_rate,
            "viewport_screenshot": str(single.SCREENSHOT) if single.SCREENSHOT is not None else None,
            "physics_rate_target_hz": 1.0 / single.PHYSICS_DT,
            "gui_render_rate_target_hz": None if single.ARGS.headless else single.ARGS.gui_render_rate,
            "physical_truth_scope": "one kinematic PhysX capsule per humanoid; ray intersection/occupancy proxy only, no human-mesh contact truth",
            "mobile_robot": mobile is not None,
            "command_topic": single.ARGS.command_topic if mobile is not None else None,
            "command_receive_count": 0 if ros is None else ros.command_receive_count,
            "robot_travel_m": robot_travel,
            "final_robot_pose": final_robot_pose,
            "arm_pose_mode": None if mobile is None else mobile.arm_lock_mode,
            "arm_dynamics_enabled": False if mobile is not None else None,
            "robot_visual_layer": None if mobile is None else str(mobile.robot_visual_usd),
            "minimum_robot_visual_bottom_z_m": None if mobile is None else mobile.minimum_visual_bottom_z,
            "maximum_robot_visual_bottom_z_m": None if mobile is None else mobile.maximum_visual_bottom_z,
            "deinstanced_robot_visual_count": None if mobile is None else mobile.deinstanced_visual_count,
            "minimum_robot_person_center_distance_m": min_robot_person_distance,
            "minimum_robot_person_rectangle_clearance_m": min_robot_person_rectangle_clearance,
            "narrow_wait_seconds_by_track": narrow_wait_seconds,
            "narrow_route_progress_at_block_end": narrow_route_progress_at_block_end,
            "robot_collision_proxy_dimensions_m": None if mobile is None else mobile.proxy_dimensions.tolist(),
            "disabled_canonical_robot_collision_count": None if mobile is None else mobile.disabled_canonical_collisions,
            "checks": checks,
            "teardown": None,
        }
        return 0 if status == "PASS" else 2
    except Exception as exc:
        result = {"status": "FAIL", "gate": "6B", "error": repr(exc), "teardown": None}
        return 1
    finally:
        if native_hunav_controller is not None and native_hunav_agent_ids:
            try:
                if not native_hunav_controller.destroy_agents(native_hunav_agent_ids):
                    teardown_errors.append("native_controller_destroy: one or more agents were missing")
            except Exception as exc:
                teardown_errors.append(f"native_controller_destroy: {exc!r}")
        if ros is not None:
            try:
                ros.close()
            except Exception as exc:
                teardown_errors.append(f"ros_close: {exc!r}")
        if world is not None:
            try:
                world.stop()
            except Exception as exc:
                teardown_errors.append(f"world_stop: {exc!r}")
        result["teardown"] = {
            "status": "PASS" if not teardown_errors else "FAIL",
            "errors": teardown_errors,
        }
        if teardown_errors and result.get("status") == "PASS":
            result["status"] = "FAIL"
        print("GATE6B_CROWD_RESULT=" + json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    exit_code = main()
    single.simulation_app.close()
    raise SystemExit(exit_code)
