#!/usr/bin/env python3
"""Bounded, read-only 10+10 evaluation of a frozen CrowdSim PPO checkpoint."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import random
import sys
from datetime import datetime, timezone

sys.dont_write_bytecode = True


def utc() -> str:
    return datetime.now(timezone.utc).isoformat()


def digest(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def write_json(path: Path, value) -> None:
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--expected-sha256", required=True)
    parser.add_argument("--expected-step", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=701)
    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--max-control-steps", type=int, default=6000)
    args = parser.parse_args()

    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    checkpoint = args.checkpoint.resolve(strict=True)
    actual_sha = digest(checkpoint)
    if actual_sha != args.expected_sha256:
        raise RuntimeError("CHECKPOINT_SHA256_MISMATCH")

    source = Path("/home/user/workspace/NavIsaaclab2.0/strict_public_dependency_source_57f98a9_20261003")
    if Path.cwd().resolve() != source:
        raise RuntimeError("RUN_FROM_FROZEN_SOURCE_ROOT")

    # CrowdSim imports the simulator shim before Torch; preserve that order.
    from CrowdSim.crowd_sim import cfg_path, get_network_config, load_config
    from CrowdSim.world.builder import build_env
    from CrowdSim.ppo.ppo_policy import (
        RobotPPOConfig, RobotPPOTrainer, bounded_robot_action, robot_network_kwargs,
    )
    from CrowdSim.protomotions_runtime import shutdown_runtime
    import numpy as np
    import torch

    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    torch.cuda.manual_seed_all(args.seed)
    config = load_config(cfg_path("CrowdSim/config/env.yaml"))
    if config["scene"] != "warehouse":
        raise RuntimeError("EXPECTED_WAREHOUSE_SCENE")
    nav_cfg = config["navigation"]
    if (nav_cfg["num_humanoids"], nav_cfg["num_robots"]) != (10, 10):
        raise RuntimeError("EXPECTED_10_HUMANS_10_ROBOTS")
    # Keep the final training stage fixed for every compared checkpoint.
    nav_cfg["path"]["seed"] = args.seed
    nav_cfg["rl"]["goal_curriculum"]["enabled"] = False
    nav_cfg["recording"]["enabled"] = False
    nav_cfg["recording"]["output_dir"] = str(output / "navigation")
    if (nav_cfg["path"]["min_start_goal_distance"],
            nav_cfg["path"]["max_start_goal_distance"]) != (7.0, 10.0):
        raise RuntimeError("EXPECTED_FULL_DISTANCE_RANGE")
    if (nav_cfg["rl"]["initial_heading_min_offset_degrees"],
            nav_cfg["rl"]["initial_heading_max_offset_degrees"]) != (30.0, 120.0):
        raise RuntimeError("EXPECTED_FULL_HEADING_RANGE")
    write_json(output / "CONFIG_EVAL.json", config)

    runtime = None
    manager = None
    try:
        result = build_env(config, num_envs=10, headless=True)
        if result is None:
            raise RuntimeError("BUILD_ENV_RETURNED_NONE")
        env, agent, manager, runtime = result
        if (manager.config.num_humanoids, manager.config.num_robots) != (10, 10):
            raise RuntimeError("RUNTIME_AGENT_COUNT_MISMATCH")
        if manager.goal_curriculum.spec.enabled:
            raise RuntimeError("CURRICULUM_NOT_FIXED")
        net_cfg = get_network_config(config)
        ppo = RobotPPOTrainer(
            RobotPPOConfig(
                obs_dim=manager.robot_rl_obs_dim,
                vector_obs_dim=manager.robot_rl_vector_obs_dim,
                **robot_network_kwargs(
                    net_cfg,
                    depth_enabled=manager.config.rl_depth_enabled,
                    depth_size=manager.config.rl_depth_size,
                    map_enabled=manager.config.rl_map_size > 0,
                    map_size=manager.config.rl_map_size,
                    num_neighbors=manager.config.rl_num_neighbors,
                ),
            ),
            manager.config.device,
        )
        loaded_step = ppo.load(checkpoint)
        if loaded_step != args.expected_step:
            raise RuntimeError(f"CHECKPOINT_STEP_MISMATCH: {loaded_step}")
        ppo.model.eval()
        agent.eval()

        robot_count = manager.config.num_robots
        offset = manager.config.num_humanoids
        initial_routes = {
            "starts_xy": manager.starts_xy.tolist(),
            "goals_xy": manager.goals_xy.tolist(),
            "robot_spawn_yaws": manager._robot_spawn_yaws.tolist(),
        }
        route_digest = hashlib.sha256(
            json.dumps(initial_routes, sort_keys=True).encode()
        ).hexdigest()
        write_json(output / "INITIAL_ROUTES.json", initial_routes)
        map_digest = hashlib.sha256(manager.obstacle_map.tobytes()).hexdigest()
        print(f"EVAL_READY checkpoint_step={loaded_step} route_sha256={route_digest}", flush=True)

        episode_numbers = [0] * robot_count
        episode_steps = [0] * robot_count
        episode_returns = [0.0] * robot_count
        episode_empty_depth = [0] * robot_count
        episode_neighbor_active = [0] * robot_count
        count = Counter()
        done_indices = None
        control_steps = 0
        episodes = []
        with (output / "episodes.jsonl").open("x") as stream:
            while len(episodes) < args.episodes and control_steps < args.max_control_steps:
                obs, _ = env.reset(done_indices)
                obs = agent.add_agent_info_to_obs(obs)
                with torch.no_grad():
                    model_outs = agent.model(agent.obs_dict_to_tensordict(obs))
                    humanoid_action = model_outs.get("mean_action", model_outs["action"])
                    vector, neighbors, mask, depth, local_map = manager.get_robot_rl_observations()
                    for key, tensor in (("vector", vector), ("neighbors", neighbors),
                                        ("depth", depth), ("map", local_map)):
                        if tensor is None or not bool(torch.isfinite(tensor).all()):
                            raise RuntimeError(f"INVALID_ACTOR_INPUT_{key.upper()}")
                    depth_nonzero = (depth > 0).flatten(1).any(dim=1).tolist()
                    neighbors_active = mask.any(dim=1).tolist()
                    mean, _, _ = ppo.model(vector, depth, local_map, neighbors, mask)
                    action = bounded_robot_action(mean)
                    if not bool(torch.isfinite(action).all()):
                        raise RuntimeError("NONFINITE_POLICY_ACTION")
                manager.set_robot_rl_actions(action)
                _, _, human_done, _, _ = env.step(humanoid_action)
                env.simulator._sim.render()
                _, reward, robot_done, info, *_ = manager.get_robot_rl_feedback()
                count["actor_reads"] += robot_count
                count["empty_depth_reads"] += sum(not item for item in depth_nonzero)
                count["neighbor_active_reads"] += sum(neighbors_active)
                for robot_id in range(robot_count):
                    episode_steps[robot_id] += 1
                    episode_returns[robot_id] += float(reward[robot_id])
                    episode_empty_depth[robot_id] += int(not depth_nonzero[robot_id])
                    episode_neighbor_active[robot_id] += int(neighbors_active[robot_id])
                    if not bool(robot_done[robot_id]):
                        continue
                    reason = {key: bool(info[key][robot_id])
                              for key in ("reached", "collision", "timeout", "stuck")}
                    row = {
                        "episode_index": len(episodes), "robot_id": robot_id,
                        "robot_episode_number": episode_numbers[robot_id],
                        "control_step_terminal": control_steps + 1,
                        "steps": episode_steps[robot_id],
                        "seconds_sim": episode_steps[robot_id] / manager.config.update_hz,
                        "start_xy": manager.starts_xy[offset + robot_id].tolist(),
                        "goal_xy": manager.goals_xy[offset + robot_id].tolist(),
                        "return": episode_returns[robot_id],
                        "empty_depth_reads": episode_empty_depth[robot_id],
                        "neighbor_active_reads": episode_neighbor_active[robot_id],
                        **reason,
                        "safe_success": reason["reached"] and not reason["collision"],
                    }
                    stream.write(json.dumps(row, allow_nan=False) + "\n")
                    stream.flush()
                    episodes.append(row)
                    episode_numbers[robot_id] += 1
                    episode_steps[robot_id] = 0
                    episode_returns[robot_id] = 0.0
                    episode_empty_depth[robot_id] = 0
                    episode_neighbor_active[robot_id] = 0
                manager.reset_robot_rl_episodes(robot_done)
                done_indices = human_done.nonzero(as_tuple=False).squeeze(-1)
                control_steps += 1
                if control_steps % 100 == 0:
                    print(f"EVAL_PROGRESS steps={control_steps} episodes={len(episodes)} "
                          f"safe_success={sum(e['safe_success'] for e in episodes)} "
                          f"collision={sum(e['collision'] for e in episodes)}", flush=True)

        metrics = {
            "status": "COMPLETE" if len(episodes) >= args.episodes else "INCONCLUSIVE_MAX_STEPS",
            "checkpoint": str(checkpoint), "checkpoint_sha256": actual_sha,
            "checkpoint_step": loaded_step, "seed": args.seed,
            "scenario": "warehouse_10_humanoids_10_robots_full_stage_deterministic_policy",
            "route_sha256": route_digest, "map_sha256": map_digest,
            "requested_episodes": args.episodes, "completed_episodes": len(episodes),
            "control_steps": control_steps,
            "reached": sum(e["reached"] for e in episodes),
            "collision": sum(e["collision"] for e in episodes),
            "timeout": sum(e["timeout"] for e in episodes),
            "stuck": sum(e["stuck"] for e in episodes),
            "safe_success": sum(e["safe_success"] for e in episodes),
            "mean_sim_seconds": sum(e["seconds_sim"] for e in episodes) / max(len(episodes), 1),
            "actor_reads": count["actor_reads"],
            "empty_depth_reads": count["empty_depth_reads"],
            "neighbor_active_reads": count["neighbor_active_reads"],
            "ended_utc": utc(),
            "validity_notes": [
                "Held-out path seed and full-stage bounds; no PPO update.",
                "Shared seed fixes initial route set, later resets can diverge by policy outcome.",
                "Collision is CrowdSim task detector/drive guard; not PhysX contact proof.",
                "Human policy and robot actor use deterministic means in this evaluation.",
            ],
        }
        write_json(output / "metrics.json", metrics)
        print("EVAL_RESULT " + json.dumps(metrics, allow_nan=False), flush=True)
        return 0 if metrics["status"] == "COMPLETE" else 2
    finally:
        if runtime is not None:
            shutdown_runtime(runtime, manager)


if __name__ == "__main__":
    raise SystemExit(main())
