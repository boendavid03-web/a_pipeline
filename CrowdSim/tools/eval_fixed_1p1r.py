#!/usr/bin/env python3
"""Frozen one-human/one-robot evaluation for the PPO pilot.

Each invocation runs one prevalidated episode in a fresh Isaac process.  The
training configuration is loaded unchanged, then only the fixed evaluation
scenario and deterministic route seed are injected into the in-memory copy.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import random
import sys
import time

import cv2
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def validate_routes(route_file: Path) -> dict:
    suite = json.loads(route_file.read_text(encoding="utf-8"))
    map_path = ROOT / suite["map"]
    image = np.asarray(Image.open(map_path).convert("L"))
    free = (image >= 250).astype(np.uint8)
    clearance = cv2.distanceTransform(free, cv2.DIST_L2, 5) * float(suite["map_resolution_m"])
    resolution = float(suite["map_resolution_m"])
    ox, oy = suite["map_origin_xy"]
    minimum = float(suite["planning_clearance_m"])
    result = {}
    for episode in suite["episodes"]:
        distances = {}
        for actor in ("robot", "human"):
            start = np.asarray(episode[actor]["start"], dtype=np.float64)
            goal = np.asarray(episode[actor]["goal"], dtype=np.float64)
            length = float(np.linalg.norm(goal - start))
            samples = max(2, math.ceil(length / (0.5 * resolution)) + 1)
            observed = []
            for x, y in np.linspace(start, goal, samples):
                px = round((x - ox) / resolution)
                py = round(image.shape[0] - 1 - (y - oy) / resolution)
                if not (0 <= px < image.shape[1] and 0 <= py < image.shape[0]):
                    raise ValueError(f"{episode['id']} {actor} route is outside the map")
                observed.append(float(clearance[py, px]))
            distances[actor] = min(observed)
            if distances[actor] < minimum:
                raise ValueError(f"{episode['id']} {actor} clearance {distances[actor]:.3f} < {minimum}")
        robot_start = np.asarray(episode["robot"]["start"], dtype=np.float64)
        human_start = np.asarray(episode["human"]["start"], dtype=np.float64)
        start_distance = float(np.linalg.norm(robot_start - human_start))
        if start_distance < 1.5:
            raise ValueError(f"{episode['id']} spawn spacing {start_distance:.3f} < 1.5")
        result[episode["id"]] = {"clearance_m": distances, "start_spacing_m": start_distance}
    return {"suite": suite, "validation": result}


def run_episode(args, suite: dict, episode: dict) -> None:
    from protomotions.utils.simulator_imports import import_simulator_before_torch
    import_simulator_before_torch("isaaclab")
    import torch
    torch.set_float32_matmul_precision("high")
    from CrowdSim.crowd_sim import get_network_config
    from CrowdSim.ppo.ppo_policy import RobotPPOConfig, RobotPPOTrainer, bounded_robot_action, robot_network_kwargs
    from CrowdSim.protomotions_runtime import shutdown_runtime
    from CrowdSim.utils.config_loader import load_config
    from CrowdSim.world.builder import build_env

    random.seed(episode["seed"])
    np.random.seed(episode["seed"])
    torch.manual_seed(episode["seed"])
    torch.cuda.manual_seed_all(episode["seed"])
    config = load_config(ROOT / args.env_config)
    nav_cfg = config["navigation"]
    if (nav_cfg["num_humanoids"], nav_cfg["num_robots"]) != (1, 1):
        raise ValueError("Fixed evaluation requires exactly one human and one robot")
    nav_cfg["path"]["seed"] = int(episode["seed"])
    nav_cfg["scenario"] = {
        "type": "fixed_routes",
        "humanoid_routes": [episode["human"]],
        "robot_routes": [episode["robot"]],
    }
    nav_cfg["rl"]["initial_heading_min_offset_degrees"] = 0.0
    nav_cfg["rl"]["initial_heading_max_offset_degrees"] = 0.0
    nav_cfg["recording"]["output_dir"] = str(args.output.parent / f"navigation_{episode['id']}")
    runtime = nav = None
    started = time.monotonic()
    try:
        env, agent, nav, runtime = build_env(config, num_envs=1, headless=True)
        network_cfg = get_network_config(config)
        trainer = RobotPPOTrainer(
            RobotPPOConfig(
                obs_dim=nav.robot_rl_obs_dim,
                vector_obs_dim=nav.robot_rl_vector_obs_dim,
                **robot_network_kwargs(
                    network_cfg,
                    depth_enabled=nav.config.rl_depth_enabled,
                    depth_size=nav.config.rl_depth_size,
                    map_enabled=nav.config.rl_map_size > 0,
                    map_size=nav.config.rl_map_size,
                    num_neighbors=nav.config.rl_num_neighbors,
                ),
            ), runtime.fabric.device,
        )
        checkpoint_step = trainer.load(args.ckpt)
        agent.eval()
        trainer.model.eval()
        obs, _ = env.reset(None)
        if env.simulator.headless:
            env.simulator._sim.render()
        positions, _ = nav._read_agent_state()
        first_robot = positions[1].copy()
        first_human = positions[0].copy()
        prev_robot = first_robot.copy()
        path_length = 0.0
        min_distance = float(np.linalg.norm(first_human - first_robot))
        active_distances = []
        actions = []
        requested = []
        executed = []
        depth_valid = 0
        map_valid = 0
        sensor_steps = 0
        neighbor_active = 0
        terminal = "NOT_TERMINAL"
        done_indices = None
        for step in range(int(nav.config.rl_max_episode_steps)):
            if step:
                obs, _ = env.reset(done_indices)
            obs = agent.add_agent_info_to_obs(obs)
            obs_td = agent.obs_dict_to_tensordict(obs)
            with torch.no_grad():
                human_out = agent.model(obs_td)
                human_action = human_out.get("mean_action", human_out["action"])
                robot_obs, neighbors, neighbor_mask, depth, map_patch = nav.get_robot_rl_observations()
                mean, _, _ = trainer.model(robot_obs, depth, map_patch, neighbors, neighbor_mask)
                action = bounded_robot_action(mean)
            sensor_steps += 1
            depth_valid += int(depth is not None and bool(torch.isfinite(depth).all()) and bool((depth > 0).any()))
            map_valid += int(map_patch is not None and bool(torch.isfinite(map_patch).all()) and bool((map_patch != 0).any()))
            mask = bool(neighbor_mask[0].any())
            neighbor_active += int(mask)
            nav.set_robot_rl_actions(action)
            command = nav.drive._commands_from_actions()[0].copy()
            _, _, human_dones, _, _ = env.step(human_action)
            if env.simulator.headless:
                env.simulator._sim.render()
            _, _, robot_done, info, _, _, _, _ = nav.get_robot_rl_feedback()
            positions, _ = nav._read_agent_state()
            robot_xy, human_xy = positions[1], positions[0]
            delta = float(np.linalg.norm(robot_xy - prev_robot))
            path_length += delta
            prev_robot = robot_xy.copy()
            human_distance = float(np.linalg.norm(human_xy - robot_xy))
            min_distance = min(min_distance, human_distance)
            if mask:
                active_distances.append(human_distance)
            actions.append(action[0].detach().cpu().numpy().tolist())
            requested.append(command.tolist())
            executed.append(delta * float(nav.config.update_hz))
            done_indices = human_dones.nonzero(as_tuple=False).squeeze(-1)
            if bool(robot_done[0]):
                terminal = next(
                    (reason for reason in ("reached", "collision", "timeout", "stuck")
                     if bool(info.get(reason, torch.zeros(1, device=robot_done.device))[0])),
                    "other_terminal",
                )
                break
        final_positions, _ = nav._read_agent_state()
        goal = nav.goals_xy[1].copy()
        actions_np = np.asarray(actions, dtype=np.float64)
        requested_np = np.asarray(requested, dtype=np.float64)
        output = {
            "route_id": episode["id"], "layer": episode["layer"], "route_type": episode["type"],
            "seed": episode["seed"], "checkpoint_step": checkpoint_step,
            "requested_start_xy": episode["robot"]["start"], "actual_start_xy": first_robot.tolist(),
            "goal_xy": goal.tolist(), "human_start_xy": first_human.tolist(),
            "terminal_reason": terminal, "success": terminal == "reached",
            "collision": terminal == "collision", "timeout": terminal == "timeout",
            "stuck": terminal == "stuck", "steps": len(actions),
            "time_s": len(actions) / float(nav.config.update_hz),
            "final_goal_error_m": float(np.linalg.norm(final_positions[1] - goal)),
            "path_length_m": path_length,
            "human_displacement_m": float(np.linalg.norm(final_positions[0] - first_human)),
            "min_human_robot_distance_m": min_distance,
            "mean_human_robot_distance_when_active_m": float(np.mean(active_distances)) if active_distances else None,
            "neighbor_active_fraction": neighbor_active / max(len(actions), 1),
            "mean_linear_action": float(actions_np[:, 0].mean()),
            "mean_abs_angular_action": float(np.abs(actions_np[:, 1]).mean()),
            "action_saturation_fraction": float(np.mean((actions_np[:, 0] > .95) | (np.abs(actions_np[:, 1]) > .95))),
            "mean_requested_linear_velocity_mps": float(requested_np[:, 0].mean()),
            "mean_requested_abs_angular_velocity_radps": float(np.abs(requested_np[:, 1]).mean()),
            "mean_executed_speed_mps": float(np.mean(executed)),
            "executed_displacement_m": path_length,
            "depth_valid_fraction": depth_valid / max(sensor_steps, 1),
            "map_nonzero_fraction": map_valid / max(sensor_steps, 1),
            "wall_time_s_before_shutdown": time.monotonic() - started,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(output, indent=2), flush=True)
    finally:
        if runtime is not None:
            shutdown_runtime(runtime, nav)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--routes", type=Path, default=ROOT / "CrowdSim/config/fixed_eval_routes_1p1r_20261001.json")
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--route-id")
    parser.add_argument("--ckpt", type=Path)
    parser.add_argument("--env-config", default="CrowdSim/config/env_ppo_baseline_1p1r.yaml")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    checked = validate_routes(args.routes)
    if args.validate_only:
        print(json.dumps(checked["validation"], indent=2))
        return
    if not args.route_id or args.ckpt is None or args.output is None:
        parser.error("--route-id, --ckpt and --output are required for runtime evaluation")
    route = next((e for e in checked["suite"]["episodes"] if e["id"] == args.route_id), None)
    if route is None:
        parser.error(f"Unknown route id {args.route_id!r}")
    run_episode(args, checked["suite"], route)


if __name__ == "__main__":
    main()
