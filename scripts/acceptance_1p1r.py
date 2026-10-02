#!/usr/bin/env python3
"""Runtime acceptance for one moving humanoid and one stationary Nova Carter."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time
import traceback

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from protomotions.utils.simulator_imports import import_simulator_before_torch

import_simulator_before_torch("isaaclab")

import torch
from PIL import Image, ImageDraw

from CrowdSim.ppo.ppo_policy import RobotPPOTrainer, bounded_robot_action
from CrowdSim.utils.config_loader import load_config


def _quat_matrix_wxyz(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = (float(value) for value in quat)
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _camera_projection(env, human_xyz: np.ndarray) -> dict:
    camera = env.crowdsim_robot_camera
    camera_pos = camera.data.pos_w[0].detach().cpu().numpy().astype(np.float64)
    camera_quat_ros = camera.data.quat_w_ros[0].detach().cpu().numpy().astype(np.float64)
    rotation_world_from_optical = _quat_matrix_wxyz(camera_quat_ros)
    optical = rotation_world_from_optical.T @ (human_xyz.astype(np.float64) - camera_pos)
    intrinsics = camera.data.intrinsic_matrices[0].detach().cpu().numpy().astype(np.float64)
    z_forward = float(optical[2])
    if z_forward <= 1.0e-6:
        pixel = None
    else:
        pixel = [
            float(intrinsics[0, 0] * optical[0] / z_forward + intrinsics[0, 2]),
            float(intrinsics[1, 1] * optical[1] / z_forward + intrinsics[1, 2]),
        ]
    return {
        "camera_pos_w": camera_pos.tolist(),
        "camera_quat_w_ros": camera_quat_ros.tolist(),
        "intrinsics": intrinsics.tolist(),
        "human_root_optical_xyz": optical.tolist(),
        "projected_pixel_uv": pixel,
    }


def _save_sensor_sample(
    sample_dir: Path,
    label: str,
    step: int,
    env,
    human_xyz: np.ndarray,
    metadata: dict,
) -> dict:
    sample_dir.mkdir(parents=True, exist_ok=True)
    camera = env.crowdsim_robot_camera
    output = camera.data.output
    rgb = output["rgb"][0].detach().cpu()
    if rgb.dtype != torch.uint8:
        rgb = (rgb.clamp(0.0, 1.0) * 255.0).to(torch.uint8)
    rgb_np = rgb[..., :3].numpy()
    depth = output["distance_to_image_plane"][0].detach().cpu()
    if depth.ndim == 3:
        depth = depth.squeeze(-1)
    depth_np = depth.numpy().astype(np.float32)
    projection = _camera_projection(env, human_xyz)

    stem = f"{step:04d}_{label}"
    rgb_path = sample_dir / f"rgb_{stem}.png"
    annotated_path = sample_dir / f"rgb_{stem}_projected.png"
    depth_tensor_path = sample_dir / f"depth_{stem}.pt"
    depth_png_path = sample_dir / f"depth_{stem}.png"
    Image.fromarray(rgb_np).save(rgb_path)
    annotated = Image.fromarray(rgb_np.copy())
    draw = ImageDraw.Draw(annotated)
    pixel = projection["projected_pixel_uv"]
    roi_stats: dict[str, float | int | bool | None] = {
        "projection_in_frame": False,
        "roi_valid_depth_count": 0,
        "roi_closest_depth_error_m": None,
        "roi_rgb_std": None,
    }
    if pixel is not None:
        u, v = pixel
        height, width = depth_np.shape
        if 0 <= u < width and 0 <= v < height:
            roi_stats["projection_in_frame"] = True
            radius = 24
            x0, x1 = max(0, int(u) - radius), min(width, int(u) + radius + 1)
            y0, y1 = max(0, int(v) - radius), min(height, int(v) + radius + 1)
            roi_depth = depth_np[y0:y1, x0:x1]
            valid = np.isfinite(roi_depth) & (roi_depth > 0.1)
            roi_stats["roi_valid_depth_count"] = int(valid.sum())
            expected_depth = float(projection["human_root_optical_xyz"][2])
            if valid.any():
                roi_stats["roi_closest_depth_error_m"] = float(
                    np.min(np.abs(roi_depth[valid] - expected_depth))
                )
            roi_stats["roi_rgb_std"] = float(rgb_np[y0:y1, x0:x1].std())
            draw.ellipse((u - 12, v - 12, u + 12, v + 12), outline=(255, 0, 0), width=3)
            draw.line((u - 18, v, u + 18, v), fill=(255, 0, 0), width=2)
            draw.line((u, v - 18, u, v + 18), fill=(255, 0, 0), width=2)
    annotated.save(annotated_path)
    torch.save(depth, depth_tensor_path)
    finite = np.isfinite(depth_np)
    depth_vis = np.zeros_like(depth_np, dtype=np.uint16)
    valid_vis = finite & (depth_np > 0.0) & (depth_np <= 5.0)
    depth_vis[valid_vis] = np.clip(depth_np[valid_vis] / 5.0 * 65535.0, 0, 65535).astype(np.uint16)
    Image.fromarray(depth_vis, mode="I;16").save(depth_png_path)

    record = {
        **metadata,
        **projection,
        **roi_stats,
        "rgb_path": str(rgb_path),
        "rgb_annotated_path": str(annotated_path),
        "depth_tensor_path": str(depth_tensor_path),
        "depth_png_path": str(depth_png_path),
        "raw_rgb_shape": list(rgb_np.shape),
        "raw_depth_shape": list(depth_np.shape),
        "raw_depth_finite_ratio": float(finite.mean()),
    }
    (sample_dir / f"metadata_{stem}.json").write_text(
        json.dumps(record, indent=2), encoding="utf-8"
    )
    return record


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="CrowdSim/config/env_acceptance_1p1r.yaml")
    parser.add_argument(
        "--checkpoint",
        default=(
            "output/crowdsim_robot_ppo_smoke_20261001/20261001_003402/"
            "robot_ppo_latest.pt"
        ),
    )
    parser.add_argument("--max-steps", type=int, default=140)
    parser.add_argument("--run-label", default="01")
    parser.add_argument(
        "--log-dir",
        default=str(Path(__file__).resolve().parents[2] / "Assets/repro_logs/acceptance_1p1r"),
    )
    args = parser.parse_args()

    config = load_config(ROOT / args.config)
    assert config["navigation"]["num_humanoids"] == 1
    assert config["navigation"]["num_robots"] == 1
    assert config["humanoid"]["human_mesh"] is False
    log_dir = Path(args.log_dir).expanduser().resolve()
    sample_dir = log_dir / f"sensor_samples_{args.run_label}"
    interaction_path = log_dir / f"interaction_{args.run_label}.jsonl"
    summary_path = log_dir / f"acceptance_summary_{args.run_label}.json"
    checkpoint = (ROOT / args.checkpoint).resolve()

    runtime = None
    nav = None
    summary: dict = {
        "status": "FAILED",
        "config": str((ROOT / args.config).resolve()),
        "checkpoint": str(checkpoint),
        "human_representation": "bare smoke skeleton (human_mesh=false)",
    }
    log_file = None
    try:
        from CrowdSim.world.builder import build_env

        build_started = time.monotonic()
        env, agent, nav, runtime = build_env(config, num_envs=1, headless=True)
        summary["build_env_seconds"] = time.monotonic() - build_started
        agent.eval()

        payload = torch.load(checkpoint, map_location=runtime.fabric.device, weights_only=False)
        trainer = RobotPPOTrainer(payload["config"], runtime.fabric.device)
        loaded_step = trainer.load(checkpoint)
        trainer.model.eval()
        summary["checkpoint_loaded_step"] = loaded_step

        reset_started = time.monotonic()
        humanoid_obs, _ = env.reset(None)
        summary["initial_reset_seconds"] = time.monotonic() - reset_started
        if getattr(env.simulator, "headless", False):
            env.simulator._sim.render()

        log_file = interaction_path.open("w", encoding="utf-8")
        initial_positions, _ = nav._read_agent_state()
        initial_distance = float(np.linalg.norm(initial_positions[0] - initial_positions[1]))
        initial_starts = nav.starts_xy.copy()
        initial_goals = nav.goals_xy.copy()
        initial_human_start = initial_starts[0].copy()
        initial_robot_start = initial_starts[1].copy()
        initial_robot_goal = initial_goals[1].copy()
        initial_mask = int(nav.get_robot_rl_observations()[2].sum().item())
        summary["initial_human_robot_distance_m"] = initial_distance
        summary["initial_neighbor_mask_count"] = initial_mask

        episode_return = 0.0
        minimum_distance = initial_distance
        mask_counts = [initial_mask]
        human_positions = [initial_positions[0].copy()]
        sensor_samples: list[dict] = []
        captured_labels: set[str] = set()
        terminal_record = None
        previous_robot_xy = initial_positions[1].copy()
        done_indices = None

        for step in range(args.max_steps):
            if step > 0:
                humanoid_obs, _ = env.reset(done_indices)
            obs = agent.add_agent_info_to_obs(humanoid_obs)
            obs_td = agent.obs_dict_to_tensordict(obs)
            with torch.no_grad():
                model_outs = agent.model(obs_td)
                humanoid_action = model_outs.get("mean_action", model_outs["action"])
                robot_obs, neighbors, neighbor_mask, depth, map_patch = (
                    nav.get_robot_rl_observations()
                )
                raw_mean, _, policy_value = trainer.model(
                    robot_obs, depth, map_patch, neighbors, neighbor_mask
                )
                bounded_policy_action = bounded_robot_action(raw_mean)

            # Controlled diagnostic: keep Carter stationary while the human
            # traverses its fixed route.  The checkpoint still performs real
            # inference; policy output and the explicit execution override are
            # logged separately so requested and executed motion are not mixed.
            requested_action = torch.zeros_like(bounded_policy_action)
            nav.set_robot_rl_actions(requested_action)
            _, _, humanoid_dones, _, _ = env.step(humanoid_action)
            if getattr(env.simulator, "headless", False):
                env.simulator._sim.render()
            _, reward, robot_done, info, depth_after, _, neighbors_after, mask_after = (
                nav.get_robot_rl_feedback()
            )
            positions, velocities = nav._read_agent_state()
            robot_yaw = float(nav.robot_yaws()[0])
            relative = positions[0] - positions[1]
            distance = float(np.linalg.norm(relative))
            bearing = math.atan2(float(relative[1]), float(relative[0])) - robot_yaw
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            relative_velocity = velocities[0] - velocities[1]
            mask_value = int(mask_after[0].sum().item())
            neighbor_row = neighbors_after[0, 0].detach().cpu().numpy()
            observed_distance = (
                float(neighbor_row[0]) * float(nav.config.neighbor_radius)
                if mask_value
                else None
            )
            observed_bearing = (
                math.atan2(float(neighbor_row[1]), float(neighbor_row[2]))
                if mask_value
                else None
            )
            robot_xy = positions[1].copy()
            actual_delta = robot_xy - previous_robot_xy
            previous_robot_xy = robot_xy
            drive_velocity = nav.drive.velocities_xy()[0].copy()
            drive_angular = float(nav.drive.angular_velocities()[0])
            camera = env.crowdsim_robot_camera
            human_root = env.simulator.get_root_state().root_pos[0].detach().cpu().numpy()
            rgb = nav._read_camera_rgb()
            depth_now = nav._read_camera_depth()
            record = {
                "step": step + 1,
                "sim_time": float((step + 1) * nav._dt()),
                "robot_xy": robot_xy.tolist(),
                "robot_yaw": robot_yaw,
                "policy_raw_mean": raw_mean[0].detach().cpu().tolist(),
                "policy_bounded_action": bounded_policy_action[0].detach().cpu().tolist(),
                "requested_action": requested_action[0].detach().cpu().tolist(),
                "requested_linear_angular_velocity": nav.drive._commands_from_actions()[0].tolist(),
                "actual_robot_pose_delta_xy": actual_delta.tolist(),
                "executed_velocity_xy": drive_velocity.tolist(),
                "executed_angular_velocity": drive_angular,
                "wall_guard_blocked": bool(nav.drive.wall_blocked[0]),
                "human_xy": positions[0].tolist(),
                "human_yaw": float(nav._humanoid_target_yaws[0]),
                "human_velocity_xy": velocities[0].tolist(),
                "neighbors": neighbors_after[0].detach().cpu().tolist(),
                "neighbor_mask": mask_after[0].detach().cpu().tolist(),
                "relative_distance_world_m": distance,
                "relative_distance_observation_m": observed_distance,
                "relative_bearing_world_rad": bearing,
                "relative_bearing_observation_rad": observed_bearing,
                "relative_velocity_world_xy": relative_velocity.tolist(),
                "depth_min": float(depth_now.min()) if depth_now is not None else None,
                "depth_max": float(depth_now.max()) if depth_now is not None else None,
                "depth_valid_ratio": (
                    float((depth_now > 0).float().mean()) if depth_now is not None else None
                ),
                "rgb_frame_id": step + 1,
                "rgb_min": int(rgb.min()) if rgb is not None else None,
                "rgb_max": int(rgb.max()) if rgb is not None else None,
                "reward": float(reward[0]),
                "policy_value": float(policy_value[0]),
                "done": bool(robot_done[0]),
                "reached": bool(info["reached"][0]),
                "collision": bool(info["collision"][0]),
                "timeout": bool(info["timeout"][0]),
                "stuck": bool(info["stuck"][0]),
            }
            for key, value in info.items():
                if key.startswith("reward_") and isinstance(value, torch.Tensor):
                    record[key] = float(value[0])
            log_file.write(json.dumps(record) + "\n")
            log_file.flush()

            episode_return += float(reward[0])
            minimum_distance = min(minimum_distance, distance)
            mask_counts.append(mask_value)
            human_positions.append(positions[0].copy())

            label = None
            if step == 0:
                label = "pre_entry"
            elif mask_value and "neighbor_active" not in captured_labels:
                label = "neighbor_active"
            elif distance <= 3.0 and "interaction_3m" not in captured_labels:
                label = "interaction_3m"
            if bool(robot_done[0]):
                label = "terminal"
            if label is not None and label not in captured_labels:
                sensor_samples.append(
                    _save_sensor_sample(
                        sample_dir,
                        label,
                        step + 1,
                        env,
                        human_root,
                        {
                            "label": label,
                            "step": step + 1,
                            "sim_time": float((step + 1) * nav._dt()),
                            "robot_xy": robot_xy.tolist(),
                            "robot_yaw": robot_yaw,
                            "human_root_xyz": human_root.tolist(),
                            "human_xy": positions[0].tolist(),
                            "relative_distance_m": distance,
                        },
                    )
                )
                captured_labels.add(label)

            done_indices = humanoid_dones.nonzero(as_tuple=False).squeeze(-1)
            if bool(robot_done[0]):
                reasons = [
                    key
                    for key in ("reached", "collision", "timeout", "stuck")
                    if bool(info[key][0])
                ]
                terminal_record = {
                    "step": step + 1,
                    "sim_time": float((step + 1) * nav._dt()),
                    "reason": "+".join(reasons),
                    "position": robot_xy.tolist(),
                    "distance_to_goal": float(info["distance_to_goal"][0]),
                    "episode_return": episode_return,
                    "reward_breakdown": {
                        key: float(value[0])
                        for key, value in info.items()
                        if key.startswith("reward_") and isinstance(value, torch.Tensor)
                    },
                }
                nav.reset_robot_rl_episodes(robot_done)
                # Exercise a real full environment reset after the robot episode
                # terminates.  The robot episode reset above validates the
                # independent robot lifecycle; this full reset must also return
                # the humanoid to its configured route start.
                all_env_ids = torch.arange(
                    env.num_envs, dtype=torch.long, device=runtime.fabric.device
                )
                env.reset(all_env_ids)
                if getattr(env.simulator, "headless", False):
                    env.simulator._sim.render()
                reset_positions, _ = nav._read_agent_state()
                reset_obs = nav.get_robot_rl_observations()
                reset_robot_yaw = float(nav.robot_yaws()[0])
                reset_camera_pos = (
                    env.crowdsim_robot_camera.data.pos_w[0]
                    .detach().cpu().numpy()
                )
                expected_camera_pos = np.asarray(
                    [
                        reset_positions[1, 0] + 0.25 * math.cos(reset_robot_yaw),
                        reset_positions[1, 1] + 0.25 * math.sin(reset_robot_yaw),
                        0.6,
                    ],
                    dtype=np.float32,
                )
                summary["reset_verification"] = {
                    "robot_xy": reset_positions[1].tolist(),
                    "human_xy": reset_positions[0].tolist(),
                    "robot_goal_xy": nav.goals_xy[1].tolist(),
                    "robot_start_matches": bool(
                        np.linalg.norm(reset_positions[1] - initial_robot_start) < 0.15
                    ),
                    "human_start_matches": bool(
                        np.linalg.norm(reset_positions[0] - initial_human_start) < 0.15
                    ),
                    "goal_matches": bool(
                        np.linalg.norm(nav.goals_xy[1] - initial_robot_goal) < 1.0e-5
                    ),
                    "episode_steps_zero": bool(nav._robot_episode_steps[0] == 0),
                    "previous_action_zero": bool(
                        np.allclose(nav._robot_rl_prev_actions[0], 0.0)
                    ),
                    "neighbor_mask_count": int(reset_obs[2][0].sum()),
                    "depth_finite": bool(torch.isfinite(reset_obs[3]).all()),
                    "map_finite": bool(torch.isfinite(reset_obs[4]).all()),
                    "camera_mount_error_m": float(
                        np.linalg.norm(reset_camera_pos - expected_camera_pos)
                    ),
                }
                break

        human_displacement = float(np.linalg.norm(human_positions[-1] - human_positions[0]))
        active_records = [sample for sample in sensor_samples if sample["label"] != "pre_entry"]
        depth_human_match = any(
            sample.get("roi_closest_depth_error_m") is not None
            and float(sample["roi_closest_depth_error_m"]) < 0.5
            for sample in active_records
        )
        summary.update(
            {
                "status": "PASS" if terminal_record is not None else "FAILED",
                "minimum_human_robot_distance_m": minimum_distance,
                "neighbor_mask_transition_0_to_1": bool(0 in mask_counts and max(mask_counts) > 0),
                "neighbor_mask_ever_active": bool(max(mask_counts) > 0),
                "human_displacement_m": human_displacement,
                "human_movement_verified": human_displacement > 0.1,
                "terminal": terminal_record,
                "sensor_samples": sensor_samples,
                "depth_human_correspondence": depth_human_match,
                "rgb_valid": all(sample["raw_rgb_shape"] == [480, 640, 3] for sample in sensor_samples),
                "human_projected_in_rgb": any(
                    bool(sample["projection_in_frame"])
                    and sample["roi_rgb_std"] is not None
                    for sample in active_records
                ),
                "neighbor_distance_max_abs_error_m": max(
                    (
                        abs(
                            float(json.loads(line)["relative_distance_world_m"])
                            - float(json.loads(line)["relative_distance_observation_m"])
                        )
                        for line in interaction_path.read_text().splitlines()
                        if json.loads(line)["relative_distance_observation_m"] is not None
                    ),
                    default=None,
                ),
            }
        )
        assert loaded_step == 128
        assert summary["human_movement_verified"]
        assert summary["neighbor_mask_transition_0_to_1"]
        assert terminal_record is not None
        reset_check = summary["reset_verification"]
        assert reset_check["robot_start_matches"]
        assert reset_check["human_start_matches"]
        assert reset_check["goal_matches"]
        assert reset_check["episode_steps_zero"]
        assert reset_check["previous_action_zero"]
        assert reset_check["depth_finite"] and reset_check["map_finite"]
        assert reset_check["camera_mount_error_m"] < 0.02
    except BaseException as exc:
        summary["status"] = "FAILED"
        summary["exception"] = repr(exc)
        traceback.print_exc()
    finally:
        if log_file is not None and not log_file.closed:
            log_file.close()
        summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(f"[acceptance] summary={summary_path}", flush=True)
        print(f"[acceptance] status={summary['status']}", flush=True)
        from CrowdSim.protomotions_runtime import shutdown_runtime

        shutdown_runtime(runtime, nav)


if __name__ == "__main__":
    main()
