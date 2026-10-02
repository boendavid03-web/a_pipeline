#!/usr/bin/env python3
"""One humanoid and one robot Isaac bring-up with observation checks."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from protomotions.utils.simulator_imports import import_simulator_before_torch

import_simulator_before_torch("isaaclab")

import torch

from CrowdSim.utils.config_loader import load_config


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="CrowdSim/config/env_smoke.yaml")
    parser.add_argument("--steps", type=int, default=2)
    args = parser.parse_args()
    config = load_config(ROOT / args.config)
    assert config["navigation"]["num_humanoids"] == 1
    assert config["navigation"]["num_robots"] == 1
    print("STAGE config OK", flush=True)

    from CrowdSim.world.builder import build_env

    runtime = None
    nav = None
    try:
        build_started = time.monotonic()
        env, agent, nav, runtime = build_env(config, num_envs=1, headless=True)
        print(
            "STAGE build_env OK (AppLauncher, ProtoMotions, MaskedMimic, scene) "
            f"duration_seconds={time.monotonic() - build_started:.3f}",
            flush=True,
        )
        agent.eval()
        done_indices = None
        for step in range(args.steps):
            reset_started = time.monotonic()
            obs, _ = env.reset(done_indices)
            print(
                f"STAGE reset {step} OK "
                f"duration_seconds={time.monotonic() - reset_started:.3f}",
                flush=True,
            )
            obs = agent.add_agent_info_to_obs(obs)
            obs_td = agent.obs_dict_to_tensordict(obs)
            with torch.no_grad():
                model_outs = agent.model(obs_td)
                action = model_outs.get("mean_action", model_outs["action"])
            _, _, dones, _, _ = env.step(action)
            print(f"STAGE step {step} OK", flush=True)
            done_indices = dones.nonzero(as_tuple=False).squeeze(-1)

        vector, neighbors, mask, depth, map_patch = nav.get_robot_rl_observations()
        expected = ((1, 5), (1, 4, 5), (1, 4), (1, 224, 224), (1, 24, 24))
        observed = tuple(tuple(x.shape) for x in (vector, neighbors, mask, depth, map_patch))
        print("observation_shapes", observed, flush=True)
        assert observed == expected, (observed, expected)
        for tensor in (vector, neighbors, depth, map_patch):
            assert torch.isfinite(tensor).all()
        rgb = nav._read_camera_rgb()
        assert rgb is not None and rgb.shape == (1, 3, 224, 224)
        assert torch.any(rgb != 0), "RGB frame is all zero"
        assert torch.any(depth != 0), "depth observation is all zero"
        positions, _ = nav._read_agent_state()
        print("rgb_min_max", int(rgb.min()), int(rgb.max()), flush=True)
        print("depth_min_max_valid_ratio", float(depth.min()), float(depth.max()),
              float((depth > 0).float().mean()), flush=True)
        print("map_occupied_ratio", float((map_patch > 0).float().mean()), flush=True)
        print("neighbor_count", int(mask.sum()), flush=True)
        print("goal_distance_normalized", float(vector[0, 0]), flush=True)
        print("human_xy", positions[0].tolist(), "robot_xy", positions[1].tolist(), flush=True)
        print("STAGE observation OK", flush=True)
    finally:
        from CrowdSim.protomotions_runtime import shutdown_runtime

        shutdown_runtime(runtime, nav)


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        raise SystemExit(1)
