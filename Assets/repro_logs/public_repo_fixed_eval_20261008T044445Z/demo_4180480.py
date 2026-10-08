#!/usr/bin/env python3
"""Bounded GUI demo of the strongest checkpoint in the seed-701 screening."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import random
import sys

sys.dont_write_bytecode = True

SOURCE = Path("/home/user/workspace/NavIsaaclab2.0/strict_public_dependency_source_57f98a9_20261003")
CHECKPOINT = Path("/home/user/workspace/NavIsaaclab2.0/Assets/repro_logs/public_repo_resume_prepared_20261007T094706Z/training/20261007_024859/robot_ppo_4180480.pt")
SHA256 = "60bf5199690bcfb32b696a3fd9364b497ddd2a03974feb9bda5711162259cecc"
OUTPUT = Path(__file__).resolve().parent / "demo_4180480_gui"


def main() -> None:
    if Path.cwd().resolve() != SOURCE:
        raise RuntimeError("RUN_FROM_FROZEN_SOURCE_ROOT")
    if hashlib.sha256(CHECKPOINT.read_bytes()).hexdigest() != SHA256:
        raise RuntimeError("CHECKPOINT_SHA256_MISMATCH")
    OUTPUT.mkdir(exist_ok=False)

    from CrowdSim.crowd_sim import (
        cfg_path, load_config, run_masked_mimic_with_robot_ppo,
    )
    from CrowdSim.world.builder import build_env
    from CrowdSim.protomotions_runtime import shutdown_runtime
    import numpy as np
    import torch

    random.seed(701)
    np.random.seed(701)
    torch.manual_seed(701)
    torch.cuda.manual_seed_all(701)
    config = load_config(cfg_path("CrowdSim/config/env.yaml"))
    assert config["scene"] == "warehouse"
    assert (config["navigation"]["num_humanoids"], config["navigation"]["num_robots"]) == (10, 10)
    config["car"]["policy_checkpoint"] = str(CHECKPOINT)
    config["navigation"]["path"]["seed"] = 701
    config["navigation"]["rl"]["goal_curriculum"]["enabled"] = False
    config["navigation"]["rl"]["deterministic"] = True
    config["navigation"]["recording"]["enabled"] = True
    config["navigation"]["recording"]["output_dir"] = str(OUTPUT / "navigation")
    (OUTPUT / "config.json").write_text(json.dumps(config, indent=2) + "\n")

    runtime = manager = None
    try:
        result = build_env(config, num_envs=10, headless=False)
        if result is None:
            raise RuntimeError("BUILD_ENV_RETURNED_NONE")
        _, _, manager, runtime = result
        assert not manager.goal_curriculum.spec.enabled
        print("DEMO_READY_10_HUMANOIDS_10_ROBOTS_FULL_STAGE", flush=True)
        run_masked_mimic_with_robot_ppo(
            runtime, manager, config, max_episodes=10,
            gif_out=OUTPUT / "birdseye.gif", gif_every=4, gif_fps=7.5,
        )
        (OUTPUT / "DEMO_LOOP_COMPLETE.txt").write_text("10 robot episodes; see native log and GIF\n")
    finally:
        if runtime is not None:
            shutdown_runtime(runtime, manager)


if __name__ == "__main__":
    main()
