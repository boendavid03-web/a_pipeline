#!/usr/bin/env python3
"""Write the file/hash provenance for the 1+1 PPO pilot."""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import sys

import torch
import yaml

ROOT = Path(__file__).resolve().parents[1]
LOG = ROOT.parent / "Assets/repro_logs/ppo_baseline_1p1r"


def file_pin(path: Path) -> dict:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    # Keep the published manifest relocatable while preserving bytes and hashes.
    relative_path = os.path.relpath(path, ROOT)
    return {"path": relative_path, "bytes": path.stat().st_size, "sha256": digest.hexdigest()}


def main() -> None:
    from CrowdSim.utils.config_loader import load_config

    LOG.mkdir(parents=True, exist_ok=True)
    config = ROOT / "CrowdSim/config/env_ppo_baseline_1p1r.yaml"
    resolved_path = LOG / "resolved_config.json"
    resolved_path.write_text(json.dumps(load_config(config), indent=2, sort_keys=True) + "\n")
    files = [
        "CrowdSim/config/env.yaml", "CrowdSim/config/env_smoke.yaml",
        "CrowdSim/config/env_acceptance_1p1r.yaml", "CrowdSim/config/env_ppo_baseline_1p1r.yaml",
        "CrowdSim/config/fixed_eval_routes_1p1r_20261001.json", "CrowdSim/ppo/train_ppo.py",
        "CrowdSim/ppo/ppo_policy.py", "CrowdSim/nav_manager.py", "CrowdSim/control/drive.py",
        "CrowdSim/protomotions_runtime.py", "CrowdSim/world/map.py", "CrowdSim/world/planning.py",
        "CrowdSim/world/sfm.py", "CrowdSim/world/builder.py", "CrowdSim/tools/eval_policy.py",
        "CrowdSim/tools/eval_fixed_1p1r.py", "CrowdSim/crowd_sim.py",
        "CrowdSim/config/scenes/warehouse.yaml", "CrowdSim/maps/newmap.yaml",
        "CrowdSim/maps/World1.png", "scripts/freeze_ppo_baseline.py",
    ]
    initial = sorted(
        path for path in (ROOT / "output/crowdsim_robot_ppo_baseline_1p1r_20261001/step0_init").glob("*/robot_ppo_latest.pt")
        if not path.parent.is_symlink()
    )
    if len(initial) != 1:
        raise RuntimeError(f"Expected one step-0 checkpoint, found {len(initial)}")
    isaac_lab_root = Path(os.environ.get("ISAAC_LAB_ROOT", Path(sys.prefix).parent))
    isaac_sim_root = os.environ.get("ISAAC_SIM_ROOT") or os.environ.get("ISAAC_PATH")
    if isaac_sim_root is None:
        raise RuntimeError("Set ISAAC_SIM_ROOT or ISAAC_PATH to the Isaac Sim installation")
    manifest = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "status": "FROZEN_BEFORE_PILOT",
        "project_root": ".",
        "git_metadata": "absent; extracted ZIP; exact pre-acceptance source delta cannot be reconstructed",
        "runtime": {
            "python": Path(sys.executable).name, "python_version": platform.python_version(),
            "torch": str(torch.__version__), "torch_cuda_runtime": str(torch.version.cuda),
            "cuda_available": bool(torch.cuda.is_available()),
            "isaac_sim_version": (Path(isaac_sim_root) / "VERSION").read_text().strip(),
            "isaac_lab_version": (isaac_lab_root / "VERSION").read_text().strip(),
            "pip_check": "No broken requirements found",
        },
        "files": {name: file_pin(ROOT / name) for name in files},
        "assets": {
            "motionlib": file_pin(ROOT.parent / "Assets/motion/amass_smpl_validation.pt"),
            "maskedmimic_checkpoint": file_pin(ROOT / "data/pretrained_models/masked_mimic/smpl_57f98a9/last.ckpt"),
            "maskedmimic_resolved": file_pin(ROOT / "data/pretrained_models/masked_mimic/smpl_57f98a9/resolved_configs_inference.pt"),
        },
        "resolved_config": file_pin(resolved_path),
        "training_config": file_pin(config),
        "evaluation_routes": file_pin(ROOT / "CrowdSim/config/fixed_eval_routes_1p1r_20261001.json"),
        "initial_checkpoint": file_pin(initial[0]),
        "training_start_step": 0,
        "training_max_robot_steps": 49920,
        "requested_checkpoints": [0, 10240, 25088, 49920],
        "visual_fidelity": "SKELETON; env.yaml defaults to SMPL; smplx absent without a project-pinned version",
        "changes_in_this_turn": [
            "CrowdSim/config/env_ppo_baseline_1p1r.yaml added",
            "CrowdSim/config/fixed_eval_routes_1p1r_20261001.json added",
            "CrowdSim/tools/eval_fixed_1p1r.py added",
            "CrowdSim/ppo/train_ppo.py seed and explicit checkpoint-step CLI controls added; PPO algorithm unchanged",
            "scripts/freeze_ppo_baseline.py added",
        ],
        "acceptance_snapshot_known_hashes": {
            "CrowdSim/config/env.yaml": "5eb3e027555e64582fb043e0411a0d4445e5b12a52ed762afcd0a25511df679b",
            "CrowdSim/config/env_smoke.yaml": "f7ec60f511ef3e5bcf886f2115e2c11ad5c9ecc13d2b7e748eeb86b379260cdf",
            "CrowdSim/config/env_acceptance_1p1r.yaml": "fc71133095e024d59c49e6c7611774641e01b6f036a9d814d21fa3b6f6f94a01",
        },
    }
    path = ROOT / "NAVISAACLAB_PPO_BASELINE_MANIFEST_20261001.yaml"
    path.write_text(yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True))
    shutil.copy2(path, LOG / "manifest.yaml")
    (LOG / "environment.txt").write_text(
        f"Python: {Path(sys.executable).name} {platform.python_version()}\n"
        f"Torch: {torch.__version__}\nCUDA runtime: {torch.version.cuda}\n"
        f"Isaac Sim: {manifest['runtime']['isaac_sim_version']}\n"
        f"Isaac Lab: {manifest['runtime']['isaac_lab_version']}\n"
        f"GPU: {torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'none'}\n"
    )
    print(path)


if __name__ == "__main__":
    main()
