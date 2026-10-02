#!/usr/bin/env python3
"""Read-only NavIsaacLab host, dependency, and configured-asset probe."""
from __future__ import annotations

import argparse
import importlib.util
import os
import platform
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def field(path: Path, key: str) -> str | None:
    if not path.is_file():
        return None
    match = re.search(rf"(?m)^\s*{re.escape(key)}:\s*([^#\n]+)", path.read_text())
    return match.group(1).strip().strip('"\'') if match else None


def path_for(value: str | None) -> Path | None:
    if not value:
        return None
    path = Path(value).expanduser()
    return (path if path.is_absolute() else ROOT / path).resolve()


def status(label: str, path: Path | None, required: bool = True) -> None:
    if path is None:
        print(f"MISSING {label}: no configured path")
        return
    if not path.exists():
        print(f"{'MISSING' if required else 'OPTIONAL MISSING'} {label}: {path}")
    elif path.is_file() and is_lfs_pointer(path):
        print(f"LFS POINTER {label}: {path}")
    else:
        print(f"OK {label}: {path}")


def is_lfs_pointer(path: Path) -> bool:
    with path.open("rb") as handle:
        return handle.read(80).startswith(b"version https://git-lfs.github.com/spec/v1")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "CrowdSim/config/env.yaml")
    args = parser.parse_args()
    config = args.config.resolve()
    scene_name = field(config, "scene") or "warehouse"
    scene_cfg = ROOT / "CrowdSim/config/scenes" / f"{scene_name}.yaml"
    scene_map = path_for(field(scene_cfg, "scene_map"))
    print(f"Host: {platform.platform()} | Python: {sys.version.split()[0]} ({sys.executable})")
    probe = subprocess.run(
        ["nvidia-smi", "--query-gpu=name,memory.total,memory.used,driver_version", "--format=csv,noheader"],
        capture_output=True, text=True, check=False,
    ) if shutil.which("nvidia-smi") else None
    print("GPU/driver:", ("DETECTED " + probe.stdout.strip() if probe and probe.returncode == 0 else "nvidia-smi unavailable"))
    isaacsim_root = Path(os.environ.get("ISAAC_SIM_ROOT", Path.home() / "isaacsim/5.1.0"))
    isaaclab_root = Path(os.environ.get("ISAACLAB_ROOT", Path.home() / "IsaacLab"))
    status("Isaac Sim install directory", isaacsim_root / "python.sh", required=False)
    status("Isaac Lab checkout VERSION", isaaclab_root / "VERSION", required=False)
    if (isaaclab_root / "VERSION").is_file():
        print("Isaac Lab checkout version:", (isaaclab_root / "VERSION").read_text().strip())
    for name in ("torch", "torchvision", "isaaclab", "lightning", "wandb", "cv2", "rtree", "smplx", "hydra"):
        try:
            installed = importlib.util.find_spec(name) is not None
        except (ImportError, ValueError):
            installed = False
        print(f"{'OK' if installed else 'MISSING'} Python module {name}")
    if importlib.util.find_spec("torch"):
        import torch
        print(f"Torch: {torch.__version__} | wheel CUDA: {torch.version.cuda} | CUDA available: {torch.cuda.is_available()}")
    print(f"ProtoMotions source: {ROOT / 'protomotions'}")
    status("MaskedMimic checkpoint", path_for(field(config, "checkpoint")))
    checkpoint = path_for(field(config, "checkpoint"))
    status("MaskedMimic resolved config", checkpoint.parent / "resolved_configs_inference.pt" if checkpoint else None)
    status("motion library", path_for(field(config, "motion_file")))
    status("scene USD", path_for(field(scene_cfg, "scene_usd")))
    status("scene map YAML", scene_map)
    map_image = field(scene_map, "image") if scene_map else None
    status("scene map PNG", (scene_map.parent / map_image).resolve() if map_image and scene_map else None)
    car = field(config, "usd")
    if car and car.lower() in {"nova_carter", "novacarter", "nova-carter"}:
        status("local Nova Carter USD (remote fallback exists)", path_for("../Assets/Robots/NovaCarter/nova_carter.usd"), required=False)
    elif car:
        status("robot USD", path_for(car))
    mesh_enabled = (field(config, "human_mesh") or "false").lower() == "true"
    model_dir = path_for(field(config, "smpl_model_dir") or "data/smpl")
    smpl_candidates = [model_dir / "SMPL_NEUTRAL.pkl", model_dir / "smpl/SMPL_NEUTRAL.pkl"] if model_dir else []
    model_file = next((p for p in smpl_candidates if p.exists()), smpl_candidates[0] if smpl_candidates else None)
    status("SMPL neutral visual model (human_mesh=true)", model_file, required=mesh_enabled)
    status("robot PPO checkpoint for inference", path_for(field(config, "policy_checkpoint")), required=False)
    usd_files = list((ROOT / "protomotions/data/assets/usd").rglob("*.usd")) + list((ROOT / "protomotions/data/assets/usd").rglob("*.usda"))
    pointers = [p for p in usd_files if is_lfs_pointer(p)]
    print(f"ProtoMotions USD assets: {len(usd_files)} files, {len(pointers)} LFS pointers")
    status("SMPL humanoid USD", ROOT / "protomotions/data/assets/usd/smpl_humanoid.usda")
    print("This probe checks files and Python packages; it does not launch Isaac Sim or validate loaded geometry.")


if __name__ == "__main__":
    main()
