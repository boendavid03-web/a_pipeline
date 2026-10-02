#!/usr/bin/env python3
"""Extract bounded PPO pilot metrics from raw logs and checkpoint payloads."""
from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path
import re

import torch

ROOT = Path(__file__).resolve().parents[1]
LOG = ROOT.parent / "Assets/repro_logs/ppo_baseline_1p1r"
PILOT = ROOT / "output/crowdsim_robot_ppo_baseline_1p1r_20261001/pilot"
METRIC = re.compile(r"([A-Za-z][A-Za-z_0-9]*)=([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def checkpoint_info(path: Path) -> dict:
    payload = torch.load(path, map_location="cpu", weights_only=False)
    model = payload["model"]
    optimizer = payload["optimizer"]
    finite_model = all(bool(torch.isfinite(value).all()) for value in model.values())
    finite_optimizer = all(
        bool(torch.isfinite(value).all())
        for state in optimizer["state"].values()
        for value in state.values()
        if isinstance(value, torch.Tensor)
    )
    return {
        "path": str(path), "sha256": sha256(path), "bytes": path.stat().st_size,
        "step": int(payload["step"]), "schema_version": int(payload["schema_version"]),
        "model_tensors_finite": finite_model, "optimizer_tensors_finite": finite_optimizer,
        "completed_episodes": payload.get("extra_state", {}).get("goal_curriculum", {}).get("completed_episodes"),
    }


def main() -> None:
    records = []
    nonfinite_lines = []
    for line in (LOG / "training.log").read_text(errors="replace").splitlines():
        if "[CrowdSim][PPO] robot_steps=" not in line:
            continue
        if re.search(r"=[+-]?(?:nan|inf)(?:\b|$)", line, re.IGNORECASE):
            nonfinite_lines.append(line)
        record = {key: float(value) for key, value in METRIC.findall(line)}
        record["robot_steps"] = int(record["robot_steps"])
        records.append(record)
    if records:
        fields = ["robot_steps"] + sorted({key for row in records for key in row if key != "robot_steps"})
        with (LOG / "training_rollouts.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(records)
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        steps = [row["robot_steps"] for row in records]
        fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
        axes[0, 0].plot(
            steps, [row.get("return", float("nan")) for row in records],
            color="tab:blue", label="mean completed return",
        )
        goal_ax = axes[0, 0].twinx()
        goal_ax.plot(
            steps, [row.get("goal_dist", float("nan")) for row in records],
            color="tab:orange", label="mean step goal distance",
        )
        goal_ax.set_ylabel("goal distance (m)")
        axes[0, 0].set_title("Return and goal distance")
        axes[0, 0].legend(loc="lower left")
        goal_ax.legend(loc="upper right")
        for key in ("reached", "collision", "timeout", "stuck"):
            axes[0, 1].plot(steps, [row.get(key, 0) for row in records], label=key)
        axes[0, 1].set_title("Terminal flags per rollout")
        axes[0, 1].legend()
        for key in ("policy_loss", "value_loss"):
            axes[1, 0].plot(steps, [row.get(key, float("nan")) for row in records], label=key)
        axes[1, 0].set_title("PPO losses")
        axes[1, 0].legend()
        for key in ("entropy", "kl", "ev"):
            axes[1, 1].plot(steps, [row.get(key, float("nan")) for row in records], label=key)
        axes[1, 1].set_title("Entropy, approximate KL, explained variance")
        axes[1, 1].legend()
        for ax in axes.flat:
            ax.grid(alpha=0.2)
            ax.set_xlabel("robot steps")
        fig.tight_layout()
        fig.savefig(LOG / "training_curves.png", dpi=160)
        plt.close(fig)
    runs = sorted(path for path in PILOT.iterdir() if path.is_dir() and not path.is_symlink())
    checkpoints = {}
    initial = sorted(
        path for path in (ROOT / "output/crowdsim_robot_ppo_baseline_1p1r_20261001/step0_init").glob("*/robot_ppo_latest.pt")
        if not path.parent.is_symlink()
    )
    if len(initial) == 1:
        checkpoints["0"] = checkpoint_info(initial[0])
    if runs:
        run = runs[-1]
        for path in sorted(run.glob("robot_ppo_*.pt")):
            if path.name == "robot_ppo_latest.pt":
                continue
            info = checkpoint_info(path)
            checkpoints[str(info["step"])] = info
        latest = run / "robot_ppo_latest.pt"
        if latest.exists():
            checkpoints["latest"] = checkpoint_info(latest)
    human_motion = None
    if runs:
        trajectory = runs[-1] / "navigation/trajectory_latest.jsonl"
        if trajectory.exists():
            import numpy as np
            previous = None
            frame_count = 0
            nonteleport_steps = 0
            moving_steps = 0
            nonteleport_path = 0.0
            teleport_steps = 0
            quiet_run = longest_quiet_run = 0
            with trajectory.open() as stream:
                for line in stream:
                    try:
                        frame = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    if frame.get("type") != "frame":
                        continue
                    current = np.asarray(frame["positions_xy"][0], dtype=np.float64)
                    if previous is not None:
                        delta = float(np.linalg.norm(current - previous))
                        if delta >= 0.5:
                            teleport_steps += 1
                            quiet_run = 0
                        else:
                            nonteleport_steps += 1
                            nonteleport_path += delta
                            moving = delta > 0.005
                            moving_steps += int(moving)
                            quiet_run = 0 if moving else quiet_run + 1
                            longest_quiet_run = max(longest_quiet_run, quiet_run)
                    previous = current
                    frame_count += 1
            human_motion = {
                "frames": frame_count, "teleport_steps_excluded": teleport_steps,
                "nonteleport_path_m": nonteleport_path,
                "mean_nonteleport_speed_mps": nonteleport_path * 25 / max(nonteleport_steps, 1),
                "fraction_steps_moving_over_5mm": moving_steps / max(nonteleport_steps, 1),
                "longest_stationary_run_steps_at_5mm": longest_quiet_run,
            }
    evaluations = {}
    for step in (0, 10240, 25088, 49920):
        episodes = []
        for route in ("short_control", "crossing", "head_on", "same_direction"):
            candidates = [LOG / f"eval_{step:05d}_{route}_rerun.json", LOG / f"eval_{step:05d}_{route}.json"]
            path = next((value for value in candidates if value.exists()), None)
            if path is not None:
                episodes.append(json.loads(path.read_text()))
        if episodes:
            evaluations[str(step)] = {
                "episodes": episodes,
                "n": len(episodes),
                "success": sum(bool(e["success"]) for e in episodes),
                "collision": sum(bool(e["collision"]) for e in episodes),
                "timeout": sum(bool(e["timeout"]) for e in episodes),
                "stuck": sum(bool(e["stuck"]) for e in episodes),
                "mean_goal_error_m": sum(e["final_goal_error_m"] for e in episodes) / len(episodes),
                "min_human_distance_m": min(e["min_human_robot_distance_m"] for e in episodes),
            }
    nonfinite = [
        {"step": row["robot_steps"], "field": key, "value": value}
        for row in records for key, value in row.items()
        if isinstance(value, (int, float)) and not math.isfinite(value)
    ]
    summary = {
        "rollouts": len(records), "last_logged_step": records[-1]["robot_steps"] if records else 0,
        "rollout_nonfinite": nonfinite,
        "rollout_nonfinite_raw_lines": nonfinite_lines,
        "terminal_flag_totals": {
            key: int(sum(row.get(key, 0) for row in records))
            for key in ("reached", "collision", "timeout", "stuck")
        },
        "checkpoints": checkpoints,
        "human_motion": human_motion,
        "evaluations": evaluations,
    }
    (LOG / "final_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({key: value for key, value in summary.items() if key != "evaluations"}, indent=2))


if __name__ == "__main__":
    main()
