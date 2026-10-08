#!/usr/bin/env python3
"""Validate and summarize the three immutable CrowdSim evaluation outputs."""

from __future__ import annotations

import json
from math import sqrt
from pathlib import Path


ROOT = Path(__file__).resolve().parent
LABELS = ("step_4180480_100", "step_4940800_100", "final_100")


def wilson(k: int, n: int) -> list[float]:
    z = 1.959963984540054
    p = k / n
    d = 1 + z * z / n
    center = (p + z * z / (2 * n)) / d
    half = z * sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return [round(center - half, 4), round(center + half, 4)]


def main() -> None:
    rows = []
    for label in LABELS:
        folder = ROOT / label
        metric = json.loads((folder / "metrics.json").read_text())
        episodes = [json.loads(line) for line in (folder / "episodes.jsonl").read_text().splitlines()]
        exit_code = int((ROOT / f"{label}_EXIT_CODE.txt").read_text())
        assert exit_code == 0 and metric["status"] == "COMPLETE"
        assert len(episodes) == metric["completed_episodes"] == 100
        assert [episode["episode_index"] for episode in episodes] == list(range(100))
        for field in ("reached", "collision", "timeout", "stuck", "safe_success"):
            assert sum(bool(episode[field]) for episode in episodes) == metric[field]
        assert all(
            episode["safe_success"] == (episode["reached"] and not episode["collision"])
            for episode in episodes
        )
        assert sum(episode["empty_depth_reads"] for episode in episodes) <= metric["empty_depth_reads"]
        rows.append({
            "label": label,
            "checkpoint_step": metric["checkpoint_step"],
            "checkpoint_sha256": metric["checkpoint_sha256"],
            "route_sha256": metric["route_sha256"],
            "map_sha256": metric["map_sha256"],
            "seed": metric["seed"],
            "control_steps": metric["control_steps"],
            "episodes": 100,
            "safe_success": metric["safe_success"],
            "safe_success_wilson95": wilson(metric["safe_success"], 100),
            "reached": metric["reached"],
            "collision": metric["collision"],
            "timeout": metric["timeout"],
            "stuck": metric["stuck"],
            "mean_sim_seconds": round(metric["mean_sim_seconds"], 3),
            "empty_depth_reads": metric["empty_depth_reads"],
            "episodes_with_empty_depth": sum(episode["empty_depth_reads"] > 0 for episode in episodes),
            "failed_episodes_with_empty_depth": sum(
                not episode["safe_success"] and episode["empty_depth_reads"] > 0
                for episode in episodes
            ),
            "terminal_reached_collision_overlap": sum(
                episode["reached"] and episode["collision"] for episode in episodes
            ),
        })

    assert len({row["route_sha256"] for row in rows}) == 1
    assert len({row["map_sha256"] for row in rows}) == 1
    assert len({row["seed"] for row in rows}) == 1
    assert len({row["checkpoint_sha256"] for row in rows}) == 3
    (ROOT / "COMPARISON.json").write_text(json.dumps({
        "validity": "SCREENING_ONLY_DEPTH_COVERAGE_ANOMALIES",
        "notes": [
            "Held-out seed 701 and identical initial routes/map; later reset routes can differ.",
            "Wilson intervals assume independent episodes and may understate dependence across robots.",
            "All-zero processed depth reads are retained; no samples are dropped.",
            "CrowdSim collision flags are not direct PhysX contact measurements.",
        ],
        "results": rows,
    }, indent=2) + "\n")
    print(json.dumps(rows, indent=2))


if __name__ == "__main__":
    main()
