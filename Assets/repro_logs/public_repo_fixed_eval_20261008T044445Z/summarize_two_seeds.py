#!/usr/bin/env python3
"""Check both held-out seeds and aggregate the 3x2 checkpoint screen."""

from __future__ import annotations

import json
from pathlib import Path

from summarize import wilson


ROOT = Path(__file__).resolve().parent
PAIRS = (
    (4180480, "step_4180480_100", "step_4180480_seed702_100"),
    (4940800, "step_4940800_100", "step_4940800_seed702_100"),
    (5002240, "final_100", "final_seed702_100"),
)


def load(label: str) -> dict:
    folder = ROOT / label
    metric = json.loads((folder / "metrics.json").read_text())
    episodes = [json.loads(line) for line in (folder / "episodes.jsonl").read_text().splitlines()]
    assert int((ROOT / f"{label}_EXIT_CODE.txt").read_text()) == 0
    assert metric["status"] == "COMPLETE"
    assert metric["completed_episodes"] == len(episodes) == 100
    assert [row["episode_index"] for row in episodes] == list(range(100))
    for key in ("reached", "collision", "timeout", "stuck", "safe_success"):
        assert metric[key] == sum(bool(row[key]) for row in episodes)
    assert all(row["safe_success"] == (row["reached"] and not row["collision"]) for row in episodes)
    return {"label": label, "metrics": metric, "episodes": episodes}


def main() -> None:
    out = []
    all_runs = {}
    for step, label701, label702 in PAIRS:
        runs = [load(label701), load(label702)]
        for seed, run in zip((701, 702), runs):
            assert run["metrics"]["seed"] == seed
            assert run["metrics"]["checkpoint_step"] == step
            all_runs[(step, seed)] = run
        metrics = [run["metrics"] for run in runs]
        safe = sum(m["safe_success"] for m in metrics)
        rows = [r for run in runs for r in run["episodes"]]
        out.append({
            "checkpoint_step": step,
            "checkpoint_sha256": metrics[0]["checkpoint_sha256"],
            "seed701_safe_success": metrics[0]["safe_success"],
            "seed702_safe_success": metrics[1]["safe_success"],
            "episodes": len(rows),
            "safe_success": safe,
            "safe_success_wilson95_independence_assumption": wilson(safe, len(rows)),
            "collision": sum(m["collision"] for m in metrics),
            "timeout": sum(m["timeout"] for m in metrics),
            "stuck": sum(m["stuck"] for m in metrics),
            "empty_depth_reads": sum(m["empty_depth_reads"] for m in metrics),
            "episodes_with_empty_depth": sum(r["empty_depth_reads"] > 0 for r in rows),
            "failed_episodes_with_empty_depth": sum(
                r["empty_depth_reads"] > 0 and not r["safe_success"] for r in rows
            ),
        })
    for seed in (701, 702):
        runs = [all_runs[(step, seed)]["metrics"] for step, *_ in PAIRS]
        assert len({m["route_sha256"] for m in runs}) == 1
        assert len({m["map_sha256"] for m in runs}) == 1
    assert len({all_runs[(4180480, seed)]["metrics"]["route_sha256"] for seed in (701, 702)}) == 2
    assert len({run["metrics"]["map_sha256"] for run in all_runs.values()}) == 1
    result = {
        "status": "TWO_SEED_WAREHOUSE_SCREEN_COMPLETE_WITH_DEPTH_COVERAGE_LIMIT",
        "selection_rule": "Largest safe-success count across two held-out seeds is the provisional demo candidate, not a proven global optimum.",
        "notes": [
            "Routes match within a seed only at initialization; policy-dependent resets can diverge.",
            "Wilson bounds treat 200 episodes as independent and may understate within-robot/seed dependence.",
            "All-zero processed depth reads are included in every numerator and denominator.",
            "Collision flags are CrowdSim detector/drive-guard outputs, not measured PhysX contact.",
            "Both seeds use the same warehouse, privileged local map, and simulator neighbor state.",
        ],
        "results": out,
    }
    (ROOT / "COMPARISON_TWO_SEEDS.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
