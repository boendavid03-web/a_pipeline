#!/usr/bin/env python3
"""Audit the Isaac5-owned people USD dependency closure in a headless Kit app."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


BACKEND_ROOT = Path(__file__).resolve().parents[1]
ASSET_ROOT = BACKEND_ROOT / "assets/people"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "renderer": "RaytracedLighting", "fast_shutdown": True})
    try:
        from pxr import UsdUtils

        inputs = [
            ASSET_ROOT / "retarget/biped_demo_meters.usd",
            ASSET_ROOT / "animations/stand_walk_loop_in_place.skelanim.usd",
            *sorted((ASSET_ROOT / "characters").glob("*/*.usd")),
        ]
        rows = []
        for path in inputs:
            layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(path.resolve()))
            rows.append(
                {
                    "path": str(path),
                    "layers": sorted(str(value.identifier) for value in layers),
                    "assets": sorted(str(value) for value in assets),
                    "unresolved": sorted(str(value) for value in unresolved),
                }
            )
        result = {
            "status": "PASS" if all(not row["unresolved"] for row in rows) else "FAIL",
            "asset_root": str(ASSET_ROOT),
            "inputs": rows,
        }
    except Exception as exc:
        result = {"status": "FAIL", "asset_root": str(ASSET_ROOT), "error": repr(exc)}
    encoded = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output is not None:
        output = args.output.resolve()
        generated = (BACKEND_ROOT / "generated").resolve()
        if generated not in output.parents:
            raise ValueError(f"output must be below {generated}: {output}")
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(encoded + "\n", encoding="utf-8")
    print("ISAAC5_PEOPLE_ASSET_AUDIT=" + json.dumps(result, ensure_ascii=False), flush=True)
    exit_code = 0 if result["status"] == "PASS" else 2
    app.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
