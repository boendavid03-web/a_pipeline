#!/usr/bin/env python3
"""Probe the installed Isaac Lab/Sim shutdown path with staged diagnostics."""

from __future__ import annotations

import argparse
import faulthandler
import inspect
import os
from pathlib import Path
import sys
import time

from isaaclab.app import AppLauncher


def _source(label: str, value: object) -> None:
    print(f"[source] {label}: {inspect.getfile(value)}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=2)
    parser.add_argument("--traceback-after", type=float, default=30.0)
    args = parser.parse_args()

    faulthandler.enable()
    faulthandler.dump_traceback_later(args.traceback_after, repeat=True)

    launcher = AppLauncher({"headless": True, "device": "cuda:0"})
    simulation_app = launcher.app

    from isaaclab.sim import SimulationCfg, SimulationContext
    from isaacsim import SimulationApp

    isaaclab_root = Path(os.environ.get("ISAAC_LAB_ROOT", Path(sys.prefix).parent))
    isaacsim_root = os.environ.get("ISAAC_SIM_ROOT") or os.environ.get("ISAAC_PATH")
    isaaclab_version_file = isaaclab_root / "VERSION"
    isaacsim_version_file = Path(isaacsim_root) / "VERSION" if isaacsim_root else None
    isaaclab_version = (
        isaaclab_version_file.read_text().strip()
        if isaaclab_version_file.is_file() else "unknown"
    )
    isaacsim_version = (
        isaacsim_version_file.read_text().strip()
        if isaacsim_version_file and isaacsim_version_file.is_file() else "unknown"
    )
    print(f"[source] Isaac Lab version: {isaaclab_version}", flush=True)
    print(f"[source] Isaac Sim version: {isaacsim_version}", flush=True)
    _source("SimulationContext", SimulationContext)
    _source(
        "SimulationContext._app_control_on_stop_handle_fn",
        SimulationContext._app_control_on_stop_handle_fn,
    )
    _source("SimulationContext.clear_all_callbacks", SimulationContext.clear_all_callbacks)
    _source("SimulationContext.clear_instance", SimulationContext.clear_instance)
    _source("SimulationApp", SimulationApp)
    _source("SimulationApp.close", SimulationApp.close)

    sim = SimulationContext(SimulationCfg(device="cuda:0"))
    print("[probe] SimulationContext created", flush=True)
    sim.reset()
    print("[probe] reset done", flush=True)
    for step in range(args.steps):
        sim.step(render=False)
        print(f"[probe] step {step + 1}/{args.steps}", flush=True)

    shutdown_started = time.monotonic()
    print("[shutdown] begin", flush=True)
    print("[shutdown] disable stop callback", flush=True)
    sim._disable_app_control_on_stop_handle = True
    print("[shutdown] clear callbacks", flush=True)
    sim.clear_all_callbacks()
    print("[shutdown] clear SimulationContext", flush=True)
    SimulationContext.clear_instance()
    print("[shutdown] close SimulationApp begin", flush=True)
    simulation_app.close(wait_for_replicator=False, skip_cleanup=False)
    print("[shutdown] close SimulationApp done", flush=True)
    print(
        f"[shutdown] duration_seconds={time.monotonic() - shutdown_started:.3f}",
        flush=True,
    )
    faulthandler.cancel_dump_traceback_later()
    print("[shutdown] python exiting", flush=True)


if __name__ == "__main__":
    main()
