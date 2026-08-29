#!/usr/bin/env python3
"""Run a standalone Python launch file without installing another ROS package."""

from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path

from launch import LaunchService


def parse_args() -> tuple[Path, list[str]]:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("launch_file", type=Path)
    args, launch_arguments = parser.parse_known_args()
    return args.launch_file.expanduser().resolve(), launch_arguments


def main() -> int:
    launch_file, launch_arguments = parse_args()
    if not launch_file.is_file():
        raise FileNotFoundError(launch_file)
    spec = importlib.util.spec_from_file_location("a_pipeline_level3_launch", launch_file)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import launch file: {launch_file}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    launch_description = module.generate_launch_description()
    service = LaunchService(argv=launch_arguments, noninteractive=False)
    service.include_launch_description(launch_description)
    return int(service.run())


if __name__ == "__main__":
    raise SystemExit(main())

