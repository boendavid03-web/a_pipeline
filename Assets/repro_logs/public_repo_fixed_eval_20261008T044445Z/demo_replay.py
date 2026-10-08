#!/usr/bin/env python3
"""Run the verified 4180480-step GUI demo into a fresh output directory."""

from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path
import sys

sys.dont_write_bytecode = True


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"OUTPUT_ALREADY_EXISTS: {output}")
    original = Path(__file__).with_name("demo_4180480.py")
    spec = importlib.util.spec_from_file_location("verified_demo_4180480", original)
    if spec is None or spec.loader is None:
        raise RuntimeError("DEMO_MODULE_UNAVAILABLE")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.OUTPUT = output
    module.main()


if __name__ == "__main__":
    main()
