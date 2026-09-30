#!/usr/bin/env python3
"""Convert one Arena mesh asset to a standalone USD with Isaac Sim 5.1.

This is intentionally an offline helper.  It does not open a ROS graph or
modify a running stage; the generated USD can then be referenced by the Arena
Isaac runtime.
"""

from __future__ import annotations

import argparse
import asyncio
from pathlib import Path

from isaacsim import SimulationApp


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_mesh", type=Path)
    parser.add_argument("output_usd", type=Path)
    parser.add_argument(
        "--ignore-materials",
        action="store_true",
        help="Drop source materials/textures; useful only for a geometry smoke test.",
    )
    return parser


async def _convert(input_mesh: Path, output_usd: Path, ignore_materials: bool) -> bool:
    import omni.kit.asset_converter

    context = omni.kit.asset_converter.AssetConverterContext()
    context.ignore_materials = ignore_materials
    context.use_meter_as_world_unit = True
    converter = omni.kit.asset_converter.get_instance()
    task = converter.create_converter_task(
        str(input_mesh),
        str(output_usd),
        lambda _progress, _steps: None,
        context,
    )
    while not await task.wait_until_finished():
        await asyncio.sleep(0.1)
    return output_usd.is_file() and output_usd.stat().st_size > 0


def main() -> int:
    args = _parser().parse_args()
    input_mesh = args.input_mesh.resolve()
    output_usd = args.output_usd.resolve()
    if not input_mesh.is_file():
        raise FileNotFoundError(input_mesh)
    output_usd.parent.mkdir(parents=True, exist_ok=True)

    app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})
    try:
        # The extension is loaded after SimulationApp, matching NVIDIA's
        # converter example and avoiding import-time Kit conflicts.
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("omni.kit.asset_converter")
        app.update()
        ok = asyncio.get_event_loop().run_until_complete(
            _convert(input_mesh, output_usd, args.ignore_materials)
        )
        print(f"ARENA_MESH_CONVERSION {'PASS' if ok else 'FAIL'}")
        print(f"ARENA_MESH_INPUT={input_mesh}")
        print(f"ARENA_MESH_OUTPUT={output_usd}")
        return 0 if ok else 1
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
