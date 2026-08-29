"""Open the local Isaac Sim 6.0.1 Simple Warehouse in the running GUI."""

from __future__ import annotations

import asyncio
import os
from pathlib import Path

import carb
import omni.kit.app
import omni.usd


asset_root = Path(os.environ["ISAACSIM_ASSET_ROOT"]).resolve()
scene_path = asset_root / "Isaac" / "Environments" / "Simple_Warehouse" / "full_warehouse.usd"

if not scene_path.is_file():
    raise FileNotFoundError(f"Local warehouse scene is missing: {scene_path}")

context = omni.usd.get_context()
carb.log_info(f"[WAREHOUSE-GUI] Opening local stage: {scene_path}")
context.open_stage(str(scene_path))


async def verify_open_stage() -> None:
    while context.get_stage_loading_status()[2] > 0:
        await omni.kit.app.get_app().next_update_async()

    stage = context.get_stage()
    root_identifier = stage.GetRootLayer().identifier if stage else ""
    if str(scene_path) != root_identifier:
        carb.log_error(
            f"[WAREHOUSE-GUI] Unexpected root layer after load: {root_identifier or '<none>'}"
        )
        return

    carb.log_info(f"[WAREHOUSE-GUI] Stage ready: {root_identifier}")
    print(f"[WAREHOUSE-GUI] Stage ready: {root_identifier}", flush=True)


asyncio.ensure_future(verify_open_stage())
