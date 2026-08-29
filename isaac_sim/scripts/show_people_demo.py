#!/usr/bin/env python3
"""Open Isaac Sim's NvBlox sample with the animated-people extensions enabled."""

from __future__ import annotations

import argparse


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--renderer", default="Wireframe")
    return parser.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp


simulation_app = SimulationApp(
    {
        "headless": ARGS.headless,
        "renderer": ARGS.renderer,
        "multi_gpu": False,
        "width": 1280,
        "height": 720,
    }
)

import carb
import omni.timeline
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import is_stage_loading
from isaacsim.storage.native import get_assets_root_path


PEOPLE_EXTENSIONS = [
    "omni.anim.people",
    "omni.anim.navigation.bundle",
    "omni.anim.timeline",
    "omni.anim.graph.bundle",
    "omni.anim.graph.core",
    "omni.anim.retarget.bundle",
    "omni.anim.retarget.core",
    "omni.kit.scripting",
]
if not ARGS.headless:
    PEOPLE_EXTENSIONS.extend(
        [
            "omni.anim.graph.ui",
            "omni.anim.retarget.ui",
        ]
    )

for extension_name in PEOPLE_EXTENSIONS:
    enable_extension(extension_name)
    simulation_app.update()

assets_root = get_assets_root_path()
if not assets_root:
    carb.log_error("Isaac Sim assets root was not found")
    simulation_app.close()
    raise SystemExit(1)

scene_url = assets_root + "/Isaac/Samples/NvBlox/nvblox_sample_scene.usd"
print(f"Opening animated-people sample: {scene_url}")
omni.usd.get_context().open_stage(scene_url)

simulation_app.update()
simulation_app.update()
while is_stage_loading():
    simulation_app.update()

timeline = omni.timeline.get_timeline_interface()
timeline.play()
print("Scene loaded. Close the Isaac Sim window or press Ctrl+C to exit.")

elapsed = 0.0
dt = 1.0 / 60.0
try:
    while simulation_app.is_running():
        simulation_app.update()
        elapsed += dt
        if ARGS.duration > 0.0 and elapsed >= ARGS.duration:
            break
finally:
    timeline.stop()
    simulation_app.close()

