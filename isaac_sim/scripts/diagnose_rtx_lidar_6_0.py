#!/usr/bin/env python3
"""Minimal local-asset RTX lidar probe for Isaac Sim 6.0.1."""

from __future__ import annotations

import os
import sys
from pathlib import Path

from isaacsim import SimulationApp


ASSET_ROOT = Path(os.environ["ISAACSIM_ASSET_ROOT"]).resolve()
LIDAR_USD = ASSET_ROOT / "Isaac/Sensors/NVIDIA/Example_Rotary_2D.usda"
simulation_app = SimulationApp(
    {
        "headless": True,
        "renderer": "RaytracedLighting",
        "enable_motion_bvh": True,
        "multi_gpu": False,
        "extra_args": [
            "--/app/runLoops/main/manualModeEnabled=true",
            "--/app/player/useFixedTimeStepping=true",
            "--/rtx/hydra/supportMultiTickRate=true",
            "--/rtx/rendering/perSensorTickTlas=true",
            "--/app/settings/fabricDefaultStageFrameHistoryCount=3",
            "--/telemetry/enableAnonymousData=false",
        ],
    }
)

import carb
import numpy as np
import omni.replicator.core as rep
import omni.timeline
from isaacsim.core.experimental.objects import Cube
from isaacsim.sensors.experimental.rtx import (
    Lidar,
    LidarSensor,
    parse_generic_model_output_data,
)
from omni.replicator.core import Writer


class ProbeWriter(Writer):
    def __init__(self) -> None:
        self.data_structure = "renderProduct"
        self.annotators = [rep.annotators.get("GenericModelOutput")]
        self.callbacks = 0
        self.valid = 0
        self.zeros = 0
        self.last = None

    def write(self, data) -> None:
        self.callbacks += 1
        for render_product in data.get("renderProducts", {}).values():
            raw = render_product.get("GenericModelOutput")
            if isinstance(raw, dict):
                raw = raw.get("data")
            gmo = parse_generic_model_output_data(raw)
            self.last = (
                int(gmo.frameId),
                int(gmo.timestampNs),
                int(gmo.scanComplete),
                int(gmo.numElements),
            )
            if int(gmo.numElements) > 0:
                self.valid += 1
            else:
                self.zeros += 1


def main() -> int:
    timeline = omni.timeline.get_timeline_interface()
    sensor = None
    try:
        Cube("/World/front", positions=np.array([5.0, 0.0, 1.0]), scales=np.array([2.0, 2.0, 2.0]))
        Cube("/World/back", positions=np.array([-5.0, 0.0, 1.0]), scales=np.array([2.0, 2.0, 2.0]))
        Cube("/World/left", positions=np.array([0.0, 5.0, 1.0]), scales=np.array([2.0, 2.0, 2.0]))
        Cube("/World/right", positions=np.array([0.0, -5.0, 1.0]), scales=np.array([2.0, 2.0, 2.0]))
        rep.WriterRegistry.register(ProbeWriter)
        rep.orchestrator.set_capture_on_play(True)
        lidar = Lidar.create(
            "/World/lidar",
            usd_path=str(LIDAR_USD),
            translations=np.array([0.0, 0.0, 1.0]),
            accumulate_outputs=True,
            aux_output_level="FULL",
            tick_rate=10.0,
        )
        sensor = LidarSensor(lidar, annotators=["generic-model-output"])
        writer = sensor.attach_writer("ProbeWriter")
        timeline.play()
        direct_valid = 0
        direct_zeros = 0
        direct_last = None
        for _ in range(240):
            simulation_app.update()
            raw, _info = sensor.get_data("generic-model-output")
            if raw is None:
                continue
            gmo = parse_generic_model_output_data(raw)
            direct_last = (
                int(gmo.frameId),
                int(gmo.timestampNs),
                int(gmo.scanComplete),
                int(gmo.numElements),
            )
            if int(gmo.numElements) > 0:
                direct_valid += 1
            else:
                direct_zeros += 1
        result = {
            "writer_callbacks": writer.callbacks,
            "writer_valid": writer.valid,
            "writer_zeros": writer.zeros,
            "writer_last": writer.last,
            "direct_valid": direct_valid,
            "direct_zeros": direct_zeros,
            "direct_last": direct_last,
            "timeline_time": float(timeline.get_current_time()),
        }
        print("RTX_LIDAR_PROBE=" + repr(result), flush=True)
        return 0 if writer.valid > 0 or direct_valid > 0 else 2
    except Exception as exc:
        carb.log_error(f"RTX lidar probe failed: {exc}")
        print(f"RTX_LIDAR_PROBE_ERROR={exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        timeline.stop()
        simulation_app.update()
        if sensor is not None:
            sensor._invalidate_sensor()
            simulation_app.update()
        simulation_app.close()


raise SystemExit(main())
