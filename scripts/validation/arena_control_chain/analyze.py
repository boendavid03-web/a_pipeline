#!/usr/bin/env python3
"""Summarize the predeclared angular windows of a Jackal control probe."""

import argparse
import json
import statistics
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("probe_json", type=Path)
args = parser.parse_args()
data = json.loads(args.probe_json.read_text())
if data.get("status") != "CAPTURED":
    raise SystemExit(f"Probe is not captured: {data.get('error')}")

output = {"probe_json": str(args.probe_json), "floor_present": data["floor_present"],
          "wheel_order": ["front_left", "front_right", "rear_left", "rear_right"],
          "windows": []}
for index, spec in enumerate(data["windows"]):
    if spec["angular_z"] == 0:
        continue
    samples = [item for item in data["samples"] if item["window"] == index]
    if len(samples) < 2:
        raise SystemExit(f"Too few samples in window {index}")
    first, last = samples[0], samples[-1]
    duration = last["sim_time"] - first["sim_time"]
    delta = [b - a for a, b in zip(first["actual_position_rad"],
                                   last["actual_position_rad"])]
    wheel_model_yaw = 0.098 / 0.54 * ((delta[1] + delta[3]) / 2 -
                                      (delta[0] + delta[2]) / 2)
    output["windows"].append({
        "input_angular_z_rad_s": spec["angular_z"],
        "measured_sim_s": duration,
        "median_target_rad_s": [statistics.median(item["targets_rad_s"][i]
                                                      for item in samples)
                                  for i in range(4)],
        "wheel_displacement_rad": delta,
        "mean_actual_wheel_rad_s": [value / duration for value in delta],
        "no_slip_wheel_model_yaw_rad": wheel_model_yaw,
        "chassis_yaw_delta_rad": last["chassis_yaw"] - first["chassis_yaw"],
        "chassis_z_first_last_m": [first["chassis_xyz"][2],
                                   last["chassis_xyz"][2]],
    })
print(json.dumps(output, indent=2))
