#!/usr/bin/env bash
set -euo pipefail
base_frame=${1:-base_link}
for source_frame in "${@:2}"; do
  printf '\n=== %s -> %s ===\n' "$source_frame" "$base_frame"
  ros2 run tf2_ros tf2_echo "$base_frame" "$source_frame" --once || true
done
