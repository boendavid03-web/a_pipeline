#!/usr/bin/env bash
set -euo pipefail
for topic in /scan_01 /scan_02 /odom /plan /semantic_cnn/local_subgoal /pedestrian_tracks /sim_to_real/semantic_cnn/cmd_vel_shadow /sim_to_real/drl_vo/cmd_vel_shadow; do
  printf '\n=== %s ===\n' "$topic"; ros2 topic info -v "$topic" || true
done
