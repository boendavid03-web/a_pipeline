#!/usr/bin/env bash
# Select exactly one shadow-only navigation method for an A/B hardware experiment.
set -euo pipefail

bundle_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
method=${1:-}
if [[ $# -gt 0 ]]; then shift; fi

case "$method" in
  semantic_cnn|semantic|a)
    exec "$bundle_root/run_semantic_cnn_shadow.sh" "$@"
    ;;
  drl_vo|drlvo|drl_vo_drspaam|b)
    exec "$bundle_root/run_drl_vo_drspaam_shadow.sh" "$@"
    ;;
  *)
    cat >&2 <<'USAGE'
Usage: ./run_shadow_experiment.sh {semantic_cnn|drl_vo} [ROS launch arguments...]

Runs exactly one method in shadow mode:
  semantic_cnn  dual LiDAR -> online S3-Net -> SemanticCNN
  drl_vo        dual LiDAR -> merger -> DR-SPAAM -> tracker -> DRL-VO

Examples:
  ./run_shadow_experiment.sh semantic_cnn base_frame:=base_link
  ./run_shadow_experiment.sh drl_vo scan_01_topic:=/lidar_1/scan
USAGE
    exit 2
    ;;
esac
