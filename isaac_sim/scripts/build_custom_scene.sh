#!/usr/bin/env bash
# Rebuild a project-owned Isaac USDA scene from a planar Gazebo static-box world.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
DEFAULT_SOURCE="$PROJECT_ROOT/workspaces/ros2_ws/src/semantic_nav_gazebo/worlds/gazebo_eng_lobby.world"
DEFAULT_OUTPUT="$PROJECT_ROOT/isaac_sim/scenes/a_pipeline_eng_lobby.usda"
SOURCE="${1:-$DEFAULT_SOURCE}"
OUTPUT="${2:-$DEFAULT_OUTPUT}"

arguments=(
    "$SOURCE"
    --output "$OUTPUT"
    --scene-name a_pipeline_eng_lobby
)
if [[ "$SOURCE" == "$DEFAULT_SOURCE" ]]; then
    arguments+=(--expected-boxes 79)
fi

exec python3 "$SCRIPT_DIR/convert_gazebo_boxes_to_usda.py" "${arguments[@]}"
