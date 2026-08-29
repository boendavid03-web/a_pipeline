#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_SIM_ROOT="${ISAAC_SIM_6_ROOT:-/home/user/navigation_project/a_pipeline/isaac_sim/isaacsim-6.0.1}"
ASSET_ROOT="${ISAACSIM_ASSET_ROOT:-/home/user/navigation_project/a_pipeline/isaac_sim/assets-6.0.1/Assets/Isaac/6.0}"
OPEN_SCRIPT="$SCRIPT_DIR/open_warehouse_scene_6_0.py"
SCENE="$ASSET_ROOT/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"

if [[ ! -x "$ISAAC_SIM_ROOT/isaac-sim.sh" ]]; then
    echo "ERROR: Isaac Sim 6.0.1 launcher is missing: $ISAAC_SIM_ROOT/isaac-sim.sh" >&2
    exit 1
fi
if [[ ! -f "$SCENE" ]]; then
    echo "ERROR: local warehouse scene is missing: $SCENE" >&2
    exit 1
fi

export ISAACSIM_ASSET_ROOT="$ASSET_ROOT"
export OMNI_KIT_DISABLE_TELEMETRY=1

echo "Opening Isaac Sim 6.0.1 GUI with: $SCENE"
exec "$ISAAC_SIM_ROOT/isaac-sim.sh" \
    --exec "$OPEN_SCRIPT" \
    --/app/file/ignoreUnsavedStage=1 \
    --/telemetry/enableAnonymousData=false \
    --/privacy/usage=false
