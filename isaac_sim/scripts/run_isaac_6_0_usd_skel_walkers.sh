#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_SIM_ROOT="${ISAAC_SIM_6_ROOT:-/home/user/navigation_project/a_pipeline/isaac_sim/isaacsim-6.0.1}"
ASSET_ROOT="${ISAACSIM_ASSET_ROOT:-/home/user/navigation_project/a_pipeline/isaac_sim/assets-6.0.1/Assets/Isaac/6.0}"
PYTHON_SCRIPT="$SCRIPT_DIR/show_usd_skel_walkers_6_0.py"
LOG_DIR="$SCRIPT_DIR/logs"

if [[ ! -x "$ISAAC_SIM_ROOT/python.sh" ]]; then
    echo "ERROR: Isaac Sim 6.0.1 python.sh not found or not executable: $ISAAC_SIM_ROOT/python.sh" >&2
    exit 1
fi

required_assets=(
    "$ASSET_ROOT/Isaac/Samples/BehaviorTree/IRA_OBT_Sample_Warehouse.usd"
    "$ASSET_ROOT/Isaac/People/MotionLibrary/BuiltinActions/MoveWalk/WalkForward.usd"
    "$ASSET_ROOT/Isaac/People/Characters/F_Business_02/F_Business_02.usd"
    "$ASSET_ROOT/Isaac/People/Characters/male_adult_police_04/male_adult_police_04.usd"
    "$ASSET_ROOT/Isaac/People/Characters/male_adult_construction_05_new/male_adult_construction_05_new.usd"
)
for asset in "${required_assets[@]}"; do
    if [[ ! -f "$asset" ]]; then
        echo "ERROR: required local asset is missing: $asset" >&2
        exit 1
    fi
done

mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/usd_skel_walkers_6_0_$(date +%Y%m%d_%H%M%S).log"
export ISAACSIM_ASSET_ROOT="$ASSET_ROOT"
export OMNI_KIT_DISABLE_TELEMETRY=1

echo "Isaac Sim: $ISAAC_SIM_ROOT"
echo "Local assets: $ISAACSIM_ASSET_ROOT"
echo "Run log: $LOG_FILE"
set +e
"$ISAAC_SIM_ROOT/python.sh" "$PYTHON_SCRIPT" "$@" 2>&1 | tee "$LOG_FILE"
status=${PIPESTATUS[0]}
set -e

if rg -i 'segmentation fault|fatal signal|libomni\.anim\.skelJoint|motion_matching::|crash detected' "$LOG_FILE" >/dev/null 2>&1; then
    echo "WARNING: a possible native animation crash signature was found in $LOG_FILE" >&2
    exit 2
fi
exit "$status"
