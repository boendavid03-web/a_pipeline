#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_ROOT="$(cd "$BACKEND_ROOT/../../.." && pwd)"
ISAAC_SIM_5_ROOT="${ISAAC_SIM_5_ROOT:-/home/user/isaacsim/5.1.0}"
ACTIVE_SCENE_USD="$PROJECT_ROOT/isaac_sim/scenes/a_pipeline_eng_lobby.usda"

if [[ ! -f "$ISAAC_SIM_5_ROOT/VERSION" ]] || ! grep -q '^5\.1\.0' "$ISAAC_SIM_5_ROOT/VERSION"; then
    echo "ERROR: ISAAC_SIM_5_ROOT is not Isaac Sim 5.1.0: $ISAAC_SIM_5_ROOT" >&2
    exit 1
fi
if [[ ! -x "$ISAAC_SIM_5_ROOT/python.sh" ]]; then
    echo "ERROR: Isaac Sim 5.1 python.sh not found: $ISAAC_SIM_5_ROOT/python.sh" >&2
    exit 1
fi
if [[ ! -f "$ACTIVE_SCENE_USD" ]]; then
    echo "ERROR: active project scene not found: $ACTIVE_SCENE_USD" >&2
    exit 1
fi

unset ROS_DISTRO AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH COLCON_CURRENT_PREFIX \
    RMW_IMPLEMENTATION ISAAC_SIM_ROOT ISAAC_SIM_PATH ISAAC_PATH ISAACSIM_PATH \
    ISAAC_ASSET_ROOT ISAAC_ASSETS_ROOT ISAACSIM_ASSET_ROOT OMNI_KIT_ROOT \
    OMNI_KIT_ALLOW_ROOT CARB_APP_PATH EXP_PATH OMNI_KIT_APP \
    OMNI_KIT_EXTENSIONS_PATH OMNI_KIT_EXT_PATH OMNI_KIT_CACHE_DIR OMNI_USER_PATH \
    PXR_PLUGINPATH_NAME PYTHONHOME PYTHONPATH LD_PRELOAD VK_LAYER_PATH || true
export PYTHONNOUSERSITE=1

if ! "$ISAAC_SIM_5_ROOT/python.sh" -c 'import sys; assert sys.version_info[:2] == (3, 11)' >/dev/null; then
    echo "ERROR: Isaac Sim 5.1 embedded Python is not 3.11" >&2
    exit 1
fi

STATUS_CAPTURE="$(mktemp -p "${TMPDIR:-/tmp}" isaac5-validate-scene.XXXXXX.log)"
cleanup_status_capture() { rm -f -- "$STATUS_CAPTURE"; }
trap cleanup_status_capture EXIT

set +e
"$ISAAC_SIM_5_ROOT/python.sh" "$BACKEND_ROOT/runtime/validate_scene.py" "$@" 2>&1 \
    | tee "$STATUS_CAPTURE"
PYTHON_STATUS="${PIPESTATUS[0]}"
set -e
if grep -F 'SCENE_VALIDATION_RESULT={"status": "FAIL"' "$STATUS_CAPTURE" >/dev/null; then
    exit 2
fi
exit "$PYTHON_STATUS"
