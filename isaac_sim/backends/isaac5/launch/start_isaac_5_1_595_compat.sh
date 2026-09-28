#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ISAAC_SIM_5_ROOT="${ISAAC_SIM_5_ROOT:-/home/user/isaacsim/5.1.0}"
CACHE_BASE="${XDG_CACHE_HOME:-${HOME}/.cache}/vulkan-rtx-compat"

if [[ ! -f "${ISAAC_SIM_5_ROOT}/VERSION" ]] || ! grep -q '^5\.1\.0' "${ISAAC_SIM_5_ROOT}/VERSION"; then
    echo "ISAAC_SIM_5_ROOT is not an Isaac Sim 5.1.0 installation: ${ISAAC_SIM_5_ROOT}" >&2
    exit 2
fi
if [[ ! -x "${ISAAC_SIM_5_ROOT}/isaac-sim.sh" ]]; then
    echo "Isaac Sim 5.1 launcher not found: ${ISAAC_SIM_5_ROOT}/isaac-sim.sh" >&2
    exit 2
fi

unset ISAAC_SIM_ROOT ISAAC_SIM_PATH ISAAC_PATH ISAACSIM_PATH ISAAC_ASSET_ROOT ISAAC_ASSETS_ROOT \
    ISAACSIM_ASSET_ROOT OMNI_KIT_ROOT OMNI_KIT_ALLOW_ROOT CARB_APP_PATH EXP_PATH \
    OMNI_KIT_APP OMNI_KIT_EXTENSIONS_PATH OMNI_KIT_EXT_PATH OMNI_KIT_CACHE_DIR \
    OMNI_USER_PATH PXR_PLUGINPATH_NAME PYTHONHOME PYTHONPATH LD_PRELOAD VK_LAYER_PATH || true
export PYTHONNOUSERSITE=1

python3 "${BACKEND_ROOT}/compat/vulkan_595/isaacsim_rtx_595_compat.py"

export RTX_VULKAN_COMPAT_ENABLE_LAYER=1
unset RTX_VULKAN_COMPAT_DISABLE_LAYER || true
export VK_KHRONOS_PROFILES_PROFILE_NAME=VP_RTX_driver_compat_generated
export VK_KHRONOS_PROFILES_SIMULATE_CAPABILITIES=SIMULATE_PROPERTIES_BIT
export VK_KHRONOS_PROFILES_DEBUG_REPORTS=DEBUG_REPORT_ERROR_BIT
export VK_KHRONOS_PROFILES_PROFILE_DIRS="${CACHE_BASE}/profiles${VK_KHRONOS_PROFILES_PROFILE_DIRS:+:${VK_KHRONOS_PROFILES_PROFILE_DIRS}}"
export VK_ADD_IMPLICIT_LAYER_PATH="${CACHE_BASE}/xdg-data/vulkan/implicit_layer.d${VK_ADD_IMPLICIT_LAYER_PATH:+:${VK_ADD_IMPLICIT_LAYER_PATH}}"
export XDG_DATA_DIRS="${CACHE_BASE}/xdg-data${XDG_DATA_DIRS:+:${XDG_DATA_DIRS}}"
export RTX_VULKAN_COMPAT_ACTIVE=1
export RTX_VULKAN_COMPAT_MAX_MEMORY_ALLOCATION_SIZE_ACTIVE=4292870144

python3 "${BACKEND_ROOT}/compat/vulkan_595/isaacsim_rtx_595_compat.py" --verify

echo "Starting Isaac Sim 5.1 with the process-local RTX 595 compatibility profile"
exec "${ISAAC_SIM_5_ROOT}/isaac-sim.sh" "$@"
