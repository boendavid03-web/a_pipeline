#!/usr/bin/env bash
"true" ':' # Keep shellcheck from treating the next re-exec branch as unreachable.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ARENA_WS="${ARENA_WS:-/home/user/arena_full_ws}"
if [[ "${1:-}" != "--owned-session" ]]; then
    exec setsid --wait "$0" --owned-session "$@"
fi
shift

source "$SCRIPT_DIR/hunav_host_env.sh"
run_stamp="$(date +%Y%m%d_%H%M%S)"
run_dir="$BACKEND_ROOT/generated/hunav_host_$run_stamp"
overlay="$run_dir/host_ament_overlay"
mkdir -p "$overlay/share/ament_index/resource_index/packages" "$overlay/share/hunav_agent_manager/behavior_trees"
: > "$overlay/share/ament_index/resource_index/packages/hunav_agent_manager"
cp -a "$ARENA_WS/src/deps/hunav/hunav_sim/hunav_agent_manager/behavior_trees/." \
    "$overlay/share/hunav_agent_manager/behavior_trees/"
export AMENT_PREFIX_PATH="$overlay:$AMENT_PREFIX_PATH"

manager="${HUNAV_MANAGER_EXECUTABLE:-$ARENA_WS/build_isaac_goal_sync/hunav_agent_manager/hunav_agent_manager}"
[[ -x "$manager" ]] || { echo "ERROR: HuNav manager missing: $manager" >&2; exit 1; }
printf '%s\n' "$$" > "$run_dir/pgid"
echo "HUNAV_HOST_RUN_DIR=$run_dir"
echo "HUNAV_HOST_PGID=$$"
echo "STOP_COMMAND=kill -TERM -- -$$"

children=()
cleanup() {
    trap - EXIT INT TERM
    for pid in "${children[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
    wait "${children[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

"$manager" --ros-args -r __ns:=/isaac_hunav \
    -p publish_tf:=false -p publish_sfm_forces:=false \
    -p hunav_loader.publish_people:=false \
    -p external_goals_authoritative:=true >"$run_dir/manager.log" 2>&1 &
children+=("$!")
sleep 1

if [[ "${1:-}" == "--probe" ]]; then
    shift
    python3 "$BACKEND_ROOT/runtime/hunav_service_probe.py" "$@" | tee "$run_dir/service_matrix.log"
else
    python3 "$BACKEND_ROOT/runtime/hunav_isaac_adapter.py" "$@" 2>&1 | tee "$run_dir/adapter.log"
fi
