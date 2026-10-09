#!/usr/bin/env bash
# One original map_empty/1.json episode, with a read-only action observer.
set -Eeo pipefail

repo=[LOCAL_PATH]
domain="${1:?ROS domain, e.g. 221}"
run_id="${2:?unique run ID}"
[[ "$domain" =~ ^[0-9]+$ ]] && (( 10#$domain <= 232 )) || { echo 'ROS domain must be 0..232' >&2; exit 2; }
[[ "$run_id" =~ ^[A-Za-z0-9_-]+$ ]] || { echo 'Invalid run ID' >&2; exit 2; }
scenario=[LOCAL_PATH]
scenario="$(realpath -e "$scenario")"
run_dir="[LOCAL_PATH]$(date +%Y%m%d)_scenario1_${run_id}"
[[ ! -e "$run_dir" ]] || { echo "Run directory exists: $run_dir" >&2; exit 2; }
mkdir -p "$run_dir/ros_logs"
printf '%s\n' "$scenario" > "$run_dir/scenario_absolute_path.txt"
printf 'command=%q %q %q\n' "$0" "$domain" "$run_id" > "$run_dir/command.txt"

source [LOCAL_PATH]
set -u
unset ARENA_NAV2_DIAG_OVERLAY
export ROS_DOMAIN_ID="$domain" ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1
export ROS_LOG_DIR="$run_dir/ros_logs" DISPLAY=:1
hunav_overlay=[LOCAL_PATH]
export AMENT_PREFIX_PATH="${AMENT_PREFIX_PATH}:$hunav_overlay"
export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}$hunav_overlay/local/lib/python3.10/dist-packages"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$hunav_overlay/lib"
for runtime_var in AMENT_PREFIX_PATH LD_LIBRARY_PATH; do
  [[ "${!runtime_var:-}" != *"[LOCAL_PATH]"* ]] || {
    echo "Diagnostic overlay leaked into $runtime_var" >&2; exit 2;
  }
done

export ARENA_SINGLE_EPISODE=1 ARENA_WORLD=map_empty ARENA_HUMAN=hunav
export ARENA_TM_ROBOTS=scenario ARENA_TM_OBSTACLES=scenario ARENA_SCENARIO_FILE="$scenario"
export ARENA_SCENARIO_EXPECTED_SHA256=f1e55f8a874e5ad3f7ad5f6f4f36e62e113386a1eafae50e7e5c4f4f6f48b718
export ARENA_RUN_DIR="$run_dir"

/usr/bin/python3 "$repo/scripts/validation/arena_dwb_baseline/preflight.py" \
  > "$run_dir/preflight.json" || { echo 'Preflight failed; see preflight.json' >&2; exit 2; }

for pkg in nav2_costmap_2d nav2_controller dwb_core nav2_navfn_planner hunav_msgs; do
  ros2 pkg prefix "$pkg" >> "$run_dir/package_prefixes.txt"
done
if [[ -n "$(timeout 5 ros2 node list)" ]]; then
  echo "Domain $domain already has ROS nodes" >&2
  exit 2
fi

# Record every discovered topic, including hidden action status and custom HuNav types.
bag_pid= launch_pid= capture_pid= libraries_pid= monitor_pid=
cleanup() {
  trap - EXIT INT TERM
  for pid in "$launch_pid" "$bag_pid" "$capture_pid" "$libraries_pid" "$monitor_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then kill -INT -- "-$pid" 2>/dev/null || true; fi
  done
  for _ in $(seq 1 20); do
    if [[ -z "$bag_pid" ]] || ! kill -0 "$bag_pid" 2>/dev/null; then break; fi
    sleep 1
  done
  for pid in "$launch_pid" "$bag_pid" "$capture_pid" "$libraries_pid" "$monitor_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then kill -TERM -- "-$pid" 2>/dev/null || true; fi
  done
  wait "$launch_pid" 2>/dev/null || true
  wait "$bag_pid" 2>/dev/null || true
  wait "$capture_pid" 2>/dev/null || true
  wait "$libraries_pid" 2>/dev/null || true
  wait "$monitor_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

setsid ros2 bag record -a --include-hidden-topics -o "$run_dir/raw_rosbag" > "$run_dir/bag.log" 2>&1 &
bag_pid=$!
setsid "$repo/scripts/validation/arena_tf_throttle/run_gui.bash" > "$run_dir/launch.log" 2>&1 &
launch_pid=$!
setsid /usr/bin/python3 "$repo/scripts/validation/arena_tf_throttle/capture_effective.py" "$run_dir" \
  > "$run_dir/capture.log" 2>&1 &
capture_pid=$!
setsid "$repo/scripts/validation/arena_tf_throttle/capture_libraries.bash" \
  "$run_dir/controller_libraries.txt" "$launch_pid" > "$run_dir/libraries.log" 2>&1 &
libraries_pid=$!
setsid "$repo/scripts/validation/arena_tf_throttle/resource_monitor.bash" \
  > "$run_dir/resources.log" 2>&1 &
monitor_pid=$!
printf 'bag=%s launch=%s capture=%s libraries=%s monitor=%s\n' \
  "$bag_pid" "$launch_pid" "$capture_pid" "$libraries_pid" "$monitor_pid" > "$run_dir/owned_pids.txt"

guard_exit=0
/usr/bin/python3 "$repo/scripts/validation/arena_dwb_baseline/terminal_guard.py" \
  "$run_dir" "$launch_pid" > "$run_dir/guard.log" 2>&1 || guard_exit=$?
printf 'guard_exit=%s\n' "$guard_exit" > "$run_dir/exit_status.txt"
exit "$guard_exit"
