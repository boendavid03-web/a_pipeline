#!/usr/bin/env bash
set -Eeo pipefail

diag_root=/home/user/arena_local_costmap_diag_20261001
kind="${1:?kind: diagnostic or short or default}"
domain="${2:?ROS domain}"
target="${3:?sim seconds}"
case "$kind" in diagnostic|short|default) ;; *) exit 2;; esac
run_id="${4:?unique run ID}"
run_dir="/home/user/arena_tf_throttle_validation_20261001/${run_id}"
if [[ -e "$run_dir" ]]; then
  echo "Run directory already exists: $run_dir" >&2
  exit 2
fi
mkdir -p "$run_dir/ros_logs"

source /home/user/arena_isaac5_host_runtime/setup.bash
if [[ "$kind" == diagnostic ]]; then
  source "$diag_root/install/local_setup.bash"
  export ARENA_NAV2_DIAG_OVERLAY="$diag_root/install"
else
  unset ARENA_NAV2_DIAG_OVERLAY
fi
set -u
export ROS_DOMAIN_ID="$domain" ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1
export ARENA_CLOCK_TARGET="$target"
export ARENA_CLOCK_WALL_LIMIT="${ARENA_CLOCK_WALL_LIMIT:-720}"
hunav_overlay=/home/user/arena_full_ws/install_harmonic_host_overlay
export AMENT_PREFIX_PATH="${AMENT_PREFIX_PATH}:$hunav_overlay"
export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}$hunav_overlay/local/lib/python3.10/dist-packages"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$hunav_overlay/lib"
export ROS_LOG_DIR="$run_dir/ros_logs"
export DISPLAY=:1

/usr/bin/python3 - <<'PY' > "$run_dir/preflight.txt"
from pathlib import Path
import hashlib, subprocess, yaml
m=yaml.safe_load(Path('/home/user/navigation_project/a_pipeline/ARENA_ISAAC_FROZEN_BASELINE_MANIFEST_20261001.yaml').read_text())
checks={
 'scenario':m['scenario_set']['files']['default.json']['live_sha256'],
 'hunav':m['hunav']['arena_bringup_default_behavior']['sha256'],
 'nav2':m['simulation_setup']['live_nav2_sha256'],
 'installed_child':m['current_adapter']['isaac_child_installed_module']['sha256'],
 'adapter_source':m['current_adapter']['factory_source']['sha256'],
}
paths={
 'scenario':Path('/home/user/arena_isaac5_host_overlay_ws/src/arena_simulation_setup/worlds/map_empty/scenarios/default.json'),
 'hunav':Path(m['hunav']['arena_bringup_default_behavior']['path']),
 'nav2':Path(m['simulation_setup']['live_overlay'])/'configs/nav2/nav2.yaml',
 'installed_child':Path(m['current_adapter']['isaac_child_installed_module']['path']),
 'adapter_source':Path(m['current_adapter']['factory_source']['path']),
}
for name,path in paths.items():
    actual=hashlib.sha256(path.read_bytes()).hexdigest()
    assert actual==checks[name],f'{name} hash mismatch: {actual} != {checks[name]}'
    print(name,path,actual)
cfg=yaml.safe_load(paths['nav2'].read_text())['controller_server']['ros__parameters']
assert cfg['controller_frequency']==1.0
assert cfg['progress_checker']['movement_time_allowance']==10.0
print('frequency',cfg['controller_frequency'],'allowance',cfg['progress_checker']['movement_time_allowance'])
head=subprocess.check_output(['git','-C',m['current_adapter']['repo'],'rev-parse','HEAD'],text=True).strip()
assert head==m['current_adapter']['commit']
print('current_adapter_HEAD',head)
PY

prefix="$(ros2 pkg prefix nav2_costmap_2d)"
if [[ "$kind" == diagnostic ]]; then
  expected_prefix="$diag_root/install/nav2_costmap_2d"
else
  expected_prefix=/opt/ros/humble
  for runtime_var in AMENT_PREFIX_PATH LD_LIBRARY_PATH; do
    [[ "${!runtime_var:-}" != *"$diag_root/install"* ]] || {
      echo "Diagnostic overlay leaked into formal $runtime_var" >&2; exit 2;
    }
  done
fi
[[ "$prefix" == "$expected_prefix" ]] || {
  echo "Wrong costmap package prefix: $prefix" >&2; exit 2;
}
printf 'costmap_prefix=%s\n' "$prefix" >> "$run_dir/preflight.txt"
printf 'domain=%s local_only=%s\n' "$ROS_DOMAIN_ID" "$ROS_LOCALHOST_ONLY" >> "$run_dir/preflight.txt"
nvidia-smi --query-gpu=index,name,memory.free --format=csv,noheader >> "$run_dir/preflight.txt"
sha256sum /home/user/arena_tf_throttle_validation_20261001/runtime/site-packages/isaac_utils/graphs/tf.py >> "$run_dir/preflight.txt"
sha256sum /home/user/arena_tf_throttle_validation_20261001/runtime/site-packages/ros2isaacsim/run_isaacsim.py >> "$run_dir/preflight.txt"
sha256sum /home/user/arena_isaac5_py311_factory/build_ws/humble/isaac_sim_ros_ws/install/ros2isaacsim/lib/python3.11/site-packages/isaac_utils/graphs/tf.py >> "$run_dir/preflight.txt"
dpkg-query -W -f='${Package} ${Version}\n' ros-humble-nav2-costmap-2d ros-humble-nav2-controller ros-humble-dwb-core >> "$run_dir/preflight.txt"
printf 'steady_window_sim_s=20..28 (diagnostic predeclared)\n' >> "$run_dir/preflight.txt"
if [[ -n "$(timeout 5 ros2 node list)" ]]; then
  echo "Domain $ROS_DOMAIN_ID already has nodes" >&2
  exit 2
fi

bag_pid= launch_pid= capture_pid= monitor_pid= libraries_pid=
cleanup() {
  trap - EXIT INT TERM
  for pid in "$launch_pid" "$bag_pid" "$capture_pid" "$monitor_pid" "$libraries_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT -- "-$pid" 2>/dev/null || true
    fi
  done
  sleep 3
  for pid in "$launch_pid" "$bag_pid" "$capture_pid" "$monitor_pid" "$libraries_pid"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -TERM -- "-$pid" 2>/dev/null || true
    fi
  done
  sleep 2
  for pid in "$launch_pid" "$bag_pid" "$capture_pid" "$monitor_pid" "$libraries_pid"; do
    if [[ -n "$pid" ]] && ps -eo pgid= | grep -qE "^[[:space:]]*$pid$"; then
      kill -KILL -- "-$pid" 2>/dev/null || true
    fi
  done
  wait "$launch_pid" 2>/dev/null || true
  wait "$bag_pid" 2>/dev/null || true
  wait "$capture_pid" 2>/dev/null || true
  wait "$monitor_pid" 2>/dev/null || true
  wait "$libraries_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

setsid ros2 bag record -a --include-hidden-topics -o "$run_dir/raw_rosbag" \
  > "$run_dir/bag.log" 2>&1 &
bag_pid=$!

scenario=/home/user/arena_isaac5_host_overlay_ws/src/arena_simulation_setup/worlds/map_empty/scenarios/default.json
human=hunav
if [[ "$kind" == short ]]; then
  scenario=/home/user/arena_isaac5_host_overlay_ws/src/arena_simulation_setup/worlds/map_empty/scenarios/nav_short_smoke_debug.json
  human=isaac
fi
sha256sum "$scenario" >> "$run_dir/preflight.txt"
setsid env ARENA_SINGLE_EPISODE=1 ARENA_WORLD=map_empty ARENA_HUMAN="$human" \
  ARENA_TM_ROBOTS=scenario ARENA_TM_OBSTACLES=scenario ARENA_SCENARIO_FILE="$scenario" \
  /home/user/navigation_project/a_pipeline/scripts/validation/arena_tf_throttle/run_gui.bash > "$run_dir/launch.log" 2>&1 &
launch_pid=$!

setsid /usr/bin/python3 "/home/user/navigation_project/a_pipeline/scripts/validation/arena_tf_throttle/capture_effective.py" "$run_dir" \
  > "$run_dir/capture.log" 2>&1 &
capture_pid=$!
setsid /home/user/navigation_project/a_pipeline/scripts/validation/arena_tf_throttle/resource_monitor.bash \
  > "$run_dir/resources.log" 2>&1 &
monitor_pid=$!
setsid /home/user/navigation_project/a_pipeline/scripts/validation/arena_tf_throttle/capture_libraries.bash \
  "$run_dir/controller_libraries.txt" "$launch_pid" > "$run_dir/libraries.log" 2>&1 &
libraries_pid=$!
printf 'bag_pid=%s launch_pid=%s capture_pid=%s domain=%s\n' \
  "$bag_pid" "$launch_pid" "$capture_pid" "$ROS_DOMAIN_ID" > "$run_dir/owned_pids.txt"

guard_exit=0
/usr/bin/python3 "/home/user/navigation_project/a_pipeline/scripts/validation/arena_tf_throttle/clock_guard.py" > "$run_dir/clock_guard.log" 2>&1 || guard_exit=$?
printf 'clock_guard_exit=%s\n' "$guard_exit" > "$run_dir/exit_status.txt"
exit "$guard_exit"
