#!/usr/bin/env bash
# Read-only deployment readiness checks. It never publishes any ROS message.
set -euo pipefail
bundle_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
method=${1:-semantic}
if [[ "$method" != semantic && "$method" != drlvo ]]; then printf 'Usage: %s [semantic|drlvo]\n' "$0" >&2; exit 2; fi
ros_setup="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [[ ! -r "$ros_setup" ]]; then
  printf 'ERROR: ROS 2 setup not found: %s\n' "$ros_setup" >&2
  exit 2
fi
set +u
# shellcheck disable=SC1090
source "$ros_setup"
if [[ -r "$bundle_root/ros2_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$bundle_root/ros2_ws/install/setup.bash"
else
  printf 'WARN: workspace is not built: %s\n' "$bundle_root/ros2_ws/install/setup.bash" >&2
fi
set -u
printf 'ROS_DISTRO=%s\nPython=%s\n' "${ROS_DISTRO:-unset}" "$(python3 --version)"
status=0
python3 - <<'PY' || status=1
try:
 import torch; print(f'PyTorch={torch.__version__} CUDA={torch.cuda.is_available()}')
except Exception as e:
 print(f'FAIL: PyTorch import failed: {e}')
 raise SystemExit(1)
PY
if (cd "$bundle_root" && sha256sum -c SHA256SUMS); then
  echo 'PASS bundle integrity'
else
  echo 'FAIL bundle integrity'
  status=1
fi

scan_01_topic=${SCAN_01_TOPIC:-/scan_01}
scan_02_topic=${SCAN_02_TOPIC:-/scan_02}
odom_topic=${ODOM_TOPIC:-/odom}
path_topic=${PATH_TOPIC:-/plan}
goal_topic=${GOAL_TOPIC:-/goal_pose}
map_frame=${MAP_FRAME:-map}
odom_frame=${ODOM_FRAME:-odom}
base_frame=${BASE_FRAME:-base_link}

for topic in "$scan_01_topic" "$scan_02_topic" "$odom_topic" "$path_topic" "$goal_topic"; do
  if ros2 topic info "$topic" >/dev/null 2>&1; then
    echo "PASS topic $topic"
  else
    echo "FAIL missing topic $topic"
    status=1
  fi
done
for target_frame in "$map_frame" "$odom_frame"; do
  if timeout 3 ros2 run tf2_ros tf2_echo "$target_frame" "$base_frame" --once >/dev/null 2>&1; then
    echo "PASS TF $target_frame <- $base_frame"
  else
    echo "FAIL TF $target_frame <- $base_frame"
    status=1
  fi
done
if [[ "$method" == semantic ]]; then
  ros2 topic info /semantic_cnn/local_subgoal >/dev/null 2>&1 && echo 'PASS local-subgoal topic exists' || echo 'WARN local-subgoal not currently advertised'
  ros2 topic info /sim_to_real/semantic_cnn/cmd_vel_shadow >/dev/null 2>&1 && echo 'PASS shadow command topic exists' || echo 'WARN shadow command not currently advertised'
else
  ros2 topic info /pedestrian_tracks >/dev/null 2>&1 && echo 'PASS pedestrian tracks topic exists' || echo 'WARN pedestrian tracks not currently advertised'
  ros2 topic info /sim_to_real/drl_vo/cmd_vel_shadow >/dev/null 2>&1 && echo 'PASS shadow command topic exists' || echo 'WARN shadow command not currently advertised'
fi
if ros2 topic info /cmd_vel 2>/dev/null | grep -Eq 'Publisher count: [1-9]'; then echo 'WARN /cmd_vel has publishers; inspect them before testing.'; else echo 'PASS this preflight did not find an active /cmd_vel publisher.'; fi
exit "$status"
