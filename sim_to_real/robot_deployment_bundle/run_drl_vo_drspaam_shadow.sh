#!/usr/bin/env bash
set -euo pipefail
bundle_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export SIM_TO_REAL_BUNDLE_ROOT="$bundle_root"
set +u
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source "$bundle_root/ros2_ws/install/setup.bash"
set -u
exec ros2 launch semantic_nav_runtime drl_vo_drspaam_shadow.launch.py "$@"
