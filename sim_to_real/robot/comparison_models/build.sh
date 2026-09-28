#!/usr/bin/env bash
set -euo pipefail

model_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
ros_setup="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

if [[ ! -r "$ros_setup" ]]; then
  printf 'ERROR: ROS setup not found: %s\n' "$ros_setup" >&2
  exit 2
fi
command -v colcon >/dev/null || {
  printf 'ERROR: colcon is not installed.\n' >&2
  exit 2
}

set +u
# shellcheck disable=SC1090
source "$ros_setup"
set -u

cd "$model_root/ros2_ws"
colcon build --symlink-install
printf 'PASS: comparison model ROS workspace built.\n'
