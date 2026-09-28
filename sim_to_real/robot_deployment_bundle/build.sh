#!/usr/bin/env bash
set -euo pipefail
bundle_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
ros_distro=${ROS_DISTRO:-humble}
ros_setup="/opt/ros/${ros_distro}/setup.bash"
if [[ ! -r "$ros_setup" ]]; then
  printf 'ERROR: ROS 2 %s was not found at %s. Install/source ROS 2 Humble first.\n' "$ros_distro" "$ros_setup" >&2; exit 2
fi
# shellcheck disable=SC1090
set +u
source "$ros_setup"
set -u
command -v colcon >/dev/null || { printf 'ERROR: colcon is missing; install python3-colcon-common-extensions.\n' >&2; exit 2; }
python3 -m pip install --user --no-deps "$bundle_root/third_party/dr_spaam"
if command -v rosdep >/dev/null; then
  printf 'INFO: rosdep is available. If dependencies are missing, run:\n  rosdep install --from-paths ros2_ws/src --ignore-src -r -y\n'
else
  printf 'WARN: rosdep not found. Install rosdep and resolve package.xml dependencies before building.\n' >&2
fi
cd "$bundle_root/ros2_ws"
colcon build --symlink-install
printf 'PASS: build completed. Source %s/ros2_ws/install/setup.bash before direct ros2 commands.\n' "$bundle_root"
