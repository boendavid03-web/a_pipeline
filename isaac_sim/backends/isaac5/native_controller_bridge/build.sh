#!/usr/bin/env bash
set -euo pipefail

bridge_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
isaac_root="${ISAAC_SIM_5_ROOT:-/home/user/isaacsim/5.1.0}"
nav_ext="$(find "$isaac_root/extscache" -maxdepth 1 -type d -name 'omni.anim.navigation.core-*' -print -quit)"

test -n "$nav_ext"
test -f "$isaac_root/kit/python/include/python3.11/Python.h"
test -f "$nav_ext/include/omni/anim/navigation/INavigation.h"
test -f "$nav_ext/bin/libmesh_tools.so"

output="$bridge_dir/_isaac5_native_controller.cpython-311-x86_64-linux-gnu.so"
g++ -std=c++17 -O2 -fPIC -shared \
  "$bridge_dir/native_controller_bridge.cpp" \
  -I"$isaac_root/kit/dev/include" \
  -I"$isaac_root/kit/python/include/python3.11" \
  -I"$isaac_root/exts/omni.isaac.ml_archive/pip_prebundle/torch/include" \
  -I"$nav_ext/include" \
  -L"$isaac_root/kit" \
  -L"$nav_ext/bin" \
  -Wl,-rpath,"$isaac_root/kit" \
  -Wl,-rpath,"$nav_ext/bin" \
  -lcarb -lmesh_tools \
  -o "$output"

printf '%s\n' "$output"
