#!/usr/bin/env bash
# Select this shim only for an explicitly launched Arena TF validation run.
set -eo pipefail
export ROS_DISTRO=humble
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONNOUSERSITE=1
export ISAAC_ASSETS_ROOT=https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1
set +u
source /home/user/arena_isaac5_py311_factory/build_ws/humble/humble_ws/install/local_setup.bash
source /home/user/arena_isaac5_py311_factory/build_ws/humble/isaac_sim_ros_ws/install/local_setup.bash
set -u
isolated=/home/user/arena_tf_throttle_validation_20261001/runtime/site-packages
[[ -f "$isolated/isaac_utils/graphs/tf.py" ]] || { echo "isolated tf.py missing" >&2; exit 2; }
export PYTHONPATH="$isolated${PYTHONPATH:+:$PYTHONPATH}"
echo "ARENA_TF_ISOLATED_PYTHONPATH=$isolated" >&2
exec /home/user/isaacsim/5.1.0/python.sh "$@"
