#!/usr/bin/env bash
set -eo pipefail

source /home/user/arena_isaac5_host_runtime/setup.bash
if [[ -n "${ARENA_NAV2_DIAG_OVERLAY:-}" ]]; then
    source "$ARENA_NAV2_DIAG_OVERLAY/local_setup.bash"
fi
export ISAAC_PATH=/home/user/navigation_project/a_pipeline/scripts/validation/arena_tf_throttle
echo "ARENA_TF_FIX_ISAAC_PATH=$ISAAC_PATH"

export DISPLAY="${DISPLAY:-:1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-173}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/home/user/arena_isaac5_py311_factory/gate8_ros_logs_abi_clean_20260916}"
export ARENA_CHARACTER_ASSETS_ROOT="${ARENA_CHARACTER_ASSETS_ROOT:-/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/assets/people/characters}"
# The Arena character mirror contains model USDs only.  Isaac Sim 5.1's
# pedestrian package also needs the shared Biped setup explicitly.
export ARENA_BIPED_SETUP_PATH="${ARENA_BIPED_SETUP_PATH:-/home/user/arena_isaac5_host_runtime/assets/Biped_Setup.usda}"
export ARENA_SOURCE_WORLDS_ROOT="${ARENA_SOURCE_WORLDS_ROOT:-/home/user/navigation_project/a_pipeline/isaac_sim/arena_ws/src/arena/simulation-setup/worlds}"
mkdir -p "$ROS_LOG_DIR"

arena_world="${ARENA_WORLD:-map_empty}"
arena_human="${ARENA_HUMAN:-isaac}"
if [[ "$arena_human" == hunav ]]; then
    hunav_overlay=/home/user/arena_full_ws/install_harmonic_host_overlay
    export AMENT_PREFIX_PATH="${AMENT_PREFIX_PATH}:$hunav_overlay"
    export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}$hunav_overlay/local/lib/python3.10/dist-packages"
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$hunav_overlay/lib"
fi
arena_tm_obstacles="${ARENA_TM_OBSTACLES:-random}"
arena_tm_robots="${ARENA_TM_ROBOTS:-explore}"
arena_scenario_file="${ARENA_SCENARIO_FILE:-}"
if [[ "$arena_human" == hunav && -n "$arena_scenario_file" ]]; then
    arena_tm_obstacles="${ARENA_TM_OBSTACLES:-scenario}"
    arena_tm_robots="${ARENA_TM_ROBOTS:-scenario}"
fi
if [[ -z "${ARENA_WORLD_USD:-}" && "$arena_world" == "factory" ]]; then
    export ARENA_WORLD_USD="/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/arena_worlds/factory/factory_full_scene.usda"
elif [[ -z "${ARENA_WORLD_USD:-}" && "$arena_world" == "ignc" ]]; then
    export ARENA_WORLD_USD="/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/arena_worlds/ignc/ignc_scene.usda"
elif [[ -z "${ARENA_WORLD_USD:-}" && "$arena_world" == "hospital" ]]; then
    export ARENA_WORLD_USD="/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/arena_worlds/hospital/hospital_full_scene.usda"
fi
echo "ARENA_WORLD_USD=${ARENA_WORLD_USD:-<unset>}"
arena_launch_args=(
  sim:=isaac "human:=${arena_human}" robot:=jackal "world:=${arena_world}"
  "tm_obstacles:=${arena_tm_obstacles}"
  "tm_robots:=${arena_tm_robots}"
  local_planner:=dwb global_planner:=navfn env_n:=1 headless:=0
)

# Scenario is a task-generator ROS parameter.  Leave it untouched for the
# historical random-obstacle default, but make scenario-mode runs explicit and
# reproducible without editing the protected active Arena workspace.
if [[ -z "$arena_scenario_file" ]]; then
  exec ros2 launch arena_bringup arena.launch.py "${arena_launch_args[@]}"
fi

# Forward the scenario into the nested task-generator launch before the node
# is created.  This is the reliable path; the parameter-set loop below is
# retained only as a compatibility marker for older launch overlays.
arena_launch_args+=("scenario_file:=${arena_scenario_file}")

# arena.launch.py does not forward arbitrary dotted launch arguments into the
# task-generator node.  Set the declared ROS parameter as soon as that node is
# discoverable, while it is still waiting for the Isaac services, then retain
# the launch process as the foreground child of this wrapper.
ros2 launch arena_bringup arena.launch.py "${arena_launch_args[@]}" &
arena_launch_pid=$!
arena_scenario_parameter_ready=0
for _arena_attempt in $(seq 1 360); do
  arena_set_result="$(
    ros2 param set /task_generator_node task.scenario.file "$arena_scenario_file" 2>&1 || true
  )"
  if [[ "$arena_set_result" == *"Set parameter successful"* ]]; then
    echo "ARENA_SCENARIO_PARAMETER_READY=$arena_scenario_file"
    arena_scenario_parameter_ready=1
    break
  fi
  if ! kill -0 "$arena_launch_pid" 2>/dev/null; then
    break
  fi
  sleep 0.25
done
if [[ "$arena_scenario_parameter_ready" -ne 1 ]]; then
  echo "ARENA_SCENARIO_PARAMETER_FAILED=$arena_scenario_file" >&2
fi
unset arena_scenario_parameter_ready arena_set_result
wait "$arena_launch_pid"
