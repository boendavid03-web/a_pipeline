#!/usr/bin/env bash
# Launch the isolated Isaac Sim 6.0.1 warehouse + optional IRA people + robot runner.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_SIM_ROOT="${ISAAC_SIM_6_ROOT:-/home/user/navigation_project/a_pipeline/isaac_sim/isaacsim-6.0.1}"
ASSET_ROOT="${ISAACSIM_ASSET_ROOT:-/home/user/navigation_project/a_pipeline/isaac_sim/assets-6.0.1/Assets/Isaac/6.0}"
ROBOT_USD="/home/user/navigation_project/robot_related/robots/chassis_arm/motion_wheel_arm_simple_sphere_usd/mecanum730_xms5_default.usd"
PYTHON_SCRIPT="$SCRIPT_DIR/show_warehouse_people_robot_6_0.py"
ROS_RELAY_SCRIPT="$SCRIPT_DIR/cmd_vel_udp_relay.py"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
DEFAULT_CUSTOM_SCENE_USD="$PROJECT_ROOT/isaac_sim/scenes/a_pipeline_eng_lobby.usda"
CUSTOM_IRA_TEMPLATE="$SCRIPT_DIR/ira_people_demo/custom_eng_lobby_people.yaml"
CUSTOM_ROUTE_GENERATOR="$SCRIPT_DIR/generate_free_space_people_config.py"
CUSTOM_ROUTE_VALIDATOR="$SCRIPT_DIR/validate_custom_people_routes.py"
CUSTOM_GAZEBO_WORLD="$PROJECT_ROOT/workspaces/ros2_ws/src/semantic_nav_gazebo/worlds/gazebo_eng_lobby.world"
CUSTOM_GAZEBO_SCENARIO="$PROJECT_ROOT/workspaces/ros2_ws/src/semantic_nav_gazebo/scenarios/lobby/eng_hall_15.xml"
DEFAULT_CUSTOM_FREE_SPACE_MAP="$PROJECT_ROOT/workspaces/ros2_ws/src/semantic_nav_gazebo/maps/gazebo_eng_lobby/gazebo_eng_lobby.yaml"
ROS_WS="$PROJECT_ROOT/workspaces/ros2_ws"
SCAN_MERGER_CONFIG="$ROS_WS/src/semantic_nav_gazebo/config/v7_dual_laser_scan_merger.yaml"
SCAN_MERGER_EXECUTABLE="$ROS_WS/install/semantic_nav_gazebo/lib/semantic_nav_gazebo/v7_dual_laser_scan_merger.py"
LOG_DIR="$SCRIPT_DIR/logs"
LOCK_FILE="${XDG_RUNTIME_DIR:-/tmp}/warehouse_people_robot_6_0.${UID}.lock"
KERNEL_FATAL_PATTERN='NVRM: Xid|BUG: soft lockup|rcu: INFO:.*stall|GPU has fallen off the bus'
KERNEL_REFERENCE_WARNING_PATTERN='NVRM: GPU[0-9]+ refcntRequestReference_IMPL: Failed to enter state'

# Reject a malformed custom sensor rate before acquiring the runtime lock,
# probing the GPU, touching ROS, or inspecting large asset trees.
export ISAAC_LIDAR_RATE_HZ="${ISAAC_LIDAR_RATE_HZ:-15}"
if [[ ! "$ISAAC_LIDAR_RATE_HZ" =~ ^[0-9]+$ ]] \
    || (( ISAAC_LIDAR_RATE_HZ < 1 || ISAAC_LIDAR_RATE_HZ > 30 )); then
    echo "ERROR: ISAAC_LIDAR_RATE_HZ must be an integer from 1 through 30." >&2
    exit 2
fi
export ISAAC_LIDAR_SAMPLE_COUNT="${ISAAC_LIDAR_SAMPLE_COUNT:-2000}"
if [[ ! "$ISAAC_LIDAR_SAMPLE_COUNT" =~ ^[0-9]+$ ]] \
    || (( ISAAC_LIDAR_SAMPLE_COUNT < 90 || ISAAC_LIDAR_SAMPLE_COUNT > 4096 )); then
    echo "ERROR: ISAAC_LIDAR_SAMPLE_COUNT must be an integer from 90 through 4096." >&2
    exit 2
fi
export ISAAC_RTX_LIDAR_PROFILE="${ISAAC_RTX_LIDAR_PROFILE:-rplidar_s2e}"
case "$ISAAC_RTX_LIDAR_PROFILE" in
    example_dense|navigation_2d_32k|rplidar_s2e) ;;
    *)
        echo "ERROR: ISAAC_RTX_LIDAR_PROFILE must be example_dense, navigation_2d_32k, or rplidar_s2e." >&2
        exit 2
        ;;
esac

# The user may launch this from a desktop terminal whose PATH does not include
# ripgrep.  Kernel monitoring and single-instance protection must not depend
# on an optional developer tool.
if command -v rg >/dev/null 2>&1; then
    match_text() { rg "$@"; }
else
    match_text() { grep -E "$@"; }
fi

# Make the single-instance check atomic.  The ps check below is still useful
# for explaining which process is already running, but two launchers can both
# pass it before either Kit process becomes visible.
if ! command -v flock >/dev/null 2>&1; then
    echo "ERROR: flock is required for the Isaac single-instance safety lock." >&2
    exit 1
fi
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
    echo "ERROR: another warehouse/robot launcher is starting or running." >&2
    echo "Lock: $LOCK_FILE" >&2
    exit 5
fi

if [[ ! -x "$ISAAC_SIM_ROOT/python.sh" ]]; then
    echo "ERROR: Isaac Sim 6.0.1 python.sh not found: $ISAAC_SIM_ROOT/python.sh" >&2
    exit 1
fi
CALLER_LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
# One /cmd_vel publisher can otherwise drive both the pre-existing 5.1 scene
# and this 6.0 scene.  Refuse to start a second Kit process rather than
# competing for the GPU or silently moving a robot in an already-open GUI.
RUNNING_ISAAC="$(ps -eo pid=,args= | match_text '/home/user/[i]saacsim/|isaacsim-6\.0\.1/(kit/)?python/bin/python3' || true)"
if [[ -n "$RUNNING_ISAAC" ]]; then
    echo "ERROR: an Isaac/Kit simulation process is already running; this launcher will not start a second scene:" >&2
    echo "$RUNNING_ISAAC" >&2
    echo "Close that Isaac Sim instance and its old teleop publisher before launching this isolated 6.0.1 scene." >&2
    exit 5
fi
# A crashed NVIDIA/Kit process can remain as a zombie while still appearing
# in nvidia-smi.  Starting another Vulkan/PhysX instance in that kernel state
# is unsafe and has already produced a kernel general-protection fault on this
# workstation.  Require a reboot instead of stacking another Isaac process.
if command -v nvidia-smi >/dev/null 2>&1; then
    if ! timeout 5s nvidia-smi -L >/dev/null 2>&1; then
        echo "ERROR: nvidia-smi did not respond within 5 seconds." >&2
        echo "Do not start Isaac Sim in this GPU-driver state; reboot Ubuntu first." >&2
        exit 6
    fi
    while IFS= read -r gpu_pid; do
        [[ "$gpu_pid" =~ ^[0-9]+$ ]] || continue
        gpu_state="$(ps -o stat= -p "$gpu_pid" 2>/dev/null | tr -d '[:space:]')"
        if [[ "$gpu_state" == Z* ]]; then
            echo "ERROR: GPU process PID $gpu_pid is a zombie after a kernel/NVIDIA fault." >&2
            echo "Reboot Ubuntu before starting Isaac Sim again; a normal process kill cannot recover it." >&2
            exit 6
        fi
    done < <(nvidia-smi --query-compute-apps=pid --format=csv,noheader,nounits 2>/dev/null || true)
fi

# A process-only check is not enough after a fatal driver or kernel fault: the
# bad process may be gone while the loaded NVIDIA module is still poisoned.
# An isolated refcnt status 0x56 also occurs during otherwise successful Kit
# startup on this RTX 5090/595.84 host, so report it but require an Xid, kernel
# fault/stall, or GPU-bus loss before refusing the run.
CURRENT_BOOT_FAULTS="$(
    journalctl -k -b 0 --no-pager -o short-iso 2>/dev/null \
        | match_text -i "$KERNEL_FATAL_PATTERN" \
        | tail -n 20 \
        || true
)"
if [[ -n "$CURRENT_BOOT_FAULTS" ]]; then
    echo "ERROR: this boot already contains a fatal GPU/kernel warning:" >&2
    echo "$CURRENT_BOOT_FAULTS" >&2
    echo "Refusing to start another Isaac process. Resolve the driver/firmware issue and reboot first." >&2
    exit 6
fi
CURRENT_BOOT_REFERENCE_WARNINGS="$(
    journalctl -k -b 0 --no-pager -o short-iso 2>/dev/null \
        | match_text -i "$KERNEL_REFERENCE_WARNING_PATTERN" \
        | tail -n 3 \
        || true
)"
if [[ -n "$CURRENT_BOOT_REFERENCE_WARNINGS" ]]; then
    echo "WARNING: isolated NVIDIA reference warning(s) exist in this boot; continuing because no fatal GPU/kernel signature followed:" >&2
    echo "$CURRENT_BOOT_REFERENCE_WARNINGS" >&2
fi

# Keep enough host RAM for GNOME, ROS 2, and the kernel.  This scene normally
# consumes roughly 10 GiB RSS while starting Vulkan, RTX, and PhysX.
MEM_AVAILABLE_KIB="$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo)"
MIN_AVAILABLE_KIB=$((16 * 1024 * 1024))
if [[ ! "$MEM_AVAILABLE_KIB" =~ ^[0-9]+$ ]] || (( MEM_AVAILABLE_KIB < MIN_AVAILABLE_KIB )); then
    echo "ERROR: at least 16 GiB of available system memory is required before launching this scene." >&2
    free -h >&2 || true
    exit 7
fi
export ISAAC_SCENE="${ISAAC_SCENE:-warehouse}"
export ISAAC_SCENE="${ISAAC_SCENE,,}"
case "$ISAAC_SCENE" in
    warehouse|simple_room|hospital|digital_twin_warehouse|custom) ;;
    *)
        echo "ERROR: ISAAC_SCENE must be warehouse, simple_room, hospital, digital_twin_warehouse, or custom." >&2
        exit 2
        ;;
esac
# Warehouse and the default project-owned engineering lobby both have an
# authored NavMesh plus a matching offline IRA patrol configuration.  An
# arbitrary custom override cannot safely reuse the lobby's coordinates.
CUSTOM_SCENE_USD="$(realpath -m -- "${ISAAC_CUSTOM_SCENE_USD:-$DEFAULT_CUSTOM_SCENE_USD}")"
if [[ "$ISAAC_SCENE" == "warehouse" \
    || ( "$ISAAC_SCENE" == "custom" && "$CUSTOM_SCENE_USD" == "$DEFAULT_CUSTOM_SCENE_USD" ) ]]; then
    export ISAAC_ENABLE_PEOPLE="${ISAAC_ENABLE_PEOPLE:-1}"
else
    export ISAAC_ENABLE_PEOPLE="${ISAAC_ENABLE_PEOPLE:-0}"
fi
case "${ISAAC_ENABLE_PEOPLE,,}" in
    1|true|yes|on) ISAAC_ENABLE_PEOPLE="1" ;;
    0|false|no|off) ISAAC_ENABLE_PEOPLE="0" ;;
    *) echo "ERROR: ISAAC_ENABLE_PEOPLE must be a boolean value." >&2; exit 2 ;;
esac

case "$ISAAC_SCENE" in
    warehouse) SCENE_USD="$ASSET_ROOT/Isaac/Samples/BehaviorTree/IRA_OBT_Sample_Warehouse.usd" ;;
    simple_room) SCENE_USD="$ASSET_ROOT/Isaac/Environments/Simple_Room/simple_room.usd" ;;
    hospital) SCENE_USD="$ASSET_ROOT/Isaac/Environments/Hospital/hospital.usd" ;;
    digital_twin_warehouse) SCENE_USD="$ASSET_ROOT/Isaac/Environments/Digital_Twin_Warehouse/small_warehouse_digital_twin.usd" ;;
    custom) SCENE_USD="$CUSTOM_SCENE_USD" ;;
esac
export ISAAC_CUSTOM_SCENE_USD="$SCENE_USD"
if [[ "$ISAAC_SCENE" != "warehouse" && "$ISAAC_SCENE" != "custom" && "$ISAAC_ENABLE_PEOPLE" == "1" ]]; then
    echo "ERROR: ISAAC_ENABLE_PEOPLE=1 is supported only with ISAAC_SCENE=warehouse or custom." >&2
    exit 2
fi
if [[ "$ISAAC_SCENE" == "custom" && "$SCENE_USD" != "$DEFAULT_CUSTOM_SCENE_USD" \
    && "$ISAAC_ENABLE_PEOPLE" == "1" ]]; then
    echo "ERROR: the bundled IRA routes match only $DEFAULT_CUSTOM_SCENE_USD." >&2
    echo "Set ISAAC_ENABLE_PEOPLE=0 for another USD, or author a matching people/NavMesh configuration." >&2
    exit 2
fi

if [[ "$ISAAC_SCENE" == "custom" ]]; then
    export ISAAC_PEDESTRIAN_COUNT="${ISAAC_PEDESTRIAN_COUNT:--1}"
    export ISAAC_PEDESTRIAN_SEED="${ISAAC_PEDESTRIAN_SEED:-7}"
    export ISAAC_PEDESTRIAN_SPEED="${ISAAC_PEDESTRIAN_SPEED:-1.0}"
    if [[ ! "$ISAAC_PEDESTRIAN_COUNT" =~ ^-?[0-9]+$ ]] \
        || (( ISAAC_PEDESTRIAN_COUNT < -1 || ISAAC_PEDESTRIAN_COUNT > 50 )); then
        echo "ERROR: ISAAC_PEDESTRIAN_COUNT must be -1 or an integer from 0 through 50." >&2
        exit 2
    fi
    if [[ ! "$ISAAC_PEDESTRIAN_SEED" =~ ^[0-9]+$ ]] \
        || (( ISAAC_PEDESTRIAN_SEED > 4294967295 )); then
        echo "ERROR: ISAAC_PEDESTRIAN_SEED must be an integer from 0 through 4294967295." >&2
        exit 2
    fi
    if ! awk -v value="$ISAAC_PEDESTRIAN_SPEED" \
        'BEGIN { exit !(value ~ /^[0-9]+([.][0-9]+)?$/ && value > 0) }'; then
        echo "ERROR: ISAAC_PEDESTRIAN_SPEED must be positive." >&2
        exit 2
    fi
    if (( ISAAC_PEDESTRIAN_COUNT == 0 )); then
        ISAAC_ENABLE_PEOPLE="0"
        export ISAAC_PEDESTRIAN_AVOIDANCE_MODE="off"
    fi
    export ISAAC_ENABLE_PEOPLE
    if (( ISAAC_PEDESTRIAN_COUNT == -1 )); then
        export ISAAC_EXPECTED_PEDESTRIAN_COUNT=15
    else
        export ISAAC_EXPECTED_PEDESTRIAN_COUNT="$ISAAC_PEDESTRIAN_COUNT"
    fi
fi

REQUIRED_ASSETS=("$SCENE_USD" "$ROBOT_USD")
if [[ "$ISAAC_ENABLE_PEOPLE" == "1" ]]; then
    REQUIRED_ASSETS+=(
        "$ASSET_ROOT/Isaac/People/Characters"
        "$ASSET_ROOT/Isaac/People/MotionLibrary/HumanMotionLibrary.usd"
        "$ASSET_ROOT/Isaac/People/MotionLibrary/BuiltinActions/MoveWalk/WalkForward.usd"
    )
fi
for required in "${REQUIRED_ASSETS[@]}"; do
    if [[ ! -e "$required" ]]; then
        echo "ERROR: required local asset is missing: $required" >&2
        exit 1
    fi
done

mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/${ISAAC_SCENE}_people_robot_6_0_$(date +%Y%m%d_%H%M%S).log"

# IRA characters are kinematic: PhysX collision geometry does not stop a
# patrol from crossing a wall.  For the project custom scene, make the saved
# SLAM free cells authoritative and generate this invocation's patrol from
# them.  Unknown and occupied cells are both excluded by the generator.
if [[ "$ISAAC_SCENE" == "custom" && "$ISAAC_ENABLE_PEOPLE" == "1" ]]; then
    export ISAAC_PEDESTRIAN_FREE_SPACE_MAP_YAML="${ISAAC_PEDESTRIAN_FREE_SPACE_MAP_YAML:-$DEFAULT_CUSTOM_FREE_SPACE_MAP}"
    # Route clearance rejects patrols near obstacles.  The smaller runtime
    # guard only detects true sustained incursions and never resets an agent.
    export ISAAC_PEDESTRIAN_FREE_SPACE_CLEARANCE_M="${ISAAC_PEDESTRIAN_FREE_SPACE_CLEARANCE_M:-0.55}"
    export ISAAC_PEDESTRIAN_FREE_SPACE_GUARD_CLEARANCE_M="${ISAAC_PEDESTRIAN_FREE_SPACE_GUARD_CLEARANCE_M:-0.20}"
    export ISAAC_PEDESTRIAN_SPAWN_CLEARANCE_M="${ISAAC_PEDESTRIAN_SPAWN_CLEARANCE_M:-1.0}"
    export ISAAC_PEDESTRIAN_MIN_PATROL_SEGMENT_M="${ISAAC_PEDESTRIAN_MIN_PATROL_SEGMENT_M:-0.5}"
    if [[ ! -f "$ISAAC_PEDESTRIAN_FREE_SPACE_MAP_YAML" ]]; then
        echo "ERROR: SLAM free-space map is missing: $ISAAC_PEDESTRIAN_FREE_SPACE_MAP_YAML" >&2
        exit 1
    fi
    if ! awk -v value="$ISAAC_PEDESTRIAN_FREE_SPACE_CLEARANCE_M" \
        'BEGIN { exit !(value ~ /^[0-9]+([.][0-9]+)?$/ && value > 0) }'; then
        echo "ERROR: ISAAC_PEDESTRIAN_FREE_SPACE_CLEARANCE_M must be positive." >&2
        exit 2
    fi
    if ! awk -v guard="$ISAAC_PEDESTRIAN_FREE_SPACE_GUARD_CLEARANCE_M" \
        -v route="$ISAAC_PEDESTRIAN_FREE_SPACE_CLEARANCE_M" \
        'BEGIN { exit !(guard ~ /^[0-9]+([.][0-9]+)?$/ && guard > 0 && guard <= route) }'; then
        echo "ERROR: ISAAC_PEDESTRIAN_FREE_SPACE_GUARD_CLEARANCE_M must be positive and no greater than ISAAC_PEDESTRIAN_FREE_SPACE_CLEARANCE_M." >&2
        exit 2
    fi
    if ! awk -v value="$ISAAC_PEDESTRIAN_SPAWN_CLEARANCE_M" \
        'BEGIN { exit !(value ~ /^[0-9]+([.][0-9]+)?$/ && value > 0) }'; then
        echo "ERROR: ISAAC_PEDESTRIAN_SPAWN_CLEARANCE_M must be positive." >&2
        exit 2
    fi
    if ! awk -v value="$ISAAC_PEDESTRIAN_MIN_PATROL_SEGMENT_M" \
        'BEGIN { exit !(value ~ /^[0-9]+([.][0-9]+)?$/ && value > 0) }'; then
        echo "ERROR: ISAAC_PEDESTRIAN_MIN_PATROL_SEGMENT_M must be positive." >&2
        exit 2
    fi
    for required in "$CUSTOM_IRA_TEMPLATE" "$CUSTOM_ROUTE_GENERATOR" "$CUSTOM_ROUTE_VALIDATOR" "$CUSTOM_GAZEBO_WORLD" "$CUSTOM_GAZEBO_SCENARIO"; do
        if [[ ! -f "$required" ]]; then
            echo "ERROR: custom pedestrian free-space input is missing: $required" >&2
            exit 1
        fi
    done
    generated_config="$LOG_DIR/custom_people_slam_free_space.yaml"
    /usr/bin/python3 "$CUSTOM_ROUTE_GENERATOR" \
        --map-yaml "$ISAAC_PEDESTRIAN_FREE_SPACE_MAP_YAML" \
        --template "$CUSTOM_IRA_TEMPLATE" \
        --output "$generated_config" \
        --clearance "$ISAAC_PEDESTRIAN_FREE_SPACE_CLEARANCE_M" \
        --spawn-clearance "$ISAAC_PEDESTRIAN_SPAWN_CLEARANCE_M" \
        --min-patrol-segment "$ISAAC_PEDESTRIAN_MIN_PATROL_SEGMENT_M" \
        --world "$CUSTOM_GAZEBO_WORLD" \
        --scenario "$CUSTOM_GAZEBO_SCENARIO" \
        --pedestrian-count "$ISAAC_PEDESTRIAN_COUNT" \
        --seed "$ISAAC_PEDESTRIAN_SEED" \
        --speed "$ISAAC_PEDESTRIAN_SPEED"
    /usr/bin/python3 "$CUSTOM_ROUTE_VALIDATOR" \
        --config "$generated_config" \
        --world "$CUSTOM_GAZEBO_WORLD" \
        --clearance "$ISAAC_PEDESTRIAN_FREE_SPACE_CLEARANCE_M" \
        --min-start-separation "$ISAAC_PEDESTRIAN_SPAWN_CLEARANCE_M"
    export ISAAC_CUSTOM_IRA_CONFIG="$generated_config"
    echo "ISAAC_SLAM_FREE_SPACE_PATROL map=$ISAAC_PEDESTRIAN_FREE_SPACE_MAP_YAML config=$generated_config"
fi
# Isaac Sim 6.0.1 embeds Python 3.12, while Ubuntu 22.04 Humble's rclpy is
# built for Python 3.10.  Keep ROS entirely in external system-Python
# processes and exchange only localhost UDP with Kit.
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
export ROS_DOMAIN_ID="${ISAAC_ROS_DOMAIN_ID:-0}"
# Isaac and keyboard teleop run on this workstation.  Keep DDS discovery on
# loopback so UFW cannot block Fast DDS multicast on the Wi-Fi interface.
export ROS_LOCALHOST_ONLY="1"
export ISAACSIM_ASSET_ROOT="$ASSET_ROOT"
export OMNI_KIT_DISABLE_TELEMETRY=1
export ISAAC_LAUNCHER_SHA256="$(sha256sum "${BASH_SOURCE[0]}" | awk '{print $1}')"
export ISAAC_CMD_VEL_UDP_PORT="${ISAAC_CMD_VEL_UDP_PORT:-15973}"
export ISAAC_TELEMETRY_UDP_PORT="${ISAAC_TELEMETRY_UDP_PORT:-15974}"
export ISAAC_RESET_UDP_PORT="${ISAAC_RESET_UDP_PORT:-15975}"
export ISAAC_ROBOT_COLLISION_PROTECTION="${ISAAC_ROBOT_COLLISION_PROTECTION:-1}"
if [[ -z "${ISAAC_ROBOT_PHYSICS+x}" ]]; then
    [[ "$ISAAC_SCENE" == "custom" ]] && ISAAC_ROBOT_PHYSICS=1 || ISAAC_ROBOT_PHYSICS=0
fi
case "${ISAAC_ROBOT_PHYSICS,,}" in
    1|true|yes|on) ISAAC_ROBOT_PHYSICS=1 ;;
    0|false|no|off) ISAAC_ROBOT_PHYSICS=0 ;;
    *) echo "ERROR: ISAAC_ROBOT_PHYSICS must be a boolean value." >&2; exit 2 ;;
esac
export ISAAC_ROBOT_PHYSICS
legacy_pedestrian_dodge_was_set=0
if [[ -n "${ISAAC_PEDESTRIAN_DODGE+x}" ]]; then
    legacy_pedestrian_dodge_was_set=1
fi
export ISAAC_PEDESTRIAN_DODGE="${ISAAC_PEDESTRIAN_DODGE:-0}"
# The four-state mode supersedes the historical boolean.  If callers only set
# ISAAC_PEDESTRIAN_DODGE, preserve its exact off/legacy behavior.  New runs
# with people default to native crowd avoidance plus the validated gentle
# robot dodge.  Explicit native remains available for A/B diagnosis.
if [[ -z "${ISAAC_PEDESTRIAN_AVOIDANCE_MODE+x}" ]]; then
    if (( legacy_pedestrian_dodge_was_set )); then
        case "${ISAAC_PEDESTRIAN_DODGE,,}" in
            1|true|yes|on) ISAAC_PEDESTRIAN_AVOIDANCE_MODE="legacy_dodge" ;;
            0|false|no|off) ISAAC_PEDESTRIAN_AVOIDANCE_MODE="off" ;;
            *) echo "ERROR: ISAAC_PEDESTRIAN_DODGE must be a boolean value." >&2; exit 2 ;;
        esac
    elif [[ "$ISAAC_ENABLE_PEOPLE" == "1" ]]; then
        ISAAC_PEDESTRIAN_AVOIDANCE_MODE="gentle"
    else
        ISAAC_PEDESTRIAN_AVOIDANCE_MODE="off"
    fi
fi
export ISAAC_PEDESTRIAN_AVOIDANCE_MODE="${ISAAC_PEDESTRIAN_AVOIDANCE_MODE,,}"
case "${ISAAC_PEDESTRIAN_AVOIDANCE_MODE}" in
    off|native|gentle|legacy_dodge) ;;
    *)
        echo "ERROR: ISAAC_PEDESTRIAN_AVOIDANCE_MODE must be off, native, gentle, or legacy_dodge." >&2
        exit 2
        ;;
esac
export ISAAC_PEDESTRIAN_SOCIAL_MODE="${ISAAC_PEDESTRIAN_SOCIAL_MODE:-legacy}"
export ISAAC_PEDESTRIAN_SOCIAL_MODE="${ISAAC_PEDESTRIAN_SOCIAL_MODE,,}"
case "$ISAAC_PEDESTRIAN_SOCIAL_MODE" in
    legacy|gazebo_social) ;;
    *)
        echo "ERROR: ISAAC_PEDESTRIAN_SOCIAL_MODE must be legacy or gazebo_social." >&2
        exit 2
        ;;
esac
if [[ "$ISAAC_ENABLE_PEOPLE" == "0" && "$ISAAC_PEDESTRIAN_AVOIDANCE_MODE" != "off" ]]; then
    echo "ERROR: pedestrian avoidance must be off when ISAAC_ENABLE_PEOPLE=0." >&2
    exit 2
fi
export ISAAC_LIDAR_MODE="${ISAAC_LIDAR_MODE:-rtx}"
case "${ISAAC_LIDAR_MODE}" in
    rtx|physx) ;;
    *) echo "ERROR: ISAAC_LIDAR_MODE must be rtx or physx." >&2; exit 2 ;;
esac

USE_ROS=1
for argument in "$@"; do
    if [[ "$argument" == "--no-ros" ]]; then
        USE_ROS=0
        break
    fi
done

relay_pid=""
merger_pid=""
stop_background_process() {
    local label="$1"
    local pid="$2"
    local attempt
    [[ "$pid" =~ ^[0-9]+$ ]] || return 0
    if kill -0 "$pid" 2>/dev/null; then
        kill -TERM "$pid" 2>/dev/null || true
        for attempt in {1..20}; do
            kill -0 "$pid" 2>/dev/null || break
            sleep 0.1
        done
    fi
    if kill -0 "$pid" 2>/dev/null; then
        echo "WARNING: $label PID $pid ignored SIGTERM; sending SIGKILL." >&2
        kill -KILL "$pid" 2>/dev/null || true
    fi
    wait "$pid" 2>/dev/null || true
}

cleanup_relay() {
    stop_background_process "dual-scan merger" "$merger_pid"
    merger_pid=""
    stop_background_process "ROS/UDP relay" "$relay_pid"
    relay_pid=""
}
trap cleanup_relay EXIT INT TERM

if (( USE_ROS )); then
    if [[ ! -f /opt/ros/humble/setup.bash ]]; then
        echo "ERROR: ROS 2 Humble setup is missing: /opt/ros/humble/setup.bash" >&2
        exit 1
    fi
    if [[ ! -f "$ROS_WS/install/setup.bash" ]]; then
        echo "ERROR: ROS 2 workspace is not built: $ROS_WS/install/setup.bash" >&2
        exit 1
    fi
    if [[ ! -f "$SCAN_MERGER_CONFIG" ]]; then
        echo "ERROR: dual-scan merger config is missing: $SCAN_MERGER_CONFIG" >&2
        exit 1
    fi
    if [[ ! -x "$SCAN_MERGER_EXECUTABLE" ]]; then
        echo "ERROR: built dual-scan merger executable is missing: $SCAN_MERGER_EXECUTABLE" >&2
        exit 1
    fi
    (
        set +u
        export LD_LIBRARY_PATH="$CALLER_LD_LIBRARY_PATH"
        # shellcheck disable=SC1091
        source /opt/ros/humble/setup.bash
        # shellcheck disable=SC1091
        source "$ROS_WS/install/setup.bash"
        set -u
        # setup.bash resets ROS_LOCALHOST_ONLY to 0; force the relay back onto
        # the exact DDS context advertised by this launcher.
        export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
        export ROS_DOMAIN_ID="${ISAAC_ROS_DOMAIN_ID:-0}"
        export ROS_LOCALHOST_ONLY="1"
        export ISAAC_CMD_VEL_UDP_PORT
        export ISAAC_TELEMETRY_UDP_PORT
        export ISAAC_RESET_UDP_PORT
        exec /usr/bin/python3 "$ROS_RELAY_SCRIPT"
    ) &
    relay_pid=$!
    sleep 0.5
    if ! kill -0 "$relay_pid" 2>/dev/null; then
        echo "ERROR: bidirectional Isaac ROS/UDP bridge failed to start." >&2
        wait "$relay_pid" || true
        exit 1
    fi
    (
        set +u
        export LD_LIBRARY_PATH="$CALLER_LD_LIBRARY_PATH"
        # shellcheck disable=SC1091
        source /opt/ros/humble/setup.bash
        # shellcheck disable=SC1091
        source "$ROS_WS/install/setup.bash"
        set -u
        export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
        export ROS_DOMAIN_ID="${ISAAC_ROS_DOMAIN_ID:-0}"
        export ROS_LOCALHOST_ONLY="1"
        # Calling the installed node directly keeps merger_pid attached to the
        # actual process. `ros2 run` leaves its child alive when only the CLI
        # wrapper receives SIGTERM, which used to leak a merger after exit.
        exec "$SCAN_MERGER_EXECUTABLE" \
            --ros-args --params-file "$SCAN_MERGER_CONFIG" \
            -p output_samples:="$ISAAC_LIDAR_SAMPLE_COUNT"
    ) &
    merger_pid=$!
    sleep 0.5
    if ! kill -0 "$merger_pid" 2>/dev/null; then
        echo "ERROR: /scan_01 + /scan_02 merger failed to start." >&2
        wait "$merger_pid" || true
        exit 1
    fi
fi

echo "Isaac Sim: $ISAAC_SIM_ROOT (6.0.1)"
echo "ROS_DISTRO: $ROS_DISTRO; RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION (external system-Humble bridge)"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID; ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"
echo "Local assets: $ISAACSIM_ASSET_ROOT"
echo "Scene: $ISAAC_SCENE ($SCENE_USD)"
echo "Robot: $ROBOT_USD"
echo "People enabled: $ISAAC_ENABLE_PEOPLE"
if [[ "$ISAAC_SCENE" == "custom" ]]; then
    echo "Gazebo-compatible pedestrians: count=$ISAAC_EXPECTED_PEDESTRIAN_COUNT seed=$ISAAC_PEDESTRIAN_SEED base_speed=$ISAAC_PEDESTRIAN_SPEED m/s"
fi
echo "A/B mode: robot collision protection=$ISAAC_ROBOT_COLLISION_PROTECTION; pedestrian avoidance=$ISAAC_PEDESTRIAN_AVOIDANCE_MODE; pedestrian social=$ISAAC_PEDESTRIAN_SOCIAL_MODE"
echo "Robot physics: $ISAAC_ROBOT_PHYSICS (dynamic rigid body in the custom scene)"
echo "LiDAR mode: $ISAAC_LIDAR_MODE (rtx=native material/angle intensity; physx=range-only fallback)"
echo "RTX LiDAR profile: ${ISAAC_RTX_LIDAR_PROFILE}"
echo "LiDAR requested rate: ${ISAAC_LIDAR_RATE_HZ} Hz"
echo "LiDAR ROS samples per scan: ${ISAAC_LIDAR_SAMPLE_COUNT}"
if (( USE_ROS )); then
    echo "ROS bridge: command UDP $ISAAC_CMD_VEL_UDP_PORT, telemetry UDP $ISAAC_TELEMETRY_UDP_PORT, reset UDP $ISAAC_RESET_UDP_PORT (PID $relay_pid)"
    echo "ROS dual-scan merger: PID $merger_pid"
fi
echo "Run log: $LOG_FILE"
echo "Host: kernel=$(uname -r); NVIDIA=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -n 1); MemAvailable=$((MEM_AVAILABLE_KIB / 1024)) MiB"

# Watch the kernel while Kit runs and terminate the isolated process group as
# soon as a genuinely fatal Xid, kernel fault/stall, or GPU-bus loss appears.
RUN_STARTED_EPOCH="$(date +%s)"
FAULT_FILE="$(mktemp "${TMPDIR:-/tmp}/warehouse_robot_kernel_fault.XXXXXX")"
SIM_PID=""
MONITOR_PID=""
TEE_PID=""

terminate_simulation() {
    if [[ "$SIM_PID" =~ ^[0-9]+$ ]] && kill -0 "$SIM_PID" 2>/dev/null; then
        kill -TERM -- "-$SIM_PID" 2>/dev/null || true
    fi
}

cleanup_runtime() {
    terminate_simulation
    if [[ "$MONITOR_PID" =~ ^[0-9]+$ ]]; then
        kill "$MONITOR_PID" 2>/dev/null || true
        wait "$MONITOR_PID" 2>/dev/null || true
    fi
    if [[ "$TEE_PID" =~ ^[0-9]+$ ]]; then
        wait "$TEE_PID" 2>/dev/null || true
    fi
    cleanup_relay
    rm -f -- "$FAULT_FILE"
}

monitor_kernel() {
    while kill -0 "$SIM_PID" 2>/dev/null; do
        new_faults="$(
            journalctl -k -b 0 --since "@$RUN_STARTED_EPOCH" --no-pager -o short-iso 2>/dev/null \
                | match_text -i "$KERNEL_FATAL_PATTERN" \
                | tail -n 20 \
                || true
        )"
        if [[ -n "$new_faults" ]]; then
            printf '%s\n' "$new_faults" >"$FAULT_FILE"
            echo "ERROR: new GPU/kernel fault detected; terminating Isaac Sim:" >&2
            echo "$new_faults" >&2
            kill -TERM -- "-$SIM_PID" 2>/dev/null || true
            sleep 2
            kill -KILL -- "-$SIM_PID" 2>/dev/null || true
            return
        fi
        sleep 1
    done
}

trap cleanup_runtime EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# A dedicated session lets the watchdog stop Kit and all of its descendants
# without touching unrelated Python or ROS processes.  Capture tee separately
# so the simulator's real exit code and complete log are both retained.
exec {LOG_FD}> >(tee "$LOG_FILE")
TEE_PID=$!
setsid "$ISAAC_SIM_ROOT/python.sh" "$PYTHON_SCRIPT" "$@" >&"$LOG_FD" 2>&1 &
SIM_PID=$!
exec {LOG_FD}>&-
monitor_kernel &
MONITOR_PID=$!
set +e
wait "$SIM_PID"
status=$?
set -e
kill "$MONITOR_PID" 2>/dev/null || true
wait "$MONITOR_PID" 2>/dev/null || true
MONITOR_PID=""
wait "$TEE_PID" 2>/dev/null || true
TEE_PID=""

if [[ -s "$FAULT_FILE" ]]; then
    echo "ERROR: Isaac Sim was stopped because the GPU/kernel fault signature returned." >&2
    echo "Reboot before any further Isaac run. Evidence:" >&2
    cat "$FAULT_FILE" >&2
    exit 6
fi

if match_text -i '\[WAREHOUSE-ROBOT\] ERROR:|Warehouse people robot integration failed|segmentation fault|fatal signal|motion_matching::|libomni\.anim\.skeljoint|crash detected|cannot find protoPath|s3://|https?://.*(asset|motion|download)' "$LOG_FILE" >/dev/null 2>&1; then
    echo "ERROR: integration failure, native crash, invalid robot prototype, or online-asset signature found in $LOG_FILE" >&2
    exit 2
fi
exit "$status"
