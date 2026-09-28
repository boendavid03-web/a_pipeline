#!/usr/bin/env bash
# Visible Isaac 5 pedestrian-only preview driven by an Arena-format scenario.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_ROOT="$(cd "$BACKEND_ROOT/../../../.." && pwd)"

count="${ISAAC_PEDESTRIAN_COUNT:-3}"
seed="${ISAAC_PEDESTRIAN_SEED:-7}"
speed="${ISAAC_PEDESTRIAN_SPEED:-}"
duration="${ISAAC5_ARENA_PREVIEW_DURATION:-120}"
width="${ISAAC5_ARENA_PREVIEW_WIDTH:-1920}"
height="${ISAAC5_ARENA_PREVIEW_HEIGHT:-1080}"
scenario="${ISAAC5_ARENA_SCENARIO:-$BACKEND_ROOT/config/arena_eng_lobby_pedestrians.json}"
social_mode="${ISAAC5_SOCIAL_MODE:-hunav}"
mobile_robot=0
dynamic_config=0
command_topic="${ISAAC_TELEOP_CMD_TOPIC:-/cmd_vel}"

while (($#)); do
    case "$1" in
        --count) count="${2:?missing count}"; shift 2 ;;
        --seed) seed="${2:?missing seed}"; shift 2 ;;
        --speed) speed="${2:?missing speed}"; shift 2 ;;
        --duration) duration="${2:?missing duration}"; shift 2 ;;
        --width) width="${2:?missing width}"; shift 2 ;;
        --height) height="${2:?missing height}"; shift 2 ;;
        --scenario) scenario="${2:?missing scenario}"; shift 2 ;;
        --dynamic) dynamic_config=1; shift ;;
        --social-mode) social_mode="${2:?missing social mode}"; shift 2 ;;
        --robot) mobile_robot=1; shift ;;
        --command-topic) command_topic="${2:?missing command topic}"; shift 2 ;;
        *)
            echo "usage: $0 [--dynamic] [--count 1..30] [--seed N] [--speed MPS] [--duration SEC] [--width PX] [--height PX] [--scenario JSON] [--social-mode off|social|hunav] [--robot] [--command-topic TOPIC]" >&2
            exit 2
            ;;
    esac
done

[[ "$count" =~ ^[0-9]+$ ]] && (( count >= 1 && count <= 30 )) || {
    echo "ERROR: Arena pedestrian count must be an integer from 1 through 30." >&2
    exit 2
}
case "$social_mode" in off|social|hunav) ;; *) echo "ERROR: social mode must be off, social, or hunav" >&2; exit 2 ;; esac
[[ "$command_topic" == /* ]] || { echo "ERROR: command topic must be an absolute ROS topic" >&2; exit 2; }

if pgrep -af '/home/user/isaacsim/5\.1\.0/.*(python|kit)|isaac-sim\.sh' >/dev/null; then
    echo "ERROR: refusing to start a second Isaac 5 instance." >&2
    exit 4
fi

if (( dynamic_config )); then
    run_dir="${ISAAC5_RUN_DIR:-$PROJECT_ROOT/runs/isaac5_hunav_dynamic_crowd_$(date +%Y%m%d_%H%M%S)}"
    if [[ -e "$run_dir" && ! -d "$run_dir" ]]; then
        echo "ERROR: dynamic run path exists and is not a directory: $run_dir" >&2
        exit 2
    fi
    mkdir -p "$run_dir"
    scenario="$run_dir/dynamic_crowd_count_${count}_seed_${seed}.json"
    generator_args=(
        --map-yaml "$PROJECT_ROOT/workspaces/ros2_ws/src/semantic_nav_gazebo/maps/gazebo_eng_lobby/gazebo_eng_lobby.yaml"
        --template "$PROJECT_ROOT/isaac_sim/scripts/ira_people_demo/custom_eng_lobby_people.yaml"
        --scenario "$PROJECT_ROOT/workspaces/ros2_ws/src/semantic_nav_gazebo/scenarios/lobby/eng_hall_15.xml"
        --world "$PROJECT_ROOT/workspaces/ros2_ws/src/semantic_nav_gazebo/worlds/gazebo_eng_lobby.world"
        --output "$scenario"
        --count "$count"
        --seed "$seed"
        --ensure-all-clusters
    )
    if [[ -n "$speed" ]]; then
        generator_args+=(--speed "$speed")
    fi
    /usr/bin/python3 "$BACKEND_ROOT/runtime/generate_dynamic_crowd_config.py" "${generator_args[@]}" \
        | tee "$run_dir/dynamic_config_generation.log"
else
    [[ -f "$scenario" ]] || { echo "ERROR: Arena scenario not found: $scenario" >&2; exit 2; }
fi

runtime_args=(
    --duration "$duration"
    --pedestrian-count "$count"
    --pedestrian-seed "$seed"
    --pedestrian-update-rate 30
    --gui-render-rate 30
    --social-mode "$social_mode"
    --arena-scenario "$scenario"
    --width "$width"
    --height "$height"
)
if [[ -n "$speed" ]]; then
    runtime_args+=(--pedestrian-speed "$speed")
fi
if (( mobile_robot )); then
    runtime_args+=(--mobile-robot --command-topic "$command_topic" --command-timeout 0.5)
fi

echo "ISAAC5_ARENA_PEDESTRIAN_PREVIEW scene=a_pipeline_eng_lobby dynamic=$dynamic_config count=$count seed=$seed speed=${speed:-generated_independent} scenario=$scenario mobile_robot=$mobile_robot command_topic=$command_topic"
exec "$SCRIPT_DIR/validate_crowd.sh" "${runtime_args[@]}"
