#!/usr/bin/env bash
# Run the existing Gazebo fixed-four launch and optionally capture/render video.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
ROS_WS="$PROJECT_ROOT/workspaces/ros2_ws"
VIDEO_RENDERER="$SCRIPT_DIR/render_fixed_four_evaluation_video.py"
GOALS_FILE="${FIXED_FOUR_GOALS_FILE:-$PROJECT_ROOT/configs/evaluation/fixed_four_goals.yaml}"
MAP_YAML="${FIXED_FOUR_MAP_YAML:-$PROJECT_ROOT/runs/20260717_042135_v7_dual/maps/semantic_label/map.yaml}"
SEMANTIC_LABEL="${FIXED_FOUR_SEMANTIC_LABEL:-$PROJECT_ROOT/runs/20260717_042135_v7_dual/maps/semantic_label/label.png}"

policy=""
run_name=""
record_video=false
launch_overrides=()
while (($#)); do
    case "$1" in
        --policy)
            policy="${2:-}"
            shift 2
            ;;
        --run-name)
            run_name="${2:-}"
            shift 2
            ;;
        --record-video)
            record_video=true
            shift
            ;;
        --)
            shift
            launch_overrides+=("$@")
            break
            ;;
        *)
            echo "ERROR: unknown argument: $1" >&2
            echo "Usage: $0 --policy semantic_cnn|drl_vo [--run-name NAME] [--record-video] [-- launch_arg:=value ...]" >&2
            exit 2
            ;;
    esac
done
case "$policy" in
    semantic_cnn) launch_file=semantic_cnn_fixed_dual_start_goal_demo.launch.py ;;
    drl_vo) launch_file=drl_vo_fixed_dual_start_goal_demo.launch.py ;;
    *) echo "ERROR: --policy must be semantic_cnn or drl_vo." >&2; exit 2 ;;
esac
for required in "$GOALS_FILE" "$MAP_YAML" "$SEMANTIC_LABEL" "$VIDEO_RENDERER"; do
    [[ -e "$required" ]] || { echo "ERROR: required input missing: $required" >&2; exit 1; }
done

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROS_WS/install/setup.bash"
set -u

if [[ -z "$run_name" ]]; then
    run_name="${policy}_gazebo_$(date +%Y%m%d_%H%M%S)"
fi
evaluation_dir="$PROJECT_ROOT/runs/unified_fixed_four/$run_name"
video_dir="$evaluation_dir/video"
if [[ -e "$evaluation_dir" ]]; then
    echo "ERROR: refusing to reuse evaluation directory: $evaluation_dir" >&2
    exit 1
fi
mkdir -p "$evaluation_dir"
echo "Evaluation: $evaluation_dir"
if [[ "$record_video" == "true" ]]; then
    echo "Video: $video_dir/evaluation_video.mp4"
fi

common_args=(
    fixed_test:=true
    fixed_goals_file:="$GOALS_FILE"
    enable_goal_picker:=false
    auto_set_initial_goal:=false
    evaluate_episode:=true
    evaluation_output_dir:="$evaluation_dir"
    evaluation_multi_episode:=true
    evaluation_timeout_sec:=86400.0
    record_trace:=false
    record_video:="$record_video"
    video_output_dir:="$video_dir"
    video_simulator_name:=gazebo
    map_yaml:="$MAP_YAML"
    semantic_label:="$SEMANTIC_LABEL"
    spawn_scene_pedestrians:=true
    pedestrian_count:=19
    pedestrian_seed:=7
    pedestrian_speed:=1.0
    robot_x:=2.0
    robot_y:=2.0
    robot_yaw:=0.0
    goal_tolerance:=0.35
    lookahead:=1.0
    inflate_radius:=0.45
)

set +e
ros2 launch semantic_nav_gazebo "$launch_file" "${common_args[@]}" "${launch_overrides[@]}"
status=$?
set -e
if [[ "$record_video" == "true" && -f "$evaluation_dir/session_summary.json" ]]; then
    mkdir -p "$video_dir"
    if ! python3 "$VIDEO_RENDERER" \
        --evaluation-dir "$evaluation_dir" \
        --map-yaml "$MAP_YAML" \
        --capture-dir "$video_dir/sync" \
        --output-mp4 "$video_dir/evaluation_video.mp4" \
        --save-episode-screenshots \
        >"$video_dir/render.log" 2>&1; then
        echo "ERROR: video rendering failed. See $video_dir/render.log" >&2
        exit 1
    fi
    echo "FIXED_FOUR_VIDEO_READY path=$video_dir/evaluation_video.mp4"
fi
exit "$status"
