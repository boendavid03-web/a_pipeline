#!/usr/bin/env bash
# Run bounded Isaac pedestrian interaction cells without the DRL-VO stack.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
CONFIG_DIR="$PROJECT_ROOT/runs/pedestrian_validation_matrix/configs"
OUTPUT_ROOT="$PROJECT_ROOT/runs/pedestrian_validation_matrix/evaluation"
DURATION_SEC="${ISAAC_MATRIX_DURATION_SEC:-60.0}"

if (( $# )); then
    cells=("$@")
else
    cells=(single head_on crossing near_wall human_robot_static)
fi

cell_count() {
    case "$1" in
        single|human_robot_static) echo 1 ;;
        head_on|crossing|near_wall) echo 2 ;;
        *) echo "ERROR: unknown validation cell: $1" >&2; return 2 ;;
    esac
}

mkdir -p "$OUTPUT_ROOT"
for cell in "${cells[@]}"; do
    count="$(cell_count "$cell")"
    config="$CONFIG_DIR/$cell.yaml"
    run_dir="$OUTPUT_ROOT/$cell"
    if [[ ! -f "$config" ]]; then
        echo "ERROR: missing validation config: $config" >&2
        exit 1
    fi
    if [[ -e "$run_dir" ]]; then
        echo "ERROR: refusing to overwrite validation run: $run_dir" >&2
        exit 1
    fi
    mkdir "$run_dir"
    echo "VALIDATION_CELL_START=$cell"
    env \
        ROS_DOMAIN_ID=78 ISAAC_ROS_DOMAIN_ID=78 \
        ISAAC_SCENE=custom ISAAC_ENABLE_PEOPLE=1 \
        ISAAC_PEDESTRIAN_COUNT="$count" ISAAC_PEDESTRIAN_SEED=7 \
        ISAAC_PEDESTRIAN_SPEED=1.0 \
        ISAAC_EXPLICIT_CUSTOM_IRA_CONFIG="$config" \
        ISAAC_EXPLICIT_CUSTOM_IRA_MIN_START_SEPARATION_M=1.0 \
        ISAAC_PEDESTRIAN_FREE_SPACE_CLEARANCE_M=0.55 \
        ISAAC_PEDESTRIAN_FREE_SPACE_GUARD_CLEARANCE_M=0.20 \
        ISAAC_PEDESTRIAN_AVOIDANCE_MODE=gentle \
        ISAAC_PEDESTRIAN_SOCIAL_MODE=gazebo_social \
        ISAAC_PEDESTRIAN_SOCIAL_MASS_KG=20.0 \
        ISAAC_PEDESTRIAN_PERSONAL_SPACE_M=1.0 \
        ISAAC_PEDESTRIAN_VISUAL_OVERLAP_M=0.45 \
        ISAAC_PEDESTRIAN_SOCIAL_NEIGHBOR_RANGE_M=10.0 \
        ISAAC_PEDESTRIAN_SOCIAL_FORCE_WEIGHT=5.1 \
        ISAAC_PEDESTRIAN_ROBOT_SOCIAL_FORCE_WEIGHT=5.1 \
        ISAAC_PEDESTRIAN_ROBOT_PERSONAL_SPACE_FORCE_WEIGHT=6.0 \
        ISAAC_PEDESTRIAN_SOCIAL_RELAXATION_TIME_SEC=0.5 \
        ISAAC_PEDESTRIAN_SOCIAL_SMOOTHING_TIME_SEC=0.35 \
        ISAAC_PEDESTRIAN_SOCIAL_MAX_ACCEL_MPS2=4.0 \
        ISAAC_PEDESTRIAN_SOCIAL_MAX_STEERING_CORRECTION_MPS=0.65 \
        ISAAC_PEDESTRIAN_SOCIAL_MAX_LATERAL_STEERING_MPS=0.45 \
        ISAAC_PEDESTRIAN_SOCIAL_MAX_STEERING_ANGLE_DEG=35.0 \
        ISAAC_PEDESTRIAN_SOCIAL_MIN_SPEED_MPS=0.15 \
        ISAAC_PEDESTRIAN_AGENT_RADIUS_M=0.35 \
        ISAAC_PEDESTRIAN_ROBOT_RADIUS_M=0.47 \
        ISAAC_PEDESTRIAN_ROBOT_CLEARANCE_M=1.0 \
        ISAAC_PEDESTRIAN_ROBOT_PERSONAL_SPACE_SIGMA_M=0.2 \
        ISAAC_PEDESTRIAN_SOCIAL_EMERGENCY_YIELD_TRIGGER_M=0.50 \
        ISAAC_PEDESTRIAN_SOCIAL_EMERGENCY_YIELD_RESUME_M=0.80 \
        ISAAC_PEDESTRIAN_SOCIAL_STEERING_LOOKAHEAD_M=1.0 \
        ISAAC_PEDESTRIAN_SOCIAL_ROUTE_LOOKAHEAD_M=1.5 \
        ISAAC_PEDESTRIAN_SOCIAL_WAYPOINT_REACH_M=0.35 \
        ISAAC_PEDESTRIAN_SOCIAL_TARGET_MIN_SHIFT_M=0.02 \
        ISAAC_PEDESTRIAN_SOCIAL_TRACE_PATH="$run_dir/pedestrian_social_steering.jsonl" \
        ISAAC_ROBOT_PHYSICS=1 ISAAC_ROBOT_COLLISION_PROTECTION=1 \
        ISAAC_PHYSX_GPU_DYNAMICS=0 \
        ISAAC_LIDAR_MODE=physx ISAAC_LIDAR_RATE_HZ=15 \
        ISAAC_LIDAR_SAMPLE_COUNT=2000 \
        bash "$SCRIPT_DIR/run_isaac_6_0_warehouse_people_robot.sh" \
        --duration "$DURATION_SEC" --no-ros >"$run_dir/isaac.log" 2>&1
    grep '^WAREHOUSE_PEOPLE_ROBOT_RESULT=' "$run_dir/isaac.log" | tail -n 1
    echo "VALIDATION_CELL_END=$cell"
    # The launcher's process-substitution logger can retain its flock fd for a
    # brief instant after Kit exits.  Avoid a false lock conflict at cell swap.
    sleep 3
done
