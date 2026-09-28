#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_ROOT="$(cd "$BACKEND_ROOT/../../.." && pwd)"
export ROS_DOMAIN_ID="${ISAAC5_GATE6B_ROS_DOMAIN_ID:-54}"
export ROS_LOCALHOST_ONLY=1

set +u
source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/workspaces/ros2_ws/install/setup.bash"
source "$PROJECT_ROOT/sim_to_real/robot/comparison_models/ros2_ws/install/setup.bash"
set -u

RUNTIME_LOG="$BACKEND_ROOT/generated/gate6b_crowd_ros_runtime_580.log"
TOPICS_LOG="$BACKEND_ROOT/generated/gate6b_crowd_ros_topics.log"
TRACK_LOG="$BACKEND_ROOT/generated/gate6b_crowd_tracks.yaml"
SCAN_LOG="$BACKEND_ROOT/generated/gate6b_scan_01_header.yaml"
JSON_LOG="$BACKEND_ROOT/generated/gate6b_crowd_ground_truth_json.yaml"
: >"$RUNTIME_LOG"
: >"$TOPICS_LOG"
: >"$TRACK_LOG"
: >"$SCAN_LOG"
: >"$JSON_LOG"

isaac_pid=""
probe_pids=()
cleanup() {
    for pid in "${probe_pids[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
        fi
    done
    if [[ -n "$isaac_pid" ]] && kill -0 "$isaac_pid" 2>/dev/null; then
        kill "$isaac_pid" 2>/dev/null || true
        wait "$isaac_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

"$SCRIPT_DIR/validate_crowd.sh" --headless --duration 6 >"$RUNTIME_LOG" 2>&1 &
isaac_pid="$!"
ready=0
for _ in $(seq 1 100); do
    if grep -F 'GATE6B_CROWD_READY=' "$RUNTIME_LOG" >/dev/null 2>&1; then
        ready=1
        break
    fi
    if ! kill -0 "$isaac_pid" 2>/dev/null; then
        break
    fi
    sleep 0.5
done
if [[ "$ready" != 1 ]]; then
    echo "GATE6B_CROWD_ROS_RESULT=FAIL reason=runtime_not_ready"
    tail -n 100 "$RUNTIME_LOG"
    exit 2
fi

topics_ready=0
for _ in $(seq 1 30); do
    ros2 topic list | sort >"$TOPICS_LOG"
    if grep -Fx '/pedestrian_tracks' "$TOPICS_LOG" >/dev/null \
        && grep -Fx '/scan_01' "$TOPICS_LOG" >/dev/null \
        && grep -Fx '/isaac_sim/pedestrian_ground_truth_json' "$TOPICS_LOG" >/dev/null; then
        topics_ready=1
        break
    fi
    if ! kill -0 "$isaac_pid" 2>/dev/null; then
        break
    fi
    sleep 0.2
done
if [[ "$topics_ready" != 1 ]]; then
    echo "GATE6B_CROWD_ROS_RESULT=FAIL reason=dds_topics_not_ready"
    cat "$TOPICS_LOG"
    tail -n 100 "$RUNTIME_LOG"
    exit 2
fi

timeout 12 ros2 topic echo --once /pedestrian_tracks >"$TRACK_LOG" 2>&1 & probe_pids+=("$!")
timeout 12 ros2 topic echo --once /scan_01 --field header >"$SCAN_LOG" 2>&1 & probe_pids+=("$!")
timeout 12 ros2 topic echo --once /isaac_sim/pedestrian_ground_truth_json --field data \
    >"$JSON_LOG" 2>&1 & probe_pids+=("$!")

probe_failed=0
for pid in "${probe_pids[@]}"; do
    wait "$pid" || probe_failed=1
done
probe_pids=()
wait "$isaac_pid" || {
    echo "GATE6B_CROWD_ROS_RESULT=FAIL reason=runtime_failed"
    tail -n 100 "$RUNTIME_LOG"
    exit 2
}
isaac_pid=""
if [[ "$probe_failed" != 0 ]]; then
    echo "GATE6B_CROWD_ROS_RESULT=FAIL reason=external_probe_failed"
    exit 2
fi

grep -F 'GATE6B_CROWD_RESULT={"status": "PASS"' "$RUNTIME_LOG" >/dev/null
grep -F 'frame_id: odom' "$TRACK_LOG" >/dev/null
for track_id in $(seq 1 8); do
    grep -F "track_id: $track_id" "$TRACK_LOG" >/dev/null
done
[[ "$(grep -Fc 'state: CONFIRMED' "$TRACK_LOG")" -eq 8 ]]
grep -F 'frame_id: base_scan_01' "$SCAN_LOG" >/dev/null
grep -F 'a_pipeline_pedestrian_ground_truth/v1' "$JSON_LOG" >/dev/null
for person_number in $(seq 1 8); do
    person_id="$(printf '%02d' "$person_number")"
    grep -F "person_$person_id" "$JSON_LOG" >/dev/null
done

echo "GATE6B_CROWD_ROS_RESULT=PASS domain=$ROS_DOMAIN_ID pedestrians=8 runtime_log=$RUNTIME_LOG track_log=$TRACK_LOG"
