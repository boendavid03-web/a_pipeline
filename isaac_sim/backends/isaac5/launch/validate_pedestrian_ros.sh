#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_ROOT="$(cd "$BACKEND_ROOT/../../.." && pwd)"
export ROS_DOMAIN_ID="${ISAAC5_GATE6_ROS_DOMAIN_ID:-53}"
export ROS_LOCALHOST_ONLY=1

set +u
source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/workspaces/ros2_ws/install/setup.bash"
source "$PROJECT_ROOT/sim_to_real/robot/comparison_models/ros2_ws/install/setup.bash"
set -u

RUNTIME_LOG="$BACKEND_ROOT/generated/gate6_pedestrian_ros_runtime_580.log"
TOPICS_LOG="$BACKEND_ROOT/generated/gate6_pedestrian_ros_topics.log"
TRACK_LOG="$BACKEND_ROOT/generated/gate6_pedestrian_track.yaml"
SCAN_01_LOG="$BACKEND_ROOT/generated/gate6_scan_01_header.yaml"
SCAN_02_LOG="$BACKEND_ROOT/generated/gate6_scan_02_header.yaml"
JSON_LOG="$BACKEND_ROOT/generated/gate6_pedestrian_ground_truth_json.yaml"
: >"$RUNTIME_LOG"
: >"$TOPICS_LOG"
: >"$TRACK_LOG"
: >"$SCAN_01_LOG"
: >"$SCAN_02_LOG"
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

"$SCRIPT_DIR/validate_pedestrian.sh" --headless --duration 6 >"$RUNTIME_LOG" 2>&1 &
isaac_pid="$!"
ready=0
for _ in $(seq 1 80); do
    if grep -F 'GATE6_PEDESTRIAN_READY=' "$RUNTIME_LOG" >/dev/null 2>&1; then
        ready=1
        break
    fi
    if ! kill -0 "$isaac_pid" 2>/dev/null; then
        break
    fi
    sleep 0.5
done
if [[ "$ready" != 1 ]]; then
    echo "GATE6_PEDESTRIAN_ROS_RESULT=FAIL reason=runtime_not_ready"
    tail -n 100 "$RUNTIME_LOG"
    exit 2
fi

# READY is emitted as soon as the publishers are constructed.  Give DDS graph
# discovery a bounded window before asking ros2 topic echo to resolve types.
topics_ready=0
for _ in $(seq 1 30); do
    ros2 topic list | sort >"$TOPICS_LOG"
    if grep -Fx '/pedestrian_tracks' "$TOPICS_LOG" >/dev/null \
        && grep -Fx '/scan_01' "$TOPICS_LOG" >/dev/null \
        && grep -Fx '/scan_02' "$TOPICS_LOG" >/dev/null \
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
    echo "GATE6_PEDESTRIAN_ROS_RESULT=FAIL reason=dds_topics_not_ready"
    cat "$TOPICS_LOG"
    tail -n 100 "$RUNTIME_LOG"
    exit 2
fi

timeout 12 ros2 topic echo --once /pedestrian_tracks >"$TRACK_LOG" 2>&1 & probe_pids+=("$!")
timeout 12 ros2 topic echo --once /scan_01 --field header >"$SCAN_01_LOG" 2>&1 & probe_pids+=("$!")
timeout 12 ros2 topic echo --once /scan_02 --field header >"$SCAN_02_LOG" 2>&1 & probe_pids+=("$!")
timeout 12 ros2 topic echo --once /isaac_sim/pedestrian_ground_truth_json --field data \
    >"$JSON_LOG" 2>&1 & probe_pids+=("$!")
probe_failed=0
for pid in "${probe_pids[@]}"; do
    wait "$pid" || probe_failed=1
done
probe_pids=()
wait "$isaac_pid" || {
    echo "GATE6_PEDESTRIAN_ROS_RESULT=FAIL reason=runtime_failed"
    tail -n 100 "$RUNTIME_LOG"
    exit 2
}
isaac_pid=""
if [[ "$probe_failed" != 0 ]]; then
    echo "GATE6_PEDESTRIAN_ROS_RESULT=FAIL reason=external_probe_failed"
    exit 2
fi

for topic in /clock /pedestrian_tracks /scan_01 /scan_02 /isaac_sim/pedestrian_ground_truth_json; do
    grep -Fx "$topic" "$TOPICS_LOG" >/dev/null || {
        echo "GATE6_PEDESTRIAN_ROS_RESULT=FAIL reason=missing_topic topic=$topic"
        exit 2
    }
done
grep -F 'GATE6_PEDESTRIAN_RESULT={"status": "PASS"' "$RUNTIME_LOG" >/dev/null
grep -F 'frame_id: odom' "$TRACK_LOG" >/dev/null
grep -F 'track_id: 1' "$TRACK_LOG" >/dev/null
grep -F 'state: CONFIRMED' "$TRACK_LOG" >/dev/null
grep -F 'frame_id: base_scan_01' "$SCAN_01_LOG" >/dev/null
grep -F 'frame_id: base_scan_02' "$SCAN_02_LOG" >/dev/null
grep -F 'a_pipeline_pedestrian_ground_truth/v1' "$JSON_LOG" >/dev/null
grep -F 'person_01' "$JSON_LOG" >/dev/null

echo "GATE6_PEDESTRIAN_ROS_RESULT=PASS domain=$ROS_DOMAIN_ID runtime_log=$RUNTIME_LOG track_log=$TRACK_LOG"
