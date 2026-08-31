#!/usr/bin/env bash
set -euo pipefail

# Recompute evaluator-only evidence from one immutable bag. This never starts
# Isaac, the scan merger, DR-SPAAM, or the point tracker.
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROS_WS="$PROJECT_ROOT/workspaces/ros2_ws"
MANIFEST="$PROJECT_ROOT/isaac_sim/config/crowded_tracking_suite_manifest_20260831.json"
RUN_DIR=""
PLAYBACK_RATE="${CROWDED_REPLAY_RATE:-5.0}"

while (( $# )); do
  case "$1" in
    --manifest) MANIFEST="$2"; shift 2 ;;
    --run-dir) RUN_DIR="$2"; shift 2 ;;
    --rate) PLAYBACK_RATE="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ -n "$RUN_DIR" ]] || { echo "--run-dir is required" >&2; exit 2; }
MANIFEST="$(realpath "$MANIFEST")"
RUN_DIR="$(realpath "$RUN_DIR")"

mapfile -t ENTRY < <(python3 - "$MANIFEST" "$RUN_DIR" <<'PY'
import hashlib, json, pathlib, sys
manifest_path = pathlib.Path(sys.argv[1])
run_dir = pathlib.Path(sys.argv[2]).resolve()
manifest = json.loads(manifest_path.read_text())
if manifest.get("schema") != "isaac_crowded_tracking_suite/v2":
    raise SystemExit("manifest schema must be v2")
root = pathlib.Path(manifest["run_root"])
if not root.is_absolute():
    root = (manifest_path.parent / root).resolve()
matches = [e for e in manifest.get("entries", []) if (root / e["run_id"]).resolve() == run_dir]
if len(matches) != 1:
    raise SystemExit(f"run must match exactly one manifest entry: {run_dir}")
entry = matches[0]
scenario = json.loads((run_dir / "scenario_metadata.json").read_text())
if scenario.get("scenario") != entry["scenario"]:
    raise SystemExit("scenario metadata mismatch")
bag_files = [(root / value).resolve() for value in entry["bag_files"]]
bag_metadata = (root / entry["metadata"]).resolve()
for path in [*bag_files, bag_metadata]:
    if not path.is_file():
        raise SystemExit(f"missing bag input: {path}")
digest = hashlib.sha256()
for path in sorted([*bag_files, bag_metadata], key=lambda item: str(item.relative_to(run_dir))):
    relative = str(path.relative_to(run_dir))
    digest.update(relative.encode()); digest.update(b"\0")
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
if digest.hexdigest() != entry["canonical_hash"]:
    raise SystemExit(f"canonical raw bag hash mismatch: {entry['cell']}")
print(entry["cell"])
print(entry["scenario"])
print(json.dumps(scenario["stress_ids"], separators=(",", ":")))
print("-1.0" if scenario.get("requested_spacing_m") is None else float(scenario["requested_spacing_m"]))
print(str(run_dir / entry.get("output_name", "evaluation_recomputed_v2")))
print(str(bag_metadata.parent))
print(entry["canonical_hash"])
print(json.dumps(entry, sort_keys=True, separators=(",", ":")))
PY
)
(( ${#ENTRY[@]} == 8 )) || { echo "manifest entry preflight failed" >&2; exit 1; }
CELL="${ENTRY[0]}"; SCENARIO="${ENTRY[1]}"; STRESS_IDS="${ENTRY[2]}"
REQUESTED_SPACING="${ENTRY[3]}"; OUTPUT_DIR="${ENTRY[4]}"; BAG_DIR="${ENTRY[5]}"
RAW_BAG_SHA="${ENTRY[6]}"; ENTRY_JSON="${ENTRY[7]}"
[[ ! -e "$OUTPUT_DIR" ]] || { echo "output already exists: $OUTPUT_DIR" >&2; exit 1; }

DOMAIN="${ROS_DOMAIN_ID:-$(( 160 + $(printf '%s' "$CELL" | cksum | awk '{print $1}') % 40 ))}"
export ROS_DOMAIN_ID="$DOMAIN"
export RCUTILS_COLORIZED_OUTPUT=0
set +u
source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
set -u
if [[ -n "$(ros2 node list 2>/dev/null || true)" ]]; then
  echo "ROS domain $DOMAIN is not isolated" >&2
  exit 1
fi

mkdir "$OUTPUT_DIR"
EVALUATOR="$ROS_WS/src/semantic_nav_gazebo/scripts/pedestrian_crowded_tracking_evaluator.py"
EVAL_PID=""
cleanup() {
  if [[ -n "$EVAL_PID" ]] && kill -0 "$EVAL_PID" 2>/dev/null; then
    kill -INT "$EVAL_PID" 2>/dev/null || true
    wait "$EVAL_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

python3 "$EVALUATOR" --ros-args \
  -p use_sim_time:=true -p scenario:="$SCENARIO" -p stress_ids:="$STRESS_IDS" \
  -p requested_spacing:="$REQUESTED_SPACING" -p target_frame:=odom \
  -p output_dir:="$OUTPUT_DIR" >"$OUTPUT_DIR/evaluator.log" 2>&1 &
EVAL_PID=$!

ready=0
for _ in $(seq 1 120); do
  kill -0 "$EVAL_PID" 2>/dev/null || { echo "evaluator exited before readiness" >&2; exit 1; }
  if rg -q 'CROWDED_EVALUATOR_READY' "$OUTPUT_DIR/evaluator.log" 2>/dev/null; then ready=1; break; fi
  sleep 0.25
done
(( ready == 1 )) || { echo "evaluator readiness timeout" >&2; exit 1; }

# The bag already contains the simulator's /clock. A second synthetic clock
# publisher would invalidate ordering, so do not pass rosbag's --clock option.
set +e
ros2 bag play "$BAG_DIR" --rate "$PLAYBACK_RATE" >"$OUTPUT_DIR/offline_replay.log" 2>&1
PLAY_RC=$?
set -e
(( PLAY_RC == 0 )) || { echo "rosbag playback failed: rc=$PLAY_RC" >&2; exit 1; }

kill -INT "$EVAL_PID" 2>/dev/null || true
wait "$EVAL_PID"
EVAL_RC=$?; EVAL_PID=""
(( EVAL_RC == 0 )) || { echo "evaluator failed: rc=$EVAL_RC" >&2; exit 1; }
test -s "$OUTPUT_DIR/summary.json"
test -s "$OUTPUT_DIR/crowded_tracking_trace.jsonl"

python3 - "$PROJECT_ROOT" "$RUN_DIR" "$OUTPUT_DIR" "$RAW_BAG_SHA" "$ENTRY_JSON" "$DOMAIN" <<'PY'
import hashlib, json, pathlib, sys
root, run_dir, output = map(pathlib.Path, sys.argv[1:4])
raw_hash, entry_json, domain = sys.argv[4:7]
summary_path = output / "summary.json"; trace_path = output / "crowded_tracking_trace.jsonl"
summary = json.loads(summary_path.read_text()); quality = summary.get("quality", {})
counts = quality.get("exact_source_stamp_counters", {}); frames = int(quality.get("evaluated_frames", 0))
errors = []
if summary.get("schema") != "isaac_crowded_tracking_evaluation/v2": errors.append("summary schema")
if frames <= 0: errors.append("no evaluated frames")
for name in ("scan_01", "scan_02", "merged", "detections", "tracks"):
    if int(counts.get(name, -1)) != frames: errors.append(f"exact stamp {name}")
for name in ("dropped_unsynchronized_frames", "dropped_wrong_frame_frames", "dropped_tf_frames", "malformed_detection_frames", "pending_track_frames"):
    if int(quality.get(name, -1)) != 0: errors.append(name)
contract = summary.get("replay_contract", {})
if not contract.get("clock_monotonic", False): errors.append("clock monotonic")
if not contract.get("tf_static_received", False): errors.append("tf_static")
if errors:
    (output / "REPLAY_INVALID").write_text("\n".join(errors) + "\n")
    raise SystemExit("replay validity failed: " + ", ".join(errors))
def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()
manifest = {
    "schema": "isaac_crowded_tracking_replay_manifest/v2",
    "cell": json.loads(entry_json)["cell"], "run_dir": str(run_dir.resolve()),
    "raw_bag_sha256": raw_hash, "ros_domain_id": int(domain), "use_sim_time": True,
    "input_manifest_entry_sha256": hashlib.sha256(entry_json.encode()).hexdigest(),
    "evaluator_source_sha256": sha(root / "workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/pedestrian_crowded_tracking_evaluator.py"),
    "analysis_core_source_sha256": sha(root / "workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/pedestrian_crowded_tracking_analysis_core.py"),
    "summary_sha256": sha(summary_path), "trace_sha256": sha(trace_path),
}
(output / "replay_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
(output / "REPLAY_VALID").write_text("PASS\n")
PY

echo "CROWDED_TRACKING_RECOMPUTE_V2=PASS cell=$CELL output=$OUTPUT_DIR"
