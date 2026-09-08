#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ISAAC_SIM_ROOT="${ISAAC_SIM_6_ROOT:-$PROJECT_ROOT/isaac_sim/isaacsim-6.0.1}"
ASSET_ROOT="${ISAACSIM_ASSET_ROOT:-$PROJECT_ROOT/isaac_sim/assets-6.0.1/Assets/Isaac/6.0}"
CONFIG="$PROJECT_ROOT/runs/pedestrian_validation_matrix/configs/single.yaml"
OUTPUT_ROOT="${BEHAVIOR_AGENT_VALIDATION_ROOT:-$PROJECT_ROOT/runs/behavior_agent_interface_validation}"
DURATION="${BEHAVIOR_AGENT_VALIDATION_DURATION_SEC:-60}"

[[ -x "$ISAAC_SIM_ROOT/python.sh" ]] || { echo "missing Isaac python.sh: $ISAAC_SIM_ROOT/python.sh" >&2; exit 1; }
[[ -f "$CONFIG" ]] || { echo "missing config: $CONFIG" >&2; exit 1; }
[[ ! -e "$OUTPUT_ROOT" ]] || { echo "refusing to overwrite: $OUTPUT_ROOT" >&2; exit 2; }
mkdir -p "$OUTPUT_ROOT/config"
cp "$CONFIG" "$OUTPUT_ROOT/config/single.yaml"
printf 'duration_sec: %s\nconfig: %s\n' "$DURATION" "$CONFIG" > "$OUTPUT_ROOT/config/experiment_manifest.txt"

export ISAACSIM_ASSET_ROOT="$ASSET_ROOT"
export OMNI_KIT_DISABLE_TELEMETRY=1
for experiment in static moving stable_subgoal; do
    run_dir="$OUTPUT_ROOT/$experiment"
    mkdir -p "$run_dir"
    "$ISAAC_SIM_ROOT/python.sh" "$SCRIPT_DIR/behavior_agent_interface_validation.py" \
        --headless --config "$CONFIG" --experiment "$experiment" --duration "$DURATION" \
        --trace "$run_dir/trace.jsonl" --metrics "$run_dir/metrics.json" \
        >"$run_dir/isaac.log" 2>&1
done

"$PROJECT_ROOT/.venvs/train/bin/python" - "$OUTPUT_ROOT" <<'PY'
import hashlib, pathlib, sys
root = pathlib.Path(sys.argv[1])
lines = []
for path in sorted(p for p in root.rglob('*') if p.is_file() and p.name != 'SHA256SUMS'):
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    lines.append(f"{digest}  {path.relative_to(root)}")
(root / 'SHA256SUMS').write_text('\n'.join(lines) + '\n', encoding='utf-8')
PY
echo "BEHAVIOR_AGENT_INTERFACE_VALIDATION_ROOT=$OUTPUT_ROOT"
