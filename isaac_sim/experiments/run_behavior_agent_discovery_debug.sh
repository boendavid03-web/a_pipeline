#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ISAAC_SIM_ROOT="${ISAAC_SIM_6_ROOT:-$PROJECT_ROOT/isaac_sim/isaacsim-6.0.1}"
ASSET_ROOT="${ISAACSIM_ASSET_ROOT:-$PROJECT_ROOT/isaac_sim/assets-6.0.1/Assets/Isaac/6.0}"
CONFIG="$PROJECT_ROOT/runs/pedestrian_validation_matrix/configs/single.yaml"
OUTPUT_ROOT="${BEHAVIOR_AGENT_DISCOVERY_ROOT:-$PROJECT_ROOT/runs/behavior_agent_discovery_debug}"

[[ -x "$ISAAC_SIM_ROOT/python.sh" ]] || exit 1
[[ ! -e "$OUTPUT_ROOT" ]] || { echo "refusing to overwrite: $OUTPUT_ROOT" >&2; exit 2; }
mkdir -p "$OUTPUT_ROOT/config"
cp "$CONFIG" "$OUTPUT_ROOT/config/single.yaml"
export ISAACSIM_ASSET_ROOT="$ASSET_ROOT"
export OMNI_KIT_DISABLE_TELEMETRY=1
"$ISAAC_SIM_ROOT/python.sh" "$SCRIPT_DIR/behavior_agent_discovery_debug.py" \
    --headless --config "$CONFIG" --output "$OUTPUT_ROOT/discovery.json" \
    >"$OUTPUT_ROOT/isaac.log" 2>&1
"$PROJECT_ROOT/.venvs/train/bin/python" - "$OUTPUT_ROOT" <<'PY'
import hashlib, pathlib, sys
root = pathlib.Path(sys.argv[1])
items = []
for path in sorted(p for p in root.rglob('*') if p.is_file() and p.name != 'SHA256SUMS'):
    items.append(f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.relative_to(root)}")
(root / 'SHA256SUMS').write_text('\n'.join(items) + '\n', encoding='utf-8')
PY
echo "BEHAVIOR_AGENT_DISCOVERY_ROOT=$OUTPUT_ROOT"
