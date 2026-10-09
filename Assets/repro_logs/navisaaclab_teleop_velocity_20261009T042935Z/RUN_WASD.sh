#!/usr/bin/env bash
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
R=/home/user/workspace/NavIsaaclab2.0/Assets/repro_logs
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"; OUT="$D/manual_$STAMP"
exec 8>"$D/.isaac.lock"; exec 9>"$R/navisaaclab_tilt_motion_20261008T173940Z/.isaac.lock"
exec 10>"$R/single_robot_teleop_20261008T064959Z/.teleop.lock"; exec 11>"$R/navisaaclab_motion_range_20261009T013250Z/.isaac.lock"
for fd in 8 9 10 11; do flock -n "$fd" || { echo ISAAC_LOCK_BUSY >&2; exit 75; }; done
[[ -z "$(nvidia-smi --query-compute-apps=pid --format=csv,noheader)" ]] || { echo GPU_COMPUTE_OWNER_PRESENT >&2; exit 75; }
free_mib="$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits | head -1 | tr -d ' ')"
(( free_mib >= 19456 )) || { echo "GPU_FREE_BELOW_GATE:$free_mib" >&2; exit 75; }
[[ -t 0 ]] || { echo INTERACTIVE_TERMINAL_REQUIRED >&2; exit 75; }
[[ -n "${DISPLAY:-}" ]] || { echo GUI_DISPLAY_UNAVAILABLE >&2; exit 76; }
mkdir -p "$OUT"
set +u; source /home/user/IsaacLab/activate_isaaclab.sh; set -u
export PYTHONPATH="/home/user/workspace/NavIsaaclab2.0/Assets/third_party/smplx_0.1.28/site:$D/../navisaaclab_tilt_motion_20261008T173940Z/source:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
cd /home/user/workspace/NavIsaaclab2.0/Assets/repro_logs/navisaaclab_tilt_motion_20261008T173940Z/source
printf 'WASD_OUTPUT=%s\n' "$OUT"
timeout --signal=TERM --kill-after=30s 1200s python -B -u "$D/wasd_teleop.py" --output "$OUT" "$@"
