#!/usr/bin/env bash
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
R=/home/user/workspace/NavIsaaclab2.0/Assets/repro_logs
exec 8>"$D/.isaac.lock"; exec 9>"$R/navisaaclab_tilt_motion_20261008T173940Z/.isaac.lock"
exec 10>"$R/navisaaclab_motion_smoke_20261008T172606Z/.isaac.lock"; exec 11>"$R/navisaaclab_ppo_ready_20261008T160517Z/.isaac.lock"
exec 12>"$R/single_robot_teleop_20261008T064959Z/.teleop.lock"; exec 13>"$R/navisaaclab_motion_range_20261009T013250Z/.isaac.lock"
exec 14>"$R/navisaaclab_camera_consistency_20261008T122712Z/.camera_consistency.lock"
exec 15>"$R/navisaaclab_diagnostic_20261008T115415Z/.diagnostic.lock"; exec 16>"$R/public_repo_fixed_eval_20261008T044445Z/.eval.lock"
for fd in 8 9 10 11 12 13 14 15 16; do flock -n "$fd" || { echo ISAAC_LOCK_BUSY >&2; exit 75; }; done
[[ -z "$(nvidia-smi --query-compute-apps=pid --format=csv,noheader)" ]] || { echo GPU_COMPUTE_OWNER_PRESENT >&2; exit 75; }
free_mib="$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits | head -1 | tr -d ' ')"
(( free_mib >= 19456 )) || { echo "GPU_FREE_BELOW_GATE:$free_mib" >&2; exit 75; }
(( $(df -Pk "$D" | awk 'NR==2{print $4}') >= 1048576 )) || { echo DISK_FREE_BELOW_GATE >&2; exit 75; }
if [[ "${1:-}" == gui ]]; then
  [[ -n "${DISPLAY:-}" ]] || { echo GUI_DISPLAY_UNAVAILABLE >&2; exit 76; }
  shift
else
  set -- --headless
fi
set +u; source /home/user/IsaacLab/activate_isaaclab.sh; set -u
export PYTHONPATH="/home/user/workspace/NavIsaaclab2.0/Assets/third_party/smplx_0.1.28/site:$D/../navisaaclab_tilt_motion_20261008T173940Z/source:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
cd /home/user/workspace/NavIsaaclab2.0/Assets/repro_logs/navisaaclab_tilt_motion_20261008T173940Z/source
date -u +%FT%TZ > "$D/START_UTC.txt"
printf '%q ' python -B -u "$D/telemetry_probe.py" "$@" > "$D/COMMAND.txt"; printf '\n' >> "$D/COMMAND.txt"
set +e
timeout --signal=TERM --kill-after=30s 1200s python -B -u "$D/telemetry_probe.py" "$@" > "$D/stdout.log" 2> "$D/stderr.log"
rc=$?
set -e
echo "$rc" > "$D/EXIT_CODE.txt"; date -u +%FT%TZ > "$D/END_UTC.txt"
exit "$rc"
