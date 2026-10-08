#!/usr/bin/env bash
set -euo pipefail

readonly RUN_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly SOURCE='/home/user/workspace/NavIsaaclab2.0/strict_public_dependency_source_57f98a9_20261003'
exec 9>"${RUN_DIR}/.eval.lock"
flock -n 9 || { echo 'ANOTHER_EVALUATION_IS_RUNNING' >&2; exit 65; }
if [[ -n "$(nvidia-smi --query-compute-apps=pid --format=csv,noheader)" ]]; then
    echo 'GPU_COMPUTE_OWNER_PRESENT' >&2
    exit 75
fi
free_mib="$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits | head -1 | tr -d ' ')"
if [[ ! "${free_mib}" =~ ^[0-9]+$ ]] || (( free_mib < 19456 )); then
    echo "GPU_FREE_BELOW_GATE: ${free_mib}" >&2
    exit 75
fi
if [[ -z "${DISPLAY:-}" ]] || [[ -z "${XAUTHORITY:-}" ]]; then
    echo 'DISPLAY_OR_XAUTHORITY_MISSING' >&2
    exit 75
fi

set +u
source /home/user/IsaacLab/activate_isaaclab.sh
set -u
export PYTHONPATH="/home/user/workspace/NavIsaaclab2.0/Assets/third_party/smplx_0.1.28/site:${SOURCE}:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
cd "${SOURCE}"
date -u +'%Y-%m-%dT%H:%M:%SZ' > "${RUN_DIR}/demo_4180480_START_UTC.txt"
set +e
python -B -u "${RUN_DIR}/demo_4180480.py" \
    > "${RUN_DIR}/demo_4180480_stdout.log" \
    2> "${RUN_DIR}/demo_4180480_stderr.log"
status=$?
set -e
printf '%s\n' "${status}" > "${RUN_DIR}/demo_4180480_EXIT_CODE.txt"
date -u +'%Y-%m-%dT%H:%M:%SZ' > "${RUN_DIR}/demo_4180480_END_UTC.txt"
exit "${status}"
