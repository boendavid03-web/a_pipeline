#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 6 ]]; then
    echo 'usage: RUN_ONE.sh LABEL CHECKPOINT SHA256 STEP EPISODES MAX_CONTROL_STEPS' >&2
    exit 64
fi

readonly RUN_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly SOURCE='/home/user/workspace/NavIsaaclab2.0/strict_public_dependency_source_57f98a9_20261003'
readonly LABEL="$1"
readonly CHECKPOINT="$2"
readonly EXPECTED_SHA="$3"
readonly EXPECTED_STEP="$4"
readonly EPISODES="$5"
readonly MAX_STEPS="$6"

exec 9>"${RUN_DIR}/.eval.lock"
flock -n 9 || { echo 'EVAL_ALREADY_RUNNING' >&2; exit 65; }

if [[ -n "$(nvidia-smi --query-compute-apps=pid --format=csv,noheader)" ]]; then
    echo 'GPU_COMPUTE_OWNER_PRESENT' >&2
    exit 75
fi
free_mib="$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits | head -1 | tr -d ' ')"
if [[ ! "${free_mib}" =~ ^[0-9]+$ ]] || (( free_mib < 19456 )); then
    echo "GPU_FREE_BELOW_GATE: ${free_mib}" >&2
    exit 75
fi

set +u
source /home/user/IsaacLab/activate_isaaclab.sh
set -u
export PYTHONPATH="/home/user/workspace/NavIsaaclab2.0/Assets/third_party/smplx_0.1.28/site:${SOURCE}:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
cd "${SOURCE}"

date -u +'%Y-%m-%dT%H:%M:%SZ' > "${RUN_DIR}/${LABEL}_START_UTC.txt"
set +e
python -B -u "${RUN_DIR}/evaluate.py" \
    --checkpoint "${CHECKPOINT}" \
    --expected-sha256 "${EXPECTED_SHA}" \
    --expected-step "${EXPECTED_STEP}" \
    --output "${RUN_DIR}/${LABEL}" \
    --seed 702 \
    --episodes "${EPISODES}" \
    --max-control-steps "${MAX_STEPS}" \
    > "${RUN_DIR}/${LABEL}_stdout.log" \
    2> "${RUN_DIR}/${LABEL}_stderr.log"
status=$?
set -e
printf '%s\n' "${status}" > "${RUN_DIR}/${LABEL}_EXIT_CODE.txt"
date -u +'%Y-%m-%dT%H:%M:%SZ' > "${RUN_DIR}/${LABEL}_END_UTC.txt"
exit "${status}"
