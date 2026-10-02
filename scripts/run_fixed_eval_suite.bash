#!/usr/bin/env bash
set -euo pipefail

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$project_root"
isaac_lab_root=${ISAAC_LAB_ROOT:?Set ISAAC_LAB_ROOT to your Isaac Lab checkout}
isaac_sim_root=${ISAAC_SIM_ROOT:-$isaac_lab_root/_isaac_sim}
python_bin=${ISAACLAB_PYTHON:-$isaac_lab_root/env_isaaclab/bin/python}
test -x "$python_bin"
test -f "$isaac_sim_root/setup_python_env.sh"
export CARB_APP_PATH="$isaac_sim_root/kit"
export ISAAC_PATH="$isaac_sim_root"
export EXP_PATH="$isaac_sim_root/apps"
export LD_PRELOAD="$isaac_sim_root/kit/libcarb.so"
source "$isaac_sim_root/setup_python_env.sh"
export PYTHONPATH="$project_root:$isaac_lab_root/source/isaaclab:$isaac_lab_root/source/isaaclab_tasks:$isaac_lab_root/source/isaaclab_rl:$isaac_lab_root/source/isaaclab_mimic${PYTHONPATH:+:$PYTHONPATH}"

evidence=${NAVISAACLAB_EVIDENCE_DIR:-$project_root/../Assets/repro_logs/ppo_baseline_1p1r}
run=${NAVISAACLAB_RUN_DIR:-$project_root/output/crowdsim_robot_ppo_baseline_1p1r_20261001/pilot/20261001_101049}
for step in 10240 25088 49920; do
  checkpoint="$run/robot_ppo_${step}.pt"
  test -s "$checkpoint"
  for route in short_control crossing head_on same_direction; do
    output="$evidence/eval_${step}_${route}.json"
    logfile="$evidence/eval_${step}_${route}.log"
    if [[ -e "$output" || -e "$logfile" ]]; then
      echo "Refusing to overwrite existing evaluation evidence: $output or $logfile" >&2
      exit 1
    fi
    /usr/bin/time -v "$python_bin" -u \
      CrowdSim/tools/eval_fixed_1p1r.py \
      --route-id "$route" --ckpt "$checkpoint" --output "$output" \
      > "$logfile" 2>&1
    test -s "$output"
    echo "evaluated checkpoint=$step route=$route"
  done
done
