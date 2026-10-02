#!/usr/bin/env bash
set -eo pipefail
while true; do
  date -u '+UTC %Y-%m-%dT%H:%M:%SZ'
  nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.free --format=csv,noheader
  ps -eo pid,pgid,%cpu,%mem,cmd | rg 'kit/python/bin/python3.*run_isaacsim|/nav2_controller/controller_server' || true
  sleep 5
done
