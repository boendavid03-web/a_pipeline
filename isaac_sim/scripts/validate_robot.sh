#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-/home/user/isaacsim/5.1.0}"

if [[ ! -x "$ISAAC_SIM_ROOT/python.sh" ]]; then
    echo "ERROR: Isaac Sim python.sh not found: $ISAAC_SIM_ROOT/python.sh" >&2
    exit 1
fi

exec "$ISAAC_SIM_ROOT/python.sh" "$SCRIPT_DIR/validate_robot.py" "$@"
