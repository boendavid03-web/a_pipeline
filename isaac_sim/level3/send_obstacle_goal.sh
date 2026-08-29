#!/usr/bin/env bash
# Deferred Test 5: the straight segment from default spawn crosses static map geometry.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/send_first_goal.sh" \
    11.075 6.425 0.142446610 --require-detour
