#!/usr/bin/env bash
# Dispatch a bounded, production-shaped Isaac5 demo chain.  Isaac6 remains an
# explicit legacy choice in the callers that select ISAAC_BACKEND=isaac6.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workflow=""
dry_run=0
while (($#)); do
    case "$1" in
        --workflow) workflow="${2:?missing workflow}"; shift 2 ;;
        --dry-run) dry_run=1; shift ;;
        *) echo "usage: $0 --workflow {drlvo|drspaam|tracking|semantic|matrix|single-person} [--dry-run]" >&2; exit 2 ;;
    esac
done
case "$workflow" in
    drlvo) target="$SCRIPT_DIR/validate_gate9_fixed_goal_gui.sh" ;;
    drspaam) target="$SCRIPT_DIR/validate_gate71_drspaam.sh" ;;
    tracking|matrix|single-person) target="$SCRIPT_DIR/validate_gate72_tracking.sh" ;;
    semantic) target="$SCRIPT_DIR/validate_gate73_semantic.sh" ;;
    *) echo "ERROR: unsupported Isaac5 production workflow: ${workflow:-unset}" >&2; exit 2 ;;
esac
if pgrep -af '/home/user/isaacsim/5\.1\.0/.*(python|kit)|isaac-sim\.sh' >/dev/null; then
    echo "ERROR: refusing to start a second Isaac instance; stop the owned run or use another terminal after it exits." >&2
    exit 4
fi
printf 'ISAAC5_PRODUCTION_DEMO workflow=%s target=%s policy_scope=%s\n' \
    "$workflow" "$target" "$([[ "$workflow" == semantic ]] && echo SemanticCNN || echo DRL-VO-or-perception-only)"
if ((dry_run)); then
    exit 0
fi
exec "$target"
