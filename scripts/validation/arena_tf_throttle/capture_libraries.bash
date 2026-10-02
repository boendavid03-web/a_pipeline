#!/usr/bin/env bash
set -eo pipefail
out="${1:?output path}"
owned_pgid="${2:?owned launch PGID}"
for _ in $(seq 1 120); do
  pid="$(ps -eo pid=,pgid=,args= | awk -v group="$owned_pgid" '$2 == group && $3 == "/opt/ros/humble/lib/nav2_controller/controller_server" {print $1; exit}')"
  if [[ -n "$pid" && -r "/proc/$pid/maps" ]] &&
     rg -q '/libdwb_core.so' "/proc/$pid/maps" &&
     rg -q '/liblayers.so' "/proc/$pid/maps"; then
    {
      printf 'pid=%s\nexe=%s\n' "$pid" "$(readlink -f "/proc/$pid/exe")"
      rg '/(nav2|dwb)[^ /]*/|lib(nav2|dwb|layers)' "/proc/$pid/maps" | sort -u
    } > "$out"
    exit 0
  fi
  sleep 1
done
echo 'controller_server library capture timed out' > "$out"
exit 124
