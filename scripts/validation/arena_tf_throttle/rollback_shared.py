#!/usr/bin/env python3
"""Manually restore the five pinned shared TF copies from saved before bytes."""
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

if sys.argv[1:] != ['--execute']:
    raise SystemExit('Usage: rollback_shared.py --execute (restores five shared TF copies)')
active = subprocess.run(['pgrep', '-f', '/kit/python/bin/python3 .*run_isaacsim'],
                        capture_output=True, text=True)
if active.returncode == 0:
    raise SystemExit('Isaac child still running; leave its files untouched')

provenance = json.loads(Path('/home/user/arena_local_costmap_tf_fix_20261001/provenance_after.json').read_text())
def sha(data):
    return hashlib.sha256(data).hexdigest()

# Check every target before writing any file.
for name, item in provenance.items():
    target = Path(item['path'])
    backup = Path(item['backup'])
    if sha(target.read_bytes()) != item['sha256_after']:
        raise SystemExit(f'{name}: target changed since validation; abort')
    if sha(backup.read_bytes()) != item['sha256_before']:
        raise SystemExit(f'{name}: backup hash mismatch; abort')

for name, item in provenance.items():
    target = Path(item['path'])
    backup = Path(item['backup'])
    temp = target.with_name(target.name + '.rollback_tmp')
    temp.write_bytes(backup.read_bytes())
    os.chmod(temp, target.stat().st_mode & 0o777)
    os.replace(temp, target)
    print(name, target, sha(target.read_bytes()))
