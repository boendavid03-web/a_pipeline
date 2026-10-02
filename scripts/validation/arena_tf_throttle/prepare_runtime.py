#!/usr/bin/env python3
"""Create an isolated Isaac isaac_utils package with the one-line TF fix."""
import hashlib
import shutil
from pathlib import Path

SOURCE_ROOT = Path('/home/user/arena_isaac5_py311_factory/build_ws/humble/isaac_sim_ros_ws/install/ros2isaacsim/lib/python3.11/site-packages')
DEST_ROOT = Path('/home/user/arena_tf_throttle_validation_20261001/runtime/site-packages')
EXPECTED = 'a1ba698b321c882226ad7b261c65e88ade709418908dea41b7a7e1212482ea5c'
CHILD_EXPECTED = 'b173a069e3603dc33797c8483ae9208b30e9e82b3229e67f2cf898afbdab1809'

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    source_tf = SOURCE_ROOT / 'isaac_utils/graphs/tf.py'
    child = SOURCE_ROOT / 'ros2isaacsim/run_isaacsim.py'
    assert sha(child) == CHILD_EXPECTED, f'Isaac child changed: {sha(child)}'
    source_code = source_tf.read_text()
    if sha(source_tf) != EXPECTED:
        # A concurrent edit made the shared installed file throttle=10 during
        # the first probe. Reconstruct the pinned original in memory only.
        assert source_code.count('    throttle: int = 10,\n') == 1
        source_code = source_code.replace('    throttle: int = 10,\n', '    throttle: int = 300,\n')
        assert hashlib.sha256(source_code.encode()).hexdigest() == EXPECTED, 'Unexpected shared TF change'
    for package in ('isaac_utils', 'ros2isaacsim'):
        dest = DEST_ROOT / package
        if dest.exists():
            shutil.rmtree(dest)
        shutil.copytree(SOURCE_ROOT / package, dest, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    target = DEST_ROOT / 'isaac_utils/graphs/tf.py'
    code = source_code
    old = '    throttle: int = 300,\n'
    assert code.count(old) == 1
    code = code.replace(old, '    throttle: int = 10,\n')
    # Read-only runtime evidence after graph creation; no change to graph inputs.
    code = code.replace('import os\n', 'import os\nimport hashlib\n', 1)
    code = code.replace('    return graph.execute(controller)\n', '''    created = graph.execute(controller)
    actual_period = og.Controller.attribute(f"{on_tick.path}.inputs:framePeriod").get()
    actual_hash = hashlib.sha256(open(__file__, 'rb').read()).hexdigest()
    print(f"ARENA_TF_RUNTIME file={__file__} sha256={actual_hash} graph={graph_path} framePeriod={actual_period} requested={throttle} created={created}", flush=True)
    return created
''', 1)
    target.write_text(code)
    print(f'PINNED_ORIGINAL_TF_SHA256={EXPECTED} shared_current_sha256={sha(source_tf)}')
    print(f'PATCHED_TF={target} sha256={sha(target)}')
    print(f'ISOLATED_CHILD={DEST_ROOT / "ros2isaacsim/run_isaacsim.py"} sha256={sha(DEST_ROOT / "ros2isaacsim/run_isaacsim.py")}')

if __name__ == '__main__':
    main()
