#!/usr/bin/env python3
"""Pin the actual scenario and frozen runtime inputs before one capture."""
import hashlib
import importlib.util
import json
import os
import shutil
import subprocess
from pathlib import Path

import yaml

repo = Path('[LOCAL_PATH]')
manifest = yaml.safe_load((repo / 'ARENA_ISAAC_FROZEN_BASELINE_MANIFEST_20261001.yaml').read_text())
startup = yaml.safe_load((repo / 'ARENA_STARTUP_GATE_20261002.yaml').read_text())
run = Path(os.environ['ARENA_RUN_DIR'])
scenario = Path(os.environ['ARENA_SCENARIO_FILE']).resolve(strict=True)
source = repo / 'isaac_sim/arena_ws/src/arena/simulation-setup/worlds/map_empty/scenarios/1.json'
live_expected = Path('[LOCAL_PATH]')
assert scenario == live_expected

paths = {
    'scenario_live': scenario,
    'scenario_source': source,
    'scenario_installed': Path('[LOCAL_PATH]'),
    'robot_manager_source': Path(startup['source']['authoritative_task_generator']) / 'task_generator/manager/robot_manager/robot_manager.py',
    'startup_patch': repo / startup['source']['reversible_patch'],
    'nav2': Path(startup['runtime']['authoritative_nav2_config']),
    'navfn': Path(manifest['nav2']['global_planner']['live']['path']),
    'dwb': Path(manifest['nav2']['local_controller']['live']['path']),
    'tf_isolated': Path('[LOCAL_PATH]'),
    'isaac_child': Path(manifest['current_adapter']['isaac_child_installed_module']['path']),
    'hunav_behavior': Path(manifest['hunav']['arena_bringup_default_behavior']['path']),
    'jackal_control': Path(manifest['scene_and_robot_assets']['jackal_control_yaml']['path']),
    'shelf_usd': Path(manifest['scene_and_robot_assets']['shelf_usd']['path']),
}
paths.update({f'map_{name}': Path(item['path']) for name, item in manifest['scenario_set']['map_files'].items()})
character_root = repo / 'isaac_sim/backends/isaac5/assets/people/characters/M_Medical_01'
for asset in sorted(character_root.rglob('*')):
    if asset.is_file():
        paths[f'human_character_{asset.relative_to(character_root)}'] = asset
motion_root = Path('[LOCAL_PATH]')
paths['human_biped_setup'] = motion_root / 'Biped_Setup.usda'
paths['human_biped_demo'] = motion_root / 'biped_demo/biped_demo_meters.usd'
for asset in sorted((motion_root / 'Animations').glob('*.usd')):
    paths[f'human_animation_{asset.name}'] = asset
hashes = {key: hashlib.sha256(path.read_bytes()).hexdigest() for key, path in paths.items()}
expected = {
    'scenario_live': manifest['scenario_set']['files']['1.json']['live_sha256'],
    'scenario_source': manifest['scenario_set']['files']['1.json']['source_sha256'],
    'scenario_installed': manifest['scenario_set']['files']['1.json']['live_sha256'],
    'robot_manager_source': startup['source']['robot_manager_sha256_after'],
    'startup_patch': startup['source']['reversible_patch_sha256'],
    'nav2': startup['runtime']['nav2_config_sha256'],
    'navfn': manifest['nav2']['global_planner']['live']['sha256'],
    'dwb': manifest['nav2']['local_controller']['live']['sha256'],
    'tf_isolated': startup['runtime']['tf_throttle_10_isolated_sha256'],
    'isaac_child': manifest['current_adapter']['isaac_child_installed_module']['sha256'],
    'hunav_behavior': manifest['hunav']['arena_bringup_default_behavior']['sha256'],
    'jackal_control': manifest['scene_and_robot_assets']['jackal_control_yaml']['sha256'],
    'shelf_usd': manifest['scene_and_robot_assets']['shelf_usd']['sha256'],
}
expected.update({f'map_{name}': item['sha256'] for name, item in manifest['scenario_set']['map_files'].items()})
drift = {key: {'actual': hashes[key], 'expected': value} for key, value in expected.items()
         if hashes[key] != value}
if os.environ['ARENA_SCENARIO_EXPECTED_SHA256'] != hashes['scenario_live']:
    drift['scenario_environment'] = os.environ['ARENA_SCENARIO_EXPECTED_SHA256']

scene = json.loads(scenario.read_text())
assert len(scene['robots']) == 1
assert len(scene['obstacles']['static']) == 3
assert len(scene['obstacles']['dynamic']) == 3
assert all(x['model'] == 'shelf' for x in scene['obstacles']['static'])
nav = yaml.safe_load(paths['nav2'].read_text())
ctrl = nav['controller_server']['ros__parameters']
assert ctrl['controller_frequency'] == 1.0
assert ctrl['progress_checker']['required_movement_radius'] == 0.5
assert ctrl['progress_checker']['movement_time_allowance'] == 10.0
assert ctrl['goal_checker']['xy_goal_tolerance'] == 0.25
assert ctrl['goal_checker']['yaw_goal_tolerance'] == 0.25
assert 'NavfnPlanner' in paths['navfn'].read_text()
assert 'DWBLocalPlanner' in paths['dwb'].read_text()
assert '10' in paths['tf_isolated'].read_text()

tg_loaded = Path(importlib.util.find_spec('task_generator.manager.robot_manager.robot_manager').origin)
assert tg_loaded.resolve() == paths['robot_manager_source'].resolve()
assert hashlib.sha256(tg_loaded.read_bytes()).hexdigest() == expected['robot_manager_source']
adapter_head = subprocess.check_output(['git', '-C', manifest['current_adapter']['repo'], 'rev-parse', 'HEAD'], text=True).strip()
assert adapter_head == manifest['current_adapter']['commit']
head = subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip()
dirty = subprocess.check_output(['git', '-C', str(repo), 'status', '--short'], text=True)
(run / 'git_status.txt').write_text(dirty)
(run / 'git_diff.patch').write_bytes(subprocess.check_output(['git', '-C', str(repo), 'diff', '--binary']))
(run / 'git_cached_diff.patch').write_bytes(subprocess.check_output(['git', '-C', str(repo), 'diff', '--cached', '--binary']))
snapshot = run / 'config_snapshot'
snapshot.mkdir()
for key in ('scenario_live', 'nav2', 'navfn', 'dwb', 'hunav_behavior', 'jackal_control'):
    shutil.copy2(paths[key], snapshot / f'{key}{paths[key].suffix}')

report = {
    'scenario_absolute_path': str(scenario), 'scenario_data': scene,
    'paths': {key: str(path) for key, path in paths.items()},
    'sha256': hashes, 'expected_sha256': expected, 'drift': drift,
    'root_head': head, 'adapter_head': adapter_head,
    'task_generator_actual_loaded_path': str(tg_loaded),
    'task_generator_actual_resolved_path': str(tg_loaded.resolve()),
    'domain': os.environ['ROS_DOMAIN_ID'],
    'source_random_seed': 'not exposed/fixed by this collector',
    'collection_limits': {'startup_wall_s': 300, 'episode_sim_s': 600,
                          'wall_protection_s': 1200, 'clock_stall_wall_s': 45,
                          'post_terminal_sim_s': 5, 'post_terminal_wall_s': 10,
                          'episode_start': 'first high-level NavigateToPose status'},
    'collection_limit_basis': 'collector rule; source Robot.TIMEOUT defaults to -1 (unbounded)',
}
print(json.dumps(report, indent=2))
if drift:
    raise SystemExit(2)
