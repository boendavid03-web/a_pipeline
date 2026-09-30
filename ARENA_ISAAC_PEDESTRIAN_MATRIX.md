# Arena + Isaac 5.1 pedestrian smoke matrix

Date: 2026-09-29

This is a pedestrian-import smoke matrix, not a final social-navigation
acceptance report.  Each native Isaac run was bounded by `timeout`; existing
Gazebo/HuNav processes were not touched.

## Runtime evidence

| world / scenario | ready people | movement evidence | result |
|---|---:|---|---|
| `map_empty/default.json` | 3 | waypoint batches for `D_test_1`, `2`, `3` | PASS (spawn + command path) |
| `map_empty/highly_social.json` | 10 | waypoint batches for all 10 | PASS (spawn + command path) |
| `map_empty/blocked_corridors.json` | 7 | batches for moving agent `20`; six no-waypoint agents remain stationary | PASS (stationary + moving import) |
| `factory/default.json` | 2 | waypoint batch for `1`, `2` | PASS (after world-root fallback) |
| `generated/default.json` | 3 | waypoint batch for `1`, `2`, `3` | PASS (after world-root fallback) |
| `house17/1.json` | 3 | waypoint batch for `1`, `2`, `3` | PASS (spawn + command path) |
| `hospital/default.json` | 17 | all 17 `ARENA_PEDESTRIAN_READY`; no batch observed before the 60-s bound | PARTIAL (spawn confirmed; motion needs a longer gate) |
| `ignc/default.json` | 3 | waypoint batch for `1`, `2`, `3` | PASS (after world-root fallback) |

The Isaac 5.1 kit logs for these runs contain no
`character_graph`/`set_variable`/`get_world_transform` error chain after the
shared `Biped_Setup.usda` was explicitly bound.  The old RTX-LiDAR render
product and PhysX tensor warnings remain and are tracked separately as robot
sensor/bridge compatibility work.

## Offline catalog coverage

`test_arena_source_scenarios.py` loads all 39 legacy JSON scenarios across
`map_empty`, `generated`, `factory`, `house17`, `hospital`, and `ignc`.
Empty scenes, `waypoint_mode=0/2`, legacy 2-D points, and repeated legacy
`id: 0` values are accepted with deterministic stationary/track-id behavior.

Command:

```bash
python3 -m pytest -q \
  isaac_sim/backends/isaac5/tests/test_arena_scenario.py \
  isaac_sim/backends/isaac5/tests/test_arena_source_scenarios.py
```

Result: `13 passed`.

## Remaining boundary

The matrix proves native Arena → Isaac People import and the existing
waypoint command path.  It does not yet prove 120-s crowd separation,
collision-free motion, robot-aware behavior, or HuNav/LightSFM integration;
those remain the next layer after the Isaac sensor/PhysX warnings are isolated.
