# Isaac Sim 5.1 versioned navigation backend

This directory is an independent copy of the native Isaac Sim 5.1 baseline.
The Isaac Sim 6.0.1 launcher and source tree remain outside this backend and
are not modified by these entry points.

## Current scope

The backend now contains the ordered Gate 0--9 migration path for the active
engineering lobby:

- canonical Mecanum730 visual plus a bounded dynamic collision proxy;
- canonical authored upright arm visual follows the dynamic base with arm
  dynamics disabled;
- active lobby plus a configurable 1--20 backend-owned textured Character
  adapter, NVIDIA-retargeted UsdSkel animation and kinematic capsules;
- ROS 2 Humble `/clock`, `/odom`, TF, dual scans and typed pedestrian data;
- two synchronized 2000-beam, 0.5--50 m PhysX scene-query scans at 15 Hz;
- DR-SPAAM, pedestrian tracking, S3-Net/SemanticCNN and base DRL-VO using the
  existing Isaac 6/project model code and checkpoints;
- explicit scan/track freshness checks and a front-obstacle safety veto;
- a bounded closed-loop smoke and one fixed odom-goal run.

The backend remains isolated from the Isaac 6 launcher, active `arena_ws`,
installed Isaac trees and checkpoints. Runtime artifacts stay below
`generated/`. People assets are a local closure at `assets/people/`: eight
Character USDs and their BaseColor/Normal/ORM textures, one walk clip, and the
81-joint source rig. It has no symlink back to an Isaac6 path. See its
`asset_manifest.json`, `SHA256SUMS`, `SOURCE.md`, copied license notice and
generated dependency audit. The license record establishes local provenance,
not a redistribution grant.

## Entry points

For a pedestrian-only visible preview, use the Arena scenario adapter. It
loads identities, starts, waypoints and `waypoint_mode=1` from
`config/arena_eng_lobby_pedestrians.json`; no DRL-VO policy or robot command
source is started. The Isaac 5 adapter remains the sole pose writer for the
Character visual and its kinematic capsule.

```bash
isaac_sim/backends/isaac5/launch/run_arena_pedestrian_scene.sh --count 3
isaac_sim/backends/isaac5/launch/run_arena_pedestrian_scene.sh --count 10
isaac_sim/backends/isaac5/launch/run_arena_pedestrian_scene.sh --count 20
isaac_sim/backends/isaac5/launch/run_arena_pedestrian_scene.sh --count 20 --robot --command-topic /cmd_vel
```

This is Arena scenario/waypoint semantics on the matched engineering-lobby
geometry. It does not claim that the full Arena Task Generator service chain
or Gazebo HuNav dynamics run inside Isaac 5.

With `--robot`, the scene adds the local Mecanum730/XMS5 visual and its bounded
dynamic PhysX collision proxy. The initial teleop contract consumes signed
`linear.x` and `angular.z` on `/cmd_vel` (forward, reverse and turn); lateral
Mecanum `linear.y` is intentionally not claimed yet.

`validate_crowd.sh` accepts `--pedestrian-count 1..20`,
`--pedestrian-seed`, `--pedestrian-speed` or min/max speed bounds, and
`--social-mode off|social`, plus independent crowd-update and GUI-render
rates.  The Gate9 launch path defaults to
`ISAAC5_GATE9_PEDESTRIAN_COUNT=20`, seed 7 and social mode; tests may select
2 or 8. Its 30 Hz crowd update does not change 60 Hz physics or the dual
2000-beam/15 Hz LiDAR contract. `runtime/crowd_social_control.py` is the sole movement writer for each
coloured visual and matching kinematic capsule. It adapts the project's pure
Gazebo social-force and Isaac6 patrol-cursor semantics without importing an
Isaac6 extension or asset at runtime.

Visible walking is distance-coupled rather than a duration-sized global USD
timeline.  Each character caches one retargeted cycle and advances its live
skeleton pose from the distance integrated by the sole crowd controller.  A
slow or stopped person therefore no longer continues a fixed-speed clip, and a
long preview no longer pre-bakes one animation sample stream per run second.
The runtime gate checks pose changes for every character; it does not claim an
idle/walk blend tree or foot-contact IK.

The 20-person result is **PASS_WITH_LIMITATIONS**. Stage D passed in headless
and the real RTX GUI; Stage E passed with GT tracks suppressed; the headless
fixed-goal Gate9F passed with 13.68 Hz wall-clock dual scans and 14.01 Hz
DRL-VO inference. The 20-person GUI Gate9G reached and stopped safely but its
wall-clock scans reached only 11.52 Hz even at a 5 Hz render request, so it is
not a formal Gate9G PASS. Three-seed 120-second pure-controller preflight also
found repeated shared-corridor deadlock and overlap, so no long-duration social
stability claim is made. See `isaac5_20_person_social_navigation_report.md`.

```bash
isaac_sim/backends/isaac5/launch/validate_robot.sh --headless --steps 120
isaac_sim/backends/isaac5/launch/validate_crowd_ros.sh --headless --duration 8
isaac_sim/backends/isaac5/launch/validate_gate73_semantic.sh
isaac_sim/backends/isaac5/launch/validate_gate81_drlvo_shadow.sh
isaac_sim/backends/isaac5/launch/validate_gate9_closed_loop_smoke.sh
isaac_sim/backends/isaac5/launch/validate_gate9_fixed_goal.sh
isaac_sim/backends/isaac5/launch/validate_gate9_fixed_goal_gui.sh
isaac_sim/backends/isaac5/launch/run_production_demo.sh --workflow drlvo
```

The final command runs the same bounded fixed-goal chain in the real Isaac 5
RTX GUI and writes `generated/gate9g_fixed_goal_gui.png`; for 20 people it is
currently a visible functional preview with the wall-rate limitation above,
not the default headless reproducibility gate.

`ISAAC_SIM_5_ROOT` is the only Isaac installation selector and defaults to
`/home/user/isaacsim/5.1.0`. The launchers verify its `VERSION`, embedded
Python 3.11, bundled Humble bridge library directory, and canonical robot
file. They clear common Isaac/host-Python inheritance variables and do not
source `/opt/ros/humble` into the Kit process.

The optional compatibility launcher is:

```bash
isaac_sim/backends/isaac5/launch/start_isaac_5_1_595_compat.sh
```

It uses the copied, checksum-verified layer in `compat/vulkan_595/` and only
sets process-local Vulkan environment variables. It is a diagnostic path, not
a claim that the driver/runtime combination is stable.

Run the numbered launchers in order. They use isolated ROS domains and refuse
to treat a static check as Isaac/ROS runtime proof. Only one Isaac instance may
run at a time.

## Default project entrypoints

The following general project entrypoints now select `ISAAC_BACKEND=isaac5` by
default and dispatch to the real Isaac5 crowd/perception/policy chain, never a
static probe: DRL-VO, DR-SPAAM smoke/tracking/stress, pedestrian validation
matrix, single-person benchmark, and the v7 SemanticCNN demo. The ROS
pedestrian-perception visualization launch also defaults to the Isaac5 custom
lobby crowd. Set `ISAAC_BACKEND=isaac6` explicitly only to retain the original
legacy/regression behavior of those scripts. Launchers explicitly named
`isaac_6_0` remain unchanged.

## Evidence boundary

`READY` and `RESULT` lines are machine-readable. Gate 9 PASS requires physical
odom motion, bounded command delivery, 2000-beam/15 Hz scans, model inference,
fixed upright arm pose, map-wide crowd checks, goal stop and clean teardown. Static checks
and Python tests do not establish Isaac runtime or ROS graph success.

Known non-equivalences remain: the Isaac5-owned local character bundle uses
NVIDIA retarget core, but its route/capsule adapter is not Isaac6 IRA
1.6/BehaviorAgent; LiDAR has no RTX intensity;
pedestrian capsule geometry is not human-mesh contact truth; no Nav2, SLAM,
training or long multi-seed benchmark is claimed. DR-SPAAM's current non-empty
runtime gate uses confidence 0.20, not 0.95. See
`isaac5_complete_scene_migration_report.md` at the repository root.
