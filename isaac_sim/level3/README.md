# Standalone Isaac Level 3 — existing map + ground truth + Nav2 MPPI Omni

This directory is deliberately independent of SLAM, AMCL, Arena, HuNav,
evaluation, and RViz. It uses only system ROS 2 Humble Nav2 1.1.20 plus the
existing `workspaces/ros2_ws` bridge/dual-scan package.

## Frozen coordinate contract

The reproducible offline fit is recorded in
`reports/map_alignment.json` and visualized in
`reports/map_alignment_overlay.png`.

```text
TF direction: parent map -> child odom
p_map = R(0.142446610) * p_odom + (0.051603052, -1.537786930)

T_map_odom:
  x   =  0.051603052 m
  y   = -1.537786930 m
  yaw =  0.142446610 rad

default Isaac/odom spawn (2, 2, 0) in map:
  x   = 1.747415569 m
  y   = 0.725887054 m
  yaw = 0.142446610 rad
```

Alignment acceptance:

```text
MAP_ALIGNMENT=PASS
median residual                = 0.043243498 m
P90 residual                   = 0.174033074 m
worst west/center/east P90     = 0.193118541 m
absolute historical-map outlier = 0.975728829 m
```

The absolute outlier means that this historical SLAM-style raster is not an
exact CAD map. Both global and local costmaps therefore retain live
`/scan_merged` obstacle marking and clearing. The first runtime goal remains in
the well-aligned, high-clearance spawn region.

## Architecture

```text
map_server(existing gazebo_eng_lobby.yaml)
  |
map -- fixed calibrated ground truth --> odom
  |                                      |
  |                             Isaac UDP bridge
  |                                      |
  +------------------------------- base_link
                                      |  |
                              base_scan_01 base_scan_02

NavFn global planner -> MPPI Omni -> cmd_vel_nav
                                      |
                              velocity_smoother
                                      |
                                   /cmd_vel
                                      |
                              Isaac UDP bridge
```

`bt_navigator` provides `/navigate_to_pose` and natively consumes standard
map-frame `/goal_pose` messages. No second adapter is launched, so one
`/goal_pose` cannot create duplicate action goals. The no-RViz acceptance
clients call the actions directly.

`odom -> base_link` and the two sensor static transforms remain owned by the
existing bridge. This bringup owns exactly one transform: `map -> odom`.

Known old fixed-map demo launch files publish identity `map -> odom`; do not run
them alongside this stack. `start_standalone_level3.sh` refuses known duplicate
Nav2/localization nodes.

## Baseline parameters

- Controller: system Nav2 1.1.20 MPPI, `motion_model: Omni`.
- 20 Hz controller, `model_dt: 0.05`.
- Velocity limits: `vx [-0.25, 0.45]`, `vy [-0.30, 0.30]`, `wz ±1.0`.
- Acceleration limits are implemented by velocity smoother. Humble MPPI does
  not consume the Arena-style `ax_max/ay_max/az_max` keys.
- Footprint: `0.70 x 0.56 m` rectangle, derived from the current Isaac
  `0.625 x 0.487 m` collision proxy plus about 3.7 cm per side.
- Global costmap: `map`, static + merged-scan obstacle + inflation, non-rolling.
- Local costmap: `odom`, merged-scan obstacle + inflation, 6 x 6 m rolling.
- Initial goal tolerance: 0.30 m and 0.35 rad.

Parameter provenance for the first runtime:

| Group | Value | Source / status |
|---|---|---|
| Frames | global `map`, local `odom`, robot `base_link` | Frozen bridge/TF contract |
| Sensor | only `/scan_merged`; `+inf` is valid clearing data | Existing merger and LaserScan contract |
| Footprint | `0.70 x 0.56 m` rectangle | Measured `0.625 x 0.487 m` Isaac collision proxy plus about 3.7 cm/side |
| MPPI speed | `vx [-0.25,0.45]`, `vy ±0.30`, `wz ±1.0` | Conservative first-run selection inside Isaac's `hypot(vx,vy)<=0.6`, `|wz|<=1.5` hard clamp |
| Smoother accel | `[0.80,0.70,1.50]`, matching negative decel | First-run engineering baseline; Isaac's kinematic bridge has no tighter acceleration contract |
| Obstacle/raytrace | `0.45..10 m` / `0.45..12 m`, height `2.0 m` | First-run indoor engineering baseline bounded below the 50 m scan contract |
| Inflation | radius `0.55 m`, scale `3.0` | First-run engineering baseline; validate live before tuning |
| Transform tolerance | `0.20 s` | First-run engineering baseline for the 10 Hz scan / 30 Hz telemetry chain |
| Costmap rate | local `10/5 Hz`, global `2/1 Hz` update/publish | First-run engineering baseline |
| Local window | `6 x 6 m`, `0.05 m` cells | First-run engineering baseline; resolution matches the existing raster |

Values marked first-run engineering baseline are intentionally frozen for the
first acceptance, but remain `RUNTIME_PENDING` for tuning. They are not claimed
as measured optima.

## Offline validation

Safe to run without Isaac:

```bash
cd /home/user/navigation_project/a_pipeline
bash isaac_sim/level3/offline_validate.sh
```

It must end with `OFFLINE_LEVEL3_PREP=PASS`. It only parses files, reruns the
CPU-light map fit, checks hashes/packages/plugins, and constructs the launch
description without executing it.

## First runtime — only after returning to the computer

Terminal 1, start the existing static custom scene:

```bash
cd /home/user/navigation_project/a_pipeline
ISAAC_SCENE=custom \
ISAAC_CUSTOM_SCENE_USD=/home/user/navigation_project/a_pipeline/isaac_sim/scenes/a_pipeline_eng_lobby.usda \
ISAAC_CUSTOM_SPAWN_X_M=2.0 \
ISAAC_CUSTOM_SPAWN_Y_M=2.0 \
ISAAC_CUSTOM_SPAWN_Z_M=0.01 \
ISAAC_ENABLE_PEOPLE=0 \
ISAAC_ROBOT_PHYSICS=1 \
ISAAC_ROBOT_COLLISION_PROTECTION=1 \
ISAAC_PEDESTRIAN_AVOIDANCE_MODE=off \
ISAAC_LIDAR_MODE=rtx \
ISAAC_RTX_LIDAR_PROFILE=rplidar_s2e \
ISAAC_LIDAR_RATE_HZ=10 \
ISAAC_LIDAR_SAMPLE_COUNT=360 \
  bash isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh --deterministic
```

Wait for `WAREHOUSE_PEOPLE_ROBOT_READY=`. Do not start teleop: Nav2 must be the
only `/cmd_vel` publisher. Keep Terminal 1 open and leave this foreground process
running for the entire Level 3 test.

Terminal 2, in a fresh shell that has never sourced `arena_ws`:

```bash
cd /home/user/navigation_project/a_pipeline
bash isaac_sim/level3/start_standalone_level3.sh
```

Keep Terminal 2 open too. Wait until its lifecycle manager prints `Managed nodes
are active` before running Terminal 3. Closing either of the first two terminals,
pressing Ctrl-C, or relying on a process started by a completed remote/IDE task
removes that part of the ROS graph. If preflight reports `RUNTIME_GRAPH_EMPTY=FAIL`
and nearly every contract fails, Terminal 1 and Terminal 2 are not both alive or
are not discoverable in the same DDS context; restart them in order rather than
changing Nav2 parameters.

Terminal 3, first check the passive runtime contracts, then submit the offline
checked 1.5 m clear-space goal:

```bash
cd /home/user/navigation_project/a_pipeline
bash isaac_sim/level3/check_runtime_preflight.sh &&
  bash isaac_sim/level3/send_first_goal.sh
```

The default goal is map `(3.232223007, 0.938835104, 0.142446610)`. Its complete
straight segment has at least 1.60 m clearance in the static raster before
footprint/inflation, so it is the minimum-variable first Level 3 test.

Expected markers:

```text
LEVEL3_RUNTIME_PREFLIGHT=PASS
LEVEL3_NAVIGATE_TO_POSE=PASS
LEVEL3_COLLISION_RESULT=PASS
```

Goal reports are written under `reports/runtime/`. After the goal, close Isaac
normally from its window and wait for `WAREHOUSE_PEOPLE_ROBOT_RESULT=...` to be
written; do not use Ctrl-C for this evidence because the launcher's interrupt
trap terminates the process group before the final RESULT. Then run:

```bash
python3 isaac_sim/level3/tools/check_isaac_collision_result.py \
  isaac_sim/scripts/logs/<custom-run>.log \
  --goal-result isaac_sim/level3/reports/runtime/<goal-result>.json
```

## Deferred tests after the first goal passes

Reset/restart at the default spawn before each test.

Static-obstacle route, whose direct segment is blocked but an inflated-map path
exists:

```bash
bash isaac_sim/level3/send_obstacle_goal.sh
```

Its checker subscribes to `/plan` and requires the successful route to be at
least 2% longer than the start-to-goal chord, in addition to the ordinary goal
arrival checks. The selected map goal is `(11.075, 6.425, 0.142446610)`;
offline conservative-grid analysis found a 15.44 m route versus a 10.93 m
chord, while the chord itself crosses occupied raster cells.

Dedicated MPPI Omni proof: a 0.8 m path along robot-local +Y while holding yaw:

```bash
bash isaac_sim/level3/send_omni_test.sh
```

The Omni checker requires `|linear.y| > 0.05 m/s`, at least 0.5 m lateral
motion, yaw change no more than 0.25 rad, action success, and final zero speed.

## Runtime status

All motion-dependent facts remain `RUNTIME_PENDING` until the above commands
are deliberately run with Isaac:

- live map/scan/TF/costmap overlay;
- lifecycle activation;
- NavigateToPose motion and arrival;
- static-obstacle avoidance;
- nonzero MPPI `linear.y` in custom Isaac;
- collision-free Isaac RESULT.
