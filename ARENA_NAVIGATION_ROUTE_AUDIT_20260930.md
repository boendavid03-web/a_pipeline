# Arena on Isaac route audit — 2026-09-30

Read-only source/runtime audit. No simulator was launched and no formal configuration was changed. A pre-existing `nav_fixed4_hunav_20260930` run on ROS domain 194 was found and stopped by signalling its dedicated PGID 851032; its 290 MB bag and log remain. Other domains and an unrelated asset extraction were not touched.

## Decision

**Choose A, with a baseline-freeze gate:** keep the working adapter and current Isaac 5.1 runtime; first restore benchmark-affecting parameters to their proven upstream values in a separately reviewed change, then run the formal baseline. The adapter is capable of closed-loop navigation. An official-branch isolation smoke is useful research, but not a prerequisite for the next benchmark. No evidence ties a remaining failure to the RC build strongly enough to justify a version migration.

Do not count the current run set as a clean official DWB baseline. It contains diagnostic changes and early task-start failures. An `ABORTED` result is a legitimate algorithm outcome only after scenario, TF, sensor, map/costmap, controller, command, and actuation gates pass for that episode.

## Actual runtime identity

| Layer | Observed path / identity |
| --- | --- |
| Host entry | `/home/user/arena_isaac5_host_runtime/run_gui.bash`; `/home/user/arena_isaac5_host_runtime/setup.bash` |
| Host Arena | `isaac_sim/arena_ws` install; source under `isaac_sim/arena_ws/src/arena/arena-rosnav` |
| Simulation setup | `/home/user/arena_isaac5_host_overlay_ws/install/arena_simulation_setup`; installed `configs/nav2/nav2.yaml` links to overlay source |
| Isaac process | `/home/user/isaacsim/5.1.0/python.sh` → `/home/user/isaacsim/5.1.0/kit/python/bin/python3`; VERSION `5.1.0-rc.19+release.26219.9c81211b.gl`; Python 3.11.13; full Kit app declares 5.1.0. Exact Kit SDK build was not established locally. |
| Isaac adapter | `isaac_sim/arena_ws/src/arena/isaac`, detached HEAD `a4beefe0203ec8a75d65d0ab70496b5e2c400605` (2025-07-10), three dirty files. Live Isaac child imports `/home/user/arena_isaac5_py311_factory/build_ws/humble/isaac_sim_ros_ws/install/ros2isaacsim/lib/python3.11/site-packages/ros2isaacsim/run_isaacsim.py`; factory source and install differ (`c07a7f...` vs `b173a0...`). |
| Official comparison | Local remote tracking ref `origin/arena5-isaac5.1.0` = `16b8e3416517d8c3dc1b5038df4fe11b9a6df46c` (2026-06-20). This is a source comparison, not the live runtime. |

## Component evidence

`Verified` refers to the cited run only. Bag `analysis.txt` files were produced from the raw bags; this audit also inspected the log/error lines and process paths. Success is action status 4, abort status 6. Early startup goals are separate UUIDs from the subsequently executed action.

| Component | Verified | Current evidence | Modified? |
| --- | --- | --- | --- |
| Arena Task Generator | Scenario dispatch, goals, repeated action lifecycle | `nav_default_hunav_numeric_fix` bag: two same-position goals, second action runs; `nav_fixed3_hunav` likewise | Yes: Isaac-specific retry guard; overlay scenario handoff |
| Isaac scene | `map_empty` ground and task shelves in fixed runs | Fixed-4 log records five shelf prims, each 14 geometry/collider paths | Yes: shelf USD/collision, world USD path, obstacle import |
| HuNav / pedestrian pose | 1 and 3-person pose delivery | `ARENA_HUNAV_ISAAC_RUNTIME_20260930.md` paired CSV; default/fixed bags contain human states | Yes: asset, service window, numeric prim, force preservation |
| Jackal spawn / physics | Spawn and motion | Raw-bag odom path from scenario start to success: default 28.889 m; long smoke 29.040 m | Yes: URDF/import/drive mapping and wheel multiplier application |
| odom / TF | Present and usable in successful episodes | Default bag 15,736 odom and 44,635 TF messages; short/long have both | Yes: old adapter compatibility and transform wiring |
| LiDAR | 640-beam scans with finite returns | Default bag 593 scans; fixed-2 574 scans; scan frame `jackal/lidar_link` | Yes: flat scan companion to RTX sensor |
| Local / global costmap | Updating in default, long, fixed-2/3 | Default 1,542 / 306 messages and occupied local cells; fixed-1 early runs report 0 occupied local cells | Yes: global extent changed; startup needs episode gate |
| NavFn | Real paths in default/fixed-2/3 | Default 577 map-frame paths; fixed-2 601; fixed-3 621 | Config retained; initial off-costmap failures occur before retry |
| DWB | Runs and produces control in default/fixed-2/3 | Default 2,877 nonzero `cmd_vel_nav`; fixed-2 2,995; fixed-3 3,098 | DWB YAML restored numerically; generic controller frequency/progress changed |
| `cmd_vel` / actuation | Command and robot response | Default 5,773 filtered commands and odom movement | Yes: drive/control graph compatibility |
| Short navigation | `SUCCEEDED` | `nav_short_smoke_debug_retry_control` bag: status 4, 2.285 m travel | Diagnostic task, not benchmark |
| Long navigation | `SUCCEEDED` | `nav_long_smoke_wheel_multiplier` bag: status 4, 29.040 m travel | Diagnostic progress/wheel state |
| Original `default.json` + 3 humans | `SUCCEEDED` | `nav_default_hunav_numeric_fix` bag: status 4, 28.889 m travel, final error 0.104 m, three human streams | Current diagnostic config; not frozen baseline |
| `1.json` | No valid baseline result | `nav_fixed1_hunav_control_limit_retry` bag: robot near start, 0 occupied local cells; log says robot off global costmap and later failed progress | Infrastructure/startup and control confounds |
| `2.json` | Executed, `ABORTED` | `nav_fixed2_hunav` bag: 601 paths, 2,995 nonzero controller commands, final XY error 0.184 m; log later says failed progress | Near-goal DWB result candidate, subject to exact config and physical clearance gate |
| `3.json` | `SUCCEEDED` in captured run | `nav_fixed3_hunav` bag: status 4, 24.258 m travel, 621 paths, final error about 0.21 m | Current diagnostic config; startup goal initially failed off costmap |
| `4.json` | Interrupted, exclude from benchmark | Dedicated run was active when audit began; early log showed NavFn failure to goal (30,1.25); bag preserved | No classification from incomplete run |

Thus the basic Arena + Isaac navigation pipeline **has closed the loop**. Requiring DWB to succeed at every fixed task would confuse algorithm outcome with infrastructure validity. `1.json` still has infrastructure confounds, while `2.json` is a plausible DWB failure. The current `3.json` success supersedes the prior “pending” note.

The historical `analysis.txt` path lengths for long/default/3 were undercounts: its `DISTANCE_TO_SUCCESS` code filters out odom samples once x falls below 20 m. This audit reread the raw bags and measured all odom samples from the first pose near each scenario start through the action result. The maximum adjacent odom step in these measured segments was 0.004, 0.011, and 0.012 m respectively. Initial origin-to-spawn jumps were excluded.

## Patch inventory and baseline contamination

| Change | Class | Evidence / baseline treatment |
| --- | --- | --- |
| Python 3.11 Isaac child separated from Humble Python 3.10; service window; Isaac 5.1 URDF API import | A, compatibility | Required by ABI/API and 10 Hz HuNav service path; keep with tests |
| HuNav pose/goal-force mapping, numeric pedestrian prim path, available character asset, `world.reset`, changed response.pose API | A, fidelity | Restores specified pedestrian state and Isaac lifecycle; keep, but log asset mapping |
| Shelf USD/collider import, Arena world USD reference | A, scene fidelity | Required for matching obstacles; retain only verified assets and map registration |
| Flat 640-ray LaserScan and RTX point cloud pair | A, sensor compatibility | Arena Jackal `urdf/jackal.gazebo` declares 640 horizontal × 16 vertical, 10 Hz, 0.08–12 m; official 5.1 code also creates separate flat scan prim. Projection semantics still need a strict sensor-equivalence check. |
| Wheel `0.36 × 1.5` in local control graph | A candidate, robot fidelity | Both Arena source and overlay Jackal `control.yaml` say `wheel_separation=0.36`, multiplier `1.5`; factory adapter applies multiplier. Keep only after wheel response/odom calibration confirms upstream semantics. |
| Isaac goal retry guard | A candidate, action lifecycle | Task Generator publishes every 3 s until success/60 s; local guard suppresses repeats while an Isaac action is active, allowing retries after a startup abort. The earlier bag shows multiple initial UUIDs. Keep with explicit episode/action semantics check. |
| Generic `movement_time_allowance` `10.0 → 30.0` | B, benchmark behavior | **Still live** in overlay `configs/nav2/nav2.yaml:82`; upstream source is 10.0. Restore before formal baseline. |
| Generic controller frequency `1.0 → 10.0 Hz` | B, benchmark behavior / unresolved compatibility | **Still live** in overlay `configs/nav2/nav2.yaml:67`; upstream 1.0. It changes DWB sampling/control behavior. Do not silently call this official baseline; decide via a bounded control-fidelity check, then record the chosen benchmark contract. |
| Global costmap `20×20`, origin `(-10,-10)` → `30×23`, origin `(0.5,0.5)` | A candidate with benchmark effect | **Still live** in overlay `configs/nav2/nav2.yaml:181-185`; original settings excluded fixed task starts near x=24–30 and produced off-costmap errors. This is map coverage fidelity, but affects planning; record as a declared Isaac compatibility deviation. Verify all task footprints remain inside and avoid changing obstacle costs. |
| DWB `min_speed_theta=0`, `vx_samples=20`, `vtheta_samples=20`, `sim_time=1.7`, critics and goal tolerances | Upstream at present | Overlay DWB file differs from Arena source only by final newline. Historical theta/vtheta/goal experiments are run-specific, not the current DWB YAML. |
| Upstream scenario positions and HuNav behavior | Upstream at present | Original overlay `default/1..5.json` were byte identical in the prior plan; no current scenario diff was found in this audit. Per-run diagnostic JSON and social-force experiments must stay outside formal suite. |

The exact generic Nav2 diff is saved separately as `ARENA_NAVIGATION_BASELINE_DIFF_20260930.patch`. No live file has been restored by this audit.

## Current versus official 5.1 adapter

| Area | Current local approach | Official 5.1 branch | Overlap / migration cost |
| --- | --- | --- | --- |
| Robot import/control | Old `ros2isaacsim`, Python differential graph consuming velocity; local wheel multiplier fix | New `arena_isaac`, URDF parser plus external `ros2_control` `JointStateTopicSystem` and topic bridge to Isaac articulation | Replaces architecture; robot work high |
| Odom/TF | Old localization/transform graph with compatibility patches | New odom/TF graphs and robot-state-publisher split | Similar goal, different implementation |
| LiDAR | Old RTX helper plus local flat scan | Rewritten sensor module creates 3D points prim and distinct 1-line scan prim, ROS 2 LaserScan publisher | **Same idea; repeated local work** |
| Reset / services | Old `Person`, `MovePed`, `MovePrim`, `GetPrimAttributes`, custom service window | New `arena_people_msgs`, `MovePedestrians`, `SpawnPrims`, `GetPrims`, `ResetWorld`, entity lifecycle; `world.reset()` at start | Similar functionality; incompatible service contract |
| Obstacles | Local shelf USD/collision repair, old import service | New walls/floors/prim/URDF services | Partial conceptual overlap; asset-specific repair still needed |
| Task Generator | Existing Humble client expects old `isaacsim_msgs` services | Official branch exposes new services/messages | Cannot switch adapter alone; host task interface needs matching port |

The local checkout is an old 2025 architecture used on Isaac 5.1 with patches. Official 2026 branch has redesigned several areas we patched, especially LiDAR, URDF, control, service lifecycle and pedestrians. Its code is not a drop-in fix for the current host overlay. The branch history says “working 5.1.0” and later “full ros2_control”; no inspected manifest pins a specific RC or GA build, and there is no current-runtime smoke evidence for it here. Current runtime is RC19; the factory repository is at a later 5.1 release commit, which does not itself identify the executable build.

## Route comparison

| Route | Expected patches | HuNav work | Robot work | LiDAR work | Scene work | Risk | Estimated disruption |
| --- | --- | --- | --- | --- | --- | --- | --- |
| A: current adapter + RC19 | Low to medium: freeze config, verify 1-start and scene gates | Low | Low | Low | Low | Medium: local patches, diagnostic contamination | Low |
| B: official adapter + matching 5.1, isolated test | Medium to high: service/Task Generator interface port | Medium to high | High: ros2_control launch/config | Medium | Medium | Medium to high: architecture switch | High |
| C: new 5.1 GA build | Unknown until isolated comparison; ABI likely same 3.11 | Low to medium | Medium | Medium | Medium | Medium: untested binary/assets | High |
| C: 6.x | High: removed deprecated APIs and adapter port | High | High | High | High | High | High |

NVIDIA's 5.1 documentation supports Humble on Ubuntu 22.04 with Isaac Python 3.11, consistent with the current host/child split. The 5.1 release notes identify API/bridge changes but do not prove RC19 caused any observed benchmark failure. The official branch was updated after the 5.1 release; its exact binary pin is still unknown. No current evidence establishes Arena 5 plus official 6.x support or scene/HuNav compatibility, so 6.x is not a justified migration.

## Why this took time and next gate

Problems came from several sources: Arena's repeated goal publication and initial off-costmap startup; old adapter API/control/sensor and pedestrian gaps; Isaac 5.1 extension/API and Python ABI changes; necessary local USD, service and overlay compatibility; genuine DWB near-goal/progress behavior; and unnecessary theta/vtheta/progress tuning aimed at making fixed tasks succeed. The last category has contaminated historical comparisons but does not invalidate the proof of a closed pipeline.

Before formal measurement: review and save a single manifest of current source/install hashes; restore B-class values with a reversible diff; explicitly decide whether 10 Hz control and global map coverage are compatibility deviations and document them; verify `1.json` with a fresh start/costmap/actuation gate only if needed to identify its infrastructure fault. Do not optimize for `1.json` or `2.json` success. Pause 4/5, MPPI, learned methods, and full evaluation until the frozen DWB run contract exists.
