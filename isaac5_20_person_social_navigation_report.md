# Isaac5 1--20 person social navigation implementation report

## Final result: PASS_WITH_LIMITATIONS

The backend now runs a configurable 1--20 coloured animated crowd through one
stateful movement authority and has real Isaac5 evidence through Stage F.
Short-window crowd, GUI, perception and the headless fixed-goal navigation gate
passed. Two limits remain: the 20-person RTX GUI cannot sustain the formal
wall-clock LiDAR rate, and the current shared-route topology fails long-term
multi-seed stability. Neither limitation is represented as a full PASS.

## Implemented control path

`PhysX-cleared open patrol -> ping-pong cursor -> desired route velocity +
human/robot social terms + route-relative closest-approach prediction +
deterministic yield -> speed/acceleration/yaw limits -> stopping-distance
free-space qualification -> coloured UsdSkel visual + kinematic capsule ->
dual 2000-beam LiDAR / ROS`.

The implementation adapts the pure-Python Gazebo social kernel, Isaac6
pedestrian social/yield and steering contracts, and the authored lobby route
topology. It does not import an Isaac6 runtime extension or reopen an Isaac6
asset path. Capsules are LiDAR/occupancy and center-distance proxies, not human
mesh contact truth or articulated-body dynamics.

Important fixes after the earlier BLOCKED handoff:

- open polylines now reverse at endpoints instead of requesting an unvalidated
  last-to-first segment;
- diagonal avoidance uses each walker's route-relative right side rather than
  the world Y axis;
- the controller checks the full bounded stopping point before committing a
  lateral step and uses bounded braking before any emergency hold;
- one deterministic yielder resolves predicted narrow-corridor conflicts;
- acceleration, yaw rate and emergency holds are formal runtime checks;
- stall ratio, average speed, avoidance-state counts and personal-space
  violation ratio are recorded;
- optional log tags preserve prior successful logs.
- the route cursor now accepts a waypoint that was reached or geometrically
  passed and acknowledges a commanded lookahead checkpoint, copied from the
  successful Isaac6 `PatrolPolylineCursor` contract.  This removes the sharp
  corner orbit in which a person turned toward the next point but could never
  satisfy the previous point's 0.38 m radius.

### Runtime gait correction after visual review

The earlier animation gate only compared two authored skeleton samples.  That
proved that retargeting produced a non-static clip, but did not prove that a
person kept changing pose in the live viewport.  It also pre-authored repeated
time samples for the whole requested run, so a 600-second, 20-person preview
spent minutes and several GiB building duration-proportional animation data.

`RuntimeGaitDriver` now caches one retargeted 80-pose cycle, removes the global
timeline samples and selects the live default skeleton pose from each agent's
actual cumulative travel distance.  Slower movement therefore slows the gait,
a stopped agent stops advancing it, initial phases remain independent, and the
animation memory no longer grows with requested duration.  This follows the
successful Isaac6 division of responsibility--the movement agent owns speed and
the character animation follows that motion--without importing IRA or an
Isaac6 runtime extension.

The new gate requires every visible character to use at least three distinct
runtime poses and records its apply/change counts, template delta and remaining
time samples.  A real 60-second RTX GUI run changed all 20 characters through
all 80 cached poses.  Frame samples approximately two seconds apart visibly
show both root displacement and changed arm/leg poses.  This fixes the reported
"fixed action while sliding" defect; it does not claim an idle/walk blend tree
or foot-contact IK.

### Arena/HuNav comparison

The local Arena tree does contain multi-person scenarios.  For example,
`highly_social.json` defines ten dynamic people with 24 destinations each, and
the legacy `obs20.json` defines twenty.  Arena's HuNav path is a useful planning
reference: persistent per-agent goals and behavior state are updated at 10 Hz,
with Behavior Trees selecting actions and lightSFM producing social motion.

Its visual path is not a better replacement for the Isaac characters: the
checked Gazebo actor wrapper currently forces a temporary `walk.dae`, while the
older Arena actor model uses a simple timed leg alternation.  Therefore this
port reuses Arena/Isaac6 ideas at the goal, route-phase and social-state layers,
but keeps NVIDIA-retargeted Isaac Character skeletons for visible gait.  Arena
coordinates are also not copied into the engineering lobby; candidates must be
routed through this lobby's inflated free-space grid and then pass actual PhysX
clearance.

Three bounded route/deadlock experiments were retained rather than promoted:

1. The Isaac6 inflated-grid generator produced 20 routes with 2,129 points at
   a 1 m maximum edge.  Greedy map-wide start phasing raised the offline minimum
   center distance to 0.566 m, but turning density left many walkers stalled.
2. Merging only mutually visible safe edges up to 5 m reduced the route set to
   621 points and passed the same 79-box/0.55 m static clearance audit, but did
   not remove shared-corridor conflicts.
3. Persistent pairwise yield hysteresis, adapted from Isaac6, increased the
   number of long-stalled walkers because one person could simultaneously be
   the yielder in several intersecting conflicts.  That experimental change
   was reverted; it is not in the delivered controller.

The evidence therefore calls for segment-level corridor reservation or a
conflict-aware route allocator, not another Social Force weight adjustment.

## Verified evidence

| Stage | Result | Evidence |
| --- | --- | --- |
| A, pure Python | PASS | 7 tests: head-on, diagonal, crossing, robot avoidance, open-route ping-pong, limits/free-space/recovery, sharp-corner lookahead progress |
| B/C regression | PASS | prior 2-person and 8-person Isaac5 headless runtime retained |
| D, 20-person headless | PASS | 20/20 moved; 28.0 x 16.87 m span; min center distance 0.683 m; 15 Hz sim schedule; zero emergency holds |
| D, 20-person RTX GUI | PASS | 1280 x 720 screenshot; min center distance 0.680 m; clean teardown |
| E, DR-SPAAM/tracker | PASS | 115 detection frames, 114 track frames, 82 nonempty, 8 IDs, 44 confirmed; GT tracks suppressed |
| F, headless fixed goal | PASS | scans 13.68 Hz wall clock, inference 14.01 Hz, goal 0.320 m, 90 post-goal zeros, min human-human 0.526 m |
| F, RTX GUI fixed goal | PASS_WITH_LIMITATIONS | goal 0.323 m and 61 post-goal zeros passed; scans only 11.52 Hz at 5 Hz render request |
| G, three-seed 120 s preflight | FAIL | min distances 0.208/0.401/0.376 m and max stall ratios 0.95--0.98; Isaac long runs were not started |
| Live gait, one person headless | PASS | one 80-pose cycle; 179 live pose changes in 6 s; zero remaining timed samples |
| Live gait, 20-person headless | GAIT_PASS / CROWD_FAIL | all 20 changed pose; overall gate still exposed sustained shared-route stalls |
| Live gait, 20-person RTX GUI, 60 s | GAIT_PASS / CROWD_FAIL | all 20 used 80 poses and moved; min center distance 0.257 m and max stall ratio 0.90 keep the overall gate failed |
| Gait + cursor, 20-person headless, 8 s | GAIT/CURSOR_PASS / CROWD_FAIL | all 20 moved; min center distance 0.634 m; 15 Hz LiDAR; overall failure only sustained-stall ratio |
| Gait + cursor, 20-person RTX GUI, 30 s | GAIT/CURSOR_PASS / CROWD_FAIL | all 20 changed pose and moved; correct screenshot; shared route 13/20 still reaches 0.257 m |

Primary artifacts:

- `isaac_sim/backends/isaac5/generated/social_crowd_20260913/stage_d_20person_headless_postfix1.log`
- `isaac_sim/backends/isaac5/generated/social_crowd_20260913/stage_d_20person_gui.log`
- `isaac_sim/backends/isaac5/generated/social_crowd_20260913/stage_d_20person_gui.png`
- `isaac_sim/backends/isaac5/generated/social_crowd_20260913/stage_e_gate72_tracking_retry1_orchestrator.log`
- `isaac_sim/backends/isaac5/generated/gate72_tracking_monitor_social20_retry1_20260913.log`
- `isaac_sim/backends/isaac5/generated/social_crowd_20260913/stage_f_gate9f_social20_control30_yield_orchestrator.log`
- `isaac_sim/backends/isaac5/generated/gate9f_drlvo_monitor_social20_control30_yield_20260913.log`
- `isaac_sim/backends/isaac5/generated/social_crowd_20260913/stage_f_gate9g_social20_final_orchestrator.log`
- `isaac_sim/backends/isaac5/generated/gate9g_drlvo_monitor_social20_final_20260913.log`
- `isaac_sim/backends/isaac5/generated/social_crowd_20260913/stage_f_gate9g_social20_final.png`
- `isaac_sim/backends/isaac5/generated/gait_runtime_single_headless_20260913.log`
- `isaac_sim/backends/isaac5/generated/gait_runtime_crowd20_headless_20260913.log`
- `isaac_sim/backends/isaac5/generated/gait_runtime_crowd20_gui_20260913.log`
- `isaac_sim/backends/isaac5/generated/isaac_sim/backends/isaac5/generated/gait_runtime_crowd20_gui_20260913.png`
- `isaac_sim/backends/isaac5/generated/gait_cursor_crowd20_headless_20260913.log`
- `isaac_sim/backends/isaac5/generated/gait_cursor_crowd20_gui_20260913.log`
- `isaac_sim/backends/isaac5/generated/gait_cursor_crowd20_gui_20260913.png`
- `isaac_sim/backends/isaac5/generated/arena_route_audit_20260913/grid_spread20_seed7_max5.yaml`

The Stage E and F ROS graphs show `/scan_01` and `/scan_02` feeding the merger,
DR-SPAAM, tracker and DRL-VO. Crowd invocations use `--suppress-tracks`; their
results record zero GT track messages. DR-SPAAM non-empty detections are runtime
facts, not an accuracy claim.

## Remaining work

Long-term completion requires replacing shared/antiparallel patrol segments
with the project-owned inflated-grid/A* route generator's per-person spread
goals and route phases, followed by persistent conflict-state handling where
paths still share a narrow corridor.  The 60-second GUI evidence identifies
`person_13`/`person_20` as the closest pair and shows that people 13--20 account
for most prolonged stalls.  After that, rerun three 2-minute seeds and one
5--10 minute run. The GUI performance path also needs profiling/batching of
skeleton/root/capsule updates or a stronger runtime; physics, beam count and
the 15 Hz sensor contract must not be reduced.
