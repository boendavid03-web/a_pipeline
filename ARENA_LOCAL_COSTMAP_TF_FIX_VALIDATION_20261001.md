# Arena Isaac local costmap TF fix validation — 2026-10-01

## Decision

**VERIFIED:** Changing only the TF graph default `throttle: 300 → 10` restores the original `map_empty/default.json + 3 HuNav` scan → local TF MessageFilter → ObservationBuffer → VoxelLayer → occupied local costmap chain. The frozen short no-human route reached its goal on the second, post-initialization UUID. The full original default run retained a separate near-goal `Failed to make progress` abort. **FROZEN_BASELINE_READY: NO.**

The original scenario, HuNav behavior, NavFn, DWB, global costmap coverage, LiDAR, local costmap parameters, `controller_frequency=1.0 Hz`, and `movement_time_allowance=10.0 s` were left unchanged. Effective local and controller parameter dumps are byte-identical before and after (`SHA-256 015492ab7352430f49fbedd810db074a8e35c5a2e39060b88a3ad01f3c87ac87` and `73230045032aaa3e057065880a6aa1d48d4dbcf498611e080ac38a6cf7d8401f`). Original default scenario, HuNav config, and live Nav2 YAML hashes remain `4970601dec3f7e56aaa2df1375d09da2f670d486244022c37af37ed5d816ff43`, `1b12b19ba0a9b75bf5b2aba2292e04c96e57b919b784d3e9e80ad8104ec38272`, and `2c7eea093eca3254d4e5c9c4e08ed044389cfd43bd097791345ae005034d2473`.

## Patch and runtime provenance

The [applied one-line source patch](ARENA_LOCAL_COSTMAP_TF_FIX_APPLIED_20261001.patch) changes only `throttle: int = 300` to `10`. It was applied to the nested current adapter source, factory source, factory build source, factory build/lib copy, and factory installed module. No workspace rebuild was run because that build would replace the separately customized installed `run_isaacsim.py`. This is a **source-plus-runtime validation deployment**, not a claim that the ignored nested adapter or external factory workspace was committed by the root Git repository. Reversible byte-for-byte backups and hashes are in [`provenance_before.json`](</home/user/arena_local_costmap_tf_fix_20261001/provenance_before.json>) and [`provenance_after.json`](</home/user/arena_local_costmap_tf_fix_20261001/provenance_after.json>).

| Copy | Absolute path | SHA-256 before | SHA-256 after |
|---|---|---|---|
| Current adapter source | `/home/user/navigation_project/a_pipeline/isaac_sim/arena_ws/src/arena/isaac/ros2isaacsim/isaac_utils/graphs/tf.py` | `8357c450e19f7d4b98d543cb945c7078ed930a4ff4fd80f3579be8adfb4a5272` | `514aa84b12ce3040dc33eb28a9a830d5418c62a1e3b3f2b064ccef9f4cc791e4` |
| Factory source | `/home/user/arena_isaac5_py311_factory/humble_ws/src/arena_local/ros2isaacsim/isaac_utils/graphs/tf.py` | `a1ba698b321c882226ad7b261c65e88ade709418908dea41b7a7e1212482ea5c` | `d48387a07d2376c25dfca6021480d31eb5a5b0b7eeeb15cb8c779c3b8fbb6afb` |
| Factory build source | `/home/user/arena_isaac5_py311_factory/build_ws/humble/isaac_sim_ros_ws/src/arena_local/ros2isaacsim/isaac_utils/graphs/tf.py` | same as factory source | same as factory source |
| Factory build/lib | `/home/user/arena_isaac5_py311_factory/build_ws/humble/isaac_sim_ros_ws/build/ros2isaacsim/build/lib/isaac_utils/graphs/tf.py` | same as factory source | same as factory source |
| Factory installed | `/home/user/arena_isaac5_py311_factory/build_ws/humble/isaac_sim_ros_ws/install/ros2isaacsim/lib/python3.11/site-packages/isaac_utils/graphs/tf.py` | same as factory source | same as factory source |

The formal Isaac launcher uses `/home/user/arena_isaac5_runtime_shim/python.sh`, which sources the factory Isaac ROS install. A read-only Python loader probe under that shim resolved `isaac_utils.graphs.tf` to `/home/user/arena_isaac5_py311_factory/build_ws/humble/isaac_sim_ros_ws/install/ros2isaacsim/lib/python3.11/site-packages/isaac_utils/graphs/tf.py` with SHA-256 `d48387a07d2376c25dfca6021480d31eb5a5b0b7eeeb15cb8c779c3b8fbb6afb`, size 4,865 bytes, and parsed default 10; see [`runtime_loader_resolution.json`](</home/user/arena_local_costmap_tf_fix_20261001/runtime_loader_resolution.json>). A separate bounded isolated child directly logged `ARENA_TF_RUNTIME file=/home/user/arena_tf_throttle_validation_20261001/runtime/site-packages/isaac_utils/graphs/tf.py sha256=c8f014b011970a07319ef871b53ebba2ff9b2d8407a3d860b664e569644f97f3 framePeriod=10 requested=10 created=True`. That isolated copy added this read-only graph confirmation log to the same one-line behavior change. Direct `inspect.getfile` inside the unmodified installed child was **NOT VERIFIED**; the shim's loader resolution and unchanged installed hash support the installed-path attribution for the short and full runs.

## BEFORE / AFTER: bounded original default diagnostic

The before run was the 30-sim-second `ROS_DOMAIN_ID=202` diagnostic in [the runtime trace](ARENA_LOCAL_COSTMAP_RUNTIME_TRACE_20261001.md). The after run used the same original default scenario and Nav2 instrumentation in isolated domain 212, stopped at 30 sim seconds. [After raw bag](</home/user/arena_tf_throttle_validation_20261001/diag_run_20261001_212/raw_rosbag/metadata.yaml>), [launch log](</home/user/arena_tf_throttle_validation_20261001/diag_run_20261001_212/launch.log>), [effective parameters](</home/user/arena_tf_throttle_validation_20261001/diag_run_20261001_212/effective_local_costmap_params.yaml>), and [recomputed per-scan trace](</home/user/arena_local_costmap_tf_fix_20261001/isolated_default_bounded_trace_analysis.json>) are retained. A second bounded run also showed 65/66 TF accepted, 113/165 occupied local grids, and a 71-cell maximum, but lacks direct import confirmation, so the table uses domain 212.

| Metric | throttle=300 before | throttle=10 after |
|---|---:|---:|
| Scan published / local callback | 57 / 57 | 65 / 65 |
| TF accepted | 0 | 62 |
| TF dropped / unresolved at stop | 55 / 2 | 2 / 1 |
| TF accept ratio of callback input | 0% | 95.4% |
| Terminal TF publish interval median | 3.350 sim s | 0.117 sim s |
| Terminal TF arrival lag median / max | 3.350 / 4.033 sim s | 0 / 0 sim s |
| Laser projected / height accepted points | 0 / 0 | 20,220 / 20,220 |
| ObservationBuffer input / stored | 0 / 0 | 62 / 62 |
| Marking / clearing observation update cycles | 0 / 0 | 333 / 333 |
| Range/map pass and markVoxel calls, summed over update cycles | 0 | 32,873 |
| Maximum marked voxels in one grid | 0 | 72 |
| Nonzero voxel grids | 0 / 358 | 328 / 429 |
| Nonzero local OccupancyGrids | 0 / 166 | 117 / 173 |
| Maximum occupied local cells | 0 | 72 |

The sums over voxel update cycles repeat persistent observations; they are **not** counts of unique LiDAR points. The two post-patch drops have diagnostic `reason_code` values 0 and 1. The filter's `OutTheBack` text is not proof of a genuinely old sensor stamp, as explained in the earlier runtime trace. The accepted scans, stored observations, marked voxels, published voxel grids, and occupied local grids directly verify that the former first failure is gone. `/clock`, `/tf`, `/tf_static`, LiDAR, local costmap, voxel grid, and clearing endpoints are in the raw bag; 328 clearing-endpoint messages were recorded.

## Short no-human regression

The unchanged derived route `map_empty/nav_short_smoke_debug.json` was run in domain 211 to 80 sim seconds with NavFn/DWB, 1 Hz/10 s controller settings. [Raw bag](</home/user/arena_local_costmap_tf_fix_20261001/run_short/raw_rosbag/metadata.yaml>), [trace](</home/user/arena_local_costmap_tf_fix_20261001/run_short/trace_analysis.json>), [action analysis](</home/user/arena_local_costmap_tf_fix_20261001/run_short/action_analysis.json>), [launch log](</home/user/arena_local_costmap_tf_fix_20261001/run_short/launch.log>), and effective parameter dumps are retained.

**PASS for the short navigation chain, with a startup retry.** A first goal UUID `3f23f3e3024a6512e7154390684763a2` aborted while the robot was still reported outside the global costmap during initialization. The task reset, and UUID `7340d9a5de890b21fb3fd6a62f6e7f85` went `EXECUTING → SUCCEEDED` (status 4). At that terminal result the XY error was 0.124 m, yaw error 0.0012 rad, and the odometry path during the final goal was 2.394 m. There were 10 nonzero navigation commands, 296/297 scan callbacks passed TF, 568/665 voxel grids and 527/613 local grids had occupancy, and the local maximum was 70 cells. All eight recorded `cmd_vel` messages after the terminal result were zero; the final command was `(0,0)`. This confirms the previously successful short control chain can still finish. It does **not** establish a clean single-UUID startup gate because the first attempt aborted.

## Original default.json + three HuNav regression

The original `map_empty/default.json` was run in domain 213 to 170 sim seconds, with the same NavFn/DWB, 1 Hz/10 s and unchanged scenario/HuNav/Nav2 hashes. [Raw bag](</home/user/arena_local_costmap_tf_fix_20261001/run_default_full/raw_rosbag/metadata.yaml>), [trace](</home/user/arena_local_costmap_tf_fix_20261001/run_default_full/trace_analysis.json>), [action analysis](</home/user/arena_local_costmap_tf_fix_20261001/run_default_full/action_analysis.json>), [launch log](</home/user/arena_local_costmap_tf_fix_20261001/run_default_full/launch.log>), and [effective parameters](</home/user/arena_local_costmap_tf_fix_20261001/run_default_full/effective_local_costmap_params.yaml>) are retained. The clock guard reached 170.000009 s and the run-owned process groups exited.

**Infrastructure PASS:** 546 scans were published and reached the local callback; 544 passed TF, one dropped, and one remained unresolved at stop. ObservationBuffer stored 544, 3,182 update cycles had marking and clearing observations, 1,548/3,277 voxel grids were nonzero, and 677/1,448 local grids were occupied with a maximum of 89 cells. The terminal TF interval median was 0.117 sim s; arrival lag median/max was 0.117/0.133 sim s. The costmap no longer stays empty under the original three-person load.

**Navigation action ABORTED:** initial UUID `7edf9c96207122a70aff8a6dd687f603` aborted during startup. The main UUID `ee5c36f61ecbb658ddb030442049218b` went `EXECUTING → ABORTED` (status 6) after `Failed to make progress`. At the terminal result the robot was `(6.155, 21.678)` m, XY error 0.197 m, yaw error 0.377 rad, and the odometry path during this goal was 28.940 m. There were 278 nonzero `cmd_vel_nav` messages; all 14 post-terminal `cmd_vel` messages were zero and the final command was `(0,0)`. HuNav states were recorded for 3,131 frames with original names `D_test_1`, `2`, `3`; each had nonzero motion (maximum speed 0.3 m/s). This result isolates the remaining near-goal/progress and yaw/action outcome from the repaired TF/costmap gate. It is not a navigation or social-safety success.

## Status and limits

- **VERIFIED:** one-line throttle change is deployed in the five enumerated copies; isolated child graph used period 10; bounded default and full original default have TF acceptance, ObservationBuffer input, marking, voxel occupancy, and local occupancy. The short route's second UUID succeeded.
- **FAILED:** original default's main NavigateToPose UUID aborted after near-goal `Failed to make progress`; clean single-UUID startup behavior also failed in the short and default runs.
- **NOT VERIFIED:** frozen baseline readiness, default navigation success, social-safety acceptance, and an installed-child `inspect.getfile` print from the standard launcher. No further Nav2/controller/scenario parameter changes were made.
- **INFERRED:** the reduced graph period caused the filter recovery: it is the only behavior change, runtime graph period is confirmed, relevant effective parameters and input hashes match, and the before/after layer counters change at the predicted first failure point. Exact thread scheduling is not measured.

The diagnostic Nav2 costmap overlay adds counters and warnings and comes from a local 1.1.19 source against installed Humble Nav2 1.1.20, as recorded in the pre-fix trace; no filter, voxel, or controller rule was changed. Raw bags remain outside the root Git commit. [Compact metrics](ARENA_LOCAL_COSTMAP_TF_FIX_METRICS_20261001.json) retains the layer and action counts; the local run harness and analyzers are at `/home/user/arena_local_costmap_tf_fix_20261001/`.

## Reproduction on this host

```bash
# Inspect the reversible patch deployment and root evidence.
cat /home/user/arena_local_costmap_tf_fix_20261001/provenance_before.json
cat /home/user/arena_local_costmap_tf_fix_20261001/provenance_after.json
cat /home/user/arena_local_costmap_tf_fix_20261001/runtime_loader_resolution.json

# Re-run one isolated gate only after confirming no other Isaac/ROS process owns the GPU/domain.
# The runner refuses an existing output directory or occupied ROS domain.
/home/user/arena_local_costmap_tf_fix_20261001/run_gate.bash default_bounded 214 30
/home/user/arena_local_costmap_tf_fix_20261001/run_gate.bash short 216 80
/home/user/arena_local_costmap_tf_fix_20261001/run_gate.bash default_full 217 170
```

These commands create new large bags outside Git; the existing runs above are sufficient for this decision. Next research task: diagnose default's near-goal/progress and yaw/action outcome as a separate failure, retaining the frozen inputs and current TF infrastructure fix.
