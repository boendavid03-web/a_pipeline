# Arena 5 + HuNavSim + Isaac 5.1: navigation reproduction plan

Date: 2026-09-30. Scope: `map_empty`, Arena Jackal, Nav2 DWB then MPPI, Arena evaluation. This is a bounded code and evidence check. **Navigation/evaluation status: Partial.** No simulator or ROS graph was started for this report. No functional code or upstream scenario was changed. The only new runtime input is an independent, clearly marked navigation diagnostic JSON in the host overlay.

## 1. Current actual state and resolved paths

The earlier [pedestrian runtime gate](ARENA_HUNAV_ISAAC_RUNTIME_20260930.md) verifies 30 paired HuNav/Isaac frames for one person, a freeze control, robot odometry reaching HuNav, a limited robot/person interaction, and 763/763/764 paired frames for three people. It does **not** establish a Nav2 action result, obstacle sensing, costmap validity, scene collision, or evaluation validity. The logged robot displacement of 9.44 m in one interaction is movement evidence only. That run used a derived scenario with `social_force_factor=5`.

| Component | Current resolved code or package | Provenance/status |
| --- | --- | --- |
| Arena bringup | `isaac_sim/arena_ws/install/arena_bringup`; source `isaac_sim/arena_ws/src/arena/arena-rosnav` | Root repo HEAD `2f42de789ba2`; nested Arena directories are ignored by root Git, so root status is incomplete. Upstream `humble` is a comparison only. |
| Task Generator / HuNav adapter | `isaac_sim/arena_ws/install/task_generator`; Python imports from `isaac_sim/arena_ws/build/task_generator`, whose checked files resolve to the local source | `benchmark.py`, HuNav adapter, and Isaac simulator build links have identical hashes to current source. Prior HuNav changes are live through links. |
| Simulation setup, scenario, Nav2 config | `/home/user/arena_isaac5_host_overlay_ws/install/arena_simulation_setup`; installed share config and original `map_empty` files link to `/home/user/arena_isaac5_host_overlay_ws/src/arena_simulation_setup` | Local overlay takes precedence over `isaac_sim/arena_ws/src/arena/simulation-setup`. Its original `map_empty` map and `empty/default/1..5.json` are byte-identical to the Arena source copy. Upstream `voshch@3f142b2` is a comparison only. |
| Isaac adapter | host ROS package `ros2isaacsim` in local `arena_ws/install`; Isaac 5.1 child imports `/home/user/arena_isaac5_py311_factory/build_ws/humble/isaac_sim_ros_ws/install/ros2isaacsim/lib/python3.11/site-packages/ros2isaacsim/run_isaacsim.py` | Nested Isaac repository HEAD `a4beefe0203e`, dirty. Factory source and installed Python module currently differ by hash; the installed module was deliberately patched for the HuNav service window in the preceding gate. Preserve both; never infer runtime from the local source alone. |
| HuNav | added host overlay `/home/user/arena_full_ws/install_harmonic_host_overlay` when wrapper selects `ARENA_HUMAN=hunav`; independent old HuNav process also exists on domain 54 | Previously verified for the narrow pedestrian gate. |
| Nav2 | `/opt/ros/humble` for `nav2_controller`, `dwb_core`, `nav2_mppi_controller` | Binaries installed; selected controller loading and autonomous action result have not been verified in this chain. |
| Arena evaluation | source `isaac_sim/arena_ws/src/arena/evaluation`; both `arena_evaluation` and `arena_evaluation_msgs` unresolved in sourced runtime | Source presence only; recorder launch will fail if enabled now. Upstream `voshch@94762429` is a comparison only. |
| Learning controllers | source `isaac_sim/arena_ws/src/planners`; `nav2py_drl_vo_controller`, `nav2py_pas_crowdnav_controller`, `nav2py_crowdnav_attngraph_controller` unresolved by `ros2 pkg prefix` | Source weights exist for PaS and AttnGraph, but their runtime packages are absent; DRL-VO's referenced zip was not found in its planner subtree. |

Resource snapshot: no Isaac/Kit process and no GPU compute app found; RTX 5090 had about 30.7 GB free VRAM, system memory about 37 GB available. Other ROS/Arena and HuNav processes were active on domains 199 and 54, respectively. They were not contacted or stopped. Recheck immediately before any later launch and use a domain not occupied then.

## 2. `map_empty` scenarios and first task

The installed overlay files match the local Arena source and the referenced upstream `voshch/arena-simulation-setup@3f142b2` `empty/default/1..5.json` byte for byte. The local `map_empty.yaml` suite matches the referenced Arena-Rosnav `humble` version byte for byte. `empty.json` contains only `{}` and is **not** a navigation task: it has no Jackal start or goal. The suite contains five Jackal fixed stages 1–5, each with one configured episode, plus random and other robot stages; it does not include `default.json`.

| File | Jackal start → goal, XY (m) | Humans | Original static shelves |
| --- | --- | ---: | --- |
| `default.json` | `(25.3,1.25)` → `(6,21.8)` | 3 `actor1` | 0 |
| `1.json` | `(24,20)` → `(2,2)` | 3 `gazebo_actor` | 3 at `(13,11),(12,12),(14,10)` |
| `2.json` | `(29.6,1.77)` → `(19.4,22.7)` | 3 | 5 at `(8,3.3),(15.5,12.2),(9.7,18.8),(21,18.3),(27,10)` |
| `3.json` | `(29.6,1.77)` → `(19.4,22.7)` | 3 | 6 at `(25.1,3),(25.9,6.07),(30.3,8.89),(24.3,13.7),(29.4,17.5),(24.2,22)` |
| `4.json` | `(17,18)` → `(30,1.25)` | 3 | 5 at `(13,11),(12,12),(14,10),(26.6,10),(20.8,5.02)` |
| `5.json` | `(25.3,1.25)` → `(6,21.8)` | 3 | 5 at `(15.3,19.6),(13.8,16.7),(14.9,17.2),(7,8.13),(13.7,15.3)` |

The PNG map is 626×481 pixels at 0.05 m/pixel, origin `(0,0)`, giving 31.3×24.05 m. All checked starts, goals, human waypoints, and shelf **centres** lie on free map pixels. This is a coordinate sanity check, not proof of complete robot footprint clearance or Isaac/world/map registration. Scenario 4 has a human waypoint near the map edge with roughly 0.2 m map clearance; real radius/geometry may make it infeasible. The Nav2 global costmap covers `(0.5,0.5)` to `(30.5,23.5)`, so the robot goals are inside; some human positions can lie outside its window. Check actual `/map`, TF, costmaps, and stage coordinates during a run.

HuNav mode passes scenario positions and waypoints through `DynamicObstacle.extra` into HuNav agent goals; the prior scoped runs establish this path for derived scenarios. The original `actor1`/`gazebo_actor` visual model strings are **not** kept as Isaac assets: the HuNav Isaac adapter uses local `M_Medical_01` for the rendered pedestrian. This is an explicit appearance mapping; IDs, start poses, and route coordinates are the task-relevant fields. Original `default` and 1–5 behaviour under HuNav is still unverified. In particular, do not assume the prior close-interaction result applies: that run supplied social force 5, whereas the default HuNav agent configuration contains an agent with social force 0.

The overlay has `shelf.usd`, and Isaac obstacle spawning requests a USD under `/World/obstacles/<name>`. A shelf asset path exists, but successful stage load, collider, RTX scan response, and costmap occupancy are unverified. The Arena shelf footprint metadata has `collision: false`; it cannot establish Isaac collision. Do not begin 1–5 acceptance until one shelf is checked in the loaded stage, scan, and costmap.

The wrapper `/home/user/arena_isaac5_host_runtime/run_gui.bash` selects `ARENA_SCENARIO_FILE`, forwards `scenario_file:=...` through `arena.launch.py` into `task_generator.launch.py`'s `task.scenario.file`, and logs `ARENA_SCENARIO_PARAMETER_READY` after a compatibility parameter set. Prior logs show the marker and the selected poses. With `ARENA_HUMAN=hunav` and a scenario, it defaults both task modes to `scenario`; for a no-human debug run with `ARENA_HUMAN=isaac`, set `ARENA_TM_OBSTACLES=scenario` and `ARENA_TM_ROBOTS=scenario` explicitly.

**First navigation diagnostic:** `/home/user/arena_isaac5_host_overlay_ws/src/arena_simulation_setup/worlds/map_empty/scenarios/nav_smoke_debug.json`, derived from the original `default` Jackal start/goal with zero obstacles or humans. It is outside the upstream task list. The absolute path loads through the currently resolved `World(...).scenario(path)` API; this was checked with one robot, zero static, zero dynamic. A filename-only reference would currently fail because the new file has not been installed into the overlay share directory. **First original three-person task:** `default.json`; then 1–5 after shelf and HuNav behaviour checks.

## 3. Navigation methods and actual control prerequisites

The effective chain is `arena.launch.py` → host-overlay `robot.launch.py` → host-overlay `nav2.launch.py` → merged generic, Jackal, local controller, navfn, and interplanner YAML → `/opt/ros/humble/nav2_bringup/navigation_launch.py` → controller server. The command path then goes through Nav2 velocity smoother and collision monitor to Isaac Jackal `/cmd_vel`. Those filters are part of the configuration and must remain enabled. Static config presence is not proof of plugin loading, topic wiring, or motion.

| Method | Config and plugin | Current proof | Missing acceptance |
| --- | --- | --- | --- |
| DWB | overlay `controllers/dwb/controller_config.yaml`; `dwb_core::DWBLocalPlanner`; `dwb_core` installed | Config and binary present; wrapper selects `local_planner:=dwb` | Runtime plugin identity/activation, `NavigateToPose` outcome, costmap/TF/scan and executed `/cmd_vel` |
| MPPI | overlay `controllers/mppi/controller_config.yaml`; `nav2_mppi_controller::MPPIController`; binary installed | Config and binary present | Same runtime gates; wrapper presently hardcodes DWB, so use a separate explicit launch choice or a small later wrapper option, without changing upstream parameters |
| `crowdnav` | overlay maps to PaS-CrowdNav `nav2py_pas_crowdnav_controller::PasCrowdNavController` | Source, config and source `models/policy.pt` (5.7 MB), `models/vae.pth` (3.3 MB); runtime package missing | Installed weight path and live input validity. Source falls back to simple obstacle avoidance if weights are not found or loading fails; such movement is not PaS reproduction. |
| `drlvo` | `nav2py_drl_vo_controller::DRL_VO_Controller` | Source and config only | Package/model absent in current sourced overlay. Source default model path points to another user's `/home/kien/.../drl_vo.zip`; `cnn_data_pub.py` consumes `/track_ped` and initializes an all-zero pedestrian map. Isaac/HuNav does not establish `/track_ped` here. |
| `crowdnav_attngraph` | `nav2py_crowdnav_attngraph_controller::TemplateController` | Source, config and source `GST_predictor_rand/checkpoints/41665.pt` (9.6 MB); runtime package missing | Weight load and observation provenance. Python inference creates an internal CrowdNav environment; no evidence that its human observation is live Isaac pedestrian state. |

Effective DWB limits: `max_vel_x=0.26 m/s`, `max_vel_theta=1.0 rad/s`, `acc_lim_x=2.5`, `acc_lim_theta=3.2`, `decel_lim_x=-2.5`, `decel_lim_theta=-3.2`, `transform_tolerance=0.2 s`, `sim_time=1.7 s`, 20 linear × 20 angular samples. Effective MPPI limits: `vx_max=0.5`, `vx_min=-0.35 m/s`, `wz_max=1.9 rad/s`, `ax_max=3`, `ax_min=-3`, `az_max=3.5`, `transform_tolerance=0.1 s`, 56 steps at `model_dt=0.25 s`, batch 2000. These are deliberately different original configurations; record them rather than silently matching them.

Jackal model params declare robot radius 0.267 m and a `/task_generator_node/jackal/lidar` scan source, while the merged Nav2 footprint is the small square `±0.1 m`; that mismatch needs a measured clearance review before collision claims. The generic local costmap is rolling 15×15 m with voxel/inflation layers; global costmap is static + obstacle + inflation. The previous launch logged repeated `TF_NO_FRAME_ID` warnings from HuNav transforms, so verify the actual `map→odom→base_link→lidar` chain separately. The BT XML exists in the selected interplanner directory; successful BT load and Nav2 lifecycle state remain live checks. For every run, record `/clock`, TF, odom, a nonempty finite scan with timestamps, costmap updates, controller plugin, action status, filtered and executed commands, and publisher count to exclude another controller.

After DWB/MPPI, **PaS-CrowdNav is the first learning candidate to investigate**, because `crowdnav` has an explicit local mapping and local weights with visible fallback checks. It is not runnable as a faithful method yet. Do not interpret its fallback as a reproduced learned policy.

## 4. Arena evaluation and benchmark chain

Default `benchmark/config.yaml` selects `simulator: gazebo`, `meta_suite.yaml`, and `basic.yaml`; `basic.yaml` contains TEB, not DWB/MPPI. Thus the default benchmark is unsuitable for the requested Isaac comparison. The fixed `map_empty.yaml` suite lists 1–5, but YAML membership is not execution proof.

The current `Mod_Benchmark` applies stage task mode, world and scenario parameter, then calls task reset. Its `_reincarnate` method does not actually restart the robot/controller, the stage `robot` field is not applied, and the contestant `local_planner` is logged but not wired to a runtime controller switch. Failed stage parameter setup still proceeds to `_reset_task`. A stage's episode counter advances after reset, but reset of Isaac people, Jackal, Nav2 costmaps and controller history has not been demonstrated. Do not run the 50-episode suite until one-stage and two-stage transitions are proven by runtime parameters and fresh state.

`arena_evaluation` and its messages package are absent from the active install. The host `robot.launch.py` starts its `record` entry point only when `record_data_dir` is nonempty; setting that now is an infrastructure failure. In source, `record` enters `BagRecorder` and writes rosbag2. `get_metrics.py`/`scripts/metrics.py` expect `odom.csv`, `scan.csv`, `episode.csv`, `start_goal.csv`; no working bag-to-CSV conversion was found in the inspected package. Its current CSV metrics code reads odometry `position` again for velocity, hardcodes `waffle` robot parameters, labels every run under timeout/collision thresholds `GOAL_REACHED` without action/terminal-pose proof, and treats LaserScan threshold crossings as collisions. `time_diff` is annotated as nanoseconds, while `episode["time"] /= 10**10` is an inconsistent scaling; it needs a unit test against known timestamps. The bag recorder subscribes to namespace-specific scan/odom/cmd/human topics and absolute `/scenario_reset`; the namespace/topic mapping and episode boundaries need live validation. Its bag path does not by itself store a trustworthy Nav2 action result or Isaac contact event.

First full episode evidence should retain raw bag/messages from the Arena recorder interface once installed, plus Nav2 action result, true terminal XY error, sim start/end time, odom path, scan/costmap validity, minimum robot–human distance, and separately labelled geometric overlap or Isaac contact if available. Missing scan/contact must remain **unknown**, never zero collision. Treat navigation failure, infrastructure failure, and invalid statistics as separate results. The smallest evaluation repair is to install the local Arena evaluation packages in an isolated compatible overlay, verify bag topic names and episode/reset IDs, add the narrow bag-to-CSV transform or adapt the existing metrics reader, then correct velocity, robot model, time units, and success source with a small known-data fixture. No new evaluation framework is proposed.

## 5. Execution sequence

| Step | Goal and prerequisites | Smallest change / run | Acceptance |
| --- | --- | --- | --- |
| 0. Freeze working base | Preserve dirty nested Isaac repo, ignored Arena sources, wrapper, factory installed patch and prior logs | Record hashes and source/install paths; no branch switch or shared rebuild | Reproducible path inventory; previous HuNav gate retained |
| 1. No-human DWB | Independent ROS domain and free Isaac/GPU; debug JSON loaded by absolute path | Use current wrapper with `ARENA_HUMAN=isaac`, both task modes `scenario`, DWB/navfn. Keep raw ROS evidence from start. No upstream JSON edit. | Lifecycle active, actual DWB plugin, valid clock/TF/lidar/costmaps, single command source, Nav2 success action, final XY error ≤0.25 m, path/time/command logs; otherwise exact first break |
| 2. Original `default` three people | Step 1 passes; HuNav overlay and native adapter available | Use `ARENA_HUMAN=hunav`, `ARENA_SCENARIO_FILE=default.json`, original start/goal and 3 people | Three named HuNav agents track Isaac poses; original routes, obstacle sensing, robot action and clearance all measured; note any default social-behaviour limitation |
| 3. Fixed tasks 1–5 | Step 2 passes; shelf USD/collider/scan/costmap checked | Run original scenarios one by one, DWB/navfn, no silent geometry changes | Per-stage start/goal/people/shelves match JSON, independent action result and recorded evidence for each |
| 4. Evaluation and MPPI | One DWB episode bag exists; evaluator installed and conversion/stat fixes checked | Enable Arena `record_data_dir` through launch; validate raw bag and metrics on one episode; then select MPPI in an isolated run using its original config | Bag has required messages and IDs; metrics agree with action/terminal error; MPPI plugin/action/control chain proven |
| 5. Learning method | DWB/MPPI controls and evaluation reliable | Investigate PaS weights, installed package, observations and fallback; no bulk dependency install | Real weights loaded, live input provenance checked, fallback absent, same task/evidence protocol |

## 6. One next task and locally valid launch form

**Next task:** run only the no-human `map_empty` diagnostic with Jackal + navfn + DWB and capture raw ROS/Nav2 evidence. The debug JSON is already created and loads by absolute path. No functional code change is needed to attempt this step. The wrapper hardcodes DWB/navfn, matching this task. Do not enable `record_data_dir` until evaluation is installed. Keep a separate raw topic/action capture for the first run so data is not lost; then connect the Arena recorder before formal comparison.

After a fresh process/GPU/RAM check and confirmation that domain `194` is unused, run from this project root. The bag starts before Isaac so the first task's raw messages are retained:

```bash
run_dir="$PWD/isaac_sim/backends/isaac5/generated/nav_smoke_debug_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$run_dir/ros_logs"
source /home/user/arena_isaac5_host_runtime/setup.bash
export ROS_DOMAIN_ID=194 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$run_dir/ros_logs"
ros2 bag record --include-hidden-topics -o "$run_dir/raw_rosbag" \
  /clock /tf /tf_static \
  /task_generator_node/jackal/odom /task_generator_node/jackal/lidar \
  /task_generator_node/jackal/cmd_vel \
  /task_generator_node/jackal/navigate_to_pose/_action/status \
  /task_generator_node/jackal/global_costmap/costmap \
  /task_generator_node/jackal/local_costmap/costmap \
  >"$run_dir/bag.log" 2>&1 &
bag_pid=$!
printf 'Owned rosbag PID: %s\n' "$bag_pid" | tee "$run_dir/owned_pids.txt"
trap 'kill -INT "$bag_pid" 2>/dev/null || true' EXIT
env ROS_DOMAIN_ID=194 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$run_dir/ros_logs" \
  ARENA_WORLD=map_empty ARENA_HUMAN=isaac \
  ARENA_TM_ROBOTS=scenario ARENA_TM_OBSTACLES=scenario \
  ARENA_SCENARIO_FILE=/home/user/arena_isaac5_host_overlay_ws/src/arena_simulation_setup/worlds/map_empty/scenarios/nav_smoke_debug.json \
  /home/user/arena_isaac5_host_runtime/run_gui.bash 2>&1 | tee "$run_dir/launch.log"
kill -INT "$bag_pid"
wait "$bag_pid" || true
ros2 bag info "$run_dir/raw_rosbag" | tee "$run_dir/bag_info.txt"
```

Run this in an owned session with its launch PID/PGID recorded; stop only that session. Verify the bag really contains the requested topics because a topic spelling mismatch can yield an empty recording; discover actual names in domain 194 and amend only the capture list if needed. The next run must also save a compact data summary with action result, terminal pose and command publishers. This command alone is a launch, not an acceptance test. Do not touch `isaac_sim/arena_ws/{build,install,log}`, factory cp311 install, any other process group, original 1–5/default JSON, or the user's planners/policies. If a live check finds Isaac already used by another session, defer the run and keep this plan.

### Source comparisons

- [Original scenario files at the cited upstream commit](https://github.com/voshch/arena-simulation-setup/tree/3f142b2/worlds/map_empty/scenarios)
- [Arena `map_empty` benchmark suite on `humble`](https://github.com/Arena-Rosnav/arena-rosnav/blob/humble/arena_bringup/configs/benchmark/suites/map_empty.yaml)
- [Arena planner repository declarations](https://github.com/Arena-Rosnav/arena-rosnav/blob/humble/.repos/planners.repos)

These are comparison references. The resolved local source/install paths above determine actual behaviour.
