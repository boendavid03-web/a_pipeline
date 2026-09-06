# Scenario topology A/B evaluation plan

This phase compares only the tracked pedestrian configurations at commit
`df76ba2`. It does not regenerate either configuration and does not change the
Social Force kernel, Isaac runtime, robot controller, policy, or evaluator.
No A/B runs were started while preparing this plan.

## Current evaluation pipeline

| Required evidence | Current source | Status and boundary |
|---|---|---|
| Pedestrian trace | `evaluation/episode_*/pedestrian_trace.csv` | Available. Each row contains simulation time, stable pedestrian ID, pose, yaw, and planar velocity from `/pedestrian_ground_truth`. |
| Social debug | `isaac.log` records named `PEDESTRIAN_GAZEBO_SOCIAL_DEBUG`; `pedestrian_social_steering.jsonl` uses schema `isaac_pedestrian_social_steering/v2` | Available when `ISAAC_PEDESTRIAN_SOCIAL_MODE=gazebo_social`. The JSONL is the preferred frame-level source. |
| Collision | Per-episode `episode_summary.json`, session `session_summary.json`, and final `WAREHOUSE_PEOPLE_ROBOT_METRICS` in `isaac.log` | Partial. Navigation `human_proxy`, `static_proxy`, and `virtual_proxy` are geometric proxies. Physical-contact truth is not configured, so these data cannot prove a contact-free run. Runtime `collision_blocked_count`, `pedestrian_inside_robot_frames`, and minimum robot clearance are additional proxy/safety evidence. |
| Route completion | `WAREHOUSE_PEOPLE_ROBOT_METRICS.pedestrian_social_motion.patrol_cursors.*.{advance_count,lap_count,point_count}` | Available only if Isaac reaches a normal finalization path. Run with a fixed `--duration` and require the final metrics marker; an externally terminated fixed-four run is insufficient. |
| Freeze/fallback | Final `pedestrian_social_motion`, `pedestrian_social_yielding`, free-space counters, and frame-level steering JSONL | Partially summarized. `maximum_freeze_sec_by_person` excludes emergency-inhibited samples. Derive an inclusive freeze duration from the JSONL and separately report emergency inhibition/yield/dodge, follow restarts, constrained targets, free-space recovery, and sustained intrusion. |

The five evidence families therefore exist, but collision remains proxy-only and
inclusive freeze must be derived offline from the collected steering samples.

## Frozen experiment contract

- Runtime duration: `600.0` simulation seconds for both variants.
- Pedestrians: count `15`, generator/config seed `7`, unchanged per-person speed
  sequence from the contract.
- Social mode: `gazebo_social`; every Social Force and steering value below is
  explicitly fixed to its current default for both variants.
- Robot: start `(2.0, 2.0, 0.0)`, goal `(6.0, 4.0)`, base DRL-VO policy,
  identical checkpoint, oracle pedestrian input, dynamic rigid body, collision
  protection enabled, CPU PhysX, two 2000-beam LiDAR scans at 15 Hz.
- Goal mode: one automatic fixed goal, not the fixed-four auto-shutdown path.
  Isaac must exit because `--duration 600.0` is reached so the final pedestrian
  route/freeze/fallback summary is emitted.
- Runs are sequential. Never run A and B concurrently or reuse an output path.
- Formal input hashes:
  - A: `76896643184fd7fbfa26c43727cce49452bbe313fdae177de1065acb3a23303d`
  - B: `a884d56465bcc489ccd5196c35bb46a9a49726556be933fc22eaddf52e682751`
  - DRL-VO checkpoint: `57cfd10e6f528f96f721480420c1d6f873b0ff53ad4bbd3e997da5d2fcd6c4e2`

## Experiment matrix

| Run name | Pedestrian config | Metrics | Artifact path |
|---|---|---|---|
| `A_baseline_seed7_r1` | `runs/scenario_topology_ab/baseline/A_baseline.yaml` | pedestrian trace; social quality/debug; route completion; inclusive/exclusive freeze; fallback/restart/intrusion; navigation and collision proxies | `runs/scenario_topology_ab/evaluation/A_baseline_seed7_r1/` |
| `B_spread_radius_seed7_r1` | `runs/scenario_topology_ab/B_spread_radius/B_spread_radius.yaml` | same metric set and thresholds as A | `runs/scenario_topology_ab/evaluation/B_spread_radius_seed7_r1/` |

This is a matched seed-7 pair. It can support a scoped seed-7 conclusion, not a
general scenario-distribution or statistical-significance claim. If runtime
nondeterminism prevents a stable conclusion, repeat the same pair as `r2` and
`r3`, alternate order, and require the direction of change to agree in at least
two of three pairs. Additional seeds are a later experiment, not part of this
frozen matrix.

## Commands to run after approval

Preflight only; these commands do not start Isaac:

```bash
cd /home/user/navigation_project/a_pipeline

test "$(git branch --show-current)" = "experiment/scenario-topology-ab-e314f56"
test "$(git rev-parse --short HEAD)" = "df76ba2"
test -z "$(git status --porcelain)"

(cd runs/scenario_topology_ab/baseline && sha256sum -c SHA256SUMS)
(cd runs/scenario_topology_ab/B_spread_radius && sha256sum -c SHA256SUMS)

/usr/bin/python3 isaac_sim/scripts/validate_custom_people_routes.py \
  --config runs/scenario_topology_ab/baseline/A_baseline.yaml \
  --world workspaces/ros2_ws/src/semantic_nav_gazebo/worlds/gazebo_eng_lobby.world \
  --clearance 0.55 --min-start-separation 1.0

/usr/bin/python3 isaac_sim/scripts/validate_custom_people_routes.py \
  --config runs/scenario_topology_ab/B_spread_radius/B_spread_radius.yaml \
  --world workspaces/ros2_ws/src/semantic_nav_gazebo/worlds/gazebo_eng_lobby.world \
  --clearance 0.55 --min-start-separation 1.0

/usr/bin/python3 -m pytest -q \
  isaac_sim/tests/test_scenario_topology_baseline_contract.py \
  isaac_sim/tests/test_scenario_topology_spread_radius_contract.py
```

Define the common frozen environment once in the terminal that will execute the
runs:

```bash
cd /home/user/navigation_project/a_pipeline
mkdir -p runs/scenario_topology_ab/evaluation

RUN_DURATION_SEC=600.0
DRL_VO_MODEL_PATH="$PWD/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/training/drl_vo/base_bc/20260727_114455/checkpoints/best.pt"

COMMON_ENV=(
  env
  ROS_DOMAIN_ID=78
  ISAAC_ROS_DOMAIN_ID=78
  ISAAC_DRLVO_PEDESTRIAN_SOURCE=oracle
  DRL_VO_MODEL="$DRL_VO_MODEL_PATH"
  ISAAC_DEMO_CONTROL_MODE=policy
  ISAAC_DEMO_AUTO_GOAL=true
  ISAAC_DEMO_GOAL_PICKER=false
  ISAAC_DEMO_FIXED_TEST=false
  ISAAC_DEMO_GOAL_X=6.0
  ISAAC_DEMO_GOAL_Y=4.0
  ISAAC_DEMO_EVALUATE=true
  ISAAC_DEMO_RECORD_TRACE=true
  ISAAC_DEMO_RECORD_BAG=0
  ISAAC_DEMO_RECORD_VIDEO=false
  ISAAC_DEMO_RVIZ=false
  ISAAC_DEMO_MAX_LINEAR=0.99
  ISAAC_DEMO_MAX_ANGULAR=1.99
  ISAAC_DEMO_INFLATE_RADIUS=0.45
  ISAAC_ROBOT_PHYSICS=1
  ISAAC_ROBOT_COLLISION_PROTECTION=1
  ISAAC_PHYSX_GPU_DYNAMICS=0
  ISAAC_LIDAR_MODE=physx
  ISAAC_LIDAR_RATE_HZ=15
  ISAAC_LIDAR_SAMPLE_COUNT=2000
  ISAAC_PEDESTRIAN_COUNT=15
  ISAAC_PEDESTRIAN_SEED=7
  ISAAC_PEDESTRIAN_SPEED=1.0
  ISAAC_PEDESTRIAN_AVOIDANCE_MODE=gentle
  ISAAC_PEDESTRIAN_FIXED_SPEED=0
  ISAAC_PEDESTRIAN_ENSURE_ALL_DIRECTIONS=0
  ISAAC_PEDESTRIAN_OPPOSED_PAIR_TEST=0
  ISAAC_PEDESTRIAN_FREE_SPACE_CLEARANCE_M=0.55
  ISAAC_PEDESTRIAN_FREE_SPACE_GUARD_CLEARANCE_M=0.20
  ISAAC_PEDESTRIAN_SPAWN_CLEARANCE_M=1.0
  ISAAC_PEDESTRIAN_MIN_PATROL_SEGMENT_M=0.5
  ISAAC_EXPLICIT_CUSTOM_IRA_MIN_START_SEPARATION_M=1.0
  ISAAC_PEDESTRIAN_SOCIAL_MODE=gazebo_social
  ISAAC_PEDESTRIAN_SOCIAL_MASS_KG=20.0
  ISAAC_PEDESTRIAN_PERSONAL_SPACE_M=1.0
  ISAAC_PEDESTRIAN_VISUAL_OVERLAP_M=0.45
  ISAAC_PEDESTRIAN_MAX_PERSONAL_SPACE_VIOLATION_RATIO=0.05
  ISAAC_PEDESTRIAN_SOCIAL_YIELD_TRIGGER_M=1.25
  ISAAC_PEDESTRIAN_SOCIAL_YIELD_RESUME_M=1.50
  ISAAC_PEDESTRIAN_SOCIAL_NEIGHBOR_RANGE_M=10.0
  ISAAC_PEDESTRIAN_SOCIAL_FORCE_WEIGHT=5.1
  ISAAC_PEDESTRIAN_ROBOT_SOCIAL_FORCE_WEIGHT=5.1
  ISAAC_PEDESTRIAN_ROBOT_PERSONAL_SPACE_FORCE_WEIGHT=6.0
  ISAAC_PEDESTRIAN_SOCIAL_RELAXATION_TIME_SEC=0.5
  ISAAC_PEDESTRIAN_SOCIAL_SMOOTHING_TIME_SEC=0.35
  ISAAC_PEDESTRIAN_SOCIAL_MAX_ACCEL_MPS2=4.0
  ISAAC_PEDESTRIAN_SOCIAL_MAX_STEERING_CORRECTION_MPS=0.65
  ISAAC_PEDESTRIAN_SOCIAL_MAX_LATERAL_STEERING_MPS=0.45
  ISAAC_PEDESTRIAN_SOCIAL_MAX_STEERING_ANGLE_DEG=35.0
  ISAAC_PEDESTRIAN_SOCIAL_MIN_SPEED_MPS=0.15
  ISAAC_PEDESTRIAN_AGENT_RADIUS_M=0.35
  ISAAC_PEDESTRIAN_ROBOT_RADIUS_M=0.47
  ISAAC_PEDESTRIAN_ROBOT_CLEARANCE_M=1.0
  ISAAC_PEDESTRIAN_ROBOT_PERSONAL_SPACE_SIGMA_M=0.2
  ISAAC_PEDESTRIAN_SOCIAL_EMERGENCY_YIELD_TRIGGER_M=0.50
  ISAAC_PEDESTRIAN_SOCIAL_EMERGENCY_YIELD_RESUME_M=0.80
  ISAAC_PEDESTRIAN_SOCIAL_EMERGENCY_DODGE_CLEARANCE_M=0.20
  ISAAC_PEDESTRIAN_SOCIAL_DEBUG_PERIOD_SEC=5.0
  ISAAC_PEDESTRIAN_SOCIAL_STEERING_LOOKAHEAD_M=1.0
  ISAAC_PEDESTRIAN_SOCIAL_ROUTE_LOOKAHEAD_M=1.5
  ISAAC_PEDESTRIAN_SOCIAL_WAYPOINT_REACH_M=0.35
  ISAAC_PEDESTRIAN_SOCIAL_TARGET_MIN_SHIFT_M=0.02
)

run_scenario_topology_cell() {
  local run_name="$1"
  local config="$2"
  local run_dir="$PWD/runs/scenario_topology_ab/evaluation/$run_name"

  "${COMMON_ENV[@]}" \
    ISAAC_EXPLICIT_CUSTOM_IRA_CONFIG="$PWD/$config" \
    ISAAC_DEMO_OUTPUT_DIR="$run_dir" \
    ISAAC_DEMO_TRACE_PATH="$run_dir/trajectory.csv" \
    ISAAC_DEMO_EVALUATION_OUTPUT_DIR="$run_dir/evaluation" \
    ISAAC_PEDESTRIAN_SOCIAL_TRACE_PATH="$run_dir/pedestrian_social_steering.jsonl" \
    bash isaac_sim/scripts/run_custom_people_drlvo_demo.sh \
    --duration "$RUN_DURATION_SEC"

  test -s "$run_dir/isaac.log"
  test -s "$run_dir/pedestrian_social_steering.jsonl"
  test -s "$run_dir/evaluation/session_summary.json"
  test -s "$run_dir/evaluation/episode_0001/pedestrian_trace.csv"
  grep -q 'pedestrian social=gazebo_social' "$run_dir/isaac.log"
  grep -q '^WAREHOUSE_PEOPLE_ROBOT_METRICS=' "$run_dir/isaac.log"

  cp --reflink=auto "$config" "$run_dir/input_people_config.yaml"
  git rev-parse HEAD > "$run_dir/git_commit.txt"
  sha256sum "$config" "$DRL_VO_MODEL_PATH" > "$run_dir/input_sha256.txt"
  grep '^WAREHOUSE_PEOPLE_ROBOT_METRICS=' "$run_dir/isaac.log" \
    | tail -n 1 | cut -d= -f2- > "$run_dir/warehouse_metrics.json"
}
```

Run A and B sequentially:

```bash
run_scenario_topology_cell \
  A_baseline_seed7_r1 \
  runs/scenario_topology_ab/baseline/A_baseline.yaml

run_scenario_topology_cell \
  B_spread_radius_seed7_r1 \
  runs/scenario_topology_ab/B_spread_radius/B_spread_radius.yaml
```

Do not start B unless A completed the six post-run `test`/`grep` gates. Do not
delete, restart, or attach to a simulator owned by another terminal; the launcher
will fail closed if the ROS domain or Isaac lock is already occupied.

## Files to collect

Keep these files from each run directory:

- `input_people_config.yaml`, `input_sha256.txt`, and `git_commit.txt`.
- `isaac.log` and the extracted `warehouse_metrics.json`.
- `pedestrian_social_steering.jsonl`.
- `sensor_preflight.log`, `controller_preflight.log`, and `drlvo.log`.
- Top-level `trajectory.csv`.
- `evaluation/session_summary.json`.
- Every `evaluation/episode_*/episode_summary.json`.
- Every `evaluation/episode_*/pedestrian_trace.csv`.
- Every episode's `trajectory.csv`, `commands.csv`, `inference_trace.csv`,
  `actuation_decisions.csv`, `simulator_actuation.csv`,
  `pose_derived_velocity.csv`, `actuation_alignment.csv`, and
  `physx_actuation_alignment.csv`.

A rosbag is intentionally not required for this minimum matrix: the existing
topics do not add physical-contact truth, and the evaluator plus steering trace
already contain the needed paired metrics. Record a bag only as a separately
approved diagnostic expansion.

## Metrics that can support improvement

First enforce validity gates. Both runs must have the expected config hash,
commit, 15 pedestrian IDs, seed 7, identical speed sequence and Social Force
parameter dictionary, `gazebo_social` mode, `duration_reached`, valid 15 Hz
dual-LiDAR preflight, comparable trace coverage, no runtime reset, and a final
metrics marker. A failed gate makes the pair incomparable.

Primary topology metrics:

1. Crowd spacing: lower
   `pedestrian_social_quality.personal_space_violation_pair_ratio` and
   `visual_overlap_pair_ratio`, no increase in visual-overlap pair samples, and
   higher or equal `min_center_distance_m`. The pair-ratio denominator is all
   unique pedestrian-pair observations across sampled frames.
2. Route liveness: per-person `lap_count`, `advance_count`, and authored loop
   length. Because A and B have different dense path point counts, do not compare
   raw `advance_count` alone. Report normalized completion as
   `lap_count * authored_loop_length_m / (preferred_speed_mps * duration_sec)`
   and the fraction of pedestrians completing at least one lap.
3. Freeze: lower duration-weighted inclusive freeze ratio and lower per-person
   maximum/p95 freeze. Inclusive freeze must count samples with actual speed
   below `0.05 m/s` while the social controller requests at least `0.2 m/s`, and
   must also include emergency-inhibited samples; keep the runtime's existing
   exclusive `maximum_freeze_sec_by_person` as a separate diagnostic.
4. Fallback/recovery: lower or equal emergency-inhibited sample ratio,
   `yield_count`, `pedestrian_robot_dodge_count`, `follow_restart_count`, and
   `free_space_constrained_target_count`; require zero sustained intrusion and
   zero pedestrian runtime recovery/reset.

Secondary robot/safety non-regression metrics:

- Both runs reach the same robot goal without timeout.
- B does not increase human/static/virtual collision proxy rates or entries.
- B does not reduce minimum human body clearance or minimum TTC, and does not
  increase time below the TTC threshold.
- Path efficiency, navigation time, failure-to-progress, safety-gated command
  ratio, and actuation-alignment coverage do not materially regress.

For this seed-7 phase, call B an improvement only if crowd-spacing metrics improve
while normalized route completion is non-inferior and all fallback, intrusion,
robot-goal, and data-quality gates pass. Navigation goal reach by itself does not
prove topology improvement. Collision fields remain geometric proxies, so this
matrix cannot establish a reduction in physical contacts.
