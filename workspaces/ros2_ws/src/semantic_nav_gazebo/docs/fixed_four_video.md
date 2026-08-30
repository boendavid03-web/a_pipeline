# Fixed-four optional evaluation video

Video recording is opt-in and independent of `record_trace`. The existing
launch defaults remain `fixed_test:=false`, `record_video:=false`, and
`record_trace:=false`; manual goal selection is unchanged.

## Gazebo

```bash
bash pipelines/v7_native_pipeline/scripts/run_fixed_four_gazebo_demo.sh \
  --policy semantic_cnn \
  --run-name semantic_cnn_gazebo_video_$(date +%Y%m%d_%H%M%S) \
  --record-video
```

```bash
bash pipelines/v7_native_pipeline/scripts/run_fixed_four_gazebo_demo.sh \
  --policy drl_vo \
  --run-name drl_vo_gazebo_video_$(date +%Y%m%d_%H%M%S) \
  --record-video
```

Extra ROS launch arguments can be appended after `--`.

## Isaac

```bash
SEMANTIC_CNN_FIXED_TEST=true \
SEMANTIC_CNN_RECORD_VIDEO=true \
SEMANTIC_CNN_RECORD_TRACE=false \
bash pipelines/v7_native_pipeline/scripts/run_isaac_semantic_cnn_demo.sh
```

```bash
ISAAC_DEMO_FIXED_TEST=true \
ISAAC_DEMO_RECORD_VIDEO=true \
ISAAC_DEMO_RECORD_TRACE=false \
bash isaac_sim/scripts/run_custom_people_drlvo_demo.sh
```

## Artifacts

Each evaluation writes only new files below its own `video/` directory:

```text
video/evaluation_video.mp4
video/video_summary.json
video/episode_01_final.png ... episode_04_final.png
video/render.log
video/sync/dual_lidar.bin.gz
video/sync/merged_lidar.bin.gz
video/sync/navigation_events.jsonl
video/sync/capture_summary.json
```

`dual_lidar.bin.gz` stores both original `LaserScan.ranges` arrays together
with their independent timestamps and angular/range layouts. The renderer uses
causal latest-at-or-before matching against `simulation_time_sec`; it does not
use wall time.

`merged_lidar.bin.gz` stores the real `/scan_merged` stream in `base_link`.
For every output frame the renderer uses the causal robot pose to transform
those returns into `map` and draws the current scan directly over the occupancy
map. The raw two-sensor polar panels remain available at the right, but they are
not the map overlay.

The current SemanticCNN and DRL-VO policies both output velocity actions, not
predicted path geometry. The video therefore shows the actual raw velocity
action and labels `model predicted trajectory` as `unavailable`. The green
path, when present, is the actually published global planner path and is never
presented as a model prediction.

Gazebo evaluation currently has no simulator actuation telemetry. Its video
uses odometry twist for actual velocity and labels applied velocity as
`unavailable`. Isaac uses `SimulatorActuationState.applied_command` and
`actual_velocity`. Physical contact truth is also labeled `unavailable` when
the evaluator has only geometric collision proxies.
