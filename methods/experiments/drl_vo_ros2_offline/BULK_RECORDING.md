# Online-goal pilot and bulk recording

Do not start the SemanticCNN closed-loop launch while recording a teleoperation
demonstration. The path-only node publishes the A* path, local subgoal, and final
goal, but does not publish `/cmd_vel`.

## Start the fixed-map capture visualization

Do not use `01_start_v7_dual_slam.sh` for demonstration capture. It starts
`slam_toolbox` and RViz will show the map being rebuilt instead of the saved full
map. Start the fixed-map capture launch instead:

```bash
RUN_MANIFEST=/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/run_manifest.env \
PEDESTRIAN_COUNT=15 \
bash pipelines/v7_native_pipeline/scripts/01_start_v7_dual_fixed_map_capture.sh
```

This launch starts Gazebo, the dual-LiDAR merger, the saved-map server, a static
`map -> odom` transform, the odometry trajectory publisher, the path-only A*
node, RViz, and the graphical goal picker. It does not start SLAM and does not
start a model that publishes `/cmd_vel`.

RViz shows the full saved map, merged scan, current A* path, and cumulative
actual trajectory. The goal-picker window accepts a map click or typed
`goal_x/goal_y`. It opens at startup and reopens only after the recorder has
finished pausing the previous episode.

Before recording, verify that the path-only node is publishing:

```text
/semantic_cnn/global_path
/semantic_cnn/local_subgoal
/semantic_cnn/final_goal
```

Keep teleop as the only `/cmd_vel` publisher. The recorder now refuses to start
unless all three online goal topics have the expected types, active publishers,
and messages. Begin driving only after it prints `CAPTURE_READY`.

## Dynamic goals and automatic episodes

The path-only node accepts new `geometry_msgs/msg/PoseStamped` goals on
`/goal_pose` (the RViz **2D Goal Pose** tool uses this topic). Each accepted goal
clears the old path and replans from the current odometry pose. Do not send a new
goal while an episode is still moving or being finalized.

For a multi-episode pilot, start the recorder with:

```bash
RUN_MANIFEST=/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/run_manifest.env \
CAPTURE_SIM_DURATION_SEC=0 \
REQUIRE_CMD_VEL_PUBLISHER=1 \
REQUIRE_PEDESTRIAN_GROUND_TRUTH=1 \
AUTO_EPISODE_RECORDING=1 \
bash pipelines/v7_native_pipeline/scripts/05_record_rosbag.sh
```

The rosbag process starts paused. The first nonzero stamped teleop command
resumes recording. At the destination, press `k`; when the robot remains within
0.35 m of the final goal and both commanded and odometry velocities remain
stopped for 0.5 seconds, the controller records an `episode_end` event and pauses
the bag. Wait for `EPISODE_SAVED` in the episode-controller log before choosing
the next RViz goal. Repeat as needed, then use Ctrl-C once in the recorder
terminal to finalize the whole bag.

Automatic episode events are recorded on:

```text
/data_collection/episode_event
```

The converter drops scan-pair races outside complete start/end intervals, saves
`episode_id` in every sample, computes hindsight subgoals independently per
episode, and splits an episodic session only by whole episode.

Keep the robot stopped briefly at both ends. Use only nonnegative forward
velocity up to 0.5 m/s and angular velocity in `[-2, 2]` rad/s; include stops,
straight motion, both turn directions, and active avoidance.

## Pilot validation

Check the bag:

```bash
bash pipelines/v7_native_pipeline/scripts/06_check_bag.sh /absolute/path/to/pilot_bag
```

Convert it with strict online subgoals:

```bash
DUAL_SLOT_BAG_DIR=/absolute/path/to/pilot_bag \
DUAL_SLOT_SUBGOAL_SOURCE=online \
DUAL_SLOT_SUBGOAL_MAX_AGE_MS=300 \
bash pipelines/v7_native_pipeline/scripts/07b_convert_bag_to_fixed_dual_lidar.sh
```

Online conversion uses the latest local subgoal whose header stamp is at or
before the `scan_01` header stamp. Leading synchronized frames before the first
causal subgoal are counted and dropped. Every retained frame must have an age no
greater than 300 ms. The output session name includes `-sgonline` and an existing
session is not overwritten.

Stop after the converter and checker pass. Export SemanticCNN/S3-Net data and
build the DRL-VO replay only after inspecting this pilot.

## Bulk capture contract

Each raw bag must contain:

```text
/scan_01
/scan_02
/scan_merged
/odom
/tf
/tf_static
/cmd_vel
/cmd_vel_stamped
/pedestrian_ground_truth
/clock
/semantic_cnn/global_path
/semantic_cnn/local_subgoal
/semantic_cnn/final_goal
/data_collection/episode_event
```

Use `PEDESTRIAN_SIMULATION_FACTOR=1.0`. Vary routes, pedestrian seeds, and
low/medium/high density, restarting the simulation after changing pedestrian
count or seed. Split train/validation/test by whole bag or route; never randomly
place frames from one trajectory into different splits.
