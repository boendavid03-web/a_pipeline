# Arena 5 / HuNav / Isaac 5.1 runtime gate, 2026-09-30

Scope: `map_empty`, Jackal, `human:=hunav`, first one pedestrian, then three. All launches used `ROS_DOMAIN_ID=193` and `ROS_LOCALHOST_ONLY=1`. The other active ROS domains and processes were left untouched. All process groups started for this gate were stopped after capture.

## Result

**Verified for this scoped runtime gate.** The one-person baseline produced 30 paired HuNav and Isaac pose samples at 10 Hz. The freeze control showed the Isaac prim stay fixed while HuNav advanced. Real Jackal odometry reached HuNav. A near robot interaction produced lateral pedestrian motion and kept the configured circular clearance. Three pedestrians then produced 763/763/764 paired samples at nominal 10 Hz.

| Gate | Measured result | Evidence |
| --- | --- | --- |
| One person, 30 paired frames | 30 samples; median `dt=0.100015 s`, maximum `0.100389 s`; HuNav x `3.000000 -> 4.739987 m`; maximum logged XY pose error `0.000000 m` | [CSV](isaac_sim/backends/isaac5/generated/hunav_arena_gate_20260930_134009/pose_samples.csv), [metrics](isaac_sim/backends/isaac5/generated/hunav_arena_gate_20260930_134009/metrics.json), [launch log](isaac_sim/backends/isaac5/generated/hunav_arena_gate_20260930_134009/launch.log) |
| Freeze, `ARENA_HUNAV_APPLY_POSE=0` | 9 paired samples before Task Generator reset; HuNav x `3.000000 -> 3.479832 m`; Isaac x stayed `3.000000 m` in every sample | [CSV](isaac_sim/backends/isaac5/generated/hunav_arena_freeze_20260930_134219/freeze_samples.csv), [metrics](isaac_sim/backends/isaac5/generated/hunav_arena_freeze_20260930_134219/metrics.json) |
| Jackal odom into HuNav | HuNav manager reported robot near `(4.39, 3.15) m` in the interaction run; every paired sample included changing robot XY and velocity from `/task_generator_node/jackal/odom` | [interaction log](isaac_sim/backends/isaac5/generated/hunav_arena_interaction_goalforce_20260930_135442/launch.log) |
| Robot and human interaction | 832 paired samples; median `dt=0.100001 s`; robot traveled `9.44 m`; pedestrian lateral deviation reached `0.573 m`; minimum center distance `0.640 m`, above the configured radii sum `0.55 m`; maximum logged pose error `0.000000 m` | [scenario](isaac_sim/backends/isaac5/generated/hunav_arena_interaction_goalforce_20260930_135442/scenario.json), [CSV](isaac_sim/backends/isaac5/generated/hunav_arena_interaction_goalforce_20260930_135442/pose_samples.csv), [metrics](isaac_sim/backends/isaac5/generated/hunav_arena_interaction_goalforce_20260930_135442/metrics.json) |
| Three pedestrians | 763, 763, 764 paired samples; first 30 median `dt=0.099951 s` per pedestrian; maximum logged XY error `0.000001 m` | [scenario](isaac_sim/backends/isaac5/generated/hunav_arena_three_20260930_135737/scenario.json), [CSV](isaac_sim/backends/isaac5/generated/hunav_arena_three_20260930_135737/pose_samples.csv), [metrics](isaac_sim/backends/isaac5/generated/hunav_arena_three_20260930_135737/metrics.json) |

The three-person run had occasional timing jitter: 8 of about 763 intervals per pedestrian exceeded `0.12 s`; the largest was `0.192435 s`. Its overall median was about `0.100000 s` and the 95th percentile was `0.100388 s`. Thus the three-person result supports nominal 10 Hz, not a hard real-time bound on every interval.

## Narrow runtime fixes made during this gate

- HuNav pedestrian spawning now uses the locally available `M_Medical_01` asset. The original random selection chose a missing character and left the Isaac animation graph invalid.
- The per-frame sample is logged at WARN because Arena launches the task generator with WARN verbosity.
- The Isaac ROS service loop handles a short extra service window when `ARENA_HUMAN=hunav`. Before this, the `move_prim` and pose-read services each waited about 100 ms for separate render frames, limiting output to about 4.4 Hz. Afterwards the one-person median interval was about 100 ms.
- HuNav agent messages now preserve the configured goal force factor. The previous hard-coded value `20.0` overpowered social force; in the first close robot run the pedestrian crossed the robot center. With the configured goal force `2.0` and scenario social force `5.0`, the measured minimum center distance was `0.640 m`.

Source touched: `task_generator/simulators/human/hunav/hunav.py`, `task_generator/simulators/sim/isaac_simulator.py`, and `ros2isaacsim/ros2isaacsim/run_isaacsim.py` under the local Arena workspace. The service-window patch was also applied to `/home/user/arena_isaac5_py311_factory/humble_ws/src/arena_local/ros2isaacsim/ros2isaacsim/run_isaacsim.py` and the installed module in `/home/user/arena_isaac5_py311_factory/build_ws/humble/isaac_sim_ros_ws/install/ros2isaacsim/lib/python3.11/site-packages/ros2isaacsim/run_isaacsim.py`, which the live launcher imports.

These results cover the specified `map_empty` runtime path. The clearance result uses configured circular radii; it is not a full Jackal footprint or navigation evaluation. Pose errors are reported at the log's six-decimal precision.
