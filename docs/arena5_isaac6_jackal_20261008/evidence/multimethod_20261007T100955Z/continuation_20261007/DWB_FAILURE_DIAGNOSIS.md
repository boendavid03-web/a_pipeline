# DWB fixed-configuration failure diagnosis, 2026-10-07

Scope: the five final-configuration headless `map_empty/quicktest` runs and one GUI run already in `RUNS/`. Jackal, zero pedestrians, seed 42, fixed 0.25 m external terminal check. The earlier experimental runs are excluded from the 1/5 figure.

| Run suffix | Terminal evidence | Native `env/stdout.log` diagnostic counts |
|---|---|---|
| `112251Z_1696308` | Native task success but 0.251 m after hold: strict FAIL | 26 `No valid trajectories out of 440`; 13 `ObstacleFootprint/Trajectory Hits Obstacle` |
| `113026Z_1717577` | 0.240 m: PASS | 18 invalid-trajectory messages; 9 footprint critic messages |
| `160732Z_2090655` | Timeout, 1.677 m: FAIL | 66 invalid-trajectory messages; 33 footprint critic messages; one `Failed to make progress` |
| `161100Z_2099130` | Timeout, 4.179 m: FAIL | 30 invalid-trajectory messages; 15 footprint critic messages; four progress failures |
| `161425Z_2107585` | Timeout, 4.536 m: FAIL | no invalid-trajectory or footprint critic message; four progress failures |
| GUI `162106Z_2124507` | Timeout, 4.357 m: FAIL | 64 invalid-trajectory messages; 32 footprint critic messages |

All six final-configuration runs recorded zero Arena 2D collision events. The two new headless failures with footprint messages show the full 440-sample DWB rollout becoming invalid near obstacles; the third failed without those messages, so it cannot be assigned the same immediate cause. The 0.251 m run crossed the fixed limit only after native task completion and post-goal drift. The native logs establish failure modes, but not a single deterministic parameter defect. No configuration was tuned and no threshold was widened; the observed fixed-configuration repeatability remains FAIL (1/5 headless). PhysX contact was not measured.

Reproduce the current controller unchanged:

```bash
[LOCAL_PATH] --method dwb --world map_empty --scenario quicktest --headless
```

Before any candidate tuning, retain per-run DWB trajectory scores and local costmap footprint geometry at the invalid-trajectory timestamps, then compare the sampled trajectories and progress checker window against robot TF/odom. This isolates a critic/geometry error from the later progress failures.
