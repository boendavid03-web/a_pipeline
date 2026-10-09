# NavIsaacLab evaluation and runtime evidence index

Read `EVALUATION_AND_DEMO_REPORT.md` first for the result and its limits.

## Motion replay and targeted candidate check (2026-10-09)

`../navisaaclab_teleop_velocity_20261009T042935Z/REPORT.md` records the A-fixed 192-step Nova Carter command-versus-motion baseline. Straight-line speeds of 0.2, 0.5, and 1.0 m/s passed every per-step gate; the robot-7 replay had 2 linear and 3 angular violations, and turn segments also had per-step angular violations. `telemetry.csv`, `telemetry.jsonl`, `SUMMARY.json`, run identity, and plots preserve the measurements. This is kinematic evidence; no PhysX contact sensor or impulse data was collected.

`../navisaaclab_motion_localfix_20261009T050247Z/REPORT.md` and `COMPARISON.json` document a same-command, same-reset 192-step comparison of one local orientation candidate against that baseline. The candidate was rejected and A-fixed retained: baseline had 2 linear / 16 angular over-limit samples (18 distinct steps), while the candidate had 6 / 36 (38 distinct steps). The candidate improved selected historical steps but regressed overall, especially turns. `telemetry.csv`, `telemetry.jsonl`, `PHYSICS_EVIDENCE.jsonl`, `SUMMARY.json`, `PATCH_DECISION.json`, `CANDIDATE.patch`, run identity, and plots contain the supporting records. Contact objects, force, and impulse remain unavailable; this does not establish a unique physical cause or physical safety.

The candidate patch is retained for review only and was not applied to the A-fixed source. Neither run is a full action-range, long-duration, PPO, cross-scene, or physical-safety acceptance result.

## Checkpoint comparison

`COMPARISON_TWO_SEEDS.json` contains the aggregate results for all three robot PPO checkpoints. Each of the six run directories contains `episodes.jsonl`, `metrics.json`, `CONFIG_EVAL.json`, and `INITIAL_ROUTES.json`:

| Checkpoint step | Seed 701 | Seed 702 |
|---:|---|---|
| 4,180,480 | `step_4180480_100/` | `step_4180480_seed702_100/` |
| 4,940,800 | `step_4940800_100/` | `step_4940800_seed702_100/` |
| 5,002,240 | `final_100/` | `final_seed702_100/` |

Root-level `*_stdout.log`, `*_stderr.log`, `*_START_UTC.txt`, `*_END_UTC.txt`, and `*_EXIT_CODE.txt` files provide run completion records. `SOURCE_INTEGRITY_FINAL.json` and `SOURCE_INTEGRITY_POST.json` record the source integrity check.

## Demo evidence and procedure

`demo_4180480_gui/` and `demo_replay_20261008T053130Z/` contain the first two `birdseye.gif` demos and their configs. `demo_replay_20261008T064331Z/` is a later replay: its exit code is 0, its completion marker records 10 robot episodes, and it produced a 69-frame GIF. The three runs' stdout/stderr and exit records are alongside them. `RUN_DEMO_REPLAY.sh`, `demo_replay.py`, `demo_4180480.py`, and the evaluation scripts document the local execution procedure; they depend on the original Isaac Sim workstation, source tree, and local checkpoints, which are not included here.

## Additional records and scope

`COMPARISON.json` and `final_pilot_10/` preserve the earlier comparison and 10-episode pilot alongside the later 600-episode two-seed evaluation. The `navigation/` directories contain timestamped path snapshots and trajectory JSONL files for evaluation and demos, including `paths_latest.json` copies. They are included as run context; use the six 100-episode records above for the reported checkpoint comparison.

Model weights, `.eval.lock`, and generated Python bytecode are not included. The reported collision flag is CrowdSim task-detector/drive-guard output; there is no frame-by-frame PhysX contact evidence. No raw camera frames for the zero-depth reads are included.
