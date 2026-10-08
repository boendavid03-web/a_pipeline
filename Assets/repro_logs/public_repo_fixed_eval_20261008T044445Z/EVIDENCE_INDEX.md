# NavIsaacLab evaluation evidence index

Read `EVALUATION_AND_DEMO_REPORT.md` first for the result and its limits.

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
