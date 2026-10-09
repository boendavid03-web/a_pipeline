# Evidence index and publication boundary

The current public branch contains the progress snapshot, textual run archive, and local source patch snapshot. Large assets and full native trace directories remain outside the branch.

## Available here

- `STATUS.json`: scope, method counts, blockers, and acceptance limits.
- `METHOD_MATRIX.md`: the stage's per-method ledger, including cited run identifiers and measured outcomes.
- `README.md`: the short handoff and recommended reading order.
- `evidence/multimethod_20261007T100955Z/RUNS/`: 61 run folders, commands, launcher logs, exit codes, case identifiers, and 53 available `result.json` files.
- `evidence/multimethod_20261007T100955Z/continuation_20261007/`: diagnostics, blocker records, before snapshots, and patches from the stage.
- `evidence/multimethod_20261007T100955Z/configs/`, `patches/`, and `logs/`: scenario/controller configuration snapshots, corrective patches, and stage-level logs.
- `source/arena_workspace/SOURCE_MANIFEST.json`: source and submodule commit identities.
- `source/arena_workspace/patches/`: tracked worktree diffs for the Arena superproject and modified submodules.
- `source/arena_workspace/working_tree_only/`: the derived table-leg scenario and two new ROS test files that were untracked in the source checkout.
- `source/arena_workspace/harness/`: the archived launcher and runtime helper scripts used by the recorded stage.
- `related_arena5_isaac5/`: older Isaac Sim 5.1/Humble reports, patches, and validation scripts that were uncommitted in the local `a_pipeline` checkout, plus five contact/drive forensic files previously published on the older `experiment/scenario-topology-ab-e314f56` branch. This is a separate historical environment and not part of the Isaac Sim 6.0.0 results.

## Not included

Full native ROS/Isaac runtime trace directories were outside the stage report archive and are not included. The 8 run folders without `result.json` remain represented by their available command/log/exit records; a missing result is not a successful result. Source patches and helper scripts are archival snapshots and require their recorded upstream repositories, matching base commits, Isaac/ROS environment, and assets to reproduce a run.

The following material was intentionally excluded:

- all 7 GUI screenshots (they show the full desktop, including unrelated applications and chat);
- multi-gigabyte Isaac/Gazebo assets, caches, checkpoints, and model weights;
- generated `egg-info`, Python caches, and other build/runtime byproducts;
- unrelated NavIsaacLab training notes and resume scripts that were mixed into the local report folder.

Local absolute paths and LAN IP addresses in copied text artifacts were redacted. `UPLOAD_MANIFEST.sha256` lists hashes for the published files in this folder.
