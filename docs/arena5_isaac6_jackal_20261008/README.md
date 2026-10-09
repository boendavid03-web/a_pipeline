# Arena5 + Isaac Sim 6.0.0 Jackal progress

This folder is the public handoff for the Arena5 zero-pedestrian Jackal reproduction stage summarized on 2026-10-08. It is deliberately kept separate from the NavIsaacLab experiment and from this repository's older Isaac Sim 5.1 material.

## Read in this order

1. [`STATUS.json`](STATUS.json) for machine-readable scope and current counts.
2. [`METHOD_MATRIX.md`](METHOD_MATRIX.md) for per-method outcomes and their evidence limits.
3. [`EVIDENCE_INDEX.md`](EVIDENCE_INDEX.md) for the evidence and source snapshot map.
4. [`evidence/multimethod_20261007T100955Z/`](evidence/multimethod_20261007T100955Z/) for the per-run archive.
5. [`source/arena_workspace/`](source/arena_workspace/) for worktree patches and source provenance.
6. [`related_arena5_isaac5/`](related_arena5_isaac5/) for earlier Isaac Sim 5.1 notes and scripts, separate from this Isaac Sim 6 stage.

## Current stage

- Scope: original `map_empty/quicktest`, Jackal, no pedestrians, seed 42, ROS domain 189, fixed external goal tolerance 0.25 m.
- Arena's 18 methods: **3 PASS, 13 FAIL, 2 BLOCKED**. This is a fixed-scope reproduction ledger, not a claim about every Arena scene or planner.
- Nav2 is tracked separately: NavFn+RPP passed the listed quicktest, table-leg, and S-bend cases; NavFn+MPPI passed its listed cases; fixed-configuration NavFn+DWB repeatability failed (1/5 headless).
- PASS means the documented task/goal and Arena 2D footprint checks passed for the cited run. It does not establish PhysX contact safety, broad repeatability, GUI equivalence, pedestrian interaction, or cross-scene generalization.
- CrowdNav remains blocked until the exact trained SARL weights are available. SoNIC remains blocked as an original-method claim until the delivered policy identity is resolved. A CADRL follow-up was refused by a resource preflight and has no navigation result; it is not an additional navigation failure.

## Provenance and limits

The stage report and source checkout were maintained separately from this repository. This branch now includes the textual per-run archive and local source patches. The main `a_pipeline` checkout contains separate historical Isaac Sim 5.1 work. Do not assume its source, launch scripts, or assets reproduce this Isaac Sim 6.0.0 stage.

The method matrix is the contemporaneous ledger. Run directory names, commands, launcher logs, exit codes, and available `result.json` files are included. Local absolute paths and LAN addresses were redacted. The 61 run folders contain 53 result JSON files; full native runtime traces are not included. The source folder contains patches against recorded upstream commits, rather than a second copy of the multi-gigabyte source and asset tree. The exact source submodules remain separate upstream repositories. The related Isaac Sim 5.1 folder adds older reports and validation scripts from the local `a_pipeline` worktree, plus the Jackal contact/drive forensics already published on `experiment/scenario-topology-ab-e314f56` at `776ff4b`.

All 7 GUI screenshot files were omitted because they capture the full desktop, including unrelated applications and chat. No model weights, simulator assets, checkpoints, caches, or generated Python metadata were included. The legacy worktree's checkpoint and ZIP artifacts were excluded.

## Next work

1. Attach the missing native runtime traces and the remaining 8 result JSON files if they can be recovered and reviewed.
2. Resolve the CrowdNav checkpoint and SoNIC policy identity blockers without substituting random or unrelated weights.
3. Revisit the CADRL radius candidate only after its resource preflight clears; run one bounded case and preserve the original result.
4. Keep original-scene, S-bend, pedestrian, and PhysX-contact gates as separate follow-up work. Do not promote the current quicktest ledger to those layers.

Snapshot date: 2026-10-09 (source ledger timestamp: 2026-10-08 02:00 UTC).
