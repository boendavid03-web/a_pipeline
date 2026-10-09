# Evidence index and publication boundary

The current public branch contains a progress snapshot, not the raw experiment archive.

## Available here

- `STATUS.json`: scope, method counts, blockers, and acceptance limits.
- `METHOD_MATRIX.md`: the stage's per-method ledger, including cited run identifiers and measured outcomes.
- `README.md`: the short handoff and recommended reading order.

## Referenced but not included

The matrix refers to run IDs under `RUNS/` and evidence under `continuation_20261007/` in the private local report tree. This branch does not include their raw `result.json`, ROS/Isaac logs, command transcripts, trajectories, or exact source/config snapshots. The references are useful identifiers, not downloadable links.

For independent review, a follow-up evidence bundle should include, per cited run:

- `result.json` and its hash;
- exact launch command, relevant config/source revision, and environment identity;
- the native case terminal status and concise log excerpt;
- trajectory and collision-event data needed to verify the stated endpoint and Arena 2D result.

Keep any follow-up bundle limited to the files needed to substantiate the claims. Do not include model weights, credentials, unrelated checkpoints, or full simulator assets unless separately reviewed and needed.
