# Temporal dual-LiDAR pedestrian detector

This model has one online purpose:

```text
N causal dual-LiDAR frames
  -> pedestrian position (x, y)
  -> absolute pedestrian velocity (vx, vy), expressed on robot axes
  -> confidence
  -> KF/Hungarian track_id
```

The neural network does not receive semantic labels, pedestrian truth, robot
commands, or absolute map coordinates during inference. Robot poses are used
only to compensate ego motion across the checkpoint-declared LiDAR history.

Training-only truth fields construct visible-pedestrian heatmaps, sub-cell
center offsets, and velocity regression targets. `infer_dataset.py` creates the
dataset with `build_targets=False` and therefore never loads those fields.

Run all commands from the `a_pipeline` root with the project PyTorch
interpreter:

```bash
source environment/activate.sh
"${TORCH_PY}" -m methods.experiments.dual_lidar_pedestrian_bev.train --smoke
```

Training and smoke artifacts are created under a new timestamped directory.
Existing datasets and checkpoints are never overwritten.

Evaluate the best checkpoint on the development bag:

```bash
"${TORCH_PY}" -m methods.experiments.dual_lidar_pedestrian_bev.evaluate \
  --dataset-root <semantic2d-root> \
  --checkpoint <run-dir>/checkpoints/best.pt \
  --split dev \
  --output-json <run-dir>/dev_evaluation.json
```

Inference applies deterministic `0.30 m` metric non-maximum suppression before
the KF/Hungarian tracker. This suppresses duplicate center peaks belonging to
the same pedestrian.

## Selected 2026-07-31 development configuration

The selected checkpoint uses 12 causal occupancy frames and a velocity loss
weight of 1.0:

```text
runs/20260717_042135_v7_dual/training/dual_lidar_pedestrian_bev/
  20260731_opt_velw100_h12_c24_v1/checkpoints/epoch_014.pt
```

Truth-free inference:

```bash
"${TORCH_PY}" -m methods.experiments.dual_lidar_pedestrian_bev.infer_dataset \
  --dataset-root runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/semantic2d \
  --checkpoint runs/20260717_042135_v7_dual/training/dual_lidar_pedestrian_bev/20260731_opt_velw100_h12_c24_v1/checkpoints/epoch_014.pt \
  --split dev \
  --output-jsonl <new-output.jsonl> \
  --confidence-threshold 0.40 \
  --nms-radius-m 0.30 \
  --position-gate-m 0.50 \
  --velocity-gate-mps 1.50 \
  --acceleration-sigma-mps2 3.0 \
  --position-measurement-scale 0.75 \
  --velocity-measurement-scale 2.0 \
  --association-velocity-weight 0.40 \
  --device cuda
```

The same-frame fixed-threshold unseen-test comparison improved detection F1
from 0.9203 to 0.9300 and single-frame velocity vector RMSE from 0.2195 m/s to
0.2030 m/s. Confirmed-track velocity RMSE improved from 0.1887 m/s to
0.1635 m/s. The test ID-switch count increased from 7 to 10, so close
pedestrian crossings remain a known limitation rather than a solved case.
Both models were evaluated on the same 11,580 frame keys; the 8-frame baseline
skipped the first four otherwise-unavailable 12-frame windows per episode.

The complete report is:

```text
runs/20260717_042135_v7_dual/evaluation/
  dual_lidar_pedestrian_optimization_20260731/optimization_summary.json
```

Training also supports isolated experiments with:

- `--velocity-loss-weight`
- `--input-encoding occupancy|current_plus_deltas`
- `--init-checkpoint` with `--trainable-components velocity_head`
- `--save-every-epochs`
- `--lr-schedule constant|cosine`

`tune_tracker.py` replays cached truth-free detections through tracker parameter
grids. Ground truth is used only by this offline evaluator, never by the
tracker update.
