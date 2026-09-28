#!/usr/bin/env python3
"""CPU-only checkpoint deserialization check; it does not start ROS nodes."""
from pathlib import Path


root = Path(__file__).resolve().parents[1]
try:
    import torch
except Exception as exc:
    raise SystemExit(f'ERROR: PyTorch import failed: {exc}')

models = (
    root / 'checkpoints/s3net/s3net_native_stats_best_dev.pth',
    root / 'checkpoints/semantic_cnn/semantic_cnn_native_cmd_best_dev.pth',
    root / 'checkpoints/dr_spaam/ckpt_jrdb_ann_ft_dr_spaam_e20.pth',
    root / 'checkpoints/drl_vo/base_bc_best.pt',
)
for path in models:
    try:
        payload = torch.load(path, map_location='cpu', weights_only=False)
    except Exception as exc:
        raise SystemExit(f'FAIL {path.name}: {exc}')
    kind = type(payload).__name__
    count = len(payload) if hasattr(payload, '__len__') else 'n/a'
    print(f'PASS {path.name}: {kind}, entries={count}')
