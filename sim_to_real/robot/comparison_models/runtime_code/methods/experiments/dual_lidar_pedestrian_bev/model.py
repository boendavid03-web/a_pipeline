
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

import numpy as np
import torch
from torch import nn
from torch.nn import functional as F

from .dataset import BEVSpec


class _ConvBlock(nn.Module):
    def __init__(self, input_channels: int, output_channels: int) -> None:
        super().__init__()
        self.block = nn.Sequential(
            nn.Conv2d(input_channels, output_channels, 3, padding=1, bias=False),
            nn.BatchNorm2d(output_channels),
            nn.SiLU(inplace=True),
            nn.Conv2d(output_channels, output_channels, 3, padding=1, bias=False),
            nn.BatchNorm2d(output_channels),
            nn.SiLU(inplace=True),
        )

    def forward(self, inputs: torch.Tensor) -> torch.Tensor:
        return self.block(inputs)


class TemporalBEVPedestrianDetector(nn.Module):
    """U-Net-style dense center/velocity predictor."""

    def __init__(self, history_frames: int = 8, base_channels: int = 24) -> None:
        super().__init__()
        if history_frames < 1 or base_channels < 8:
            raise ValueError("invalid detector dimensions")
        self.history_frames = int(history_frames)
        self.base_channels = int(base_channels)
        input_channels = self.history_frames * 2
        self.stem = _ConvBlock(input_channels, base_channels)
        self.down1 = nn.Sequential(
            nn.MaxPool2d(2), _ConvBlock(base_channels, base_channels * 2)
        )
        self.down2 = nn.Sequential(
            nn.MaxPool2d(2), _ConvBlock(base_channels * 2, base_channels * 4)
        )
        self.up1 = nn.ConvTranspose2d(
            base_channels * 4, base_channels * 2, kernel_size=2, stride=2
        )
        self.decode1 = _ConvBlock(base_channels * 4, base_channels * 2)
        self.up2 = nn.ConvTranspose2d(
            base_channels * 2, base_channels, kernel_size=2, stride=2
        )
        self.decode2 = _ConvBlock(base_channels * 2, base_channels)
        self.heatmap_head = nn.Sequential(
            nn.Conv2d(base_channels, base_channels, 3, padding=1),
            nn.SiLU(inplace=True),
            nn.Conv2d(base_channels, 1, 1),
        )
        self.offset_head = nn.Sequential(
            nn.Conv2d(base_channels, base_channels, 3, padding=1),
            nn.SiLU(inplace=True),
            nn.Conv2d(base_channels, 2, 1),
        )
        self.velocity_head = nn.Sequential(
            nn.Conv2d(base_channels, base_channels, 3, padding=1),
            nn.SiLU(inplace=True),
            nn.Conv2d(base_channels, 2, 1),
        )
        nn.init.constant_(self.heatmap_head[-1].bias, -2.19)

    def forward(self, inputs: torch.Tensor) -> Dict[str, torch.Tensor]:
        if inputs.ndim != 4 or inputs.shape[1] != self.history_frames * 2:
            raise ValueError(
                f"input must have shape [B,{self.history_frames * 2},H,W]"
            )
        level0 = self.stem(inputs)
        level1 = self.down1(level0)
        level2 = self.down2(level1)
        decoded1 = self.decode1(torch.cat((self.up1(level2), level1), dim=1))
        decoded2 = self.decode2(torch.cat((self.up2(decoded1), level0), dim=1))
        return {
            "heatmap_logits": self.heatmap_head(decoded2),
            "offset": torch.sigmoid(self.offset_head(decoded2)),
            "velocity": self.velocity_head(decoded2),
        }


def _heatmap_focal_loss(
    logits: torch.Tensor, target: torch.Tensor
) -> torch.Tensor:
    prediction = torch.sigmoid(logits).clamp(1e-4, 1.0 - 1e-4)
    positive = target.eq(1.0).to(logits.dtype)
    negative = target.lt(1.0).to(logits.dtype)
    negative_weight = torch.pow(1.0 - target, 4.0)
    positive_loss = (
        torch.log(prediction) * torch.pow(1.0 - prediction, 2.0) * positive
    )
    negative_loss = (
        torch.log(1.0 - prediction)
        * torch.pow(prediction, 2.0)
        * negative_weight
        * negative
    )
    positive_count = positive.sum()
    if float(positive_count.detach()) > 0.0:
        return -(positive_loss.sum() + negative_loss.sum()) / positive_count
    return -negative_loss.sum()


def detection_loss(
    outputs: Dict[str, torch.Tensor],
    batch: Dict[str, torch.Tensor],
    *,
    offset_weight: float = 1.0,
    velocity_weight: float = 0.5,
) -> Dict[str, torch.Tensor]:
    heatmap_loss = _heatmap_focal_loss(
        outputs["heatmap_logits"], batch["heatmap"]
    )
    mask = batch["regression_mask"]
    denominator = mask.sum().clamp_min(1.0)
    offset_loss = (
        F.smooth_l1_loss(outputs["offset"], batch["offset"], reduction="none")
        * mask
    ).sum() / denominator
    velocity_loss = (
        F.smooth_l1_loss(
            outputs["velocity"], batch["velocity"], reduction="none"
        )
        * mask
    ).sum() / denominator
    total = (
        heatmap_loss
        + float(offset_weight) * offset_loss
        + float(velocity_weight) * velocity_loss
    )
    return {
        "loss": total,
        "heatmap_loss": heatmap_loss,
        "offset_loss": offset_loss,
        "velocity_loss": velocity_loss,
    }


@dataclass(frozen=True)
class DecodedDetection:
    position_xy_base: np.ndarray
    velocity_xy_robot_axes_absolute: np.ndarray
    confidence: float


def decode_detections(
    outputs: Dict[str, torch.Tensor],
    bev_spec: BEVSpec,
    *,
    confidence_threshold: float = 0.30,
    topk: int = 30,
    nms_radius_m: float = 0.30,
) -> List[List[DecodedDetection]]:
    if topk < 1:
        raise ValueError("topk must be positive")
    if nms_radius_m < 0.0:
        raise ValueError("nms_radius_m cannot be negative")
    scores = torch.sigmoid(outputs["heatmap_logits"])
    local_maximum = scores.eq(F.max_pool2d(scores, 3, stride=1, padding=1))
    scores = scores * local_maximum
    batch_size, _, height, width = scores.shape
    candidate_multiplier = 4 if nms_radius_m > 0.0 else 1
    count = min(int(topk) * candidate_multiplier, height * width)
    top_scores, top_indices = torch.topk(scores.reshape(batch_size, -1), count)
    decoded: List[List[DecodedDetection]] = []
    for batch_index in range(batch_size):
        items: List[DecodedDetection] = []
        for rank in range(count):
            confidence = float(top_scores[batch_index, rank].detach().cpu())
            if confidence < confidence_threshold:
                continue
            flat_index = int(top_indices[batch_index, rank].detach().cpu())
            row = flat_index // width
            col = flat_index % width
            offset = (
                outputs["offset"][batch_index, :, row, col]
                .detach()
                .cpu()
                .numpy()
            )
            grid_x = float(col) + float(offset[0])
            grid_y = float(row) + float(offset[1])
            position = bev_spec.grid_to_metric(
                np.asarray(grid_x), np.asarray(grid_y)
            ).astype(np.float64)
            velocity = (
                outputs["velocity"][batch_index, :, row, col]
                .detach()
                .cpu()
                .numpy()
                .astype(np.float64)
            )
            if any(
                np.linalg.norm(position - item.position_xy_base)
                < nms_radius_m
                for item in items
            ):
                continue
            items.append(
                DecodedDetection(
                    position_xy_base=position,
                    velocity_xy_robot_axes_absolute=velocity,
                    confidence=confidence,
                )
            )
            if len(items) >= topk:
                break
        decoded.append(items)
    return decoded
