#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：未检测到明显的大写环境变量。
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/experiments/dual_lidar_pedestrian_bev/model.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-30 12:36:31.422662451 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:18.377546444 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/render_semantic_cnn_offline_demo.py（导入其函数、类或模型）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/smoke_fixed_dual_training.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh（ros2 launch 启动该场景）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/s3net_fixed_dual_perception_demo.launch.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/s3net_fixed_dual_inference_node.py（导入其函数、类或模型）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_cnn_fixed_dual_inference_node.py（导入其函数、类或模型）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/experiments/dual_lidar_pedestrian_bev/model.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/render_semantic_cnn_offline_demo.py; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/smoke_fixed_dual_training.py; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/s3net_fixed_dual_perception_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/s3net_fixed_dual_inference_node.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_cnn_fixed_dual_inference_node.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
"""Small temporal BEV detector for pedestrian center and velocity prediction."""

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
