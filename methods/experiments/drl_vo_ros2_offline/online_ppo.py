"""Dependency-free PPO utilities for ROS 2 online DRL-VO fine-tuning."""
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：EXPECTED_POLICY_ITEMS, OBSERVATION_SIZE
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/online_ppo.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-31 12:40:47.903650164 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:00.813228349 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/online_ppo.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
import torch
from torch import nn

from methods.experiments.drl_vo_ros2_offline.drlvo_model import (
    DrlVoPolicy,
    load_policy_strict,
)


OBSERVATION_SIZE = 19202
EXPECTED_POLICY_ITEMS = 163


@dataclass(frozen=True)
class RewardConfig:
    progress_scale: float = 5.0
    step_penalty: float = -0.01
    proximity_distance_m: float = 0.9
    proximity_scale: float = 0.25
    angular_scale: float = 0.01
    success_reward: float = 20.0
    collision_reward: float = -20.0
    timeout_reward: float = -10.0
    goal_tolerance_m: float = 0.35
    collision_distance_m: float = 0.30

    def validate(self) -> None:
        numeric = asdict(self)
        if not all(math.isfinite(float(value)) for value in numeric.values()):
            raise ValueError("reward parameters must be finite")
        if self.proximity_distance_m <= self.collision_distance_m:
            raise ValueError(
                "proximity distance must exceed collision distance"
            )
        if self.goal_tolerance_m <= 0.0 or self.collision_distance_m <= 0.0:
            raise ValueError("goal and collision distances must be positive")


@dataclass(frozen=True)
class PPOConfig:
    learning_rate: float = 5e-5
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_range: float = 0.2
    value_coefficient: float = 0.5
    entropy_coefficient: float = 0.0
    max_grad_norm: float = 0.5
    update_epochs: int = 4
    batch_size: int = 64
    freeze_feature_extractor: bool = True

    def validate(self) -> None:
        if self.learning_rate <= 0.0:
            raise ValueError("learning rate must be positive")
        if not 0.0 < self.gamma <= 1.0:
            raise ValueError("gamma must be in (0,1]")
        if not 0.0 <= self.gae_lambda <= 1.0:
            raise ValueError("gae_lambda must be in [0,1]")
        if self.clip_range <= 0.0 or self.max_grad_norm <= 0.0:
            raise ValueError("clip range and max grad norm must be positive")
        if self.update_epochs < 1 or self.batch_size < 2:
            raise ValueError("update epochs and batch size are invalid")


def compute_training_reward(
    previous_goal_distance_m: float,
    goal_distance_m: float,
    minimum_scan_range_m: float,
    angular_velocity_radps: float,
    *,
    timed_out: bool,
    config: RewardConfig,
) -> tuple[float, bool, str]:
    """Compute a truth-free reward from goal progress and LiDAR clearance."""

    config.validate()
    values = (
        previous_goal_distance_m,
        goal_distance_m,
        angular_velocity_radps,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("reward state must be finite")
    if math.isnan(float(minimum_scan_range_m)):
        raise ValueError("minimum scan range cannot be NaN")
    if goal_distance_m <= config.goal_tolerance_m:
        return config.success_reward, True, "goal"
    if minimum_scan_range_m <= config.collision_distance_m:
        return config.collision_reward, True, "collision"
    if timed_out:
        return config.timeout_reward, True, "timeout"

    reward = (
        config.progress_scale
        * (previous_goal_distance_m - goal_distance_m)
        + config.step_penalty
        - config.angular_scale * abs(angular_velocity_radps)
    )
    if minimum_scan_range_m < config.proximity_distance_m:
        reward -= config.proximity_scale * (
            config.proximity_distance_m - minimum_scan_range_m
        )
    return float(reward), False, "running"


def generalized_advantage_estimate(
    rewards: np.ndarray,
    dones: np.ndarray,
    values: np.ndarray,
    bootstrap_value: float,
    gamma: float,
    gae_lambda: float,
) -> tuple[np.ndarray, np.ndarray]:
    rewards = np.asarray(rewards, dtype=np.float32).reshape(-1)
    dones = np.asarray(dones, dtype=np.float32).reshape(-1)
    values = np.asarray(values, dtype=np.float32).reshape(-1)
    if rewards.shape != dones.shape or rewards.shape != values.shape:
        raise ValueError("rewards, dones, and values must have equal shapes")
    if not len(rewards):
        raise ValueError("cannot estimate advantages for an empty rollout")
    if (
        not np.isfinite(rewards).all()
        or not np.isfinite(dones).all()
        or not np.isfinite(values).all()
        or not math.isfinite(float(bootstrap_value))
    ):
        raise ValueError("GAE inputs must be finite")
    if np.any((dones < 0.0) | (dones > 1.0)):
        raise ValueError("done flags must be in [0,1]")

    advantages = np.zeros_like(rewards)
    last_advantage = 0.0
    next_value = float(bootstrap_value)
    for index in range(len(rewards) - 1, -1, -1):
        nonterminal = 1.0 - float(dones[index])
        delta = (
            float(rewards[index])
            + gamma * next_value * nonterminal
            - float(values[index])
        )
        last_advantage = (
            delta + gamma * gae_lambda * nonterminal * last_advantage
        )
        advantages[index] = last_advantage
        next_value = float(values[index])
    returns = advantages + values
    return advantages.astype(np.float32), returns.astype(np.float32)


class RolloutBuffer:
    def __init__(self) -> None:
        self.clear()

    def clear(self) -> None:
        self.observations: list[np.ndarray] = []
        self.actions: list[np.ndarray] = []
        self.rewards: list[float] = []
        self.dones: list[float] = []
        self.log_probabilities: list[float] = []
        self.values: list[float] = []

    def __len__(self) -> int:
        return len(self.rewards)

    def add(
        self,
        observation: np.ndarray,
        action: np.ndarray,
        reward: float,
        done: bool,
        log_probability: float,
        value: float,
    ) -> None:
        observation = np.asarray(observation, dtype=np.float32).reshape(-1)
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if observation.shape != (OBSERVATION_SIZE,) or action.shape != (2,):
            raise ValueError("invalid rollout observation or action shape")
        scalars = (reward, log_probability, value)
        if (
            not np.isfinite(observation).all()
            or not np.isfinite(action).all()
            or not all(math.isfinite(float(item)) for item in scalars)
        ):
            raise ValueError("rollout transition must be finite")
        self.observations.append(observation.copy())
        self.actions.append(action.copy())
        self.rewards.append(float(reward))
        self.dones.append(float(bool(done)))
        self.log_probabilities.append(float(log_probability))
        self.values.append(float(value))


def load_online_policy(path: Path) -> tuple[DrlVoPolicy, int]:
    path = Path(path).expanduser().resolve()
    if path.suffix == ".zip":
        policy, count = load_policy_strict(path)
    else:
        state = torch.load(path, map_location="cpu", weights_only=True)
        if not isinstance(state, dict):
            raise TypeError("online PPO checkpoint must contain a state dict")
        policy = DrlVoPolicy()
        policy.load_state_dict(state, strict=True)
        count = len(state)
    if count != EXPECTED_POLICY_ITEMS:
        raise RuntimeError(
            f"DRL-VO policy has {count} items; expected {EXPECTED_POLICY_ITEMS}"
        )
    return policy, count


class OnlinePPO:
    """Small PyTorch PPO implementation for one ROS 2 environment."""

    def __init__(
        self,
        policy: DrlVoPolicy,
        device: torch.device,
        config: PPOConfig,
        seed: int,
    ) -> None:
        config.validate()
        device = torch.device(device)
        self.policy = policy.to(device)
        self.device = device
        self.config = config
        torch.manual_seed(int(seed))
        np.random.seed(int(seed))
        if device.type == "cuda":
            torch.cuda.manual_seed_all(int(seed))
        if config.freeze_feature_extractor:
            for parameter in self.policy.features_extractor.parameters():
                parameter.requires_grad = False
            self.policy.features_extractor.eval()
        parameters = [
            parameter
            for parameter in self.policy.parameters()
            if parameter.requires_grad
        ]
        self.optimizer = torch.optim.Adam(
            parameters, lr=config.learning_rate
        )

    def _distribution(
        self, observations: torch.Tensor
    ) -> tuple[torch.distributions.Normal, torch.Tensor]:
        means, values = self.policy(observations)
        log_std = self.policy.log_std.clamp(-20.0, 2.0)
        distribution = torch.distributions.Normal(
            means, log_std.exp().expand_as(means)
        )
        return distribution, values.squeeze(-1)

    def select_action(
        self, observation: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, float, float]:
        observation = np.asarray(observation, dtype=np.float32).reshape(-1)
        if (
            observation.shape != (OBSERVATION_SIZE,)
            or not np.isfinite(observation).all()
        ):
            raise ValueError("policy observation must be finite [19202]")
        self.policy.eval()
        with torch.no_grad():
            tensor = torch.from_numpy(observation).unsqueeze(0).to(self.device)
            distribution, value = self._distribution(tensor)
            raw_action = distribution.mean + distribution.stddev * torch.randn_like(
                distribution.mean
            )
            log_probability = distribution.log_prob(raw_action).sum(dim=1)
        raw = raw_action.squeeze(0).cpu().numpy().astype(np.float32)
        clipped = np.clip(raw, -1.0, 1.0).astype(np.float32)
        return (
            raw,
            clipped,
            float(log_probability.item()),
            float(value.item()),
        )

    def value(self, observation: np.ndarray) -> float:
        observation = np.asarray(observation, dtype=np.float32).reshape(-1)
        self.policy.eval()
        with torch.no_grad():
            tensor = torch.from_numpy(observation).unsqueeze(0).to(self.device)
            _, value = self._distribution(tensor)
        return float(value.item())

    def update(
        self,
        buffer: RolloutBuffer,
        bootstrap_value: float,
    ) -> dict[str, float | int]:
        if len(buffer) < 2:
            raise ValueError("PPO update requires at least two transitions")
        advantages, returns = generalized_advantage_estimate(
            np.asarray(buffer.rewards),
            np.asarray(buffer.dones),
            np.asarray(buffer.values),
            bootstrap_value,
            self.config.gamma,
            self.config.gae_lambda,
        )
        advantages = (
            advantages - float(np.mean(advantages))
        ) / (float(np.std(advantages)) + 1e-8)
        observations = np.stack(buffer.observations)
        actions = np.stack(buffer.actions)
        old_log_probabilities = np.asarray(
            buffer.log_probabilities, dtype=np.float32
        )
        count = len(buffer)
        totals = {
            "policy_loss": 0.0,
            "value_loss": 0.0,
            "entropy": 0.0,
            "approx_kl": 0.0,
            "clip_fraction": 0.0,
        }
        batches = 0
        self.policy.train()
        if self.config.freeze_feature_extractor:
            self.policy.features_extractor.eval()
        for _epoch in range(self.config.update_epochs):
            permutation = np.random.permutation(count)
            for start in range(0, count, self.config.batch_size):
                indices = permutation[start : start + self.config.batch_size]
                obs_batch = torch.from_numpy(observations[indices]).to(self.device)
                action_batch = torch.from_numpy(actions[indices]).to(self.device)
                old_log_batch = torch.from_numpy(
                    old_log_probabilities[indices]
                ).to(self.device)
                advantage_batch = torch.from_numpy(advantages[indices]).to(
                    self.device
                )
                return_batch = torch.from_numpy(returns[indices]).to(self.device)

                distribution, values = self._distribution(obs_batch)
                log_probabilities = distribution.log_prob(action_batch).sum(dim=1)
                entropy = distribution.entropy().sum(dim=1).mean()
                ratio = torch.exp(log_probabilities - old_log_batch)
                unclipped = ratio * advantage_batch
                clipped = torch.clamp(
                    ratio,
                    1.0 - self.config.clip_range,
                    1.0 + self.config.clip_range,
                ) * advantage_batch
                policy_loss = -torch.minimum(unclipped, clipped).mean()
                value_loss = torch.nn.functional.mse_loss(values, return_batch)
                loss = (
                    policy_loss
                    + self.config.value_coefficient * value_loss
                    - self.config.entropy_coefficient * entropy
                )
                if not torch.isfinite(loss):
                    raise ValueError("PPO loss became NaN or Inf")
                self.optimizer.zero_grad(set_to_none=True)
                loss.backward()
                nn.utils.clip_grad_norm_(
                    [
                        parameter
                        for parameter in self.policy.parameters()
                        if parameter.requires_grad
                    ],
                    self.config.max_grad_norm,
                )
                self.optimizer.step()

                with torch.no_grad():
                    log_ratio = log_probabilities - old_log_batch
                    totals["policy_loss"] += float(policy_loss)
                    totals["value_loss"] += float(value_loss)
                    totals["entropy"] += float(entropy)
                    totals["approx_kl"] += float(
                        ((torch.exp(log_ratio) - 1.0) - log_ratio).mean()
                    )
                    totals["clip_fraction"] += float(
                        (torch.abs(ratio - 1.0) > self.config.clip_range)
                        .float()
                        .mean()
                    )
                batches += 1
        buffer.clear()
        return {
            "transitions": count,
            "batches": batches,
            **{key: value / batches for key, value in totals.items()},
        }


def cpu_policy_state(policy: DrlVoPolicy) -> dict[str, torch.Tensor]:
    return {
        name: value.detach().cpu().clone()
        for name, value in policy.state_dict().items()
    }
