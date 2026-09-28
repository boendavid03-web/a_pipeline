from __future__ import annotations

import io
import zipfile
from collections import OrderedDict
from pathlib import Path

import torch
from torch import nn


def conv3x3(
    in_planes: int,
    out_planes: int,
    stride: int = 1,
    groups: int = 1,
    dilation: int = 1,
) -> nn.Conv2d:
    return nn.Conv2d(
        in_planes,
        out_planes,
        kernel_size=3,
        stride=stride,
        padding=dilation,
        groups=groups,
        bias=False,
        dilation=dilation,
    )


def conv1x1(in_planes: int, out_planes: int, stride: int = 1) -> nn.Conv2d:
    return nn.Conv2d(in_planes, out_planes, kernel_size=1, stride=stride, bias=False)


class Bottleneck(nn.Module):
    expansion = 2

    def __init__(
        self,
        inplanes: int,
        planes: int,
        stride: int = 1,
        downsample: nn.Module | None = None,
        groups: int = 1,
        base_width: int = 64,
        dilation: int = 1,
        norm_layer: type[nn.Module] = nn.BatchNorm2d,
    ) -> None:
        super().__init__()
        width = int(planes * (base_width / 64.0)) * groups
        self.conv1 = conv1x1(inplanes, width)
        self.bn1 = norm_layer(width)
        self.conv2 = conv3x3(width, width, stride, groups, dilation)
        self.bn2 = norm_layer(width)
        self.conv3 = conv1x1(width, planes * self.expansion)
        self.bn3 = norm_layer(planes * self.expansion)
        self.relu = nn.ReLU(inplace=True)
        self.downsample = downsample

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        identity = x
        out = self.relu(self.bn1(self.conv1(x)))
        out = self.relu(self.bn2(self.conv2(out)))
        out = self.bn3(self.conv3(out))
        if self.downsample is not None:
            identity = self.downsample(x)
        return self.relu(out + identity)


class CustomCNN(nn.Module):
    """Exact network topology from the original custom_cnn_full.CustomCNN."""

    def __init__(self, features_dim: int = 256) -> None:
        super().__init__()
        self.inplanes = 64
        self.groups = 1
        self.base_width = 64
        self.conv1 = nn.Conv2d(3, 64, kernel_size=3, stride=1, padding=1, bias=False)
        self.bn1 = nn.BatchNorm2d(64)
        self.relu = nn.ReLU(inplace=True)
        self.maxpool = nn.MaxPool2d(kernel_size=3, stride=1, padding=1)
        self.layer1 = self._make_layer(64, blocks=2)
        self.layer2 = self._make_layer(128, blocks=1, stride=2)
        self.layer3 = self._make_layer(256, blocks=1, stride=2)

        self.conv2_2 = nn.Sequential(
            nn.Conv2d(256, 128, kernel_size=1),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),
            nn.Conv2d(128, 128, kernel_size=3, padding=1),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),
            nn.Conv2d(128, 256, kernel_size=1),
            nn.BatchNorm2d(256),
        )
        self.downsample2 = nn.Sequential(
            nn.Conv2d(128, 256, kernel_size=1, stride=2),
            nn.BatchNorm2d(256),
        )
        self.relu2 = nn.ReLU(inplace=True)

        self.conv3_2 = nn.Sequential(
            nn.Conv2d(512, 256, kernel_size=1),
            nn.BatchNorm2d(256),
            nn.ReLU(inplace=True),
            nn.Conv2d(256, 256, kernel_size=3, padding=1),
            nn.BatchNorm2d(256),
            nn.ReLU(inplace=True),
            nn.Conv2d(256, 512, kernel_size=1),
            nn.BatchNorm2d(512),
        )
        self.downsample3 = nn.Sequential(
            nn.Conv2d(64, 512, kernel_size=1, stride=4),
            nn.BatchNorm2d(512),
        )
        self.relu3 = nn.ReLU(inplace=True)
        self.avgpool = nn.AdaptiveAvgPool2d((1, 1))
        self.linear_fc = nn.Sequential(
            nn.Linear(514, features_dim),
            nn.ReLU(),
        )

    def _make_layer(self, planes: int, blocks: int, stride: int = 1) -> nn.Sequential:
        downsample = None
        if stride != 1 or self.inplanes != planes * Bottleneck.expansion:
            downsample = nn.Sequential(
                conv1x1(self.inplanes, planes * Bottleneck.expansion, stride),
                nn.BatchNorm2d(planes * Bottleneck.expansion),
            )
        layers = [
            Bottleneck(
                self.inplanes,
                planes,
                stride,
                downsample,
                self.groups,
                self.base_width,
            )
        ]
        self.inplanes = planes * Bottleneck.expansion
        for _ in range(1, blocks):
            layers.append(
                Bottleneck(
                    self.inplanes,
                    planes,
                    groups=self.groups,
                    base_width=self.base_width,
                )
            )
        return nn.Sequential(*layers)

    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        ped = observations[:, :12800].reshape(-1, 2, 80, 80)
        scan = observations[:, 12800:19200].reshape(-1, 1, 80, 80)
        goal = observations[:, 19200:].reshape(-1, 2)
        x = torch.cat((scan, ped), dim=1)
        x = self.maxpool(self.relu(self.bn1(self.conv1(x))))
        identity3 = self.downsample3(x)
        x = self.layer1(x)
        identity2 = self.downsample2(x)
        x = self.relu2(self.conv2_2(self.layer2(x)) + identity2)
        x = self.relu3(self.conv3_2(self.layer3(x)) + identity3)
        x = torch.flatten(self.avgpool(x), 1)
        return self.linear_fc(torch.cat((x, goal), dim=1))


class MlpExtractor(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.policy_net = nn.Sequential(nn.Linear(256, 256), nn.Tanh())
        self.value_net = nn.Sequential(nn.Linear(256, 128), nn.Tanh())


class DrlVoPolicy(nn.Module):
    """SB3 0.10 ActorCriticCnnPolicy state-dict-compatible reconstruction."""

    def __init__(self) -> None:
        super().__init__()
        self.log_std = nn.Parameter(torch.zeros(2))
        self.features_extractor = CustomCNN(features_dim=256)
        self.mlp_extractor = MlpExtractor()
        self.action_net = nn.Linear(256, 2)
        self.value_net = nn.Linear(128, 1)

    def forward(self, observations: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        features = self.features_extractor(observations)
        policy_latent = self.mlp_extractor.policy_net(features)
        value_latent = self.mlp_extractor.value_net(features)
        return self.action_net(policy_latent), self.value_net(value_latent)

    def deterministic_action(self, observations: torch.Tensor) -> torch.Tensor:
        action_mean, _ = self(observations)
        return torch.clamp(action_mean, -1.0, 1.0)


class SemanticFeatureFusion(nn.Module):
    """Encode categorical semantics without changing the pretrained DRL-VO trunk."""

    def __init__(
        self,
        num_classes: int,
        embedding_dim: int = 4,
        features_dim: int = 256,
    ) -> None:
        super().__init__()
        if num_classes <= 0:
            raise ValueError("num_classes must be positive")
        self.num_classes = num_classes
        self.embedding = nn.Embedding(
            num_classes + 1,
            embedding_dim,
            padding_idx=0,
        )
        self.encoder = nn.Sequential(
            nn.Conv2d(embedding_dim, 16, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(16, 32, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((1, 1)),
        )
        self.projection = nn.Linear(64, features_dim)
        # The optional branch starts as an exact no-op. This preserves the
        # pretrained policy output until semantic training is explicitly run.
        nn.init.zeros_(self.projection.weight)
        nn.init.zeros_(self.projection.bias)

    def forward(
        self,
        base_features: torch.Tensor,
        semantic_maps: torch.Tensor,
    ) -> torch.Tensor:
        if semantic_maps.ndim != 3 or semantic_maps.shape[1:] != (80, 80):
            raise ValueError(
                f"Expected semantic maps (batch, 80, 80), got {tuple(semantic_maps.shape)}"
            )
        labels = semantic_maps.to(dtype=torch.long)
        if torch.any(labels < -1) or torch.any(labels >= self.num_classes):
            minimum = int(torch.min(labels))
            maximum = int(torch.max(labels))
            raise ValueError(
                f"Semantic labels must be in [-1, {self.num_classes - 1}], "
                f"got [{minimum}, {maximum}]"
            )
        embedded = self.embedding(labels + 1).permute(0, 3, 1, 2)
        semantic_features = torch.flatten(self.encoder(embedded), 1)
        return base_features + self.projection(semantic_features)


class SemanticDrlVoPolicy(DrlVoPolicy):
    """DRL-VO policy with an optional late-fusion categorical semantic branch."""

    def __init__(self, semantic_num_classes: int) -> None:
        super().__init__()
        self.semantic_fusion = SemanticFeatureFusion(semantic_num_classes)

    def forward(
        self,
        observations: torch.Tensor,
        semantic_maps: torch.Tensor | None = None,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        features = self.features_extractor(observations)
        if semantic_maps is not None:
            features = self.semantic_fusion(features, semantic_maps)
        policy_latent = self.mlp_extractor.policy_net(features)
        value_latent = self.mlp_extractor.value_net(features)
        return self.action_net(policy_latent), self.value_net(value_latent)

    def deterministic_action(
        self,
        observations: torch.Tensor,
        semantic_maps: torch.Tensor | None = None,
    ) -> torch.Tensor:
        action_mean, _ = self(observations, semantic_maps)
        return torch.clamp(action_mean, -1.0, 1.0)


def read_policy_state(model_zip: str | Path) -> OrderedDict[str, torch.Tensor]:
    with zipfile.ZipFile(model_zip, "r") as archive:
        payload = archive.read("policy.pth")
    state = torch.load(io.BytesIO(payload), map_location="cpu", weights_only=False)
    if not isinstance(state, OrderedDict):
        raise TypeError(f"Expected OrderedDict policy state, got {type(state)!r}")
    return state


def load_policy_strict(model_zip: str | Path) -> tuple[DrlVoPolicy, int]:
    state = read_policy_state(model_zip)
    policy = DrlVoPolicy()
    policy.load_state_dict(state, strict=True)
    policy.eval()
    return policy, len(state)


def load_semantic_policy(
    model_zip: str | Path,
    semantic_num_classes: int,
) -> tuple[SemanticDrlVoPolicy, int]:
    """Load all original weights and leave only the new semantic branch initialized."""

    state = read_policy_state(model_zip)
    policy = SemanticDrlVoPolicy(semantic_num_classes)
    incompatible = policy.load_state_dict(state, strict=False)
    expected_missing = {
        name
        for name in policy.state_dict()
        if name.startswith("semantic_fusion.")
    }
    if set(incompatible.missing_keys) != expected_missing:
        raise RuntimeError(
            "Unexpected missing pretrained weights: "
            f"{sorted(set(incompatible.missing_keys) - expected_missing)}"
        )
    if incompatible.unexpected_keys:
        raise RuntimeError(
            f"Unexpected pretrained weights: {sorted(incompatible.unexpected_keys)}"
        )
    policy.eval()
    return policy, len(state)


def load_trained_semantic_policy(
    checkpoint: str | Path,
    semantic_num_classes: int,
) -> tuple[SemanticDrlVoPolicy, int]:
    """Strictly load a checkpoint produced by semantic behavior cloning."""

    state = torch.load(
        checkpoint,
        map_location="cpu",
        weights_only=True,
    )
    if not isinstance(state, dict):
        raise TypeError(f"Expected checkpoint state dict, got {type(state)!r}")
    policy = SemanticDrlVoPolicy(semantic_num_classes)
    policy.load_state_dict(state, strict=True)
    policy.eval()
    return policy, len(state)
