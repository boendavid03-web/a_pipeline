"""Lightweight pairwise quality metrics for pedestrian simulations.

The module deliberately has no Isaac Sim or ROS dependency so that the metric
contract can be unit-tested and reused by both interactive demos and offline
analysis.  One ``update`` call represents one sampled simulation frame.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence


Pair = tuple[str, str]


@dataclass(frozen=True)
class PairwiseSnapshot:
    """Pairwise measurements for one sampled frame."""

    sample_index: int
    pedestrian_count: int
    pair_count: int
    min_center_distance_m: float | None
    closest_pair: Pair | None
    visual_overlap_pairs: int
    personal_space_violation_pairs: int

    @property
    def has_visual_overlap(self) -> bool:
        return self.visual_overlap_pairs > 0

    @property
    def has_personal_space_violation(self) -> bool:
        return self.personal_space_violation_pairs > 0


@dataclass(frozen=True)
class YieldDecision:
    """Pedestrians that should begin or end a social yielding pause."""

    begin_yielding: tuple[str, ...]
    end_yielding: tuple[str, ...]
    active_yielders: tuple[str, ...]


class SocialYieldPlanner:
    """Choose one deterministic yielder before two people become too close.

    This class only makes decisions; the Isaac adapter owns animation tasks.
    A yielder remains paused until it is beyond ``resume_distance_m`` from
    every other pedestrian, preventing rapid stop/start oscillation.
    """

    def __init__(
        self,
        *,
        trigger_distance_m: float = 0.90,
        resume_distance_m: float = 1.10,
    ) -> None:
        self.trigger_distance_m = _finite_positive(
            "trigger_distance_m", trigger_distance_m
        )
        self.resume_distance_m = _finite_positive(
            "resume_distance_m", resume_distance_m
        )
        if self.resume_distance_m <= self.trigger_distance_m:
            raise ValueError(
                "resume_distance_m must be greater than trigger_distance_m"
            )
        self._active: set[str] = set()

    def update(self, positions: Mapping[str, Sequence[float]]) -> YieldDecision:
        points = _validated_planar_positions(positions)
        names = sorted(points)
        present = set(names)
        self._active.intersection_update(present)

        end: list[str] = []
        for name in sorted(self._active):
            x, y = points[name]
            if all(
                other == name
                or math.hypot(points[other][0] - x, points[other][1] - y)
                >= self.resume_distance_m
                for other in names
            ):
                self._active.remove(name)
                end.append(name)

        begin: list[str] = []
        for left_index, left_name in enumerate(names):
            left_x, left_y = points[left_name]
            for right_name in names[left_index + 1 :]:
                distance = math.hypot(
                    points[right_name][0] - left_x,
                    points[right_name][1] - left_y,
                )
                if distance >= self.trigger_distance_m:
                    continue
                if left_name in self._active or right_name in self._active:
                    continue
                # Stable ordering makes seeded runs reproducible.  The other
                # person keeps moving and native avoidance routes around the
                # now-stationary, still-visible BehaviorAgent obstacle.
                yielder = right_name
                self._active.add(yielder)
                begin.append(yielder)

        return YieldDecision(
            begin_yielding=tuple(begin),
            end_yielding=tuple(end),
            active_yielders=tuple(sorted(self._active)),
        )


class SocialQualityTracker:
    """Accumulate pairwise clearance metrics from planar pedestrian positions.

    ``positions`` passed to :meth:`update` maps stable pedestrian names to an
    ``(x, y, ...)`` sequence in metres.  A violation uses a strict ``<``
    comparison, so a pair exactly on a configured boundary is not counted.
    Pair ratios use all unique pair observations across all sampled frames as
    their denominator.  They are ``0.0`` when no pair has been observed.
    """

    def __init__(
        self,
        *,
        personal_space_m: float = 1.0,
        visual_overlap_m: float = 0.45,
    ) -> None:
        personal_space_m = _finite_positive(
            "personal_space_m", personal_space_m
        )
        visual_overlap_m = _finite_positive(
            "visual_overlap_m", visual_overlap_m
        )
        if visual_overlap_m > personal_space_m:
            raise ValueError(
                "visual_overlap_m must be less than or equal to personal_space_m"
            )

        self.personal_space_m = personal_space_m
        self.visual_overlap_m = visual_overlap_m
        self.reset()

    def reset(self) -> None:
        """Clear all accumulated samples while preserving thresholds."""

        self._sample_frames = 0
        self._pair_samples = 0
        self._min_center_distance_m: float | None = None
        self._closest_pair: Pair | None = None
        self._visual_overlap_frames = 0
        self._visual_overlap_pair_samples = 0
        self._personal_space_violation_frames = 0
        self._personal_space_violation_pair_samples = 0

    def update(
        self, positions: Mapping[str, Sequence[float]]
    ) -> PairwiseSnapshot:
        """Validate and record one frame, returning that frame's measurements.

        Validation happens before counters are changed, so a rejected frame
        never leaves the tracker partially updated.
        """

        points = _validated_planar_positions(positions)
        names = sorted(points)
        pair_count = len(names) * (len(names) - 1) // 2
        min_distance: float | None = None
        closest_pair: Pair | None = None
        visual_pairs = 0
        personal_pairs = 0

        for left_index, left_name in enumerate(names):
            left_x, left_y = points[left_name]
            for right_name in names[left_index + 1 :]:
                right_x, right_y = points[right_name]
                distance = math.hypot(right_x - left_x, right_y - left_y)
                pair = (left_name, right_name)
                if min_distance is None or distance < min_distance:
                    min_distance = distance
                    closest_pair = pair
                if distance < self.visual_overlap_m:
                    visual_pairs += 1
                if distance < self.personal_space_m:
                    personal_pairs += 1

        self._sample_frames += 1
        self._pair_samples += pair_count
        if min_distance is not None and (
            self._min_center_distance_m is None
            or min_distance < self._min_center_distance_m
        ):
            self._min_center_distance_m = min_distance
            self._closest_pair = closest_pair
        if visual_pairs:
            self._visual_overlap_frames += 1
        self._visual_overlap_pair_samples += visual_pairs
        if personal_pairs:
            self._personal_space_violation_frames += 1
        self._personal_space_violation_pair_samples += personal_pairs

        return PairwiseSnapshot(
            sample_index=self._sample_frames,
            pedestrian_count=len(names),
            pair_count=pair_count,
            min_center_distance_m=min_distance,
            closest_pair=closest_pair,
            visual_overlap_pairs=visual_pairs,
            personal_space_violation_pairs=personal_pairs,
        )

    def summary(self) -> dict[str, object]:
        """Return all accumulated metrics as a JSON-safe dictionary."""

        denominator = self._pair_samples
        return {
            "personal_space_m": self.personal_space_m,
            "visual_overlap_m": self.visual_overlap_m,
            "sample_frames": self._sample_frames,
            "pair_samples": denominator,
            "min_center_distance_m": self._min_center_distance_m,
            "closest_pair": (
                list(self._closest_pair) if self._closest_pair is not None else None
            ),
            "visual_overlap_frames": self._visual_overlap_frames,
            "visual_overlap_pair_samples": self._visual_overlap_pair_samples,
            "visual_overlap_pair_ratio": (
                self._visual_overlap_pair_samples / denominator
                if denominator
                else 0.0
            ),
            "personal_space_violation_frames": (
                self._personal_space_violation_frames
            ),
            "personal_space_violation_pair_samples": (
                self._personal_space_violation_pair_samples
            ),
            "personal_space_violation_pair_ratio": (
                self._personal_space_violation_pair_samples / denominator
                if denominator
                else 0.0
            ),
        }


def _finite_positive(name: str, value: float) -> float:
    try:
        numeric = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite positive number") from exc
    if not math.isfinite(numeric) or numeric <= 0.0:
        raise ValueError(f"{name} must be a finite positive number")
    return numeric


def _validated_planar_positions(
    positions: Mapping[str, Sequence[float]],
) -> dict[str, tuple[float, float]]:
    if not isinstance(positions, Mapping):
        raise TypeError("positions must be a mapping of pedestrian names to positions")

    validated: dict[str, tuple[float, float]] = {}
    for name, position in positions.items():
        if not isinstance(name, str) or not name:
            raise ValueError("pedestrian names must be non-empty strings")
        if isinstance(position, (str, bytes)):
            raise ValueError(f"position for {name!r} must contain finite x and y values")
        try:
            x = float(position[0])
            y = float(position[1])
        except (IndexError, KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                f"position for {name!r} must contain finite x and y values"
            ) from exc
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError(f"position for {name!r} must contain finite x and y values")
        validated[name] = (x, y)
    return validated
