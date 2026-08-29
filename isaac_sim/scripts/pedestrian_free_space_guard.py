"""Free-space intrusion tracking and last-safe live-pose recovery."""

from __future__ import annotations

from dataclasses import dataclass
import math
from collections.abc import Callable, Mapping, Sequence
from typing import Any


@dataclass(frozen=True)
class IntrusionSnapshot:
    """Free-space status for one sampled frame."""

    sample_index: int
    unsafe_people: tuple[str, ...]
    sustained_intrusions: tuple[str, ...]


class SustainedIntrusionTracker:
    """Count distinct sustained intrusions without mutating the simulation.

    An event begins when a person has been unsafe for ``sustained_samples``
    consecutive observations.  It is counted once until that person returns to
    free space, even if the unsafe run continues for many more samples.
    """

    def __init__(self, *, sustained_samples: int = 3) -> None:
        if isinstance(sustained_samples, bool) or not isinstance(
            sustained_samples, int
        ):
            raise ValueError("sustained_samples must be a positive integer")
        if sustained_samples <= 0:
            raise ValueError("sustained_samples must be a positive integer")
        self.sustained_samples = sustained_samples
        self.reset()

    def reset(self) -> None:
        self._sample_frames = 0
        self._unsafe_sample_count = 0
        self._event_count = 0
        self._consecutive_unsafe: dict[str, int] = {}
        self._active_intrusions: set[str] = set()
        self._involved_people: set[str] = set()

    def update(self, safe_by_person: Mapping[str, bool]) -> IntrusionSnapshot:
        if not isinstance(safe_by_person, Mapping):
            raise TypeError("safe_by_person must be a mapping")
        validated: dict[str, bool] = {}
        for name, is_safe in safe_by_person.items():
            if not isinstance(name, str) or not name:
                raise ValueError("pedestrian names must be non-empty strings")
            if not isinstance(is_safe, bool):
                raise ValueError(f"free-space status for {name!r} must be boolean")
            validated[name] = is_safe

        self._sample_frames += 1
        unsafe_people: list[str] = []
        new_intrusions: list[str] = []
        for name in sorted(validated):
            if validated[name]:
                self._consecutive_unsafe[name] = 0
                self._active_intrusions.discard(name)
                continue
            unsafe_people.append(name)
            self._unsafe_sample_count += 1
            consecutive = self._consecutive_unsafe.get(name, 0) + 1
            self._consecutive_unsafe[name] = consecutive
            if (
                consecutive >= self.sustained_samples
                and name not in self._active_intrusions
            ):
                self._active_intrusions.add(name)
                self._involved_people.add(name)
                self._event_count += 1
                new_intrusions.append(name)

        # Retire agents no longer present so a later reappearance starts a new
        # observation run instead of inheriting stale state.
        missing = set(self._consecutive_unsafe) - set(validated)
        for name in missing:
            self._consecutive_unsafe.pop(name, None)
            self._active_intrusions.discard(name)

        return IntrusionSnapshot(
            sample_index=self._sample_frames,
            unsafe_people=tuple(unsafe_people),
            sustained_intrusions=tuple(new_intrusions),
        )

    def summary(self) -> dict[str, object]:
        return {
            "sustained_samples": self.sustained_samples,
            "sample_frames": self._sample_frames,
            "unsafe_person_samples": self._unsafe_sample_count,
            "sustained_intrusion_count": self._event_count,
            "involved_people": sorted(self._involved_people),
            "active_intrusions": sorted(self._active_intrusions),
            "pending_unsafe_samples": {
                name: count
                for name, count in sorted(self._consecutive_unsafe.items())
                if count and name not in self._active_intrusions
            },
        }


class LastSafePositionGuard:
    """Recover live agents from unsafe points or unsafe last-safe edges.

    The tracker receives only the real point-membership observation.  A
    recovery is never fed back as a synthetic observation; the next live
    readback must prove both point and edge safety before pending recovery is
    cleared.  One transient post-teleport unsafe readback is tolerated while
    the simulator settles; a second consecutive unsafe readback still fails.
    """

    def __init__(
        self,
        *,
        free_space: Any,
        agents: Mapping[str, Any],
        tracker: SustainedIntrusionTracker,
        target_factory: Callable[[Sequence[float]], Any],
    ) -> None:
        if not isinstance(agents, Mapping):
            raise TypeError("agents must be a mapping")
        if not isinstance(tracker, SustainedIntrusionTracker):
            raise TypeError("tracker must be a SustainedIntrusionTracker")
        if not callable(target_factory):
            raise TypeError("target_factory must be callable")
        self.free_space = free_space
        self.agents = agents
        self.tracker = tracker
        self.target_factory = target_factory
        self.last_safe_positions: dict[str, tuple[float, ...]] = {}
        self.pending_recovery: set[str] = set()
        self.recovery_count = 0

    @staticmethod
    def _finite_position(name: str, position: Sequence[float]) -> tuple[float, ...]:
        if isinstance(position, (str, bytes)):
            raise RuntimeError(f"non-finite live position for {name!r}")
        try:
            values = tuple(float(value) for value in position)
        except (TypeError, ValueError) as exc:
            raise RuntimeError(f"invalid live position for {name!r}") from exc
        if len(values) < 2 or not all(math.isfinite(value) for value in values):
            raise RuntimeError(f"non-finite live position for {name!r}")
        return values

    def _point_safe(self, position: tuple[float, ...]) -> bool:
        return bool(self.free_space.contains_world(position[0], position[1]))

    def _edge_safe(
        self, start: tuple[float, ...], end: tuple[float, ...]
    ) -> bool:
        return bool(
            self.free_space.segment_world_free(
                start[0], start[1], end[0], end[1]
            )
        )

    def _recover(self, name: str) -> None:
        safe_position = self.last_safe_positions.get(name)
        if safe_position is None:
            raise RuntimeError(f"unsafe pedestrian {name!r} has no last-safe position")
        agent = self.agents.get(name)
        if agent is None:
            raise RuntimeError(f"BehaviorAgent missing for unsafe pedestrian {name!r}")
        try:
            teleported = bool(
                agent.teleport(target=self.target_factory(safe_position))
            )
        except Exception as exc:
            raise RuntimeError(
                f"BehaviorAgent teleport failed for unsafe pedestrian {name!r}"
            ) from exc
        if not teleported:
            raise RuntimeError(
                f"BehaviorAgent refused last-safe teleport for {name!r}"
            )
        self.pending_recovery.add(name)
        self.recovery_count += 1

    def update(
        self, live_positions: Mapping[str, Sequence[float]]
    ) -> IntrusionSnapshot:
        if not isinstance(live_positions, Mapping):
            raise TypeError("live_positions must be a mapping")
        positions = {
            name: self._finite_position(name, position)
            for name, position in live_positions.items()
        }
        if any(not isinstance(name, str) or not name for name in positions):
            raise RuntimeError("pedestrian names must be non-empty strings")
        missing_live_positions = set(self.agents) - set(positions)
        if missing_live_positions:
            raise RuntimeError(
                "live positions missing for pedestrians: "
                + ", ".join(sorted(missing_live_positions))
            )
        point_safe = {name: self._point_safe(position) for name, position in positions.items()}
        snapshot = self.tracker.update(point_safe)
        for name in sorted(positions):
            if self.agents.get(name) is None:
                raise RuntimeError(f"BehaviorAgent missing for pedestrian {name!r}")
            current = positions[name]
            if name in self.pending_recovery:
                safe_position = self.last_safe_positions.get(name)
                if safe_position is None:
                    raise RuntimeError(
                        f"unsafe live readback after recovery for pedestrian {name!r}"
                    )
                if point_safe[name] and self._edge_safe(safe_position, current):
                    self.pending_recovery.remove(name)
                    self.last_safe_positions[name] = current
                    continue
                raise RuntimeError(
                    f"unsafe live readback after recovery for pedestrian {name!r}"
                )
            if point_safe[name]:
                safe_position = self.last_safe_positions.get(name)
                if safe_position is None:
                    if not self._edge_safe(current, current):
                        raise RuntimeError(
                            f"safe point has unsafe self-edge for pedestrian {name!r}"
                        )
                    self.last_safe_positions[name] = current
                elif self._edge_safe(safe_position, current):
                    self.last_safe_positions[name] = current
                else:
                    self._recover(name)
            else:
                self._recover(name)
        return snapshot

    def summary(self) -> dict[str, object]:
        return {
            "recovery_count": self.recovery_count,
            "pending_recovery": sorted(self.pending_recovery),
            "last_safe_people": sorted(self.last_safe_positions),
        }
