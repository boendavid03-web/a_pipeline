"""Low-overhead opt-in rolling phase profiler for Isaac runtime diagnostics."""

from __future__ import annotations

import math
import time
from collections import defaultdict
from dataclasses import dataclass, field


def _percentile(sorted_values: list[float], percentile: float) -> float:
    if not sorted_values:
        raise ValueError("percentile requires at least one value")
    position = (len(sorted_values) - 1) * percentile / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return sorted_values[lower]
    weight = position - lower
    return sorted_values[lower] * (1.0 - weight) + sorted_values[upper] * weight


@dataclass
class _PhaseSamples:
    window: list[float] = field(default_factory=list)
    total: list[float] = field(default_factory=list)


class PerformanceProfiler:
    """Collect phase durations and emit bounded-frequency rolling summaries.

    The profiler is deliberately single-threaded: every current call site runs
    on Kit's main thread.  Avoiding locks keeps enabled profiling overhead low.
    """

    def __init__(
        self,
        enabled: bool,
        report_interval_sec: float = 15.0,
        *,
        clock=time.perf_counter,
    ) -> None:
        if report_interval_sec <= 0.0 or not math.isfinite(report_interval_sec):
            raise ValueError("report_interval_sec must be positive and finite")
        self.enabled = bool(enabled)
        self.report_interval_sec = float(report_interval_sec)
        self._clock = clock
        self._started = self._clock()
        self._window_started = self._started
        self._phases: dict[str, _PhaseSamples] = defaultdict(_PhaseSamples)

    def restart(self) -> None:
        """Discard setup samples and anchor both windows at the current time."""
        current = self._clock()
        self._started = current
        self._window_started = current
        self._phases.clear()

    def record(self, phase: str, elapsed_sec: float) -> None:
        if not self.enabled:
            return
        value = float(elapsed_sec)
        if not phase or value < 0.0 or not math.isfinite(value):
            raise ValueError("phase must be non-empty and elapsed_sec non-negative")
        samples = self._phases[phase]
        samples.window.append(value)
        samples.total.append(value)

    def due(self, now: float | None = None) -> bool:
        if not self.enabled:
            return False
        current = self._clock() if now is None else float(now)
        return current - self._window_started >= self.report_interval_sec

    @staticmethod
    def _phase_summary(values: list[float], wall_sec: float) -> dict[str, float | int]:
        ordered = sorted(values)
        count = len(ordered)
        total = sum(ordered)
        return {
            "calls": count,
            "calls_per_sec": count / wall_sec,
            "mean_ms": total * 1000.0 / count,
            "p50_ms": _percentile(ordered, 50.0) * 1000.0,
            "p95_ms": _percentile(ordered, 95.0) * 1000.0,
            "max_ms": ordered[-1] * 1000.0,
            "total_ms": total * 1000.0,
            "wall_share": total / wall_sec,
        }

    def summary(
        self,
        *,
        total: bool = False,
        reset_window: bool = False,
        now: float | None = None,
    ) -> dict[str, object]:
        current = self._clock() if now is None else float(now)
        started = self._started if total else self._window_started
        wall_sec = max(1.0e-9, current - started)
        phase_summaries = {}
        for name, samples in sorted(self._phases.items()):
            values = samples.total if total else samples.window
            if values:
                phase_summaries[name] = self._phase_summary(values, wall_sec)
        result: dict[str, object] = {
            "schema": "isaac_phase_profile/v1",
            "scope": "total" if total else "rolling",
            "wall_sec": wall_sec,
            "phases": phase_summaries,
        }
        if reset_window and not total:
            for samples in self._phases.values():
                samples.window.clear()
            self._window_started = current
        return result
