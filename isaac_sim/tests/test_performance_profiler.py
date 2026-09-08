import math
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from performance_profiler import PerformanceProfiler  # noqa: E402


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


def test_disabled_profiler_is_inert() -> None:
    clock = FakeClock()
    profiler = PerformanceProfiler(False, clock=clock)
    profiler.record("ignored", 1.0)
    clock.now = 20.0
    assert not profiler.due()
    assert profiler.summary(total=True)["phases"] == {}


def test_rolling_statistics_and_reset() -> None:
    clock = FakeClock()
    profiler = PerformanceProfiler(True, report_interval_sec=10.0, clock=clock)
    for value in (0.001, 0.002, 0.003, 0.004):
        profiler.record("phase", value)
    clock.now = 10.0
    assert profiler.due()
    result = profiler.summary(reset_window=True)
    phase = result["phases"]["phase"]
    assert phase["calls"] == 4
    assert math.isclose(phase["calls_per_sec"], 0.4)
    assert math.isclose(phase["mean_ms"], 2.5)
    assert math.isclose(phase["p50_ms"], 2.5)
    assert math.isclose(phase["p95_ms"], 3.85)
    assert math.isclose(phase["max_ms"], 4.0)
    assert profiler.summary()["phases"] == {}


def test_total_survives_rolling_reset() -> None:
    clock = FakeClock()
    profiler = PerformanceProfiler(True, report_interval_sec=5.0, clock=clock)
    profiler.record("a", 0.1)
    clock.now = 5.0
    profiler.summary(reset_window=True)
    profiler.record("a", 0.2)
    clock.now = 10.0
    total = profiler.summary(total=True)
    assert total["phases"]["a"]["calls"] == 2
    assert math.isclose(total["phases"]["a"]["mean_ms"], 150.0)
