#!/usr/bin/env python3
"""Run bounded 1+1 Isaac bring-up repetitions and record parent-side metrics."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import re
import subprocess
import sys
import time


def _rss_mib(pid: int) -> float:
    try:
        text = Path(f"/proc/{pid}/status").read_text()
    except (FileNotFoundError, ProcessLookupError):
        return 0.0
    match = re.search(r"^VmRSS:\s+(\d+)\s+kB$", text, re.MULTILINE)
    return float(match.group(1)) / 1024.0 if match else 0.0


def _gpu_mib(pid: int) -> float:
    query = subprocess.run(
        [
            "nvidia-smi",
            "--query-compute-apps=pid,used_gpu_memory",
            "--format=csv,noheader,nounits",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    total = 0.0
    for line in query.stdout.splitlines():
        fields = [field.strip() for field in line.split(",")]
        if len(fields) == 2 and fields[0] == str(pid):
            try:
                total += float(fields[1])
            except ValueError:
                pass
    return total


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--steps", type=int, default=5)
    parser.add_argument("--timeout", type=float, default=600.0)
    parser.add_argument("--config", default="CrowdSim/config/env_smoke.yaml")
    parser.add_argument(
        "--log-dir",
        default=str(Path(__file__).resolve().parents[2] / "Assets/repro_logs/acceptance_1p1r"),
    )
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    log_dir = Path(args.log_dir).expanduser().resolve()
    log_dir.mkdir(parents=True, exist_ok=True)

    overall_ok = True
    for run_id in range(1, args.runs + 1):
        log_path = log_dir / f"shutdown_repeat_{run_id:02d}.log"
        command = [
            sys.executable,
            "-u",
            "scripts/smoke_isaac_bringup.py",
            "--config",
            args.config,
            "--steps",
            str(args.steps),
        ]
        started_unix = time.time()
        started_iso = datetime.now(timezone.utc).isoformat()
        peak_rss_mib = 0.0
        peak_gpu_mib = 0.0
        timed_out = False
        with log_path.open("w", encoding="utf-8") as log:
            header = (
                f"[runner] run={run_id} started={started_iso}\n"
                f"[runner] command={' '.join(command)}\n"
            )
            log.write(header)
            log.flush()
            print(header, end="", flush=True)
            proc = subprocess.Popen(
                command,
                cwd=root,
                env=os.environ.copy(),
                stdout=log,
                stderr=subprocess.STDOUT,
                text=True,
            )
            while proc.poll() is None:
                peak_rss_mib = max(peak_rss_mib, _rss_mib(proc.pid))
                peak_gpu_mib = max(peak_gpu_mib, _gpu_mib(proc.pid))
                if time.time() - started_unix > args.timeout:
                    timed_out = True
                    proc.terminate()
                    try:
                        proc.wait(timeout=30.0)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait()
                    break
                time.sleep(0.5)
            exit_code = int(proc.returncode)
            elapsed = time.time() - started_unix
            text = log_path.read_text(encoding="utf-8", errors="replace")
            close_match = re.search(r"\[shutdown\] close_started_unix=([0-9.]+)", text)
            shutdown_seconds = (
                time.time() - float(close_match.group(1)) if close_match else float("nan")
            )
            summary = (
                f"[runner] exit_code={exit_code} timed_out={timed_out} "
                f"elapsed_seconds={elapsed:.3f} shutdown_seconds={shutdown_seconds:.3f} "
                f"max_rss_mib={peak_rss_mib:.1f} max_vram_mib={peak_gpu_mib:.1f}\n"
            )
            log.write(summary)
            log.flush()
            print(summary, end="", flush=True)
        ok = exit_code == 0 and not timed_out and "STAGE observation OK" in text
        overall_ok &= ok
        print(f"[runner] run={run_id} result={'PASS' if ok else 'FAIL'} log={log_path}", flush=True)
        if not ok:
            break
    return 0 if overall_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
