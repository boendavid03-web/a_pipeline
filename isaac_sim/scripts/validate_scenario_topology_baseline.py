#!/usr/bin/env python3
"""Read-only validation of the sealed e314f56 scenario-topology baseline."""

from __future__ import annotations

import hashlib
import subprocess
import tempfile
from pathlib import Path

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
MANIFEST_PATH = PROJECT_ROOT / "runs/scenario_topology_ab/baseline_manifest.yaml"
EXPECTED_SCHEMA = "scenario_topology_baseline_manifest/v1"


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    return sha256_bytes(path.read_bytes())


def git_output(*arguments: str) -> str:
    return subprocess.check_output(
        ["git", *arguments], cwd=PROJECT_ROOT, text=True
    ).strip()


def main() -> int:
    errors: list[str] = []
    manifest = yaml.safe_load(MANIFEST_PATH.read_text(encoding="utf-8"))
    if manifest.get("schema") != EXPECTED_SCHEMA:
        errors.append(f"manifest schema must be {EXPECTED_SCHEMA}")

    baseline_commit = str(manifest["baseline_commit"])
    resolved_commit = git_output("rev-parse", baseline_commit)
    if resolved_commit != baseline_commit:
        errors.append(
            f"baseline commit resolved to {resolved_commit}, expected {baseline_commit}"
        )
    ancestor = subprocess.run(
        ["git", "merge-base", "--is-ancestor", baseline_commit, "HEAD"],
        cwd=PROJECT_ROOT,
        check=False,
    )
    if ancestor.returncode != 0:
        errors.append(f"baseline commit {baseline_commit} is not an ancestor of HEAD")

    expected_branch = str(manifest["branch"])
    current_branch = git_output("branch", "--show-current")
    if current_branch != expected_branch:
        errors.append(
            f"current branch is {current_branch!r}, expected {expected_branch!r}"
        )

    provenance = manifest["provenance"]
    for role, record in provenance.items():
        relative_path = str(record["path"])
        expected_hash = str(record["sha256"])
        path = PROJECT_ROOT / relative_path
        if not path.is_file():
            errors.append(f"{role}: missing file {relative_path}")
            continue
        current_hash = sha256_file(path)
        # The generator may gain a B-only branch later. Its current bytes may
        # change, but its default output must still reproduce this sealed A.
        if role != "generator" and current_hash != expected_hash:
            errors.append(
                f"{role}: SHA256 {current_hash} does not match {expected_hash}"
            )

    generator_record = provenance["generator"]
    generator_path = str(generator_record["path"])
    baseline_generator = subprocess.check_output(
        ["git", "show", f"{baseline_commit}:{generator_path}"], cwd=PROJECT_ROOT
    )
    baseline_generator_hash = sha256_bytes(baseline_generator)
    if baseline_generator_hash != str(generator_record["sha256"]):
        errors.append(
            "generator: e314f56 blob SHA256 "
            f"{baseline_generator_hash} does not match manifest"
        )

    output_record = manifest["outputs"]["baseline_config"]
    baseline_path = PROJECT_ROOT / str(output_record["path"])
    if not baseline_path.is_file():
        errors.append(f"baseline config is missing: {baseline_path}")
    else:
        baseline_hash = sha256_file(baseline_path)
        if baseline_hash != str(output_record["sha256"]):
            errors.append(
                f"baseline config SHA256 {baseline_hash} does not match manifest"
            )

    if not errors:
        with tempfile.TemporaryDirectory(prefix="scenario_topology_baseline_") as tmp:
            regenerated_path = Path(tmp) / "A_baseline.yaml"
            invocation = manifest["generator_invocation"]
            arguments = [
                str(regenerated_path) if value == "{output}" else str(value)
                for value in invocation["arguments"]
            ]
            completed = subprocess.run(
                [
                    str(invocation["executable"]),
                    str(PROJECT_ROOT / str(invocation["script"])),
                    *arguments,
                ],
                cwd=PROJECT_ROOT,
                check=False,
                capture_output=True,
                text=True,
            )
            if completed.returncode != 0:
                errors.append(
                    "baseline regeneration failed: "
                    + (completed.stderr.strip() or completed.stdout.strip())
                )
            elif regenerated_path.read_bytes() != baseline_path.read_bytes():
                errors.append(
                    "current generator output is not byte-identical to sealed A_baseline.yaml"
                )

    if errors:
        print("SCENARIO_TOPOLOGY_BASELINE_CHECK=FAIL")
        for error in errors:
            print(f"- {error}")
        return 1

    print(
        "SCENARIO_TOPOLOGY_BASELINE_CHECK=PASS "
        f"commit={baseline_commit} "
        f"branch={current_branch} "
        f"config_sha256={output_record['sha256']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
