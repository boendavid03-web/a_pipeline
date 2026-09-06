#!/usr/bin/env python3
"""Validate the tracked, immutable scenario-topology A baseline."""

from __future__ import annotations

import hashlib
import json
import math
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
BASELINE_DIR = PROJECT_ROOT / "runs/scenario_topology_ab/baseline"
MANIFEST_PATH = BASELINE_DIR / "baseline_manifest.yaml"
CHECKSUMS_PATH = BASELINE_DIR / "SHA256SUMS"
EXPECTED_SCHEMA = "scenario_topology_baseline_manifest/v2"


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    return sha256_bytes(path.read_bytes())


def canonical_path_sha256(points: list[list[float]]) -> str:
    payload = json.dumps(
        points, ensure_ascii=False, separators=(",", ":")
    ).encode("utf-8")
    return sha256_bytes(payload)


def git_output(*arguments: str) -> str:
    return subprocess.check_output(
        ["git", *arguments], cwd=PROJECT_ROOT, text=True
    ).strip()


def load_manifest() -> dict[str, Any]:
    return yaml.safe_load(MANIFEST_PATH.read_text(encoding="utf-8"))


def load_baseline_config(manifest: dict[str, Any]) -> dict[str, Any]:
    record = manifest["outputs"]["baseline_config"]
    return yaml.safe_load(
        (PROJECT_ROOT / str(record["path"])).read_text(encoding="utf-8")
    )


def config_groups(config: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return config["isaacsim.replicator.agent"]["character"]["groups"]


def group_patrol(group: dict[str, Any]) -> dict[str, Any]:
    patrols = [routine["patrol"] for routine in group["routines"] if "patrol" in routine]
    if len(patrols) != 1:
        raise ValueError(f"expected exactly one patrol, found {len(patrols)}")
    return patrols[0]


def validate_checksums() -> list[str]:
    errors: list[str] = []
    if not CHECKSUMS_PATH.is_file():
        return [f"missing checksum file {CHECKSUMS_PATH.relative_to(PROJECT_ROOT)}"]
    expected_names = {"A_baseline.yaml", "baseline_manifest.yaml"}
    records: dict[str, str] = {}
    for line_number, line in enumerate(
        CHECKSUMS_PATH.read_text(encoding="utf-8").splitlines(), start=1
    ):
        fields = line.split("  ", 1)
        if len(fields) != 2 or len(fields[0]) != 64:
            errors.append(f"SHA256SUMS:{line_number}: malformed record")
            continue
        digest, name = fields
        records[name] = digest
    if set(records) != expected_names:
        errors.append(
            f"SHA256SUMS entries are {sorted(records)}, expected {sorted(expected_names)}"
        )
    for name, expected_hash in records.items():
        path = BASELINE_DIR / name
        if not path.is_file():
            errors.append(f"SHA256SUMS: missing {name}")
        elif sha256_file(path) != expected_hash:
            errors.append(f"SHA256SUMS: {name} hash mismatch")
    return errors


def validate_environment(manifest: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    expected = manifest["environment"]
    actual_python = ".".join(str(value) for value in sys.version_info[:3])
    if actual_python != str(expected["python"]["version"]):
        errors.append(
            f"python version is {actual_python}, expected {expected['python']['version']}"
        )
    if yaml.__version__ != str(expected["yaml_parser"]["version"]):
        errors.append(
            f"PyYAML version is {yaml.__version__}, expected {expected['yaml_parser']['version']}"
        )
    isaac = expected["isaac_sim"]
    evidence_path = PROJECT_ROOT / str(isaac["evidence"])
    version_marker = f"Isaac Sim version: {isaac['version']}"
    if version_marker not in evidence_path.read_text(encoding="utf-8"):
        errors.append(
            f"Isaac version evidence {version_marker!r} missing from {isaac['evidence']}"
        )
    return errors


def validate_provenance(manifest: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    baseline_commit = str(manifest["baseline_commit"])
    for role, record in manifest["provenance"].items():
        relative_path = str(record["path"])
        expected_hash = str(record["sha256"])
        if role == "generator_at_baseline_commit":
            try:
                payload = subprocess.check_output(
                    ["git", "show", f"{baseline_commit}:{relative_path}"],
                    cwd=PROJECT_ROOT,
                )
            except subprocess.CalledProcessError:
                errors.append(f"{role}: cannot read {relative_path} at {baseline_commit}")
                continue
            actual_hash = sha256_bytes(payload)
        else:
            path = PROJECT_ROOT / relative_path
            if not path.is_file():
                errors.append(f"{role}: missing file {relative_path}")
                continue
            actual_hash = sha256_file(path)
        if actual_hash != expected_hash:
            errors.append(
                f"{role}: SHA256 {actual_hash} does not match {expected_hash}"
            )
    return errors


def validate_git_anchor(manifest: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if manifest.get("schema") != EXPECTED_SCHEMA:
        errors.append(f"manifest schema must be {EXPECTED_SCHEMA}")
    baseline_commit = str(manifest["baseline_commit"])
    if git_output("rev-parse", baseline_commit) != baseline_commit:
        errors.append(f"baseline commit is not canonical: {baseline_commit}")
    ancestor = subprocess.run(
        ["git", "merge-base", "--is-ancestor", baseline_commit, "HEAD"],
        cwd=PROJECT_ROOT,
        check=False,
    )
    if ancestor.returncode != 0:
        errors.append(f"baseline commit {baseline_commit} is not an ancestor of HEAD")
    current_branch = git_output("branch", "--show-current")
    if current_branch != str(manifest["branch"]):
        errors.append(
            f"current branch is {current_branch!r}, expected {manifest['branch']!r}"
        )
    for name in ("A_baseline.yaml", "baseline_manifest.yaml", "SHA256SUMS"):
        relative_path = (BASELINE_DIR / name).relative_to(PROJECT_ROOT)
        result = subprocess.run(
            ["git", "check-ignore", "--quiet", str(relative_path)],
            cwd=PROJECT_ROOT,
            check=False,
        )
        if result.returncode == 0:
            errors.append(f"tracked baseline anchor remains ignored: {name}")
        tracked = subprocess.run(
            ["git", "ls-files", "--error-unmatch", "--", str(relative_path)],
            cwd=PROJECT_ROOT,
            check=False,
            capture_output=True,
        )
        if tracked.returncode != 0:
            errors.append(f"baseline anchor is not tracked by Git: {name}")
    ignored_runtime_copy = PROJECT_ROOT / "runs/scenario_topology_ab/configs/A_baseline.yaml"
    if subprocess.run(
        ["git", "check-ignore", "--quiet", str(ignored_runtime_copy)],
        cwd=PROJECT_ROOT,
        check=False,
    ).returncode != 0:
        errors.append("runtime configs directory must remain ignored")
    return errors


def build_logical_topology(manifest: dict[str, Any], config: dict[str, Any]) -> dict[str, Any]:
    provenance = manifest["provenance"]
    xml_path = PROJECT_ROOT / str(provenance["scenario_xml"]["path"])
    template_path = PROJECT_ROOT / str(provenance["template"]["path"])
    root = ET.parse(xml_path).getroot()
    waypoint_elements = {
        element.attrib["id"]: element for element in root if element.tag == "waypoint"
    }
    agents = [
        element
        for element in root
        if element.tag == "agent" and int(element.attrib.get("type", "0")) != 2
    ]
    template = yaml.safe_load(template_path.read_text(encoding="utf-8"))
    route_families = list(config_groups(template))
    if len(agents) != len(route_families):
        raise ValueError(
            f"XML/template cluster count mismatch: {len(agents)} != {len(route_families)}"
        )
    clusters: list[dict[str, Any]] = []
    for index, (agent, route_family) in enumerate(zip(agents, route_families)):
        cluster_id = f"gazebo_{chr(ord('a') + index)}"
        waypoint_ids = [
            child.attrib["id"] for child in agent if child.tag == "addwaypoint"
        ]
        clusters.append(
            {
                "cluster_id": cluster_id,
                "source_agent_order": index,
                "route_family": route_family,
                "waypoints": [
                    {
                        "waypoint_id": waypoint_id,
                        "waypoint_order": order,
                        "waypoint_radius_m": float(waypoint_elements[waypoint_id].attrib["r"]),
                    }
                    for order, waypoint_id in enumerate(waypoint_ids)
                ],
            }
        )
    clusters_by_id = {cluster["cluster_id"]: cluster for cluster in clusters}
    pedestrians: list[dict[str, Any]] = []
    for pedestrian_id in config_groups(config):
        source_cluster, person_number = pedestrian_id.rsplit("_", 1)
        cluster = clusters_by_id[source_cluster]
        pedestrians.append(
            {
                "id": pedestrian_id,
                "source_cluster": source_cluster,
                "route_family": cluster["route_family"],
                "route_phase": (int(person_number) - 1) % len(cluster["waypoints"]),
            }
        )
    return {"waypoint_clusters": clusters, "pedestrians": pedestrians}


def validate_topology_snapshot(
    manifest: dict[str, Any], config: dict[str, Any]
) -> list[str]:
    try:
        actual = build_logical_topology(manifest, config)
    except (KeyError, TypeError, ValueError) as exc:
        return [f"logical topology extraction failed: {exc}"]
    if actual != manifest["logical_topology_snapshot"]:
        return ["logical topology snapshot does not match XML/template/pedestrian IDs"]
    return []


def validate_semantic_snapshot(
    manifest: dict[str, Any], config: dict[str, Any]
) -> list[str]:
    errors: list[str] = []
    snapshot = manifest["semantic_snapshot"]
    pedestrian = snapshot["pedestrian"]
    groups = config_groups(config)
    ordered_ids = list(groups)
    if ordered_ids != pedestrian["ordered_ids"]:
        errors.append("ordered pedestrian ID list changed")
    seed = config["isaacsim.replicator.agent"]["seed"]
    if seed != pedestrian["seed"] or seed != manifest["parameters"]["seed"]:
        errors.append("pedestrian seed changed")
    counts: list[int] = []
    topology = manifest["logical_topology_snapshot"]
    for cluster in topology["waypoint_clusters"]:
        prefix = str(cluster["cluster_id"]) + "_"
        counts.append(sum(name.startswith(prefix) for name in ordered_ids))
    if counts != pedestrian["allocation"]:
        errors.append(f"pedestrian allocation changed: {counts}")
    speeds: list[float] = []
    records: list[dict[str, Any]] = []
    spawn_positions: list[list[float]] = []
    total_points = 0
    for pedestrian_id, group in groups.items():
        try:
            patrol = group_patrol(group)
        except ValueError as exc:
            errors.append(f"{pedestrian_id}: {exc}")
            continue
        speed_range = [float(value) for value in patrol["speed_range"]]
        if len(speed_range) != 2 or speed_range[0] != speed_range[1]:
            errors.append(f"{pedestrian_id}: speed range is not a fixed sequence value")
        points = [[float(value) for value in point] for point in patrol["path_points"]]
        speed = speed_range[0]
        speeds.append(speed)
        spawn_positions.append(points[0])
        total_points += len(points)
        records.append(
            {
                "id": pedestrian_id,
                "spawn_position": points[0],
                "path_point_count": len(points),
                "path_sha256": canonical_path_sha256(points),
            }
        )
    if speeds != pedestrian["speed_sequence_mps"]:
        errors.append("pedestrian speed sequence changed")
    if records != pedestrian["records"]:
        errors.append("pedestrian spawn/path semantic records changed")
    output = manifest["outputs"]["baseline_config"]
    if len(groups) != int(output["people"]):
        errors.append(f"people count is {len(groups)}, expected {output['people']}")
    if total_points != int(output["patrol_points"]):
        errors.append(f"patrol point total is {total_points}, expected {output['patrol_points']}")
    scenario = snapshot["scenario"]
    duration = config["isaacsim.replicator.agent"]["simulation_duration"]
    if duration != scenario["simulation_duration_s"]:
        errors.append("scenario duration changed")
    if scenario["free_space_clearance_m"] != manifest["parameters"]["free_space_clearance_m"]:
        errors.append("scenario clearance contract is inconsistent")
    minimum_separation = float(scenario["minimum_spawn_separation_m"])
    if minimum_separation != manifest["parameters"]["spawn_clearance_m"]:
        errors.append("minimum spawn separation contract is inconsistent")
    for index, first in enumerate(spawn_positions):
        for second in spawn_positions[index + 1 :]:
            if math.dist(first[:2], second[:2]) + 1.0e-9 < minimum_separation:
                errors.append("baseline contains spawn positions below minimum separation")
                break
    expected_families = [
        {
            "route_family": cluster["route_family"],
            "canonical_cyclic_order": [
                waypoint["waypoint_id"] for waypoint in cluster["waypoints"]
            ],
            "direction": "forward",
        }
        for cluster in topology["waypoint_clusters"]
    ]
    if snapshot["route"]["families"] != expected_families:
        errors.append("route family/order/direction snapshot is inconsistent")
    expected_phases = {
        record["id"]: record["route_phase"] for record in topology["pedestrians"]
    }
    if snapshot["route"]["phases"] != expected_phases:
        errors.append("route phase snapshot is inconsistent")
    return errors


def generator_command(
    manifest: dict[str, Any], output: Path, script: Path, *, explicit: bool
) -> list[str]:
    contract = manifest["generator_contract"]
    arguments = [
        str(output) if value == "{output}" else str(value)
        for value in contract["arguments"]
    ]
    if explicit:
        arguments.extend(str(value) for value in contract["explicit_baseline_arguments"])
    return [str(contract["executable"]), str(script), *arguments]


def run_generator(command: list[str]) -> tuple[bytes | None, str | None]:
    completed = subprocess.run(
        command,
        cwd=PROJECT_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip()
        return None, detail
    output_index = command.index("--output") + 1
    return Path(command[output_index]).read_bytes(), None


def validate_generator_byte_identity(manifest: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    contract = manifest["generator_contract"]
    generator_path = PROJECT_ROOT / str(contract["script"])
    baseline_commit = str(manifest["baseline_commit"])
    old_payload = subprocess.check_output(
        ["git", "show", f"{baseline_commit}:{contract['script']}"], cwd=PROJECT_ROOT
    )
    sealed_path = PROJECT_ROOT / str(manifest["outputs"]["baseline_config"]["path"])
    sealed_payload = sealed_path.read_bytes()
    temporary_old_script: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            dir=generator_path.parent,
            prefix=".scenario_topology_baseline_commit_",
            suffix=".py",
            delete=False,
        ) as handle:
            handle.write(old_payload)
            temporary_old_script = Path(handle.name)
        with tempfile.TemporaryDirectory(prefix="scenario_topology_baseline_") as directory:
            root = Path(directory)
            variants = (
                ("baseline_commit_generator", temporary_old_script, False),
                ("current_default_generator", generator_path, False),
                ("current_explicit_baseline_generator", generator_path, True),
            )
            payloads: dict[str, bytes] = {}
            for name, script, explicit in variants:
                output = root / f"{name}.yaml"
                payload, failure = run_generator(
                    generator_command(manifest, output, script, explicit=explicit)
                )
                if failure is not None:
                    errors.append(f"{name} failed: {failure}")
                elif payload is not None:
                    payloads[name] = payload
            for name, payload in payloads.items():
                if payload != sealed_payload:
                    errors.append(f"{name} output is not byte-identical to sealed A")
            if len(payloads) == 3 and len(set(payloads.values())) != 1:
                errors.append("three generator variants are not byte-identical")
    finally:
        if temporary_old_script is not None:
            try:
                os.unlink(temporary_old_script)
            except FileNotFoundError:
                pass
    return errors


def main() -> int:
    errors: list[str] = []
    try:
        manifest = load_manifest()
        config = load_baseline_config(manifest)
    except (KeyError, OSError, TypeError, yaml.YAMLError) as exc:
        print("SCENARIO_TOPOLOGY_BASELINE_CHECK=FAIL")
        print(f"- cannot load baseline anchor: {exc}")
        return 1
    errors.extend(validate_checksums())
    errors.extend(validate_git_anchor(manifest))
    errors.extend(validate_environment(manifest))
    errors.extend(validate_provenance(manifest))
    output = manifest["outputs"]["baseline_config"]
    baseline_path = PROJECT_ROOT / str(output["path"])
    if sha256_file(baseline_path) != str(output["sha256"]):
        errors.append("sealed A_baseline.yaml hash does not match manifest")
    errors.extend(validate_semantic_snapshot(manifest, config))
    errors.extend(validate_topology_snapshot(manifest, config))
    if not errors:
        errors.extend(validate_generator_byte_identity(manifest))
    if errors:
        print("SCENARIO_TOPOLOGY_BASELINE_CHECK=FAIL")
        for error in errors:
            print(f"- {error}")
        return 1
    print(
        "SCENARIO_TOPOLOGY_BASELINE_CHECK=PASS "
        "byte_identity=old==default==explicit_baseline "
        "semantic_snapshot=PASS topology_snapshot=PASS "
        f"config_sha256={output['sha256']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
