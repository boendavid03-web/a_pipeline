from __future__ import annotations

import copy
import hashlib
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = PROJECT_ROOT / "isaac_sim/scripts"
sys.path.insert(0, str(SCRIPT_DIR))

import validate_scenario_topology_baseline as baseline  # noqa: E402
from generate_free_space_people_config import (  # noqa: E402
    SPREAD_RADIUS_FRACTION,
    load_gazebo_clusters,
    spread_waypoint_route,
)


B_DIR = PROJECT_ROOT / "runs/scenario_topology_ab/B_spread_radius"
B_CONFIG_PATH = B_DIR / "B_spread_radius.yaml"
B_MANIFEST_PATH = B_DIR / "B_manifest.yaml"
B_CHECKSUMS_PATH = B_DIR / "SHA256SUMS"


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def groups(config: dict) -> dict:
    return config["isaacsim.replicator.agent"]["character"]["groups"]


def patrol(group: dict) -> dict:
    matches = [routine["patrol"] for routine in group["routines"] if "patrol" in routine]
    if len(matches) != 1:
        raise AssertionError(f"expected one patrol, found {len(matches)}")
    return matches[0]


def allocation(ordered_ids: list[str]) -> list[int]:
    return [
        sum(name.startswith(f"gazebo_{letter}_") for name in ordered_ids)
        for letter in "abcdef"
    ]


class ScenarioTopologySpreadRadiusContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.a_manifest = baseline.load_manifest()
        cls.a_config = baseline.load_baseline_config(cls.a_manifest)
        cls.b_manifest = yaml.safe_load(B_MANIFEST_PATH.read_text(encoding="utf-8"))
        cls.b_config = yaml.safe_load(B_CONFIG_PATH.read_text(encoding="utf-8"))
        cls.a_groups = groups(cls.a_config)
        cls.b_groups = groups(cls.b_config)

    def test_a_default_and_explicit_baseline_remain_byte_identical(self) -> None:
        self.assertEqual([], baseline.validate_generator_byte_identity(self.a_manifest))

    def test_b_artifact_checksums_are_complete(self) -> None:
        records = {}
        for line in B_CHECKSUMS_PATH.read_text(encoding="utf-8").splitlines():
            digest, name = line.split("  ", 1)
            records[name] = digest
        self.assertEqual(records.keys(), {"B_spread_radius.yaml", "B_manifest.yaml"})
        for name, digest in records.items():
            self.assertEqual(sha256_file(B_DIR / name), digest)
        self.assertEqual(self.b_manifest["outputs"]["config_sha256"], records["B_spread_radius.yaml"])

    def test_b_generator_reproduces_tracked_artifact(self) -> None:
        generator = self.b_manifest["generator"]
        with tempfile.TemporaryDirectory(prefix="scenario_topology_b_") as directory:
            output = Path(directory) / "B_spread_radius.yaml"
            arguments = list(generator["arguments"])
            output_index = arguments.index("--output") + 1
            arguments[output_index] = str(output)
            completed = subprocess.run(
                [generator["executable"], generator["script"], *arguments],
                cwd=PROJECT_ROOT,
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr or completed.stdout)
            self.assertEqual(output.read_bytes(), B_CONFIG_PATH.read_bytes())

    def test_b_changes_only_spawn_and_path_geometry(self) -> None:
        a_ids = list(self.a_groups)
        b_ids = list(self.b_groups)
        self.assertEqual(a_ids, b_ids)
        changed_spawns = []
        changed_paths = []
        changed_endpoints = []
        for pedestrian_id in a_ids:
            a_group = copy.deepcopy(self.a_groups[pedestrian_id])
            b_group = copy.deepcopy(self.b_groups[pedestrian_id])
            a_patrol = patrol(a_group)
            b_patrol = patrol(b_group)
            if a_patrol["path_points"][0] != b_patrol["path_points"][0]:
                changed_spawns.append(pedestrian_id)
            if a_patrol["path_points"] != b_patrol["path_points"]:
                changed_paths.append(pedestrian_id)
            if a_patrol["path_points"][-1] != b_patrol["path_points"][-1]:
                changed_endpoints.append(pedestrian_id)
            a_patrol.pop("path_points")
            b_patrol.pop("path_points")
            self.assertEqual(a_group, b_group, pedestrian_id)
        expected = self.b_manifest["allowed_changes"]
        self.assertEqual(len(changed_spawns), expected["spawn_changed_people"])
        self.assertEqual(len(changed_endpoints), expected["path_endpoint_changed_people"])
        self.assertEqual(len(changed_paths), expected["path_changed_people"])
        self.assertGreater(len(changed_spawns), 0)
        self.assertEqual(changed_paths, a_ids)

    def test_b_forbidden_invariants_are_unchanged(self) -> None:
        a_ids = list(self.a_groups)
        b_ids = list(self.b_groups)
        self.assertEqual(b_ids, a_ids)
        self.assertEqual(allocation(b_ids), self.a_manifest["parameters"]["allocation"])
        self.assertEqual(
            self.b_config["isaacsim.replicator.agent"]["seed"],
            self.a_config["isaacsim.replicator.agent"]["seed"],
        )
        self.assertEqual(
            [patrol(group)["speed_range"] for group in self.b_groups.values()],
            [patrol(group)["speed_range"] for group in self.a_groups.values()],
        )

        clusters = load_gazebo_clusters(
            PROJECT_ROOT / self.a_manifest["provenance"]["scenario_xml"]["path"]
        )
        topology = self.a_manifest["logical_topology_snapshot"]
        self.assertEqual(
            [cluster["route_ids"] for cluster in clusters],
            [
                [waypoint["waypoint_id"] for waypoint in cluster["waypoints"]]
                for cluster in topology["waypoint_clusters"]
            ],
        )
        expected_phases = topology["pedestrians"]
        self.assertEqual(
            [
                {
                    "id": pedestrian_id,
                    "source_cluster": pedestrian_id.rsplit("_", 1)[0],
                    "route_family": topology["waypoint_clusters"][
                        ord(pedestrian_id[7]) - ord("a")
                    ]["route_family"],
                    "route_phase": (int(pedestrian_id.rsplit("_", 1)[1]) - 1)
                    % len(
                        topology["waypoint_clusters"][
                            ord(pedestrian_id[7]) - ord("a")
                        ]["waypoints"]
                    ),
                }
                for pedestrian_id in b_ids
            ],
            expected_phases,
        )
        topology_sha = hashlib.sha256(
            json.dumps(topology, sort_keys=True, separators=(",", ":")).encode("utf-8")
        ).hexdigest()
        self.assertEqual(
            topology_sha,
            self.b_manifest["forbidden_invariants"]["logical_topology_sha256"],
        )

    def test_spread_targets_stay_inside_the_same_ordered_waypoint_radii(self) -> None:
        clusters = load_gazebo_clusters(
            PROJECT_ROOT / self.a_manifest["provenance"]["scenario_xml"]["path"]
        )
        target_count = 0
        for cluster_index, cluster in enumerate(clusters):
            count = self.a_manifest["parameters"]["allocation"][cluster_index]
            for person_index in range(count):
                targets = spread_waypoint_route(
                    cluster["route"],
                    cluster["route_radii"],
                    (0.25, -0.4),
                    person_index,
                )
                self.assertEqual(len(targets), len(cluster["route_ids"]))
                for center, radius, target in zip(
                    cluster["route"], cluster["route_radii"], targets
                ):
                    self.assertLessEqual(math.dist(center, target), radius)
                    self.assertAlmostEqual(
                        math.dist(center, target),
                        radius * SPREAD_RADIUS_FRACTION,
                    )
                    self.assertNotEqual(center, target)
                    target_count += 1
        self.assertEqual(
            target_count,
            self.b_manifest["allowed_changes"]["waypoint_arrival_targets_changed"],
        )


if __name__ == "__main__":
    unittest.main()
