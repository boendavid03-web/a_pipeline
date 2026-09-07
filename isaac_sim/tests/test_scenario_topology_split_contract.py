from __future__ import annotations

import copy
import hashlib
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
    select_scenario_ab_geometry,
    spread_spawn_point,
    spread_waypoint_route,
)


CONFIG_PATHS = {
    "A": PROJECT_ROOT / "runs/scenario_topology_ab/baseline/A_baseline.yaml",
    "S": PROJECT_ROOT / "runs/scenario_topology_ab/S_spawn_only/S_spawn_only.yaml",
    "R": PROJECT_ROOT / "runs/scenario_topology_ab/R_radius_only/R_radius_only.yaml",
    "B": PROJECT_ROOT / "runs/scenario_topology_ab/B_spread_radius/B_spread_radius.yaml",
}
MANIFEST_PATHS = {
    "S": PROJECT_ROOT / "runs/scenario_topology_ab/S_spawn_only/S_manifest.yaml",
    "R": PROJECT_ROOT / "runs/scenario_topology_ab/R_radius_only/R_manifest.yaml",
}
CHECKSUM_PATHS = {
    "S": PROJECT_ROOT / "runs/scenario_topology_ab/S_spawn_only/SHA256SUMS",
    "R": PROJECT_ROOT / "runs/scenario_topology_ab/R_radius_only/SHA256SUMS",
}
EXPECTED_SHA256 = {
    "A": "76896643184fd7fbfa26c43727cce49452bbe313fdae177de1065acb3a23303d",
    "S": "19ae11ec72633d1274caa499274fe18e42b5af83289d71b5ec1ce2a7c8dc3d73",
    "R": "80f6120db201444fbfb38e9472e9418e9a52ad8c89463d939879e5e64f9aeb5a",
    "B": "a884d56465bcc489ccd5196c35bb46a9a49726556be933fc22eaddf52e682751",
}
COMMON_GENERATOR_ARGUMENTS = [
    "--map-yaml",
    "workspaces/ros2_ws/src/semantic_nav_gazebo/maps/gazebo_eng_lobby/gazebo_eng_lobby.yaml",
    "--template",
    "isaac_sim/scripts/ira_people_demo/custom_eng_lobby_people.yaml",
    "--clearance",
    "0.55",
    "--spawn-clearance",
    "1.0",
    "--min-patrol-segment",
    "0.5",
    "--max-patrol-segment",
    "1.0",
    "--world",
    "workspaces/ros2_ws/src/semantic_nav_gazebo/worlds/gazebo_eng_lobby.world",
    "--scenario",
    "workspaces/ros2_ws/src/semantic_nav_gazebo/scenarios/lobby/eng_hall_15.xml",
    "--pedestrian-count",
    "15",
    "--seed",
    "7",
    "--speed",
    "1.0",
]


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def groups(config: dict) -> dict:
    return config["isaacsim.replicator.agent"]["character"]["groups"]


def patrol(group: dict) -> dict:
    return next(routine["patrol"] for routine in group["routines"] if "patrol" in routine)


class ScenarioTopologySplitContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.configs = {
            name: yaml.safe_load(path.read_text(encoding="utf-8"))
            for name, path in CONFIG_PATHS.items()
        }
        cls.groups = {name: groups(config) for name, config in cls.configs.items()}

    def test_frozen_a_and_b_and_split_artifact_hashes(self) -> None:
        self.assertEqual([], baseline.validate_generator_byte_identity(baseline.load_manifest()))
        for name, path in CONFIG_PATHS.items():
            self.assertEqual(sha256_file(path), EXPECTED_SHA256[name], name)

    def test_split_artifacts_are_reproducible(self) -> None:
        modes = {"S": "spawn_only", "R": "radius_only"}
        with tempfile.TemporaryDirectory(prefix="scenario_topology_split_") as directory:
            for name, mode in modes.items():
                output = Path(directory) / f"{name}.yaml"
                completed = subprocess.run(
                    [
                        sys.executable,
                        str(SCRIPT_DIR / "generate_free_space_people_config.py"),
                        *COMMON_GENERATOR_ARGUMENTS,
                        "--scenario-ab-mode",
                        mode,
                        "--output",
                        str(output),
                    ],
                    cwd=PROJECT_ROOT,
                    check=False,
                    capture_output=True,
                    text=True,
                )
                self.assertEqual(
                    completed.returncode, 0, completed.stderr or completed.stdout
                )
                self.assertEqual(output.read_bytes(), CONFIG_PATHS[name].read_bytes())

    def test_split_manifests_and_checksums_match_the_treatments(self) -> None:
        expected_modes = {"S": "spawn_only", "R": "radius_only"}
        for name, mode in expected_modes.items():
            manifest = yaml.safe_load(MANIFEST_PATHS[name].read_text(encoding="utf-8"))
            self.assertEqual(manifest["scenario_ab_mode"], mode)
            self.assertEqual(manifest["outputs"]["config_sha256"], EXPECTED_SHA256[name])
            records = {}
            for line in CHECKSUM_PATHS[name].read_text(encoding="utf-8").splitlines():
                digest, relative_path = line.split("  ", 1)
                records[relative_path] = digest
            self.assertEqual(
                records,
                {
                    CONFIG_PATHS[name].name: sha256_file(CONFIG_PATHS[name]),
                    MANIFEST_PATHS[name].name: sha256_file(MANIFEST_PATHS[name]),
                },
            )

    def test_spawn_and_radius_treatments_are_independent(self) -> None:
        center = (20.0, 19.0)
        extent = (4.0, 2.0)
        requested_start = (20.4, 18.7)
        route = [(3.0, 17.0), (3.0, 10.0), (25.0, 9.0), (25.0, 21.0)]
        radii = [3.0] * len(route)
        cluster_index = 1
        person_index = 4
        offset = (
            (requested_start[0] - center[0]) / extent[0],
            (requested_start[1] - center[1]) / extent[1],
        )
        spread_start = spread_spawn_point(
            center, extent, offset, cluster_index, person_index
        )
        spread_route = spread_waypoint_route(route, radii, offset, person_index)

        expected = {
            "baseline": (requested_start, route),
            "spawn_only": (spread_start, route),
            "radius_only": (requested_start, spread_route),
            "spread_radius": (spread_start, spread_route),
        }
        for mode, geometry in expected.items():
            self.assertEqual(
                select_scenario_ab_geometry(
                    mode,
                    center,
                    extent,
                    requested_start,
                    route,
                    radii,
                    cluster_index,
                    person_index,
                ),
                geometry,
            )

    def test_ids_allocation_seed_speed_and_non_path_fields_are_unchanged(self) -> None:
        ordered_ids = list(self.groups["A"])
        for name in ("S", "R", "B"):
            self.assertEqual(list(self.groups[name]), ordered_ids)
            self.assertEqual(
                self.configs[name]["isaacsim.replicator.agent"]["seed"],
                self.configs["A"]["isaacsim.replicator.agent"]["seed"],
            )
            for pedestrian_id in ordered_ids:
                reference = copy.deepcopy(self.groups["A"][pedestrian_id])
                candidate = copy.deepcopy(self.groups[name][pedestrian_id])
                reference_patrol = patrol(reference)
                candidate_patrol = patrol(candidate)
                reference_patrol.pop("path_points")
                candidate_patrol.pop("path_points")
                self.assertEqual(candidate, reference, f"{name}:{pedestrian_id}")

    def test_artifacts_form_the_required_geometry_factorization(self) -> None:
        ordered_ids = list(self.groups["A"])
        changed_spawns = {}
        changed_paths = {}
        for left, right in (("A", "S"), ("A", "R"), ("S", "B"), ("R", "B")):
            changed_spawns[(left, right)] = sum(
                patrol(self.groups[left][pedestrian_id])["path_points"][0]
                != patrol(self.groups[right][pedestrian_id])["path_points"][0]
                for pedestrian_id in ordered_ids
            )
            changed_paths[(left, right)] = sum(
                patrol(self.groups[left][pedestrian_id])["path_points"]
                != patrol(self.groups[right][pedestrian_id])["path_points"]
                for pedestrian_id in ordered_ids
            )

        self.assertEqual(changed_spawns[("A", "S")], 14)
        self.assertEqual(changed_spawns[("A", "R")], 0)
        self.assertEqual(changed_spawns[("S", "B")], 0)
        self.assertEqual(changed_spawns[("R", "B")], 14)
        self.assertEqual(changed_paths[("A", "S")], 14)
        self.assertEqual(changed_paths[("A", "R")], 15)
        self.assertEqual(changed_paths[("S", "B")], 15)
        self.assertEqual(changed_paths[("R", "B")], 14)


if __name__ == "__main__":
    unittest.main()
