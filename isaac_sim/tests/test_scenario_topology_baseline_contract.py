from __future__ import annotations

import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = PROJECT_ROOT / "isaac_sim/scripts"
sys.path.insert(0, str(SCRIPT_DIR))

import validate_scenario_topology_baseline as baseline  # noqa: E402


class ScenarioTopologyBaselineContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.manifest = baseline.load_manifest()
        cls.config = baseline.load_baseline_config(cls.manifest)

    def test_semantic_snapshot(self) -> None:
        self.assertEqual(
            [],
            baseline.validate_semantic_snapshot(self.manifest, self.config),
        )

    def test_logical_topology_snapshot(self) -> None:
        self.assertEqual(
            [],
            baseline.validate_topology_snapshot(self.manifest, self.config),
        )


if __name__ == "__main__":
    unittest.main()
