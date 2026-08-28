#!/usr/bin/env python3
"""Regression tests for automatic-capture provenance and approval seals."""

from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
TOOLS = PROJECT_ROOT / "scripts" / "validation" / "ros2_workspace_tools"


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


contract_tool = load_module(
    "write_auto_capture_contract",
    TOOLS / "write_auto_capture_contract.py",
)


class ArtifactContractTests(unittest.TestCase):
    @staticmethod
    def _file_record(path: Path, root: Path) -> dict:
        return {
            "file": str(path.relative_to(root)),
            "size_bytes": path.stat().st_size,
            "sha256": contract_tool.sha256(path),
        }

    def _prepare_semantic_training_session(
        self, temporary: Path, dataset_root: Path, session_name: str
    ) -> dict:
        source = temporary / f"{session_name}-source"
        source_samples = source / "samples"
        source_samples.mkdir(parents=True)
        source_sample = source_samples / "0000000.npz"
        source_sample.write_bytes(b"source-sample-v1")
        source_metadata = source / "metadata.json"
        source_metadata.write_text(
            json.dumps({"samples": 1}), encoding="utf-8"
        )
        contract_payload = {
            "schema": "semantic_nav_auto_capture_contract/v1",
            "bag": str((temporary / "source-bag").resolve()),
        }
        source_contract = source / "capture_contract.json"
        source_contract.write_text(
            json.dumps(contract_payload), encoding="utf-8"
        )

        session = dataset_root / session_name
        session.mkdir(parents=True)
        training_contract = session / "capture_contract.json"
        training_contract.write_text(
            json.dumps(contract_payload), encoding="utf-8"
        )
        metadata = session / "metadata.json"
        metadata.write_text(
            json.dumps(
                {
                    "format": "semantic2d-fixed-dual-native-v3",
                    "session_name": session_name,
                    "source_npz_session": str(source.resolve()),
                    "source_metadata_sha256": contract_tool.sha256(
                        source_metadata
                    ),
                    "samples": 1,
                    "field_map": {"scans_lidar": "raw_ranges"},
                }
            ),
            encoding="utf-8",
        )
        (session / "label_names.txt").write_text(
            "_background_\nPerson\n", encoding="utf-8"
        )
        (session / "train.txt").write_text("sample.npy\n", encoding="utf-8")
        (session / "dev.txt").write_text("", encoding="utf-8")
        (session / "test.txt").write_text("", encoding="utf-8")
        for directory in ("scans_lidar", "intensities_lidar"):
            field = session / directory
            field.mkdir()
            (field / "sample.npy").write_bytes(b"training-array-v1")
        source_approval = session / "source_QUALITY_PASS.json"
        source_approval.write_text(
            json.dumps(
                {
                    "schema": "semantic_nav_cnn_supervision_approval/v1",
                    "session": str(source.resolve()),
                    "quality_status": "PASS",
                    "sample_count": 1,
                    "core_files": [
                        self._file_record(source_metadata, source),
                        self._file_record(source_contract, source),
                    ],
                    "sample_files": [
                        self._file_record(source_sample, source)
                    ],
                }
            ),
            encoding="utf-8",
        )
        quality = session / "quality_report.json"
        quality.write_text(
            json.dumps(
                {
                    "status": "PASS",
                    "session": str(session.resolve()),
                    "source_session": str(source.resolve()),
                    "samples": 1,
                    "error_count": 0,
                    "errors": [],
                    "warnings": [],
                }
            ),
            encoding="utf-8",
        )
        return {
            "session": session,
            "source": source,
            "source_sample": source_sample,
            "training_array": session / "scans_lidar" / "sample.npy",
            "quality": quality,
            "source_approval": source_approval,
            "contract": training_contract,
            "ready": session / "CNN_READY.json",
        }

    @staticmethod
    def _seal_semantic_training(prepared: dict) -> subprocess.CompletedProcess:
        return subprocess.run(
            [
                sys.executable,
                str(TOOLS / "seal_semantic2d_training_session.py"),
                "--session",
                str(prepared["session"]),
                "--source-session",
                str(prepared["source"]),
                "--quality-report",
                str(prepared["quality"]),
                "--source-approval",
                str(prepared["source_approval"]),
                "--capture-contract",
                str(prepared["contract"]),
                "--output",
                str(prepared["ready"]),
            ],
            check=False,
            capture_output=True,
            text=True,
        )

    @staticmethod
    def _register_semantic_training(
        dataset_root: Path, session: Path
    ) -> subprocess.CompletedProcess:
        return subprocess.run(
            [
                sys.executable,
                str(TOOLS / "register_semantic2d_session.py"),
                "--dataset-root",
                str(dataset_root),
                "--session",
                str(session),
            ],
            check=False,
            capture_output=True,
            text=True,
        )

    def test_asset_manifest_detects_snapshot_tampering(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            snapshot = {}
            for key, filename in (
                ("map_yaml", "map.yaml"),
                ("occupancy_image", "occupancy.pgm"),
                ("semantic_label", "label.png"),
                ("label_names", "label_names.txt"),
            ):
                path = root / filename
                path.write_bytes(f"original-{key}".encode())
                snapshot[key] = {
                    "file": filename,
                    "sha256": contract_tool.sha256(path),
                }
            manifest_path = root / "manifest.json"
            manifest = {
                "schema": contract_tool.ASSET_SCHEMA,
                "snapshot": snapshot,
            }
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            verified = contract_tool.validate_asset_manifest(
                manifest_path, manifest
            )
            self.assertEqual(len(verified), 4)
            (root / "label.png").write_bytes(b"tampered")
            with self.assertRaisesRegex(ValueError, "checksum mismatch"):
                contract_tool.validate_asset_manifest(manifest_path, manifest)

    def test_bag_data_files_reject_path_escape(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            bag = root / "bag"
            bag.mkdir()
            metadata = bag / "metadata.yaml"
            metadata.write_text(
                "rosbag2_bagfile_information:\n"
                "  relative_file_paths:\n"
                "    - ../outside.db3\n",
                encoding="utf-8",
            )
            (root / "outside.db3").write_bytes(b"bag")
            with self.assertRaisesRegex(ValueError, "inside"):
                contract_tool.bag_data_files(bag.resolve(), metadata)

    def test_recorded_topic_inventory_checks_type_and_nonempty_count(self):
        with tempfile.TemporaryDirectory() as temporary:
            metadata = Path(temporary) / "metadata.yaml"
            entries = [
                {
                    "topic_metadata": {
                        "name": name,
                        "type": contract_tool.EXPECTED_TOPIC_TYPES[name],
                    },
                    "message_count": (
                        0
                        if name in contract_tool.OPTIONAL_EMPTY_TOPICS
                        else 1
                    ),
                }
                for name in contract_tool.RECORDED_TOPICS
            ]

            def write_inventory():
                import yaml

                metadata.write_text(
                    yaml.safe_dump(
                        {
                            "rosbag2_bagfile_information": {
                                "topics_with_message_count": entries
                            }
                        }
                    ),
                    encoding="utf-8",
                )

            write_inventory()
            self.assertEqual(
                len(contract_tool.validate_recorded_topics(metadata)),
                len(contract_tool.RECORDED_TOPICS),
            )
            entries[0]["message_count"] = 0
            write_inventory()
            with self.assertRaisesRegex(ValueError, "empty"):
                contract_tool.validate_recorded_topics(metadata)

    def test_supervision_approval_is_strict_and_hashes_samples(self):
        with tempfile.TemporaryDirectory() as temporary:
            session = Path(temporary) / "session"
            samples = session / "samples"
            samples.mkdir(parents=True)
            sample = samples / "sample_000000.npz"
            sample.write_bytes(b"sample-payload")
            (session / "train.txt").write_text(sample.name + "\n")
            (session / "dev.txt").write_text("")
            (session / "test.txt").write_text("")
            (session / "metadata.json").write_text(
                json.dumps(
                    {
                        "samples": 1,
                        "bag": str((session / "source_bag").resolve()),
                        "map_yaml_sha256": "a" * 64,
                        "occupancy_image_sha256": "b" * 64,
                        "semantic_label_sha256": "c" * 64,
                        "label_names_sha256": "d" * 64,
                    }
                ),
                encoding="utf-8",
            )
            contract = session / "capture_contract.json"
            contract.write_text(
                json.dumps(
                    {
                        "schema": "semantic_nav_auto_capture_contract/v1",
                        "bag": str((session / "source_bag").resolve()),
                        "supervision_assets": {
                            "verified_files": [
                                {"kind": "map_yaml", "sha256": "a" * 64},
                                {
                                    "kind": "occupancy_image",
                                    "sha256": "b" * 64,
                                },
                                {
                                    "kind": "semantic_label",
                                    "sha256": "c" * 64,
                                },
                                {
                                    "kind": "label_names",
                                    "sha256": "d" * 64,
                                },
                            ]
                        },
                    }
                ),
                encoding="utf-8",
            )
            report = session / "quality_report.json"
            report.write_text(
                json.dumps(
                    {
                        "status": "PASS",
                        "session": str(session.resolve()),
                        "samples": 1,
                        "error_count": 0,
                        "errors": [],
                        "warnings": [],
                        "subgoal_source": "online",
                        "person_label_mode": "ground-truth-legs",
                        "person_label_count": 1,
                        "person_ground_truth_unmatched_samples": 0,
                        "negative_linear_x_samples": 0,
                        "episode_count": 1,
                        "episode_filter": {"mode": "successful_only"},
                        "quality_gate_configuration": {
                            "minimum_samples": 1,
                            "minimum_duration_sec": 1.0,
                            "minimum_unique_command_vectors": 1,
                            "minimum_nonzero_command_fraction": 0.1,
                            "minimum_effective_sample_rate_hz": 1.0,
                            "minimum_person_positive_sample_fraction": 0.1,
                            "maximum_subgoal_age_ms": 300.0,
                            "maximum_cmd_vel_age_ms": 100.0,
                            "maximum_person_truth_unmatched_samples": 0,
                            "require_online_subgoal": True,
                            "require_successful_episodes_only": True,
                            "require_ground_truth_person_labels": True,
                            "require_person_observations": True,
                            "require_forward_only": True,
                            "require_pre_relay_command_labels": True,
                            "fail_on_warnings": True,
                        },
                    }
                ),
                encoding="utf-8",
            )
            approval = session / "QUALITY_PASS.json"
            result = subprocess.run(
                [
                    sys.executable,
                    str(TOOLS / "seal_cnn_supervision_dataset.py"),
                    "--session",
                    str(session),
                    "--quality-report",
                    str(report),
                    "--capture-contract",
                    str(contract),
                    "--output",
                    str(approval),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            payload = json.loads(approval.read_text(encoding="utf-8"))
            self.assertEqual(payload["sample_count"], 1)
            self.assertEqual(
                payload["sample_files"][0]["sha256"],
                contract_tool.sha256(sample),
            )

    def test_training_seal_rejects_tampered_source_npz(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            dataset_root = root / "dataset"
            dataset_root.mkdir()
            prepared = self._prepare_semantic_training_session(
                root, dataset_root, "session-one"
            )
            prepared["source_sample"].write_bytes(b"source-sample-v2")
            result = self._seal_semantic_training(prepared)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("sample_files checksum mismatch", result.stderr)
            self.assertFalse(prepared["ready"].exists())

    def test_registration_rejects_tampered_new_training_array(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            dataset_root = root / "dataset"
            dataset_root.mkdir()
            prepared = self._prepare_semantic_training_session(
                root, dataset_root, "session-one"
            )
            sealed = self._seal_semantic_training(prepared)
            self.assertEqual(sealed.returncode, 0, sealed.stderr)
            prepared["training_array"].write_bytes(b"training-array-v2")
            result = self._register_semantic_training(
                dataset_root, prepared["session"]
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("training_arrays checksum mismatch", result.stderr)
            self.assertFalse((dataset_root / "dataset.txt").exists())

    def test_registration_rechecks_already_indexed_session_hashes(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            dataset_root = root / "dataset"
            dataset_root.mkdir()
            first = self._prepare_semantic_training_session(
                root, dataset_root, "session-one"
            )
            sealed = self._seal_semantic_training(first)
            self.assertEqual(sealed.returncode, 0, sealed.stderr)
            registered = self._register_semantic_training(
                dataset_root, first["session"]
            )
            self.assertEqual(registered.returncode, 0, registered.stderr)
            first["training_array"].write_bytes(b"training-array-v2")

            second = self._prepare_semantic_training_session(
                root, dataset_root, "session-two"
            )
            sealed = self._seal_semantic_training(second)
            self.assertEqual(sealed.returncode, 0, sealed.stderr)
            result = self._register_semantic_training(
                dataset_root, second["session"]
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn(
                "session-one training_arrays checksum mismatch", result.stderr
            )
            self.assertEqual(
                (dataset_root / "dataset.txt").read_text(encoding="utf-8"),
                "session-one\n",
            )


if __name__ == "__main__":
    unittest.main()
