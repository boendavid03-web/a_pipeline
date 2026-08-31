from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
WRAPPER = ROOT / "isaac_sim/scripts/recompute_crowded_tracking_evaluation.sh"
MANIFEST = ROOT / "isaac_sim/config/crowded_tracking_suite_manifest_20260831.json"


def test_wrapper_contract_is_fail_closed_and_replay_only():
    source = WRAPPER.read_text()
    for text in ("canonical raw bag hash", "ROS_DOMAIN_ID", "set +u", "set -u", "use_sim_time:=true", "tf_static", "/clock", "--clock", "kill -INT", "CROWDED_TRACKING_RECOMPUTE_V2=PASS"):
        assert text in source
    assert "isaac_sim" not in source.split("ros2 bag play", 1)[1]


def test_manifest_contains_exact_run_root_and_replay_manifest():
    import json
    manifest = json.loads(MANIFEST.read_text())
    assert manifest["run_root"].endswith("runs/dr_spaam_isaac_crowded_tracking")
    assert "replay_manifest" in manifest
    assert len(manifest["entries"]) == 10


def test_existing_dirty_sources_have_recorded_sha256():
    # Baselines are intentionally immutable during this handoff.
    import hashlib
    expected = {
                "isaac_sim/scripts/cmd_vel_udp_relay.py": "b9c1389b9b8ae1dcfe50f7adf32d9d2ee2cee4bdd4b3bbd4f351adbced7d3638",
            "isaac_sim/scripts/physx_lidar_people.py": "7cb2263de509e4b12c3e6bb362ef6a86ba3ab7612e42c8065b44422e7af40995",
                    "isaac_sim/scripts/show_warehouse_people_robot_6_0.py": "781626014508c287e59ffff95e681f2f992cb86f686672ccdbd4c28d68dae0b7",
                    "isaac_sim/tests/test_crowded_tracking_stress_contract.py": "236741696e6c638b9aebfdb44b860ca0ce20fe2e273336b7431d8e45a9e42429",
                    "isaac_sim/tests/test_isaac_evaluation_shell_contract.py": "fffa23078f0074fb6ea093df56761f7945dd94b9caebce3dd6ad48df2b1243b5",
            "isaac_sim/tests/test_physx_lidar_people.py": "8735c2fdba0043ccefc257b243da5203cba321fda2d4277152b91d48a3c766b3",
    }
    for relative, digest in expected.items():
        actual = hashlib.sha256((ROOT / relative).read_bytes()).hexdigest()
        assert actual == digest, relative
