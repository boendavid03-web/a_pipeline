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
        "isaac_sim/scripts/cmd_vel_udp_relay.py": "42b5cf246c633096ca285718d1bdde3daf983811df7a001c0f0f6c1d754ff8d9",
        "isaac_sim/scripts/physx_lidar_people.py": "c0e7fbaa59fc5761886d0e644876722eaaa5fafae76da7eae251528590dcb1d7",
        "isaac_sim/scripts/show_warehouse_people_robot_6_0.py": "e8fefea611035bec1a21e96e7635637a53f3911b3954571d48c534964edb83e7",
        "isaac_sim/tests/test_crowded_tracking_stress_contract.py": "5b3ccdd8588088aad47436d0dd72262b09ad8ce280c25dbf6e6aa4f45ab5ae3c",
        "isaac_sim/tests/test_isaac_evaluation_shell_contract.py": "f3579fe493a22c85725c5439dc151a1c68f9c65061f0192aadfe035e3ce34347",
        "isaac_sim/tests/test_physx_lidar_people.py": "047ed6f0abf18673838645a01215d8ed37b09a3d2b23a1c369c4307c590b7bd1",
    }
    for relative, digest in expected.items():
        actual = hashlib.sha256((ROOT / relative).read_bytes()).hexdigest()
        assert actual == digest, relative
