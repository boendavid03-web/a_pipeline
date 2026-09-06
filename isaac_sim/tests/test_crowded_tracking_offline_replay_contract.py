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
                "isaac_sim/scripts/cmd_vel_udp_relay.py": "1e76519fe09e31cd48f363c078f143f27aa3a58f733cdea43a7d14eb1c7730d3",
            "isaac_sim/scripts/physx_lidar_people.py": "d3395182b3d5434197eb1d0ff0bfb806a5e8a60a1513377674a121542365201a",
                    "isaac_sim/scripts/show_warehouse_people_robot_6_0.py": "3f3807b52f1e03fd1d8d6a90d3715ac91643bb5ececcf5b6d2235218ac6cdc74",
                    "isaac_sim/tests/test_crowded_tracking_stress_contract.py": "8a3098e36519bd17a785c3f1be690035b32097ef833e631272912fe16aab978b",
                    "isaac_sim/tests/test_isaac_evaluation_shell_contract.py": "b8f48d604366ae28fab7c548fa4c1441b64f3f8c136fae36ee77d54d332f2616",
            "isaac_sim/tests/test_physx_lidar_people.py": "e3d450c306fa5f247fc91ec8fa827fa01f9400723499529bd2def723637d4c63",
    }
    for relative, digest in expected.items():
        actual = hashlib.sha256((ROOT / relative).read_bytes()).hexdigest()
        assert actual == digest, relative
