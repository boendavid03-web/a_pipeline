from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_launches_explicitly_use_only_shadow_outputs():
    text = '\n'.join(p.read_text() for p in (ROOT / 'ros2_ws/src/semantic_nav_runtime/launch').glob('*.launch.py'))
    assert "'/sim_to_real/semantic_cnn/cmd_vel_shadow'" in text
    assert "'/sim_to_real/drl_vo/cmd_vel_shadow'" in text
    assert "'cmd_vel_topic': '/cmd_vel'" not in text


def test_drlvo_drspaam_mode_disables_truth_and_uses_tracks():
    text = (ROOT / 'ros2_ws/src/semantic_nav_runtime/launch/drl_vo_drspaam_shadow.launch.py').read_text()
    assert "'pedestrian_source': 'dr_spaam'" in text
    assert "'require_pedestrian_truth': False" in text
    assert "'pedestrian_tracks_topic': '/pedestrian_tracks'" in text
    runtime = (ROOT / 'ros2_ws/src/semantic_nav_runtime/scripts/drl_vo_fixed_dual_inference_node.py').read_text()
    assert '/pedestrian_ground_truth' not in runtime
