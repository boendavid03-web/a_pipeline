import gzip
import importlib.util
from array import array
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[5]
RENDERER = (
    PROJECT_ROOT
    / "pipelines"
    / "v7_native_pipeline"
    / "scripts"
    / "render_fixed_four_evaluation_video.py"
)
CAPTURE = (
    PROJECT_ROOT
    / "workspaces"
    / "ros2_ws"
    / "src"
    / "semantic_nav_gazebo"
    / "scripts"
    / "fixed_four_video_capture.py"
)
LAUNCH_FILES = (
    PROJECT_ROOT
    / "workspaces"
    / "ros2_ws"
    / "src"
    / "semantic_nav_gazebo"
    / "launch"
    / "semantic_cnn_fixed_dual_start_goal_demo.launch.py",
    PROJECT_ROOT
    / "workspaces"
    / "ros2_ws"
    / "src"
    / "semantic_nav_gazebo"
    / "launch"
    / "drl_vo_fixed_dual_start_goal_demo.launch.py",
)


def load_renderer():
    spec = importlib.util.spec_from_file_location("fixed_four_video_renderer", RENDERER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_video_mode_is_opt_in_and_independent_of_record_trace():
    for launch_file in LAUNCH_FILES:
        source = launch_file.read_text(encoding="utf-8")
        assert '"record_video",\n' in source
        assert 'default_value="false"' in source
        assert 'condition=IfCondition(LaunchConfiguration("record_video"))' in source
        assert 'condition=IfCondition(LaunchConfiguration("record_trace"))' in source
        assert 'executable="fixed_four_video_capture.py"' in source


def test_video_tools_are_installed_and_executable():
    cmake = (
        PROJECT_ROOT
        / "workspaces"
        / "ros2_ws"
        / "src"
        / "semantic_nav_gazebo"
        / "CMakeLists.txt"
    ).read_text(encoding="utf-8")
    assert "scripts/fixed_four_video_capture.py" in cmake
    assert CAPTURE.stat().st_mode & 0o111
    assert RENDERER.stat().st_mode & 0o111


def test_frame_schedule_keeps_episode_order_and_final_frame():
    module = load_renderer()
    episodes = [
        {"experiment": {"simulation_time_start": 10.0, "simulation_time_end": 11.0}},
        {"experiment": {"simulation_time_start": 20.0, "simulation_time_end": 21.0}},
        {"experiment": {"simulation_time_start": 30.0, "simulation_time_end": 31.0}},
        {"experiment": {"simulation_time_start": 40.0, "simulation_time_end": 41.0}},
    ]
    schedule = module.build_frame_schedule(episodes, fps=2.0, playback_rate=1.0, max_frames=0)
    assert [episode for episode, _ in schedule] == sorted(episode for episode, _ in schedule)
    assert (1, 11.0) in schedule
    assert (4, 41.0) in schedule


def test_binary_lidar_reader_preserves_both_real_slots(tmp_path):
    module = load_renderer()
    path = tmp_path / "dual_lidar.bin.gz"
    ranges_01 = array("f", [1.0, 2.0, 3.0])
    ranges_02 = array("f", [4.0, 5.0])
    with gzip.open(path, "wb") as stream:
        stream.write(module.LIDAR_MAGIC)
        stream.write(
            module.LIDAR_RECORD.pack(
                1_000_000_000,
                1_010_000_000,
                len(ranges_01),
                len(ranges_02),
                -1.0,
                0.1,
                0.05,
                20.0,
                -0.5,
                0.2,
                0.05,
                30.0,
            )
        )
        stream.write(ranges_01.tobytes())
        stream.write(ranges_02.tobytes())
    selected = module.select_lidar_samples(path, [0.5, 1.0, 2.0])
    assert selected[0] is None
    assert list(selected[1]["ranges_01"]) == [1.0, 2.0, 3.0]
    assert list(selected[1]["ranges_02"]) == [4.0, 5.0]
    assert selected[1]["time_01"] == 1.0
    assert selected[1]["time_02"] == 1.01


def test_merged_lidar_reader_and_map_projection(tmp_path):
    module = load_renderer()
    path = tmp_path / "merged_lidar.bin.gz"
    ranges = array("f", [1.0, float("inf")])
    with gzip.open(path, "wb") as stream:
        stream.write(module.MERGED_LIDAR_MAGIC)
        stream.write(
            module.MERGED_LIDAR_RECORD.pack(
                2_000_000_000,
                len(ranges),
                0.0,
                1.0,
                0.1,
                20.0,
            )
        )
        stream.write(ranges.tobytes())
    selected = module.select_merged_lidar_samples(path, [1.0, 2.0])
    assert selected[0] is None
    assert list(selected[1]["ranges"]) == [1.0, float("inf")]
    points = module.project_merged_lidar_to_map(
        selected[1], {"x": "2.0", "y": "3.0", "yaw": str(module.math.pi / 2.0)}
    )
    assert len(points) == 1
    assert abs(points[0][0] - 2.0) < 1.0e-6
    assert abs(points[0][1] - 4.0) < 1.0e-6


def test_renderer_requires_and_reports_map_frame_lidar_overlay():
    source = RENDERER.read_text(encoding="utf-8")
    assert 'merged_lidar_path = capture_dir / "merged_lidar.bin.gz"' in source
    assert "/scan_merged capture contains no sample" in source
    assert '"map_frame_lidar_overlay": "available"' in source
    assert "project_merged_lidar_to_map" in source


def test_renderer_does_not_construct_a_model_prediction_trajectory():
    source = RENDERER.read_text(encoding="utf-8")
    assert '"model_predicted_trajectory": "unavailable"' in source
    assert "not converted into a fabricated trajectory" in source
