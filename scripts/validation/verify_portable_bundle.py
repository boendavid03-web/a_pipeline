#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：/home
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, SDF, YAML, YML
# 可能使用的关键环境变量：BANNED, FAIL, PASS, SKIP_PARTS, TEXT_SUFFIXES
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/scripts/validation/verify_portable_bundle.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:46:55.725904249 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:20:55.850391663 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/environment/04_verify_install.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/00_create_run.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/scripts/release/create_bundle.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/convert_rosbag2_to_semantic2d_native_lidar.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/environment/04_verify_install.sh（source 载入公共环境/函数/变量）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/00_create_run.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/scripts/release/create_bundle.sh（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/convert_rosbag2_to_semantic2d_native_lidar.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/convert_rosbag2_to_semantic2d_native_lidar.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/scripts/validation/verify_portable_bundle.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/00_create_run.sh; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/convert_rosbag2_to_semantic2d_native_lidar.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/tools/convert_rosbag2_to_semantic2d_native_lidar.py
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/environment/04_verify_install.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/00_create_run.sh; /home/user/navigation_project/a_pipeline/scripts/release/create_bundle.sh; /home/user/navigation_project/a_pipeline/scripts/validation/ros2_workspace_tools/convert_rosbag2_to_semantic2d_native_lidar.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜verify_portable_bundle.py】
# 用途：项目辅助脚本，用于发布、验证、转换、调试或打包等实际运行任务。
# 输入输出：输入通常是命令行参数、ROS 2 话题、bag、配置或数据集；输出通常是检查结果、转换文件、日志或辅助进程。
# 关系：由 pipeline、ROS 2 launch 或人工命令调用，依赖对应环境和数据格式；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Reject active files or symlinks that still depend on the source checkout."""

from pathlib import Path
import sys


TEXT_SUFFIXES = {".py", ".sh", ".xml", ".sdf", ".yaml", ".yml", ".json"}
BANNED = (
    str(Path("/home") / "suat_wxb"),
    "semantic_navigation/ros2_ws",
    "experiments/legacy_runs",
)
SKIP_PARTS = {"build", "install", "log", ".venvs", ".runtime", "runs", "dist", "__pycache__"}
SKIP_PREFIXES = ("build.stale-", "install.stale-", "log.stale-")


def main() -> int:
    root = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
    required = (
        "pipelines/v7_native_pipeline/scripts/00_create_run.sh",
        "workspaces/ros2_ws/src/semantic_nav_gazebo/package.xml",
        "workspaces/ros2_ws/tools/convert_rosbag2_to_semantic2d_native_lidar.py",
        "methods/baselines/s3net",
        "methods/baselines/semantic_cnn",
        "examples/smoke/test_rosbag/metadata.yaml",
    )
    errors: list[str] = []
    for relative in required:
        if not (root / relative).exists():
            errors.append(f"missing required path: {relative}")

    for path in root.rglob("*"):
        relative = path.relative_to(root)
        if relative == Path("scripts/validation/verify_portable_bundle.py"):
            continue
        if any(
            part in SKIP_PARTS or part.startswith(SKIP_PREFIXES)
            for part in relative.parts
        ):
            continue
        if path.is_symlink():
            try:
                path.resolve().relative_to(root)
            except (OSError, ValueError):
                errors.append(f"external symlink: {relative} -> {path.readlink()}")
            continue
        if not path.is_file() or path.suffix.lower() not in TEXT_SUFFIXES:
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            continue
        for token in BANNED:
            if token in text:
                errors.append(f"banned path token {token!r}: {relative}")

    model = root / "workspaces/ros2_ws/src/semantic_nav_gazebo/models/mecanum730_xms5_nav_proxy_fallback_v7_teacher_scan01"
    sdf = model / "model.sdf"
    mesh = model / "meshes/derived/mecanum730_xms5_body_with_dual_lidar_recesses.stl"
    if not mesh.is_file() or mesh.stat().st_size < 100_000_000:
        errors.append("active v7 robot mesh is missing or truncated")
    if sdf.is_file() and "model://mecanum730_xms5_nav_proxy_fallback_v7_teacher_scan01/" not in sdf.read_text():
        errors.append("active v7 model does not use its bundled model:// mesh")

    if errors:
        for error in errors:
            print(f"FAIL: {error}", file=sys.stderr)
        return 1
    print("PASS: required source closure is present")
    print("PASS: no external symlink or source-checkout absolute path in active files")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
