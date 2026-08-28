import importlib.util
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：NPY, TXT
# 可能使用的关键环境变量：CHECKER, SCRIPT, SPEC
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：测试脚本
# 推荐运行方式：python3 -m pytest /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/tests/test_semantic_cnn_training_check_helpers.py
# 输入来源：命令行参数、环境变量和上一步 pipeline 产生的文件或目录。
# 输出结果：下一阶段需要的数据、模型、日志或 ROS 2 进程。
# 前置条件：先加载项目环境，并确认上一步输出存在。
# 后续步骤：按 pipeline 编号执行后续阶段脚本。
# 副作用与安全：可能启动进程或写入实验输出；默认不应删除原始数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-27 20:43:51.377155142 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:53.597198074 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/check_semantic_cnn_training.py（正文中引用该脚本路径/名称）
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/tests/test_semantic_cnn_training_check_helpers.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/check_semantic_cnn_training.py
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
import tempfile
import unittest
from pathlib import Path

import numpy as np


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "check_semantic_cnn_training.py"
)
SPEC = importlib.util.spec_from_file_location(
    "check_semantic_cnn_training",
    SCRIPT,
)
CHECKER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECKER)


class SemanticCnnTrainingCheckHelperTests(unittest.TestCase):
    def test_composite_split_name_uses_matching_command_file(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            session = Path(temp_dir)
            command_dir = session / "cmd_velocities"
            command_dir.mkdir()
            sample_name = "20260727_074611-ep002-0000009.npy"
            np.save(
                command_dir / sample_name,
                np.asarray([0.25, 0.0, -0.5], dtype=np.float32),
            )
            (session / "train.txt").write_text(
                f"nested/{sample_name}\n",
                encoding="utf-8",
            )

            self.assertEqual(
                CHECKER.split_file_names(str(session), "train"),
                [sample_name],
            )
            np.testing.assert_allclose(
                CHECKER.target_for_dataset_sample(str(session), sample_name),
                [0.25, -0.5],
            )

    def test_numeric_split_name_preserves_legacy_window_target(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            session = Path(temp_dir)
            scan_dir = session / "scans_lidar"
            command_dir = session / "cmd_velocities"
            scan_dir.mkdir()
            command_dir.mkdir()
            for index in range(20):
                name = f"{index:07d}.npy"
                np.save(scan_dir / name, np.asarray([index], dtype=np.float32))
                np.save(
                    command_dir / name,
                    np.asarray([index, 0.0, -index], dtype=np.float32),
                )

            np.testing.assert_allclose(
                CHECKER.target_for_dataset_sample(str(session), "0000002.npy"),
                [11.0, -11.0],
            )


if __name__ == "__main__":
    unittest.main()
