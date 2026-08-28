#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：/pedestrian_ground_truth
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：PROJECT_ROOT
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/test_online_ppo.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-31 12:45:04.429148216 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:09.567386301 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/drl_vo_online_ppo_training_node.py（正文中引用该脚本路径/名称）
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/test_online_ppo.py
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/drl_vo_online_ppo_training_node.py
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。

import unittest
from pathlib import Path

import numpy as np

from methods.experiments.drl_vo_ros2_offline.online_ppo import (
    OnlinePPO,
    PPOConfig,
    RewardConfig,
    RolloutBuffer,
    compute_training_reward,
    generalized_advantage_estimate,
    load_online_policy,
)


PROJECT_ROOT = Path(__file__).resolve().parents[3]


class RewardAndAdvantageTests(unittest.TestCase):
    def test_reward_terminal_precedence_and_progress(self):
        config = RewardConfig()
        reward, done, reason = compute_training_reward(
            2.0, 0.3, 1.0, 0.0, timed_out=False, config=config
        )
        self.assertEqual((reward, done, reason), (20.0, True, "goal"))
        reward, done, reason = compute_training_reward(
            2.0, 1.9, 0.2, 0.0, timed_out=False, config=config
        )
        self.assertEqual((reward, done, reason), (-20.0, True, "collision"))
        reward, done, reason = compute_training_reward(
            2.0, 1.9, 2.0, 0.0, timed_out=False, config=config
        )
        self.assertFalse(done)
        self.assertEqual(reason, "running")
        self.assertGreater(reward, 0.0)

    def test_gae_stops_bootstrap_at_episode_boundary(self):
        advantages, returns = generalized_advantage_estimate(
            rewards=np.asarray([1.0, 2.0], dtype=np.float32),
            dones=np.asarray([0.0, 1.0], dtype=np.float32),
            values=np.asarray([0.5, 0.25], dtype=np.float32),
            bootstrap_value=99.0,
            gamma=1.0,
            gae_lambda=1.0,
        )
        np.testing.assert_allclose(advantages, [2.5, 1.75], atol=1e-6)
        np.testing.assert_allclose(returns, [3.0, 2.0], atol=1e-6)

    def test_training_node_has_no_pedestrian_truth_subscription(self):
        source = (
            PROJECT_ROOT
            / "workspaces"
            / "ros2_ws"
            / "src"
            / "semantic_nav_gazebo"
            / "scripts"
            / "drl_vo_online_ppo_training_node.py"
        ).read_text(encoding="utf-8")
        self.assertNotIn("PedestrianStateArray", source)
        self.assertNotIn('"/pedestrian_ground_truth"', source)


class PPOUpdateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        model = (
            PROJECT_ROOT
            / "github_src"
            / "drl_vo_nav-drl_vo"
            / "drl_vo"
            / "src"
            / "model"
            / "drl_vo.zip"
        )
        cls.policy, count = load_online_policy(model)
        if count != 163:
            raise AssertionError(f"unexpected weight count {count}")

    def test_real_policy_select_and_update(self):
        config = PPOConfig(
            update_epochs=1,
            batch_size=2,
            freeze_feature_extractor=True,
        )
        agent = OnlinePPO(self.policy, "cpu", config, seed=7)
        buffer = RolloutBuffer()
        before = agent.policy.action_net.weight.detach().clone()
        for index in range(2):
            observation = np.zeros(19202, dtype=np.float32)
            observation[-2] = 0.1 * index
            raw, _clipped, log_probability, value = agent.select_action(
                observation
            )
            buffer.add(
                observation,
                raw,
                reward=float(index + 1),
                done=index == 1,
                log_probability=log_probability,
                value=value,
            )
        metrics = agent.update(buffer, bootstrap_value=0.0)
        self.assertEqual(metrics["transitions"], 2)
        self.assertTrue(
            all(
                np.isfinite(float(value))
                for key, value in metrics.items()
                if key not in {"transitions", "batches"}
            )
        )
        self.assertFalse(
            np.array_equal(
                before.numpy(),
                agent.policy.action_net.weight.detach().numpy(),
            )
        )
        self.assertEqual(len(buffer), 0)


if __name__ == "__main__":
    unittest.main()
