#!/usr/bin/env python3
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：/cmd_vel, /drl_vo/episode_reset, /drl_vo/training_state, /world/default/set_pose
# 检测到的消息类型：DrlVoTrainingState; Empty; Entity; Twist
# 检测到的文件格式：JSON, PT
# 可能使用的关键环境变量：CUDA, E402, MODEL, NANOSECONDS_PER_SECOND, NAVIGATION_PROJECT_ROOT, OBSERVATION_SIZE, PROJECT_ROOT
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：ROS 2 功能节点
# 推荐运行方式：ros2 run semantic_nav_gazebo drl_vo_online_ppo_training_node.py
# 输入来源：ROS 2 参数、订阅话题、地图、模型和配置文件。
# 输出结果：ROS 2 节点、话题、服务、日志或采集文件。
# 前置条件：需要 source ROS 2 和工作空间 install/setup.bash，并确保依赖节点/话题存在。
# 后续步骤：由对应 launch/pipeline 继续运行或由下游节点消费输出。
# 副作用与安全：可能启动仿真、发布控制指令或写入数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-31 12:42:57.088222033 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:21:13.643741916 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/test_online_ppo.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/test_online_ppo.py（引用其脚本路径或名称，形成流程关联）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py（引用其脚本路径或名称，形成流程关联）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/drl_vo_online_ppo_training_node.py
# 直接依赖（脚本层面）：无项目内脚本引用（外部库/ROS 2 命令另见代码正文）。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline/test_online_ppo.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/drl_vo_fixed_dual_start_goal_demo.launch.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜drl_vo_online_ppo_training_node.py】
# 用途：ROS 2 运行节点，负责导航、感知、遥操作、数据采集或仿真辅助功能。
# 输入输出：输入为 ROS 2 参数和订阅话题；输出为发布话题、日志或实验文件。
# 关系：通常由 launch 或 pipeline 启动，依赖 ROS 2 消息及其他节点；install 下同名文件是本源文件的安装入口。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
"""Online PPO fine-tuning from truth-free DRL-VO training states."""

from __future__ import annotations

import json
import math
import os
import sys
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np
import rclpy
import torch
from geometry_msgs.msg import Twist
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from semantic_nav_gazebo.msg import DrlVoTrainingState
from std_msgs.msg import Empty


PROJECT_ROOT = Path(
    os.environ.get(
        "NAVIGATION_PROJECT_ROOT", Path(__file__).resolve().parents[5]
    )
).resolve()
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from methods.experiments.drl_vo_ros2_offline.observation_adapter import (  # noqa: E402
    normalized_to_physical,
)
from methods.experiments.drl_vo_ros2_offline.online_ppo import (  # noqa: E402
    OBSERVATION_SIZE,
    OnlinePPO,
    PPOConfig,
    RewardConfig,
    RolloutBuffer,
    compute_training_reward,
    cpu_policy_state,
    load_online_policy,
)


NANOSECONDS_PER_SECOND = 1_000_000_000


def stamp_to_nanoseconds(stamp) -> int:
    return int(stamp.sec) * NANOSECONDS_PER_SECOND + int(stamp.nanosec)


class DrlVoOnlinePPOTraining(Node):
    def __init__(self) -> None:
        super().__init__("drl_vo_online_ppo_training")
        run_root = PROJECT_ROOT / "runs" / "20260717_042135_v7_dual"
        self.declare_parameter(
            "model",
            str(
                PROJECT_ROOT
                / "github_src"
                / "drl_vo_nav-drl_vo"
                / "drl_vo"
                / "src"
                / "model"
                / "drl_vo.zip"
            ),
        )
        self.declare_parameter("output_dir", "")
        self.declare_parameter("device", "auto")
        self.declare_parameter("training_state_topic", "/drl_vo/training_state")
        self.declare_parameter("episode_reset_topic", "/drl_vo/episode_reset")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("reset_service", "/world/default/set_pose")
        self.declare_parameter(
            "robot_entity_name", "mecanum730_xms5_v7_teacher_dual_scan"
        )
        self.declare_parameter("start_x", 2.0)
        self.declare_parameter("start_y", 2.0)
        self.declare_parameter("start_z", 0.0)
        self.declare_parameter("start_yaw", 0.0)
        self.declare_parameter("reset_settle_sec", 1.0)
        self.declare_parameter("service_wait_timeout_sec", 30.0)
        self.declare_parameter("control_period_sec", 0.1)
        self.declare_parameter("total_timesteps", 100000)
        self.declare_parameter("rollout_steps", 256)
        self.declare_parameter("max_episode_steps", 512)
        self.declare_parameter("checkpoint_interval_updates", 1)
        self.declare_parameter("seed", 1337)
        self.declare_parameter("max_linear", 0.5)
        self.declare_parameter("max_angular", 2.0)
        self.declare_parameter("learning_rate", 5e-5)
        self.declare_parameter("gamma", 0.99)
        self.declare_parameter("gae_lambda", 0.95)
        self.declare_parameter("clip_range", 0.2)
        self.declare_parameter("value_coefficient", 0.5)
        self.declare_parameter("entropy_coefficient", 0.0)
        self.declare_parameter("max_grad_norm", 0.5)
        self.declare_parameter("update_epochs", 4)
        self.declare_parameter("batch_size", 64)
        self.declare_parameter("freeze_feature_extractor", True)
        self.declare_parameter("reward_progress_scale", 5.0)
        self.declare_parameter("reward_step_penalty", -0.01)
        self.declare_parameter("reward_proximity_distance", 0.9)
        self.declare_parameter("reward_proximity_scale", 0.25)
        self.declare_parameter("reward_angular_scale", 0.01)
        self.declare_parameter("reward_success", 20.0)
        self.declare_parameter("reward_collision", -20.0)
        self.declare_parameter("reward_timeout", -10.0)
        self.declare_parameter("goal_tolerance", 0.35)
        self.declare_parameter("collision_distance", 0.30)
        self.declare_parameter(
            "perception_model_contract",
            str(
                run_root
                / "training"
                / "dual_lidar_pedestrian_bev"
                / "20260731_opt_velw100_h12_c24_v1"
                / "checkpoints"
                / "epoch_014.pt"
            ),
        )

        output_text = str(self.get_parameter("output_dir").value).strip()
        if not output_text:
            raise ValueError("output_dir must be a new, explicit directory")
        self.output_dir = Path(output_text).expanduser().resolve()
        self.output_dir.mkdir(parents=True, exist_ok=False)
        self.checkpoints_dir = self.output_dir / "checkpoints"
        self.checkpoints_dir.mkdir()
        self.episodes_stream = (self.output_dir / "episodes.jsonl").open(
            "x", encoding="utf-8"
        )
        self.updates_stream = (self.output_dir / "updates.jsonl").open(
            "x", encoding="utf-8"
        )

        requested_device = str(self.get_parameter("device").value)
        if requested_device == "auto":
            requested_device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(requested_device)
        if self.device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA requested but unavailable")
        self.ppo_config = PPOConfig(
            learning_rate=float(self.get_parameter("learning_rate").value),
            gamma=float(self.get_parameter("gamma").value),
            gae_lambda=float(self.get_parameter("gae_lambda").value),
            clip_range=float(self.get_parameter("clip_range").value),
            value_coefficient=float(
                self.get_parameter("value_coefficient").value
            ),
            entropy_coefficient=float(
                self.get_parameter("entropy_coefficient").value
            ),
            max_grad_norm=float(self.get_parameter("max_grad_norm").value),
            update_epochs=int(self.get_parameter("update_epochs").value),
            batch_size=int(self.get_parameter("batch_size").value),
            freeze_feature_extractor=bool(
                self.get_parameter("freeze_feature_extractor").value
            ),
        )
        self.reward_config = RewardConfig(
            progress_scale=float(
                self.get_parameter("reward_progress_scale").value
            ),
            step_penalty=float(
                self.get_parameter("reward_step_penalty").value
            ),
            proximity_distance_m=float(
                self.get_parameter("reward_proximity_distance").value
            ),
            proximity_scale=float(
                self.get_parameter("reward_proximity_scale").value
            ),
            angular_scale=float(
                self.get_parameter("reward_angular_scale").value
            ),
            success_reward=float(self.get_parameter("reward_success").value),
            collision_reward=float(
                self.get_parameter("reward_collision").value
            ),
            timeout_reward=float(self.get_parameter("reward_timeout").value),
            goal_tolerance_m=float(self.get_parameter("goal_tolerance").value),
            collision_distance_m=float(
                self.get_parameter("collision_distance").value
            ),
        )
        self.ppo_config.validate()
        self.reward_config.validate()
        self.total_timesteps_target = int(
            self.get_parameter("total_timesteps").value
        )
        self.rollout_steps = int(self.get_parameter("rollout_steps").value)
        self.max_episode_steps = int(
            self.get_parameter("max_episode_steps").value
        )
        if min(
            self.total_timesteps_target,
            self.rollout_steps,
            self.max_episode_steps,
        ) < 2:
            raise ValueError("training and episode step counts must be >= 2")

        model_path = Path(str(self.get_parameter("model").value)).resolve()
        policy, weight_items = load_online_policy(model_path)
        self.agent = OnlinePPO(
            policy,
            self.device,
            self.ppo_config,
            int(self.get_parameter("seed").value),
        )
        self.buffer = RolloutBuffer()
        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.reset_pub = self.create_publisher(
            Empty, str(self.get_parameter("episode_reset_topic").value), 10
        )
        self.reset_client = self.create_client(
            SetEntityPose, str(self.get_parameter("reset_service").value)
        )
        self.create_subscription(
            DrlVoTrainingState,
            str(self.get_parameter("training_state_topic").value),
            self.state_callback,
            10,
        )
        self.create_timer(0.2, self.startup_timer)

        self.started_wall = time.monotonic()
        self.service_deadline = self.started_wall + float(
            self.get_parameter("service_wait_timeout_sec").value
        )
        self.initial_reset_requested = False
        self.reset_pending = False
        self.settle_until = math.inf
        self.finished = False
        self.final_saved = False
        self.last_state_timestamp_ns: int | None = None
        self.last_control_timestamp_ns: int | None = None
        self.previous_transition: dict[str, object] | None = None
        self.total_timesteps = 0
        self.update_count = 0
        self.episode_index = 0
        self.episode_steps = 0
        self.episode_reward = 0.0
        self.episode_start_wall = self.started_wall
        self.episode_returns: list[float] = []
        self.best_mean_return = -math.inf

        config = {
            "schema": "drl-vo-ros2-online-ppo/v1",
            "model": str(model_path),
            "policy_weight_items": weight_items,
            "device": str(self.device),
            "total_timesteps": self.total_timesteps_target,
            "rollout_steps": self.rollout_steps,
            "max_episode_steps": self.max_episode_steps,
            "ppo": asdict(self.ppo_config),
            "reward": asdict(self.reward_config),
            "truth_isolation": {
                "observation_source": str(
                    self.get_parameter("training_state_topic").value
                ),
                "pedestrian_ground_truth_subscription": False,
                "reward_uses_pedestrian_ground_truth": False,
                "reward_inputs": [
                    "goal_distance",
                    "minimum_scan_range",
                    "commanded_angular_velocity",
                ],
            },
            "perception_model_contract": str(
                self.get_parameter("perception_model_contract").value
            ),
        }
        (self.output_dir / "config.json").write_text(
            json.dumps(config, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        self.get_logger().warning(
            "Online PPO initialized from truth-free predicted training states; "
            f"weights={weight_items}, device={self.device}, output={self.output_dir}"
        )

    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def startup_timer(self) -> None:
        if self.finished or self.initial_reset_requested:
            return
        if self.reset_client.service_is_ready():
            self.initial_reset_requested = True
            self.request_episode_reset("startup")
        elif time.monotonic() >= self.service_deadline:
            self.get_logger().error("Gazebo reset service was not available in time")
            self.finish_training("reset_service_timeout")

    def request_episode_reset(self, reason: str) -> None:
        if self.reset_pending or self.finished:
            return
        self.publish_stop()
        self.reset_pub.publish(Empty())
        self.previous_transition = None
        self.last_control_timestamp_ns = None
        request = SetEntityPose.Request()
        request.entity.name = str(
            self.get_parameter("robot_entity_name").value
        )
        request.entity.type = Entity.MODEL
        request.pose.position.x = float(self.get_parameter("start_x").value)
        request.pose.position.y = float(self.get_parameter("start_y").value)
        request.pose.position.z = float(self.get_parameter("start_z").value)
        yaw = float(self.get_parameter("start_yaw").value)
        request.pose.orientation.z = math.sin(yaw / 2.0)
        request.pose.orientation.w = math.cos(yaw / 2.0)
        self.reset_pending = True
        self.settle_until = math.inf
        future = self.reset_client.call_async(request)
        future.add_done_callback(
            lambda completed: self.reset_done(completed, reason)
        )

    def reset_done(self, future, reason: str) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Gazebo robot reset failed: {exc}")
            self.finish_training("reset_service_error")
            return
        if response is None or not response.success:
            self.get_logger().error("Gazebo rejected the robot pose reset")
            self.finish_training("reset_rejected")
            return
        self.reset_pub.publish(Empty())
        self.reset_pending = False
        self.settle_until = time.monotonic() + float(
            self.get_parameter("reset_settle_sec").value
        )
        self.episode_steps = 0
        self.episode_reward = 0.0
        self.episode_start_wall = time.monotonic()
        self.episode_index += 1
        self.get_logger().info(
            f"episode {self.episode_index} reset complete ({reason})"
        )

    def state_callback(self, message: DrlVoTrainingState) -> None:
        if self.finished or self.reset_pending or time.monotonic() < self.settle_until:
            self.publish_stop()
            return
        timestamp_ns = stamp_to_nanoseconds(message.header.stamp)
        if (
            self.last_state_timestamp_ns is not None
            and timestamp_ns <= self.last_state_timestamp_ns
        ):
            self.get_logger().warning(
                "non-increasing training-state timestamp; resetting episode"
            )
            self.last_state_timestamp_ns = timestamp_ns
            self.request_episode_reset("timestamp_reset")
            return
        self.last_state_timestamp_ns = timestamp_ns
        period_ns = int(
            float(self.get_parameter("control_period_sec").value)
            * NANOSECONDS_PER_SECOND
        )
        if (
            self.last_control_timestamp_ns is not None
            and timestamp_ns - self.last_control_timestamp_ns < period_ns
        ):
            return
        self.last_control_timestamp_ns = timestamp_ns

        observation = np.asarray(message.observation, dtype=np.float32)
        goal_distance = float(message.goal_distance)
        minimum_scan_range = float(message.minimum_scan_range)
        if (
            observation.shape != (OBSERVATION_SIZE,)
            or not np.isfinite(observation).all()
            or not math.isfinite(goal_distance)
            or math.isnan(minimum_scan_range)
        ):
            self.get_logger().error("invalid training state; resetting episode")
            self.request_episode_reset("invalid_state")
            return

        done = False
        done_reason = "running"
        if self.previous_transition is not None:
            timed_out = self.episode_steps + 1 >= self.max_episode_steps
            reward, done, done_reason = compute_training_reward(
                float(self.previous_transition["goal_distance"]),
                goal_distance,
                minimum_scan_range,
                float(self.previous_transition["angular_velocity"]),
                timed_out=timed_out,
                config=self.reward_config,
            )
            self.buffer.add(
                self.previous_transition["observation"],
                self.previous_transition["raw_action"],
                reward,
                done,
                float(self.previous_transition["log_probability"]),
                float(self.previous_transition["value"]),
            )
            self.total_timesteps += 1
            self.episode_steps += 1
            self.episode_reward += reward

            if len(self.buffer) >= self.rollout_steps:
                self.perform_update(observation, done, timestamp_ns)

            if done:
                self.record_episode(done_reason, goal_distance, minimum_scan_range)
                self.previous_transition = None
            if self.total_timesteps >= self.total_timesteps_target:
                if len(self.buffer) >= 2:
                    self.perform_update(observation, done, timestamp_ns)
                self.finish_training("total_timesteps_reached")
                return
            if done:
                self.request_episode_reset(done_reason)
                return

        raw_action, clipped_action, log_probability, value = (
            self.agent.select_action(observation)
        )
        physical = normalized_to_physical(clipped_action)
        linear = float(
            np.clip(
                physical[0],
                0.0,
                float(self.get_parameter("max_linear").value),
            )
        )
        angular = float(
            np.clip(
                physical[1],
                -float(self.get_parameter("max_angular").value),
                float(self.get_parameter("max_angular").value),
            )
        )
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self.cmd_pub.publish(command)
        self.previous_transition = {
            "observation": observation.copy(),
            "raw_action": raw_action.copy(),
            "log_probability": log_probability,
            "value": value,
            "goal_distance": goal_distance,
            "angular_velocity": angular,
        }

    def perform_update(
        self,
        observation: np.ndarray,
        terminal: bool,
        timestamp_ns: int,
    ) -> None:
        bootstrap = 0.0 if terminal else self.agent.value(observation)
        metrics = self.agent.update(self.buffer, bootstrap)
        self.update_count += 1
        metrics.update(
            {
                "update": self.update_count,
                "total_timesteps": self.total_timesteps,
                "timestamp_ns": timestamp_ns,
            }
        )
        self.updates_stream.write(json.dumps(metrics, sort_keys=True) + "\n")
        self.updates_stream.flush()
        interval = int(
            self.get_parameter("checkpoint_interval_updates").value
        )
        if interval > 0 and self.update_count % interval == 0:
            self.save_checkpoint("latest.pt")
        self.get_logger().info(
            f"PPO update={self.update_count}, steps={self.total_timesteps}, "
            f"policy_loss={metrics['policy_loss']:.4f}, "
            f"value_loss={metrics['value_loss']:.4f}"
        )

    def record_episode(
        self,
        reason: str,
        goal_distance: float,
        minimum_scan_range: float,
    ) -> None:
        record = {
            "episode": self.episode_index,
            "total_timesteps": self.total_timesteps,
            "steps": self.episode_steps,
            "return": self.episode_reward,
            "finish_reason": reason,
            "final_goal_distance": goal_distance,
            "minimum_scan_range_at_finish": minimum_scan_range,
            "wall_seconds": time.monotonic() - self.episode_start_wall,
        }
        self.episodes_stream.write(json.dumps(record, sort_keys=True) + "\n")
        self.episodes_stream.flush()
        self.episode_returns.append(self.episode_reward)
        mean_return = float(np.mean(self.episode_returns[-100:]))
        if mean_return > self.best_mean_return:
            self.best_mean_return = mean_return
            self.save_checkpoint("best.pt")
        self.get_logger().info(
            f"episode={self.episode_index}, reason={reason}, "
            f"steps={self.episode_steps}, return={self.episode_reward:.3f}"
        )

    def save_checkpoint(self, name: str) -> None:
        destination = self.checkpoints_dir / name
        temporary = destination.with_suffix(destination.suffix + ".tmp")
        torch.save(cpu_policy_state(self.agent.policy), temporary)
        os.replace(temporary, destination)

    def write_summary(self, finish_reason: str) -> None:
        summary = {
            "schema": "drl-vo-ros2-online-ppo-summary/v1",
            "finish_reason": finish_reason,
            "total_timesteps": self.total_timesteps,
            "target_timesteps": self.total_timesteps_target,
            "updates": self.update_count,
            "unapplied_rollout_transitions": len(self.buffer),
            "episodes": len(self.episode_returns),
            "mean_episode_return": (
                float(np.mean(self.episode_returns))
                if self.episode_returns
                else None
            ),
            "best_mean_return_100": (
                self.best_mean_return
                if math.isfinite(self.best_mean_return)
                else None
            ),
            "wall_seconds": time.monotonic() - self.started_wall,
        }
        (self.output_dir / "training_summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def finish_training(self, reason: str) -> None:
        if self.finished:
            return
        self.finished = True
        self.publish_stop()
        self.save_checkpoint("final.pt")
        self.final_saved = True
        self.write_summary(reason)
        self.get_logger().warning(
            f"online PPO finished: {reason}; steps={self.total_timesteps}"
        )

    def destroy_node(self):
        if not self.final_saved:
            self.save_checkpoint("final.pt")
            self.write_summary("interrupted")
            self.final_saved = True
        if not self.episodes_stream.closed:
            self.episodes_stream.close()
        if not self.updates_stream.closed:
            self.updates_stream.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = DrlVoOnlinePPOTraining()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
