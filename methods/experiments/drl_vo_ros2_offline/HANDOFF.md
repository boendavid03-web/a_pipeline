# DRL-VO ROS 2 离线适配科研交接

更新时间：2026-07-26

项目根目录：

```text
/home/user/navigation_project/a_pipeline
```

独立 DRL-VO 实验目录：

```text
/home/user/navigation_project/a_pipeline/
methods/experiments/drl_vo_ros2_offline
```

## 1. 用户目标

使用已经录制并验证通过的 ROS 2 双雷达 bag，验证：

1. 当前数据能否转换成原始 DRL-VO 的输入；
2. 原始预训练策略能否离线回放；
3. DRL-VO 是否可以使用当前数据训练；
4. 如果方法可行，用户后续再亲自录制更丰富的数据。

当前技术可行性已经验证完成。结论是：

```text
DRL-VO 输入适配：成功
原预训练模型直接迁移：失败
当前数据行为克隆训练：成功
DRL-VO 方法线是否值得继续：是
是否已经证明闭环自主导航：否
```

## 2. 强制保护约束

不得修改原始 DRL-VO：

```text
github_src/drl_vo_nav-drl_vo
```

不得覆盖或修改：

- 原始 bag；
- 地图；
- 正式 dataset；
- manifest；
- 原始 checkpoint；
- 项目根 `README.md`。

不得自行：

- 启动 Gazebo、RViz 或 GUI；
- 启动 ROS 控制节点；
- 录制 rosbag；
- 发布 `/cmd_vel` 或其他控制话题；
- 安装或升级依赖。

当前所有 DRL-VO 代码修改都隔离在：

```text
methods/experiments/drl_vo_ros2_offline/
```

所有实验输出都位于：

```text
runs/20260717_042135_v7_dual/experiments/drl_vo_offline/<timestamp>/
```

## 3. 当前有效输入

原始只读 bag：

```text
runs/20260717_042135_v7_dual/bags/raw/
20260726_111121_v7_dual_teleop_bag
```

正式 bag 检查日志：

```text
runs/20260717_042135_v7_dual/logs/
06_check_bag_20260726_111518.log
```

只读隔离验证输出：

```text
/tmp/a_pipeline_check_20260726_111121/
```

DRL-VO 回放使用的 2739 个 NPZ：

```text
/tmp/a_pipeline_check_20260726_111121/
20260726_111121-fixed-dual-check-pedgt-v1/samples/
```

注意：`/tmp` 内容不是正式 dataset，且可能在系统重启后消失。正式回放生成的
`observations.npz` 已经保存在 `runs/` 中。

原始 checkpoint：

```text
github_src/drl_vo_nav-drl_vo/drl_vo/src/model/drl_vo.zip
```

checkpoint SHA-256：

```text
78a6bc9918b092e2bd26782664248d9ce06a5498e7cd086260dbfe6548f99645
```

## 4. 有效 bag 已知质量

- 仿真时长约 180.665 s；
- 双雷达各 2739 帧；
- 每部雷达 2000 beams、约 15.152 Hz、0.1–8 m；
- 双雷达同步误差 0 ms；
- 8 个稳定行人 ID；
- 行人真值最大间隔 0.07 s；
- `/cmd_vel` 与 `/cmd_vel_stamped` 各 3752 条；
- TF `map→base` 2739/2739 成功；
- 机器人轨迹约 59.98 m；
- 非零控制样本 1398/2739；
- Person 标签 98,691 个；
- Person 未匹配扫描为 0；
- 固定双雷达检查 PASS；
- Semantic2D 导出检查 PASS。

这包不需要重录，继续作为 DRL-VO 固定回归、输入适配和初始训练样本。

## 5. 原始 DRL-VO 情况

原项目是 ROS 1 Noetic，不是 ROS 2 Humble。

原始神经网络观测为 19202 维：

```text
pedestrian velocity map: 2 × 80 × 80 = 12800
scan history map:        1 × 80 × 80 =  6400
local goal:              2             =     2
total:                                   19202
```

动作范围：

```text
linear.x  ∈ [0, 0.5] m/s
angular.z ∈ [-2, 2] rad/s
```

模型 zip 实际保存版本：

```text
Stable-Baselines3 0.10.0
```

当前环境没有 Gym、Gymnasium 或 Stable-Baselines3。没有安装旧依赖，而是用当前
PyTorch 独立重建网络。`policy.pth` 包含 163 项完整权重：

- `features_extractor`；
- `mlp_extractor.policy_net`；
- `mlp_extractor.value_net`；
- `action_net`；
- `value_net`；
- `log_std`。

## 6. 当前局部目标来源

当前数据已经包含 `sub_goal_local`，不是缺少目标输入。

转换逻辑：

```text
当前帧 i
未来帧 j = min(i + 20, 最后一帧)
计算未来位姿相对当前位姿的 dx、dy
根据当前 yaw 旋转到 base_link
保存为 sub_goal_local = (x, y)
```

实现位置：

```text
workspaces/ros2_ws/tools/
convert_rosbag2_to_semantic2d_native_lidar.py
```

当前 2739 个局部目标：

- shape 全部为 `(2,)`；
- 全部有限；
- 最大距离约 0.948 m；
- 1309 帧为零；
- 2229 帧距离不超过 0.9 m。

录制时不要求提前设置终点。当前局部目标足够用于离线回放和训练。

限制：该目标来自未来真实轨迹，属于 hindsight subgoal。它会泄漏一部分未来走/停信息，
所以不能仅凭行为克隆高分宣称已经学会在线目标导航。

## 7. 已实现代码

独立说明：

```text
methods/experiments/drl_vo_ros2_offline/README.md
```

代码文件：

```text
drlvo_model.py
observation_adapter.py
replay.py
train_behavior_cloning.py
analyze_goal_shortcut.py
test_offline.py
```

用途：

- `drlvo_model.py`
  - 精确重建原 `CustomCNN`；
  - 重建 PPO policy/value MLP；
  - 从 zip 内存读取 `policy.pth`；
  - 163 项权重 `strict=True` 加载。

- `observation_adapter.py`
  - 行人 map/velocity 转换到 `base_link`；
  - 生成 `2×80×80` 行人速度图；
  - 双雷达转换为原模型 1080/720 束语义；
  - 复现 10 帧、每 9 束 min/mean 压缩；
  - 拼接 19202 维观测；
  - 计算最近障碍、最近行人、TTC 和最近会遇距离代理。

- `replay.py`
  - 不使用 ROS 的全量 shadow replay；
  - 输出未裁剪动作、SB3 兼容裁剪动作和原节点门控动作；
  - 生成 `observations.npz`、`predictions.csv`、`summary.json` 和报告。

- `train_behavior_cloning.py`
  - 冻结 DRL-VO 特征提取器；
  - 微调 policy MLP 和 action head；
  - 支持 goal 置零的 sensor-only 消融；
  - 保存完整 163 项 checkpoint。

- `analyze_goal_shortcut.py`
  - 训练 goal-only MLP；
  - 评估 hindsight subgoal 捷径。

- `test_offline.py`
  - 当前共有 6 项单元和集成测试。

## 8. 正式离线回放结果

正式目录：

```text
runs/20260717_042135_v7_dual/experiments/drl_vo_offline/
20260726_115055/
```

结果：

- 2739/2739 帧完成；
- 观测 shape 为 `(2739, 19202)`；
- 无 NaN/Inf；
- 163 项权重严格加载；
- CPU 推理速度 79.47 Hz；
- 所有预测动作有限并在物理范围内；
- 第一维未裁剪动作均值约 23.46；
- 第一维经 SB3 兼容裁剪后 100% 为 `+1`；
- 映射后线速度 100% 为 `0.5 m/s`；
- 动作多样性 FAIL。

因此：

```text
原始预训练模型不能直接部署到当前机器人。
```

`replay.py` 因动作多样性失败返回退出码 2，但输出完整，这是预期研究结论。

原推理节点门控复现结果：

```text
goal_within_0.9m: 2229 帧
policy:             510 帧
```

0.6 m 行人 TTC 代理：

```text
有限 TTC 帧数: 712
最小 TTC:      1.411 s
```

该 TTC 是恒速度离线代理，不是碰撞标签。

## 9. 正式行为克隆结果

正式目录：

```text
runs/20260717_042135_v7_dual/experiments/drl_vo_offline/
20260726_114731/
```

最佳 checkpoint：

```text
20260726_114731/checkpoints/best.pt
```

训练策略：

- 冻结 DRL-VO CNN 特征提取器；
- 微调 policy MLP 与 action head；
- 100 帧连续块；
- 按 `train/train/train/val/test` 循环分配；
- 每个块边界清除 20 帧；
- train 1059、val 300、test 300、purged 1080；
- 最佳 epoch 142。

测试结果：

```text
pretrained normalized MSE: 1.10864
constant normalized MSE:   0.51546
fine-tuned normalized MSE: 0.08645

linear MAE:                0.0404 m/s
angular MAE:               0.1647 rad/s
walk/stop balanced acc:    93.72%
```

最佳 checkpoint 已独立用新建模型严格重载：

```text
163/163 keys matched
```

因此：

```text
当前数据可以成功训练 DRL-VO，方法技术可行。
```

## 10. 消融结果

### 10.1 Goal-only

目录：

```text
runs/20260717_042135_v7_dual/experiments/drl_vo_offline/
20260726_114929/
```

结果：

```text
test MSE:                   0.09274
walk/stop balanced acc:     94.36%
```

goal-only 几乎达到完整输入成绩，确认 hindsight subgoal 存在明显捷径。

### 10.2 Sensor-only

目录：

```text
runs/20260717_042135_v7_dual/experiments/drl_vo_offline/
20260726_115351/
```

将 goal 强制设为零，保留雷达和行人输入：

```text
test MSE:                   0.14016
walk/stop balanced acc:     91.16%
constant balanced acc:      50.00%
```

说明当前双雷达和行人观测本身确实存在可学习控制信号。

## 11. 综合判断

已经证明：

1. ROS 2 双雷达和行人数据可以适配到原 DRL-VO；
2. 原 checkpoint 可以无旧 SB3 依赖严格重建；
3. 离线推理性能满足 15 Hz；
4. 当前 bag 可以让 DRL-VO 训练收敛；
5. 训练后动作不再恒定；
6. 传感器输入包含可学习信号。

没有证明：

1. 微调模型可以闭环控制机器人；
2. 模型可以跨路线、seed 和人流密度泛化；
3. 模型已经学会可靠的碰撞规避；
4. hindsight subgoal 的高分可以代表在线目标导航。

对用户问题“DRL-VO 可以用吗”的当前回答：

```text
方法可以用，适配和训练线已经打通；
仓库自带预训练模型不能直接用；
后续应使用用户更好的数据重新训练或微调。
```

## 12. 下一步

用户后续会亲自录制更好的数据。当前不要擅自启动仿真、GUI 或录包。

后续数据可以继续沿用普通 SLAM + teleop：

- 不强制提前设置最终终点；
- 转换器继续生成 hindsight `sub_goals_local`；
- 移动时可以固定 `0.5 m/s`；
- 需要包含停止、左转、右转和避让；
- 不要超过 DRL-VO 的 `0.5 m/s` 线速度范围；
- 使用多路线、多 seed 和多行人密度；
- 按整包或整路线划分训练、验证和测试。

用户提供新 bag 后建议顺序：

1. 先运行现有 bag 完整性和双雷达检查；
2. 转换到独立固定双雷达数据目录；
3. 对每个 bag 生成 DRL-VO 观测；
4. 按整包划分 train/val/test，避免同一轨迹泄漏；
5. 先运行原预训练回放；
6. 再从当前 `best.pt` 或原 `drl_vo.zip` 做对照微调；
7. 重做 goal-only、sensor-only 和完整输入比较；
8. 只有留出整包结果稳定后，再讨论闭环测试。

如果以后目标变为“自主到达指定终点”，再记录或在线生成：

- `local_subgoal`；
- `final_goal`；
- `global_path`；
- episode success/collision/timeout。

这不是当前离线训练可行性测试的前置条件。

## 13. 验证与安全状态

最终测试：

```text
Ran 6 tests
OK
```

保护源审计：

- 原始 DRL-VO 无新增修改时间文件；
- 根 `README.md` 未修改；
- 原始 bag metadata 时间未改变；
- 原始 checkpoint 时间未改变；
- 未安装或升级依赖；
- 未启动 ROS、Gazebo 或 GUI；
- 未发布控制话题。

综合结构化结论：

```text
runs/20260717_042135_v7_dual/experiments/drl_vo_offline/
20260726_115504/research_summary.json
```

综合可读报告：

```text
runs/20260717_042135_v7_dual/experiments/drl_vo_offline/
20260726_115504/report.md
```

完整运行方法和命令见同目录的：

```text
README.md
```
