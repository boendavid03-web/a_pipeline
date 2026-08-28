# DRL-VO ROS 2 纯离线适配、回放与训练

本目录是独立的 DRL-VO 实验线，用于将 `a_pipeline` 的 ROS 2 双雷达、行人真值、
局部目标和遥控动作转换为原始 DRL-VO 的 19202 维输入，并完成：

1. Stable-Baselines3 0.10.0 checkpoint 的无 SB3 PyTorch 重建；
2. 预训练策略纯离线 shadow replay；
3. 基于录制动作的行为克隆可行性训练；
4. goal-only 与 sensor-only 消融；
5. 动作范围、推理速度、TTC 代理指标和域差异检查。

这里的所有程序都是离线程序，不启动 ROS 节点，不发布 `/cmd_vel`，不启动 Gazebo、
RViz 或其他 GUI。

## 0. 当前结论

当前主验证使用三个 seed 隔离的 bag：

```text
train: seed 17 / 20260727_074611
dev:   seed 27 / 20260727_080451
test:  seed 47 / 20260727_084207

隔离任务根目录:
runs/20260717_042135_v7_dual/datasets/
20260727_three_bag_online_seed_split_v1/

原始 checkpoint:
github_src/drl_vo_nav-drl_vo/drl_vo/src/model/drl_vo.zip
```

结论：

| 项目 | 结果 |
|---|---:|
| 28478 帧、31 个 episode 的转换与检查 | PASS |
| 观测 NaN/Inf 检查 | PASS |
| 163 项权重 `strict=True` 加载 | PASS |
| 三 split 有界离线 replay | PASS |
| 原预训练策略直接迁移 | 线速度饱和，不能作为部署结论 |
| 四种行为克隆 smoke 链路 | PASS |
| 训练 checkpoint 严格重载 | PASS |
| ROS、仿真和控制话题使用 | 无 |

当前 replay 使用录制时的 causal online subgoal，并在每个 bag/episode 边界重置
10 帧历史。原策略在有界 smoke 上仍出现线速度饱和，因此不能直接部署。当前
2-epoch BC 结果只证明数据、split、反向传播和 checkpoint 链路可用，不是能力评估，
更不等于闭环自主导航成功。

机器可读报告位于：

```text
runs/20260717_042135_v7_dual/datasets/
20260727_three_bag_online_seed_split_v1/control/
```

## 1. 保护边界

不得修改：

```text
github_src/drl_vo_nav-drl_vo
runs/20260717_042135_v7_dual/bags/raw
原始地图、dataset、manifest 和 checkpoint
项目根 README.md
```

本实验代码只位于：

```text
methods/experiments/drl_vo_ros2_offline/
```

实验输出只写入新的时间戳目录：

```text
runs/20260717_042135_v7_dual/experiments/drl_vo_offline/<timestamp>/
```

不需要安装旧版 Gym 或 Stable-Baselines3，也不要覆盖 `drl_vo.zip`。

## 2. DRL-VO 输入

每帧观测总长度为 19202：

```text
pedestrian velocity map: 2 × 80 × 80 = 12800
scan history map:        1 × 80 × 80 =  6400
local goal:              2             =     2
total:                                   19202
```

### 2.1 行人速度图

每个行人先从 map 坐标转换到当前 `base_link`：

```text
x ∈ [0, 20] m
y ∈ [-10, 10] m
grid resolution = 0.25 m
```

两个通道分别保存机器人坐标系下的 `vx` 和 `vy`，再按原模型的 `[-2,2]` 范围归一化。

### 2.2 激光历史图

当前双 2000 束雷达先转换到 `base_link`，再构造原模型的约 270°、1080 束虚拟扫描。
取其中前方 720 束，保留最近 10 帧。每 9 束分别计算最小值和平均值，得到
`20×80`，然后复制为 `80×80`，最后按原模型的 30 m 范围归一化。

### 2.3 局部目标

当前数据已经包含：

```text
sub_goal_local: shape=(2,)
```

当前三包任务中的该字段来自录制的 online planner subgoal。转换器按 episode 做
causal hold-last，只使用不晚于 scan 的消息，最大允许年龄 300 ms；无先验或过旧的帧
会被丢弃。时间戳、年龄和 episode ID 均保存在样本与导出 metadata 中。

### 2.4 动作

原策略输出两个归一化动作，并映射为：

```text
linear.x  ∈ [0, 0.5] m/s
angular.z ∈ [-2, 2] rad/s
```

当前三包含少量倒车动作。行为克隆标签会单独裁剪到 `[0,0.5]`，全量线速度裁剪率
约为 `0.137%`；原始 `/cmd_vel_stamped` 和导出 raw action 均不会被修改。

### 2.5 可选语义接口

默认路径保持原始 19202 维观测和 163 项 checkpoint 严格加载不变。显式开启后，
适配器会从每帧 `semantic_label` 构造与 10 帧激光历史对齐的 categorical
`80×80` 语义图：

```text
每个角度组：最近有效束类别 + 有效束多数类别
未知/无标签：-1
有效类别：0 .. semantic_num_classes-1
```

模型使用独立的 embedding + 小型卷积编码器，并通过零初始化残差投影与原 256 维
DRL-VO 特征做 late fusion。语义分支初始为严格 no-op；不开启语义或尚未训练时，
不会改变原预训练策略输出。

当前 `ground-truth-legs` 数据中的 Person 标签来自仿真真值。闭环部署时必须由在线
感知产生同合同标签，或者训练与上线都屏蔽 Person，不能把真值标签直接当作在线输入。

### 2.6 语义替代行人速度图

`--drop-pedestrian-velocity` 会将原观测的两个 `80×80` 行人 `vx/vy` 通道固定为零，
并要求同时启用 `--use-semantics`。此时策略可用的输入为：

```text
scan history:          1 × 80 × 80
categorical semantics: 1 × 80 × 80（独立 embedding + CNN 分支）
local goal:            2
pedestrian vx/vy:      固定为 0，不携带信息
```

保留 19202 维张量外形是为了严格复用预训练 DRL-VO 主干；固定零通道在数学上不携带
行人速度信息。语义图是唯一的行人信息来源。当前 Person 标签仍来自仿真真值位置，
因此该消融去除了真值速度依赖，但尚未去除真值 Person 检测依赖。

## 3. 文件结构

```text
methods/experiments/drl_vo_ros2_offline/
├── README.md
├── drlvo_model.py
├── observation_adapter.py
├── replay.py
├── train_behavior_cloning.py
├── analyze_goal_shortcut.py
├── visualize_pedestrian_velocity_map.py
└── test_offline.py
```

各文件用途：

- `drlvo_model.py`：重建 `CustomCNN`、PPO policy/value MLP 和动作头，读取 zip 中的
  `policy.pth` 并严格加载。
- `observation_adapter.py`：生成行人图、双雷达历史图、局部目标和 TTC 代理指标。
- `replay.py`：全量离线回放并写出观测、预测、统计和报告。
- `train_behavior_cloning.py`：冻结 DRL-VO 特征提取器，微调 policy MLP 和 action head。
- `analyze_goal_shortcut.py`：训练 goal-only 模型并检查 hindsight subgoal 捷径。
- `visualize_pedestrian_velocity_map.py`：显示地图坐标中的行人状态、机器人坐标系位置、
  两个 `80×80` 速度通道和时间序列动画。
- `test_offline.py`：单元测试和单帧严格加载集成测试。

## 4. 激活环境

打开终端后执行：

```bash
cd /home/user/navigation_project/a_pipeline
source environment/activate.sh

cd methods/experiments/drl_vo_ros2_offline
```

本实验实际使用：

```text
.venvs/train/bin/python
Python 3.10
PyTorch 2.11.0+cu128
```

当前主机 RTX 5060 Ti 可用；S3-Net/SemanticCNN smoke 已在 CUDA 上通过。DRL-VO
有界 replay/BC smoke 为离线接口验证，不依赖 CUDA。

## 5. 运行测试

```bash
cd /home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline

/home/user/navigation_project/a_pipeline/.venvs/train/bin/python \
  -m unittest -v test_offline.py
```

当前结果：

```text
Ran 7 tests
OK
```

测试覆盖：

- map 到 base 的二维向量旋转；
- 原始 10×720 激光 min/mean 压缩；
- 10 帧语义图的最近类别/多数类别压缩；
- DRL-VO 动作映射；
- 分块数据集互斥和边界清除；
- 合成行人相遇 TTC；
- 163 项权重严格加载和单帧前向。

### 5.1 可视化行人位置与速度图

```bash
MPLCONFIGDIR=/tmp/a_pipeline_pedviz_mpl \
/home/user/navigation_project/a_pipeline/.venvs/train/bin/python \
  visualize_pedestrian_velocity_map.py \
  --samples "$DRLVO_SAMPLES" \
  --map-yaml \
    /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/maps/semantic_label/map.yaml \
  --output-root "$DRLVO_OUTPUT_ROOT"
```

默认自动选择机器人视野内最近行人距离最小的帧，生成 overview PNG、前后时间窗口 GIF、
`summary.json`、运行命令和环境记录。它只读 NPZ，不启动 ROS。

## 6. 全量离线 shadow replay

先设置输入路径：

```bash
cd /home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline

export DRLVO_SAMPLES="/tmp/a_pipeline_check_20260726_111121/20260726_111121-fixed-dual-check-pedgt-v1/samples"
export DRLVO_MODEL="/home/user/navigation_project/a_pipeline/github_src/drl_vo_nav-drl_vo/drl_vo/src/model/drl_vo.zip"
export DRLVO_OUTPUT_ROOT="/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/experiments/drl_vo_offline"
```

执行：

```bash
/home/user/navigation_project/a_pipeline/.venvs/train/bin/python \
  replay.py \
  --samples "$DRLVO_SAMPLES" \
  --model "$DRLVO_MODEL" \
  --output-root "$DRLVO_OUTPUT_ROOT" \
  --batch-size 16
```

程序自动创建：

```text
<timestamp>/
├── observations.npz
├── predictions.csv
├── summary.json
├── report.md
├── run_command.txt
└── environment.json
```

`predictions.csv` 包含：

- 未裁剪策略动作；
- SB3 兼容裁剪后的物理动作；
- 原推理节点 0.9 m 目标门控后的动作；
- bag 中录制的控制；
- local subgoal；
- 最近障碍和最近行人距离；
- 0.6 m 行人 TTC 代理；
- 最近会遇距离和时间；
- 前方扫描覆盖率；
- value estimate。

当前预训练模型会因为线速度 100% 饱和而使 `action_diversity` 失败，`replay.py`
因此返回退出码 2。这是预期的研究结论，不代表输出不完整。正式全量结果位于：

```text
runs/20260717_042135_v7_dual/experiments/drl_vo_offline/
20260726_115055/
```

## 7. 行为克隆训练

使用已经生成的离线观测：

```bash
cd /home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline

export DRLVO_REPLAY_DIR="/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/experiments/drl_vo_offline/20260726_115055"
export DRLVO_MODEL="/home/user/navigation_project/a_pipeline/github_src/drl_vo_nav-drl_vo/drl_vo/src/model/drl_vo.zip"
export DRLVO_OUTPUT_ROOT="/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/experiments/drl_vo_offline"
```

执行：

```bash
/home/user/navigation_project/a_pipeline/.venvs/train/bin/python \
  train_behavior_cloning.py \
  --replay-dir "$DRLVO_REPLAY_DIR" \
  --model "$DRLVO_MODEL" \
  --output-root "$DRLVO_OUTPUT_ROOT" \
  --epochs 200 \
  --batch-size 64 \
  --feature-batch-size 16 \
  --learning-rate 3e-4 \
  --block-size 100 \
  --purge-frames 20 \
  --patience 30 \
  --seed 1337
```

可选语义训练需要先生成包含语义缓存的新 replay：

```bash
/home/user/navigation_project/a_pipeline/.venvs/train/bin/python \
  replay.py \
  --samples "$DRLVO_SAMPLES" \
  --model "$DRLVO_MODEL" \
  --output-root "$DRLVO_OUTPUT_ROOT" \
  --batch-size 16 \
  --include-semantics

export DRLVO_SEMANTIC_REPLAY_DIR="<上一步新建的时间戳目录>"

/home/user/navigation_project/a_pipeline/.venvs/train/bin/python \
  train_behavior_cloning.py \
  --replay-dir "$DRLVO_SEMANTIC_REPLAY_DIR" \
  --model "$DRLVO_MODEL" \
  --output-root "$DRLVO_OUTPUT_ROOT" \
  --use-semantics \
  --semantic-num-classes 7 \
  --semantic-person-class 6 \
  --epochs 200 \
  --batch-size 64 \
  --feature-batch-size 16 \
  --learning-rate 3e-4 \
  --block-size 100 \
  --purge-frames 20 \
  --patience 30 \
  --seed 1337
```

语义替代行人速度图训练：

```bash
"$TORCH_PY" train_behavior_cloning.py \
  --replay-dir "$DRLVO_REPLAY" \
  --model "$DRLVO_PRETRAINED" \
  --output-root "$TASK_ROOT/training/drl_vo/semantic_no_ped_bc" \
  --use-semantics \
  --drop-pedestrian-velocity \
  --semantic-num-classes 7 \
  --semantic-person-class 6 \
  --epochs 200 \
  --batch-size 64 \
  --feature-batch-size 16 \
  --learning-rate 3e-4 \
  --patience 30 \
  --seed 1337
```

正式对照结果位于：

```text
training/drl_vo/semantic_no_ped_bc/20260730_121840/
```

相同 whole-bag seed split 下：

| 模型 | test normalized MSE | 相对 base |
|---|---:|---:|
| base（`vx/vy + scan + goal`） | 0.318941 | 基准 |
| semantic（`vx/vy + semantic + scan + goal`） | 0.319000 | +0.02% |
| semantic_no_ped（`semantic + scan + goal`） | 0.321153 | +0.69% |

`semantic_no_ped` 的语义图在 split 内随机打乱后 test MSE 为 `0.377883`，屏蔽
Person=6 后为 `0.360813`，说明训练后的策略确实使用了语义和 Person 信息。该结果仍
只是离线动作拟合，不代表闭环成功。

夜间多 seed 长训使用较小学习率和更长早停耐心：

```bash
cd /home/user/navigation_project/a_pipeline
nohup methods/experiments/drl_vo_ros2_offline/run_semantic_no_ped_overnight.sh \
  > runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/training/drl_vo/semantic_no_ped_overnight_launcher.log \
  2>&1 &
echo $!
```

该脚本顺序训练 5 个 seed，每个最多 1200 epoch、patience 200，并写入独立时间戳目录。
最终只能按 dev 的 `best_val_smooth_l1` 选择候选模型，不能用 test 挑 checkpoint；
选定后再报告一次 held-out test。

语义增强 checkpoint 包含原 DRL-VO 权重和新增 `semantic_fusion.*` 权重；它是新模型
格式，不能冒充原始 163 项 checkpoint。

当前语义接口正式验证：

```text
semantic replay: 20260726_124946/
semantic training: 20260726_125523/
best checkpoint: 20260726_125523/checkpoints/best.pt
```

- 2739 个 `80×80` 语义图全部生成，标签范围为 `-1..6`；
- 新 checkpoint 共 172 项，独立 `strict=True` 重载无缺失、无多余项；
- 正确语义测试 MSE：`0.09052`；
- split 内随机打乱语义测试 MSE：`0.11484`；
- 屏蔽 Person 测试 MSE：`0.09148`；
- 原无语义完整输入微调测试 MSE：`0.08645`。

因此语义分支确实使用了语义空间结构，且主要收益不是仿真 Person 真值造成；但当前单包
数据上它没有整体超过无语义模型，应继续保持为可选实验接口，不能替换现有最佳结果。

输出：

```text
<timestamp>/
├── checkpoints/
│   ├── best.pt
│   └── last.pt
├── dataset_split.json
├── eval_predictions.csv
├── train_metrics.csv
├── training_summary.json
├── report.md
├── run_command.txt
└── environment.json
```

训练划分不是随机逐帧切分。连续 100 帧组成一个块，按
`train/train/train/val/test` 循环分配，并在每个边界丢弃 20 帧，隔离 10 帧扫描历史和
未来 20 帧局部目标。

当前正式训练结果：

```text
runs/20260717_042135_v7_dual/experiments/drl_vo_offline/
20260726_114731/
```

最佳 checkpoint：

```text
20260726_114731/checkpoints/best.pt
```

## 8. 历史单包 hindsight goal 捷径消融

```bash
cd /home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline

/home/user/navigation_project/a_pipeline/.venvs/train/bin/python \
  analyze_goal_shortcut.py \
  --replay-dir "$DRLVO_REPLAY_DIR" \
  --training-dir "$DRLVO_OUTPUT_ROOT/20260726_114731" \
  --output-root "$DRLVO_OUTPUT_ROOT" \
  --epochs 500 \
  --learning-rate 1e-3 \
  --patience 50 \
  --seed 1337
```

当前结果：

```text
goal-only test MSE:                  0.09274
goal-only walk/stop balanced acc:   94.36%
full-input walk/stop balanced acc:  93.72%
```

goal-only 几乎达到完整输入的成绩，因此当前行为克隆分数包含明显的未来轨迹捷径。
这不影响“DRL-VO 可以适配和训练”的结论，但限制了对在线导航能力的解释。

正式消融结果：

```text
runs/20260717_042135_v7_dual/experiments/drl_vo_offline/
20260726_114929/
```

## 9. sensor-only 消融

保持雷达和行人输入，将 goal 强制设为零：

```bash
cd /home/user/navigation_project/a_pipeline/methods/experiments/drl_vo_ros2_offline

/home/user/navigation_project/a_pipeline/.venvs/train/bin/python \
  train_behavior_cloning.py \
  --replay-dir "$DRLVO_REPLAY_DIR" \
  --model "$DRLVO_MODEL" \
  --output-root "$DRLVO_OUTPUT_ROOT" \
  --epochs 200 \
  --batch-size 64 \
  --feature-batch-size 16 \
  --learning-rate 3e-4 \
  --block-size 100 \
  --purge-frames 20 \
  --patience 30 \
  --seed 1337 \
  --zero-goal
```

当前结果：

```text
sensor-only test MSE:                 0.14016
sensor-only walk/stop balanced acc:   91.16%
constant baseline balanced acc:       50.00%
```

这说明当前双雷达和行人观测中存在可学习控制信号。

正式结果：

```text
runs/20260717_042135_v7_dual/experiments/drl_vo_offline/
20260726_115351/
```

## 10. 如何解释结果

当前已经证明：

1. ROS 2 双雷达数据可以适配为原 DRL-VO 输入；
2. 原 checkpoint 可以在当前 PyTorch 中严格重建；
3. 离线推理速度满足 15 Hz 实时要求；
4. 三包可以按 seed 整包隔离并在 episode 边界重置历史；
5. 行为克隆的完整输入、goal-only、sensor-only 和 semantic smoke 共用相同 split；
6. 原预训练策略在当前域上出现动作饱和，不能直接部署。

当前尚未证明：

1. 微调模型能够闭环控制当前机器人；
2. 模型能够在新路线、新 seed 和新人流密度下泛化；
3. 模型已经学会真实碰撞规避；
4. 当前 online subgoal 的离线分数可以代表闭环在线目标导航。

因此，本目录的结论是：

```text
DRL-VO 方法可用，适配和训练链路已打通；
原预训练模型不可直接部署；
后续更丰富的数据应继续用于重新训练和严格留包评估。
```

## 11. 后续数据

继续沿用主流水线的普通 SLAM + teleop 录制方式即可。录制时不强制提前设置终点，
正式数据转换应继续记录并使用 causal online `sub_goals_local`；hindsight 模式只保留
为历史兼容和明确标注的离线实验。

建议后续数据满足：

- 移动速度不超过 DRL-VO 的 `0.5 m/s`；
- 可以在移动时固定使用 `0.5 m/s`，但要包含停止和左右转向；
- 多路线、多 seed、多行人密度；
- 按整包或整路线划分训练、验证和测试；
- 保留失败、停车、避让和转向样本；
- 如果以后验证完整目标导航，再额外记录在线 local subgoal、final goal 和 global path。

当前 bag 不需要丢弃，继续作为固定回归测试和初始微调数据。

## 12. 安全说明

这些程序不会：

- 启动 ROS 1 或 ROS 2；
- 启动 Gazebo、RViz 或 GUI；
- 发布 `/cmd_vel` 或任何控制话题；
- 录制 rosbag；
- 修改原始 bag、地图、数据集或 checkpoint；
- 安装或升级依赖。

如果未来需要闭环验证，应另建独立阶段，并在运行前明确检查控制话题发布者、速度上限、
碰撞停车和目标停车。当前离线结果不能直接作为发布控制的授权。
