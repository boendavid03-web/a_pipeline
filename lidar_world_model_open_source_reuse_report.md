# 2D LiDAR 世界模型开源复用审计

日期：2026-09-21

## 结论

可以从当前数据开始，但第一步不应从零搭一套世界模型，也不应直接搬运完整 JEPLO。当前项目最合适的组合是：

1. 复用项目已有的双雷达 4000-slot 转换、时间窗、TF/ego-motion 对齐和 BEV 数据代码；
2. 以 Meta 的 EB-JEPA `ac_video_jepa` 为训练骨架，复用 action-conditioned latent predictor、multi-step rollout、anti-collapse loss、inverse-dynamics probe 和 MPPI/CEM 评估；
3. 从 LeWorldModel 复用 SIGReg 和简洁的自回归 predictor 接口，作为第二种 anti-collapse 配置；
4. 用 NavRep 的 `VAE1D_LSTM/GPT1D` 和 SCOPE 的 stochastic OGM prediction 作为 2D LiDAR 基线；
5. 从 JEPLO 复用 LiDAR 退化增强、proprio/exteroception 联合条件和 JEPA teacher-student 思路，而不是复用其腿足机器人训练栈。

第一版的目标应是验证“当前双雷达数据能否学习出非坍塌、动作相关、可多步预测的 latent”，还不能宣称学到了行人群组或人对机器人的响应。当前 bag 只有四个 episode，动作和场景覆盖不足以验证交互因果或泛化。

## 开源项目可复用性排序

| 优先级 | 项目 | 开源状态 | 可直接复用内容 | 不能直接复用的部分 | 对当前项目的判断 |
|---|---|---|---|---|---|
| 1 | [EB-JEPA](https://github.com/facebookresearch/eb_jepa) | Apache-2.0；完整代码；无现成 LiDAR 权重 | `ac_video_jepa` 的 action-conditioned encoder/predictor、GRU rollout、variance/covariance、time-sim、inverse dynamics、MPPI/CEM、训练诊断 | 默认输入是 Two-Rooms 图像；默认配置面向 H100；需要替换 dataset 与 encoder | **最适合做第一版骨架**。它已经覆盖我们原本要自己写的大部分世界模型训练、rollout 和诊断逻辑 |
| 2 | [LeWorldModel](https://github.com/lucas-maes/le-wm) | MIT；完整代码；提供论文数据/检查点入口 | `SIGReg`、AdaLN action conditioning、`ARPredictor`、latent rollout、goal-latent cost | 官方 encoder 是像素 ViT，训练脚本依赖 `stable_pretraining`、`stable_worldmodel`、Hydra/Lightning | **适合抽取小模块**，不适合整仓接入。当前环境缺少其上层训练依赖，但核心 `module.py` 只需 PyTorch/einops，逻辑可移植 |
| 3 | [JEPLO](https://github.com/ASIG-X/JEPLO) | GPLv3；完整训练/部署代码、数据集、预训练 policy | LiDAR occlusion/sparsity/noise 增强，五帧点云累积，proprio-exteroceptive 条件，CJTS teacher-student 训练设计，SIGReg 用法 | Isaac Lab 2.3.2、RSL-RL、4096 env、四足状态和低层关节动作；不是独立的 2D LiDAR world-model 包 | **方法参考价值高，直接代码复用价值中等**。仓库中的 `training/lejepa` 主要是统计正则包，核心 PE-JEPA 与腿足 RL 深度耦合 |
| 4 | [NavRep](https://github.com/ethz-asl/navrep) | MIT；完整代码、预训练模型、环境 | 1D LiDAR VAE、rings 表示、action-conditioned LSTM/Transformer、`z`/`h`/`z+h` 对比、dream rollout | Python 3.6、Keras 2.3.1、ROS1，工程栈老旧 | **非常适合作为科学基线**，不建议直接继承运行环境。可复刻其 32-D `z` 和 64-D recurrent state 作为对照 |
| 5 | [SCOPE](https://github.com/TempleRAIL/scope) / [SOGMP](https://github.com/TempleRAIL/SOGMP) | MIT；完整代码、公开 OGM 数据；三种速度/精度分支 | LiDAR 到 occupancy、随机多未来预测、预测不确定性、0.5 s/5-step 指标和 costmap 接口 | Python 3.7、PyTorch 1.7.1；输出是栅格未来，不是 JEPA latent | **最重要的显式未来基线**。当前项目的 S3-Net 源码本身已经保留 SOGMP 血缘，可减少数据适配成本 |
| 6 | [Point-JEPA](https://github.com/Ayumu-J-S/Point-JEPA) | MIT；完整代码和预训练/下游检查点 | 点云 patch、空间邻近 sequencer、masked context-target 选择 | ShapeNet 静态 3D 物体；没有时间、动作或导航 | **只借鉴 tokenization**。不适合直接作为世界模型骨架 |
| 7 | [AD-L-JEPA](https://github.com/HaoranZhuExplorer/adljepa) | 代码和 KITTI/ONCE 权重公开；仓库未在主页明确展示许可证 | 3D LiDAR BEV masked JEPA、OpenPCDet 数据管线和权重 | Python 3.8、PyTorch 1.10/OpenPCDet 0.5；任务是 3D 检测预训练，无动作动态 | 可借鉴 BEV masking 和局部/全局上下文，第一版不值得承担 OpenPCDet 依赖 |
| 8 | [JEPA-WMs](https://github.com/facebookresearch/jepa-wms) | CC BY-NC 4.0；代码、数据、权重 | 完整 latent planning、DINO-WM/JEPA-WM/V-JEPA2-AC 对照、PointMaze 配置、PyTorch Hub 模型 | 图像 foundation encoder、依赖较重、非商业许可证 | 适合后期验证 planner 和表征评估协议，不适合当前小数据第一跑 |
| 9 | [NavThinker](https://github.com/hutslib/NavThinker) | MIT；当前属于分阶段开放 | social navigation 中共享 latent 预测 future depth、human trajectory、reward；action-conditioned imagination reward | 检查点、稳定 API、端到端安装和 standalone offline WM 尚未发布 | **idea 很贴近最终方向，但目前不能作为可靠底座** |
| 10 | [SlotFormer](https://github.com/pairlab/SlotFormer) / [HOWM](https://github.com/linfeng-z/HOWM) | 有代码；SlotFormer 有预训练 slots/weights，HOWM 有训练代码 | object slots、集合匹配、每对象 latent dynamics、组合泛化 | 图像 object discovery 和合成任务数据；迁移到 2D LiDAR 要先解决实例分解 | 留到第二阶段，用于验证“多个 z/slots 是否分别承载人、群组和静态结构” |

## 为什么首选 EB-JEPA，而不是直接以 JEPLO 开始

JEPLO 的价值在方法设计：它把 raw LiDAR 派生的深度图与本体状态一起编码，在 latent 中预测未来，并通过随机遮挡、稀疏和噪声迫使表示保留持久几何。论文与仓库还提供了 sim-to-real、MuJoCo 验证和部署链。

但它的训练入口是四足 locomotion policy：Isaac Lab 2.3.2、RSL-RL、4096 个并行环境、teacher privileged height map、关节状态与关节动作共同参与训练。把这套栈改成移动机器人双 2D 雷达，会同时修改 observation、action、teacher、reward、environment 和 deployment，复用比例反而低。

EB-JEPA 的 `ac_video_jepa` 正好提供移动机器人第一版需要的抽象接口：

```text
observation_t -- Encoder --> z_t
                                  \
action_t ---------------------- Predictor --> z_hat_(t+1:t+H)

loss = latent prediction
     + variance/covariance anti-collapse
     + temporal smoothness
     + inverse-dynamics action recovery
```

论文示例明确做了 autoregressive rollout、action swap、inverse dynamics 和 MPPI/CEM planning。尤其值得保留的是 inverse-dynamics loss：它要求 `(z_t, z_{t+1})` 能恢复动作，可直接检验 latent 是否真正保留了与机器人运动有关的信息。EB-JEPA 的 ablation 也显示去掉 inverse dynamics 会产生退化表示，因此它比“只看 latent MSE 是否下降”更可靠。

## 当前项目已经具备的可复用模块

### 1. 数据转换与动作标签

应复用：

- `scripts/validation/ros2_workspace_tools/convert_rosbag2_to_semantic2d_native_lidar.py`
- `scripts/validation/ros2_workspace_tools/prepare_semanticcnn_formal_split.py`

前者已经处理 `/scan_01`、`/scan_02`、TF/odom、`/cmd_vel`/`/cmd_vel_stamped` 与 `/clock` 映射，并输出对齐后的 `cmd_velocities/`。后者已经维护 episode、sequence index、split role 和 4000-slot 数据合同。

这批 Isaac bag 需要一个很薄的动作适配：它的 `/cmd_vel` 无 header 且时间不规则，应优先读取 `/semantic_cnn/actuation_decision` 的 `final_command`，或用 `/isaac/actuation_state` 作为实际执行动作；不能把最近的 headerless `/cmd_vel` 无条件当成真实控制标签。bag 也没有显式 `done/episode boundary`，转换时必须从运行记录或 reset/时间跳变恢复四段 episode 边界。

不应使用 `v7_rosbag_to_irregular_720_dataset.py` 作为这批数据的入口。该脚本硬编码每个雷达 360 束，而当前 S3-Net 合同为每个雷达 2000 束。

### 2. 双雷达时间窗和 BEV

应复用：

- `methods/experiments/dual_lidar_pedestrian_bev/dataset.py`
- `methods/experiments/dual_lidar_pedestrian_bev/model.py`

`TemporalDualLidarDataset` 已实现：

- 按 episode 构建严格因果窗口；
- 双雷达有效点与 base/map 坐标转换；
- ego-motion compensation；
- 前后左右各 `8 m`（总计 `16 m × 16 m`）、`0.1 m` 分辨率的可配置 BEV；
- occupancy 或 current-plus-deltas 两种时序编码。

第一版只需让 dataset 额外返回每一帧对齐的 `(linear_x, angular_z)`，并增加 future horizon，而不必重新写 LiDAR 栅格化。

### 3. 可选的现有 encoder 初始化

项目已有两个可比较的 LiDAR encoder：

- S3-Net 的 1D residual/VAE encoder：保留原始角序结构，适合做 polar/beam latent；
- `TemporalBEVPedestrianDetector` 的 U-Net stem/down blocks：保留局部空间结构，适合做 BEV latent。

首轮建议使用 BEV encoder，因为社交导航最终需要人的空间关系、群组和自由空间。1D S3-Net encoder作为消融，检验 BEV 是否真的有收益。

## 推荐的第一版复用架构

```text
现有 rosbag
  -> 现有 4000-slot converter + episode split
  -> 现有 TemporalDualLidarDataset (T=8, BEV occupancy/current+deltas)
  -> 本地 BEV encoder stem/down blocks
  -> global z_t (64 or 128 dim)
  -> EB-JEPA RNNPredictor(z_t, cmd_t)
  -> z_hat_(t+1 ... t+H)

loss:
  L_pred       = future latent prediction
  L_var/L_cov  = prevent collapse
  L_time       = temporal geometry smoothness
  L_IDM        = recover cmd_vel from (z_t, z_t+1)

diagnostics:
  latent std / covariance effective rank
  one-step and multi-step latent error
  true-action vs shuffled-action prediction gap
  inverse-dynamics MAE for linear.x/angular.z
  frozen linear probes: min range, pedestrian count, nearest-person distance
```

这不是把不同含义硬切成多个 `z`。第一轮先训练一个全局 `z`，验证世界模型信号成立。随后才加入：

```text
z_static       静态几何与可通行空间
z_people[K]    每个行人或局部人群 slot
z_interaction  人群对机器人候选动作的响应残差
```

第二阶段可从 SlotFormer/HOWM 借鉴 slots 和集合匹配，但应通过 decoder/probe/干预实验约束语义，否则“把向量切成三段”不会自动产生可解释分工。

## 第一轮最小实验和基线

同一时间切分上跑四个模型：

| 模型 | 目的 |
|---|---|
| persistence：`z_hat_(t+1)=z_t` | 判断动态预测是否超过静态复制 |
| NavRep-style：1D encoder + GRU | 2D LiDAR 经典 latent dynamics 基线 |
| EB-JEPA-BEV：BEV encoder + action-conditioned GRU | 主模型 |
| EB-JEPA-BEV shuffled action | 检查模型是否真的使用动作，而不是只靠相邻帧惯性 |

首轮通过条件建议为：

1. validation multi-step latent error 明显优于 persistence；
2. 使用真实动作优于 shuffled action；
3. latent 每维标准差、协方差有效秩不坍塌；
4. frozen probe 能恢复几何/行人风险量；
5. train/validation 按完整 episode 或连续时间段切分，不随机打散相邻帧；
6. 至少 3 个随机种子报告均值与离散程度。

当前四个 episode 足够做 pipeline smoke test 和表征诊断，但不足以支持 social generalization 结论。如果动作变化很小，true-action 与 shuffled-action 可能没有显著差异；此时需要补采带分支动作或人工扰动的轨迹，而不是继续加大模型。

## 实施边界

下一步最小实现只需要新增一个独立实验目录，不修改在线 S3-Net/DRL-VO 节点：

```text
methods/experiments/dual_lidar_jepa/
  dataset.py        # 薄封装现有 TemporalDualLidarDataset + action/future horizon
  encoder.py        # 复用现有 BEV encoder block
  world_model.py    # 移植 EB-JEPA predictor/loss；可切换 VICReg/SIGReg
  train.py
  evaluate.py
  README.md
```

外部代码移植时应保留上游版权和许可证说明。JEPLO 是 GPLv3；如果直接复制其代码并分发衍生项目，需要按 GPLv3 处理。EB-JEPA 为 Apache-2.0，LeWorldModel、NavRep、SCOPE、Point-JEPA 为 MIT，作为模块来源更容易与当前实验代码共存。

## 双雷达转虚拟 360 的实测结论（2026-09-22）

项目中确实已有虚拟扫描逻辑，但存在两代实现，不能混用：

- `workspaces/ros2_ws/src/semantic_nav_gazebo/tools/dual_laser_scan_merger.py` 是旧实现，写死两台雷达位姿，默认输出 720 束；
- `sim_to_real/robot_deployment_bundle/ros2_ws/src/semantic_nav_runtime/scripts/v7_dual_laser_scan_merger.py` 是当前部署实现，默认输出 360 束，通过 TF 获取外参，并用 50 ms 近似同步；
- `v7_rosbag_to_irregular_720_dataset.py` 保留两台 360 束雷达的 720 个独立槽位，不是“机器人中心的虚拟 360 雷达”，也不接受当前每台 2000 束的 bag。

已新增离线入口 `scripts/validation/ros2_workspace_tools/convert_rosbag2_dual_lidar_to_virtual360.py`，它复用部署实现的几何合同：

1. 从 `/tf_static` 求每台雷达到 `base_link` 的变换；
2. 将每个有效量测转成雷达端点，再变换到 `base_link`；
3. 删除底盘矩形 `x=[-0.36,0.36] m, y=[-0.32,0.32] m` 内的自身回波；
4. 端点按 `atan2(y,x)` 投入 `[-pi,+pi]` 的 360 个槽；
5. 多个端点落入同槽时保留离 `base_link` 原点最近者；空槽保留为 `+inf`。

对 `semantic_cnn_online_s3net_isaac_r045_20260903_122130` bag 的转换结果位于 `runs/lidar_world_model_virtual360_20260922_095358/`：

- 两路各 2459 帧全部配对，时间差全部为 0 ms；
- 输出 `ranges_m` 为 `(2459,360)`，平均有效槽 `358.464/360`，最少 328、最多 360；
- 平均每帧 335.561 个槽同时收到两台雷达候选，最终近距离回波获胜；
- `virtual360.npz` 保留有效掩码、获胜传感器、原始束号和两雷达来源掩码；
- 派生 `rosbag/` 含 2459 条 `/scan_merged`，可直接回放；
- 首、中、末三帧与现有部署版 merger 逐槽精确一致，NPZ 与派生 rosbag 的距离数组全帧逐元素一致。

这一处理仍是“端点重投影 + 最近槽”的虚拟扫描近似，不是从 `base_link` 原点对点云重新做连续射线投射。它会丢掉同角度的远层回波，并把两台雷达的差异压成一个距离。因此第一轮可以把它作为简单 1D 世界模型输入，但应同时保留 `valid_mask` 和 `source_sensor_mask`；原始双路 2000 束数据继续作为无损输入/消融对照。当前 360 合同还沿用两端都包含的 `[-pi,+pi]` 和 `2pi/359` 增量，首尾方向近似重复；为保持与在线 `/scan_merged` 一致，本次没有擅自改成半开区间。

### 地图射线视频

已输出两支使用同一地图、episode 和时间轴的对照视频：

- `visualization/dual_lidar_raw4000_map_rays.mp4`：原始双雷达，每帧输入槽为 `2000+2000`；蓝色为仅 `/scan_01` 占据的射线像素，橙色为仅 `/scan_02` 占据的像素，紫色为两路射线像素重叠。该合成与绘制顺序无关，避免后绘制的橙色覆盖蓝色。无效或非有限量测不伪造射线，右侧面板显示每帧真实绘制数。右下局部窗放大机器人本体，标出两台传感器安装点，并使用 Isaac 生成器相同的逐束计算恢复碰撞框外的实际查询起点；局部窗为避免遮蔽每四束显示一束，全局地图仍绘制全部有效束。
- `visualization/virtual360_map_rays.mp4`：融合后的虚拟 360；射线从 `base_link` 发出，蓝/橙表示该角槽最终由哪台雷达的最近回波获胜。右下局部窗将机器人碰撞框覆盖在合成射线上，明确表示它不是一个没有尺寸的点。
- `visualization/scan_01_only_map_rays.mp4` 与 `visualization/scan_02_only_map_rays.mp4`：分别隔离显示第一台和第二台雷达，并应用现有 V7/Gazebo 固定自遮挡标定。scan01 屏蔽 `752/2000` 束、保留约 `224.6°` 角度支持；scan02 屏蔽 `804/2000` 束、保留约 `215.3°`。这两支视频表示从历史标定重建的物理可见范围，不再错误展示单雷达穿过机器人后的完整 360°。

两支视频均为 1280×720、10 FPS、373 帧、37.3 秒，按 4 倍速连接四个有效 episode，跳过 episode 之间的重置等待。颜色已提高饱和度和不透明度；端点、机器人轨迹、行人真值和目标同时显示。逐帧解码检查通过，四段 episode 各保存一张最终帧 PNG。这是记录数据的离线可视化，不是实时 RViz 或重新运行仿真。

必须区分“传感器安装点”和“物理查询射线起点”。本次运行中两台雷达安装在机器人坐标系 `(0.20,0.13)` m 与 `(-0.20,-0.13)` m，但当前 Isaac PhysX 生成器调用 `ray_start_offsets_outside_box()`，沿每束方向把查询起点移到记录的 `0.62474 x 0.48705` m 机器人碰撞框之外，并至少满足 `range_min=0.5` m。该函数的目的就是阻止车体自回波。因此这份 bag 不含能够画成机器人矩形的自身命中点；视频中的黄色矩形来自运行记录的碰撞框尺寸，原始射线段则从逐束恢复的真实查询起点开始。若此前 RViz 中出现车体矩形回波，它对应另一套传感器后端或另一份未抑制自身命中的数据，不能从当前 bag 反推出来。

为检查颜色覆盖与历史自回波，又从项目中 86 份含双雷达话题的 bag 选取四种来源各一份，并以相同 `±8 m` 范围绘制“仅 scan01、仅 scan02、蓝后橙、橙后蓝、顺序无关”五列对照。结果保存在 `runs/lidar_draw_order_audit_20260922/dual_lidar_draw_order_contact_sheet.png` 和 `draw_order_audit.json`。当前 bag 的两路射线栅格像素交并比为 `80.69%`；旧实现固定先画蓝再画橙时，这 `22487` 个重叠像素都会显示为橙色，所以原视频确实存在橙色覆盖蓝色的问题。视频现已改为顺序无关的三色合成：仅 scan01 为蓝、仅 scan02 为橙、两路重叠为紫色。

历史数据也解释了 RViz 中的机器人矩形：2026-07 V7 bag 与 2026-08 Gazebo bag 的中间帧分别有 `320+327` 个端点落在机器人框内，较早的 Isaac DR-SPAAM bag 有 `341+341` 个；当前 2026-09 bag 为 `0+0`。所以矩形并非视觉错觉，而是旧传感器后端确实保留的自身回波；当前 PhysX 数据路径后来抑制了这些命中。

### 后续 rosbag 采集修正

用户给出的启动命令中 `ISAAC_DEMO_RECORD_BAG=0` 不会启动录包，而且 `ISAAC_LIDAR_MODE=physx` 的旧默认行为会让每台雷达逐束从车体外开始查询，形成不真实的单传感器 360° 穿透视野。现已增加 `ISAAC_LIDAR_SELF_OCCLUSION_MODE=fixed_mask`：它复用项目训练数据的固定 beam 标定，通过 UDP 传送紧凑的闭区间列表，并由 ROS 桥在发布 `/scan_01`、`/scan_02` 时将遮挡槽写成 `NaN`。这里使用 `NaN` 而不是 `+Inf`，因为本项目下游把 `+Inf` 解释为一直到 `range_max` 的自由空间；遮挡不能伪装成自由空间。

可直接使用新增启动器：

```bash
cd /home/user/navigation_project/a_pipeline
RUN_DIR="$PWD/runs/drl_oracle_manual_light_$(date +%Y%m%d_%H%M%S)"
ROS_DOMAIN_ID=78 ISAAC_DEMO_OUTPUT_DIR="$RUN_DIR" \
  bash isaac_sim/scripts/run_drl_oracle_manual_light_record_bag.sh
```

该启动器设置 `ISAAC_DEMO_RECORD_BAG=1`、`ISAAC_DEMO_RECORD_TRACE=true`、`ISAAC_LIDAR_MODE=physx` 和 `ISAAC_LIDAR_SELF_OCCLUSION_MODE=fixed_mask`，其余行人、控制、GUI 和分辨率参数保持用户给出的配置。该模式保留 2000 个固定槽以及无效槽身份，适合后续世界模型的数据合同；它重建的是标定遮挡掩码，不是重新生成旧传感器的车体表面距离。当前只完成静态与单元测试，尚未为这项新模式运行 Isaac 6 全流程采集。

录制结束后可检查前 30 对扫描是否严格保存 `752/804` 个 `NaN` 遮挡槽：

```bash
source /opt/ros/humble/setup.bash
python3 scripts/validation/ros2_workspace_tools/validate_dual_lidar_self_occlusion_bag.py \
  "$RUN_DIR/rosbag" --samples-per-sensor 30 \
  --output-json "$RUN_DIR/lidar_self_occlusion_validation.json"
```

验证器已经对当前旧 bag 做负对照：两台雷达的标定遮挡槽均为 `0` 个 `NaN`，按预期返回 `FAIL`，证明它能识别本次发现的穿透数据，而不是无条件通过。

颜色在前后区域混合不是 TF 绘制错误。该 Isaac bag 中两路消息均为 2000 束、接近完整 360° 的 panoramic scan；`base_scan_02` 相对 `base_link` 的确旋转 180°，但旋转整圈扫描只会改变束号对应方向，不会把它限制为后半圈。Isaac 生成器显式使用 `angle_min=-pi, angle_span=2pi`，S3-Net 在线合同也要求每台 `[-pi,+pi]`，当前 `v7_dual_laser_scan_merger.py` 没有半圈裁剪。实测每帧平均 335.561/360 个虚拟角槽同时有两台候选；前半圈 scan01/scan02 获胜约 52.87%/47.13%，后半圈约 47.13%/52.87%。若改成每台只使用朝外的局部 `[-pi/2,+pi/2]`，才会出现预期的两种颜色各占大半边，但那是新的 outward-hemisphere 表示，会改变当前数据和部署 merger 的语义，不能称为原始 4000 束可视化。

## 主要来源

- [EB-JEPA repository and AC Video JEPA](https://github.com/facebookresearch/eb_jepa/tree/main/examples/ac_video_jepa)
- [LeWorldModel official repository](https://github.com/lucas-maes/le-wm)
- [JEPLO paper](https://arxiv.org/abs/2609.15770) and [official repository](https://github.com/ASIG-X/JEPLO)
- [NavRep official repository](https://github.com/ethz-asl/navrep)
- [SCOPE official repository](https://github.com/TempleRAIL/scope)
- [SOGMP official repository](https://github.com/TempleRAIL/SOGMP)
- [Point-JEPA official repository](https://github.com/Ayumu-J-S/Point-JEPA)
- [JEPA-WMs official repository](https://github.com/facebookresearch/jepa-wms)
- [NavThinker official repository](https://github.com/hutslib/NavThinker)
- [SlotFormer official repository](https://github.com/pairlab/SlotFormer)
- [HOWM official repository](https://github.com/linfeng-z/HOWM)
