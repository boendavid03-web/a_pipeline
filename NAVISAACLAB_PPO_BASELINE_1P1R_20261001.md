# NavIsaacLab 1+1 PPO pilot（2026-10-01）

## 研究边界与 provenance

本实验检验当前 ZIP 来源的 NavIsaacLab 项目所带 Robot PPO 能否从随机初始化在 1 个 MaskedMimic 人体和 1 台 Nova Carter 的 warehouse 中产生导航学习信号。它沿用当前本地 `env.yaml` 的网络、奖励、动作缩放、地图、depth、邻居状态、课程和 1000 步终止设置。仓库没有 `.git` 元数据；精确的 upstream commit 及相对上一轮验收之前的全部源码变更不可重建。训练前的源码、配置和资产哈希列在冻结 manifest；本轮另新增 `scripts/analyze_ppo_pilot.py` 和 `scripts/run_fixed_eval_suite.bash` 用于事后分析及串行评测，未改变 PPO 算法。原始 `env.yaml`、smoke 和 acceptance 配置的 SHA-256 与上一轮验收记录一致。

环境：Python 3.11.13、Torch 2.7.0+cu128、PyTorch CUDA runtime 12.8、Isaac Sim 5.1.0-rc.19、Isaac Lab 2.3.2。MotionLib 与匹配 `smpl_57f98a9` MaskedMimic checkpoint 已按本机原始冻结 manifest 固定；公开仓库提供路径脱敏的 `NAVISAACLAB_PPO_BASELINE_PUBLIC_MANIFEST_20261002.yaml`，资产和运行日志留在本地。没有重新下载、转换、升级或覆盖 acceptance 证据。

## Baseline fidelity 与配置差异

| 配置差异（相对 `env.yaml`） | smoke | acceptance | 正式 1+1 pilot | 类别与影响 |
|---|---|---|---|---|
| `human_mesh: true → false` | 是 | 是 | 是 | D：PPO depth 中人体由 SMPL 表面变成 articulated skeleton。不能称完整 upstream 视觉复现。 |
| MaskedMimic checkpoint `smpl → smpl_57f98a9` | 是 | 是 | 是 | A：与当前 vendored ProtoMotions 匹配的本机 checkpoint；旧版组合不能运行。 |
| human / robot 各 10 → 各 1 | 是 | 是 | 是 | B：规模缩减，改变交互密度。 |
| 固定人车路线、起点到终点最短距离 7 → 2 m | 否 | 是 | 否 | C/D：acceptance 诊断；正式训练恢复原随机路线机制。 |
| 初始朝向偏差 30–120° → 0°、goal curriculum 关闭 | 否 | 是 | 否 | D：acceptance 诊断；正式训练保留原课程。 |
| episode 1000 → 120 步 | 否 | 是 | 否 | C/D：acceptance 为检验 reset 的短 timeout；正式训练保留 1000。 |
| rollout 256 → 64、PPO epochs 4 → 1、minibatch 256 → 64 | 否 | 是 | 否 | C：acceptance 续训烟测；正式训练保留原 PPO 更新参数。 |
| total steps 5M → 49,920；输出路径独立 | 否 | 是（192） | 是 | C：pilot 的可执行上限与证据隔离，不改变单步训练语义。 |

正式配置为 `CrowdSim/config/env_ppo_baseline_1p1r.yaml`，它 include 原 `env.yaml`，只覆盖上表所列的 1+1/兼容/视觉/预算项。`resolved_config.json` 固定了实际加载后的完整值。`human_mesh=true` 的 SMPL overlay 依赖 `smplx`；现有隔离环境缺这个模块，项目依赖文件没有固定兼容版本。本轮未升级 Torch/Isaac/CUDA 或猜版本安装。人体视觉表示判为 **SKELETON**；架构对当前本地 PPO 为 **FULL**，环境语义为 **PARTIAL**。由于没有可靠原始 Git metadata，“完全 upstream faithful”不可核实。

## PPO 输入、输出与执行链

PPO 使用 5 维 ego/goal 向量、最多 4 个 5 维邻居状态、224×224 depth、24×24 local map；RGB 已启动但不进入此 PPO 网络。显式邻居状态与 local map 含仿真 privileged information，所以这是 **privileged-input PPO baseline**。确定性评测用 actor mean 经 `bounded_robot_action()`，训练仍使用 PPO 原采样动作。线动作范围 `[0,1]`，角动作 `[-1,1]`，当前配置最高 1 m/s 与 1 rad/s。车辆通过 `DriveController` 的 kinematic root pose integration 执行；不是完整轮动力学。评测区分 policy action、requested velocity 与实际位置位移。

## 固定评测协议

固定文件 `CrowdSim/config/fixed_eval_routes_1p1r_20261001.json` 在训练前创建。四个 episode 各一次：Layer A 的短 PointGoal / non-interacting control，以及 Layer B 的 crossing、head-on、same-direction。seed 分别 3101–3104。所有 checkpoint 使用同一 start/goal、seed、human start/goal 和确定性 actor mean。训练仍随机采样，固定场景只用于评测。warehouse 地图分辨率 0.05 m，逐段以半栅格间距验证保守占用阈值下的最小净距；四类路径的最低净距 1.0 m，高于 0.35 m 机器人半径及 0.5 m 规划 clearance；起始人车间距最小 2.0 m。社交路线按沿目标方向的几何方位落在 90° 水平 FOV 内，实际帧内投影仍需单独检验。

每回合记录 seed、起终点、终止原因、reached/collision/timeout/stuck、目标误差、时间、路径长度、人车距离、neighbor 激活比例、动作、requested 速度、executed 位移/速度、depth 和 map 有效比例。四回合仅是小样本诊断；一次成败使成功率变化 25 个百分点。人类的净位移可能很小，因为固定路线到端点后会折返，不能据此认定人体静止。

## Eval@0 与训练 / checkpoint 结果

step 0 模型来自 `RobotPPOTrainer` 的随机初始化并保存为 schema-3 checkpoint；它没有加载之前的 128 或 192 步烟测模型。Eval@0 真正把 actor mean 送给车辆，未覆盖为零动作。

| Step | Success | Collision | Timeout | Stuck | Mean goal error (m) | Min human dist (m) |
|---:|---:|---:|---:|---:|---:|---:|
| 0 | 2/4 | 2/4 | 0/4 | 0/4 | 4.816 | 1.522 |
| 10,240 | 0/4 | 4/4 | 0/4 | 0/4 | 8.129 | 1.445 |
| 25,088 | 0/4 | 4/4 | 0/4 | 0/4 | 6.790 | 1.579 |
| 49,920 | 2/4 | 1/4 | 1/4 | 0/4 | 4.690 | 1.579 |

Eval@0：短 PointGoal 于 180 步到达，head-on 于 508 步到达；crossing 与 same-direction 分别在 906、888 步碰撞，目标误差较大。初始 actor mean 对应约 0.5 线动作与 0 角动作，因此短直线成功不能解释为已学习的社交策略。独立复跑的短路 / crossing 终止类别和步数一致，但最近人车距离有波动；固定种子与路线不保证物理人体轨迹逐浮点一致。

10,240 步：四条固定路线均以 collision 结束，包括无人交互短路；平均目标误差升至 8.129 m。25,088 步：四条路线仍全部 collision，且 73–95 步即终止；短路虽然到达距目标 2.436 m，仍不能算成功。49,920 步：短路和 same-direction 到达；crossing 在 1000 步 timeout，head-on 在 164 步 collision。两个到达终点的误差约 0.747 m，符合环境当前 reached 判定，但不能推断训练出了安全社交策略。该 checkpoint 的 crossing 距目标 8.954 m，head-on 距目标 8.314 m。

训练上限 49,920 robot steps，256 步 rollout，对应 195 个 rollout；里程碑在 10,240、25,088、49,920。训练日志保留每个 rollout 的 return、终止标记、目标距离、progress、policy/value loss、entropy、KL、explained variance；checkpoint 含 model、optimizer、config、goal curriculum state。总耗时 1:16:35，约 10.87 robot steps/s。checkpoint 中课程累计完成 226 个 episode：132 reached、87 collision、6 timeout、1 stuck。这些是训练分布上的终止计数，不能代替固定评测。课程最终仍为 `short_straight`，显示的 success rate 为 58.5%；10 m 的 Layer B 路线超出当前短路课程，不宜据此单独判定社交导航能力。

原始训练入口在 run 目录仅复制了薄的 `_include: env.yaml` 配置。为使保存的 run 配置可独立解析，事后在该生成目录保留薄配置为 `config/env_ppo_baseline_1p1r.yaml`，把冻结的原始 `env.yaml` 复制为 `config/env.yaml`，同时保存 `config/resolved_config.json`；训练时使用的源码、配置及 checkpoint 均未改变。独立输出路径、训练命令、全部 SHA-256 和按名义 0/10k/25k/50k 汇总的评测 JSON 见 `../Assets/repro_logs/ppo_baseline_1p1r/`。

## 训练曲线与稳定性

完整曲线在 `training_curves.png`，逐 rollout 表在 `training_rollouts.csv`。前 25 与后 25 个 rollout 的平均完成 episode return 分别为 -27.08 与 -25.89，单步平均 goal distance 为 2.415 与 2.576 m，平均 progress 为 0.0080 与 0.0035 m；这些数值没有形成一致改善。后 25 个 rollout 的平均 policy loss -0.0012、value loss 4.022、entropy 1.789、KL 0.0007、explained variance 0.0012。全程 195 条 rollout 指标和四个 checkpoint 的 model/optimizer 张量均无 NaN/Inf。最后的低 explained variance 说明当前 value function 对 return 的解释力弱，不能单凭训练内 reached 次数声称策略学会了导航。

训练期间累计完成 226 个 episode 并继续完成 reset，正式训练和固定评测进程均 exit 0；10,240 与 25,088 checkpoint 经真实 `RobotPPOTrainer.load()` 读取，49,920 checkpoint 被正式评测进程加载，三个模型均完成前向和闭环动作。step 0 的 optimizer 尚无更新状态，其“finite”检查为空集；后续 checkpoint 的 optimizer 状态有内容且全部有限。四个 checkpoint 各四个固定 episode 的 depth/map 有效比例都为 1.0，动作饱和比例均为 0。最后 checkpoint 的短路 mean requested linear speed 0.451 m/s、mean executed speed 0.417 m/s；crossing 分别为 0.285 与 0.247 m/s。差异存在，但没有出现控制链完全不执行或机器人永久静止的证据。训练原始 trajectory 排除 >=0.5 m 的重置跳变后，人体累计运动 1336.98 m，平均约 0.667 m/s，94.0% 的非跳变帧位移超过 5 mm，最长近静止连续 15 步。

`human_mesh=false` 短 probe 完成 2 步并 clean exit；RGB 取值 0–242，depth 有效像素比例 0.341，PPO 输入形状为 `(1,5)、(1,4,5)、(1,4)、(1,224,224)、(1,24,24)`。`human_mesh=true` 在建场时因缺少 `smplx` 明确失败，发生在可观测 RGB/depth/shape 前。该失败 probe 的 Isaac 退出挂起，随后仅结束了其自身进程组；上述正式训练和全部 skeleton 评测均正常退出。项目依赖文件未固定兼容的 `smplx` 版本，故本轮没有安装，也没有把骨架 depth 冒称 SMPL depth。

## 失败案例与学习结论

10,240 步甚至未通过无交互短路；25,088 步四条路线很快碰撞。最终 checkpoint 恢复短路成功，并在 same-direction 到达，但 crossing 超时，head-on 碰撞。最终固定成功率仍是 2/4，与随机初始化相同；平均目标误差只从 4.816 m 变为 4.690 m，且其中一条通过、一条失败的类型发生了变化。全部固定路线的最近人车距离至少 1.445 m，但 collision 的对象未被可靠分类为人或静态环境，因此不把该数值解释为社会安全保证。固定评测只有四个 episode，没有重复随机种子或置信区间；可检测明显回归，却不足以量化小幅泛化提升。

结论：**NO CLEAR LEARNING SIGNAL**。训练链、reset、sensor、checkpoint 和正常关机已验证，训练内也确实有成功 episode；但固定评测在 50k 步未优于 step 0，且中间 checkpoint 明显退化。当前**不值得直接把这份原版 PPO 扩大到更长训练**。本轮到此停止；是否先补齐 SMPL 视觉依赖、扩大固定评测样本或调查短路碰撞，由下一阶段另行决定。
