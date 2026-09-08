# Social Navigation 研究决策与执行记录 · 2026-09-08

研究负责人判断：继续这个方向，但把近期目标从“增加 Isaac 功能”收窄为**建立可信的行人执行与社会导航评价闭环**。现在没有证据支持立即放弃 BehaviorAgent，也没有证据支持把平台宣布为已验收。先完成有因果解释力的执行诊断，再决定是否更换接口。

本轮授权范围：用户要求先评估再实施、保留当前模式、备份并记录、禁止子 agent。本轮先实施独立诊断工具和有界运行；正式入口是否改变必须由实验结果决定。1–3 个月计划属于后续研究，不把本轮工具交付称为论文或闭环验收。

## 1. 真实评价

已解决的是工程基础：共享 kernel 的数值回归；真实的双 LiDAR→DR-SPAAM→tracker→DRL-VO 输入链；route visibility、重接和 emergency yield 的具体故障；20 人工作负载下约 60 Hz physics、15 Hz LiDAR 和 RTF≈1。性能结果支持继续研究，但**不能推出行人控制时序已满足闭环需求**。

本轮读取完整优化运行的 15 个 rolling profiler 窗口：`pedestrian_control_tick_total` 与 Social Force 调用均为加权平均 **3.8503 Hz**，单窗口 **1.367–6.193 Hz**，与 app update 一致。主程序在 app loop 中调用社会控制，所以 physics 60 Hz 不等于社会控制 60 Hz。历史 head-on 的 solver dt 中位 0.06643 s、最大 0.34121 s；本轮最小探针则为 1/60 s。这个时序差异是必须验证的新假设，尚不能认定它造成了旧 non-finite 故障。恢复稳定控制周期是控制研究问题，区别于单纯追求 GUI FPS。

必须降级的“✅”：kernel parity 不等于人类行为有效性；adapter 保留横向速度的平均幅值不等于逐时刻向量跟踪成功；感知链能够运行不等于遮挡、速度误差与延迟已达到实机要求；DRL-VO 到达四个目标不等于社会导航安全。现有最终验收仍为 `NOT_CONVERGED_STOPPED`，physical-contact coverage 为 0。15 人正式运行最近机器人–人体 clearance proxy 达 5.27 m，未形成有效人机近距交互；平均 pair ratio 还可能被大量不交互人对稀释。

最大风险是**实验有效性和误归因**：执行故障、读取错误、感知误差可能被计入 planner 的性能；反过来，“机器人没撞上人”可能只是机器人没有真正遇到人。正反两种错误都足以使论文结论失效。

停止投入：GPU PhysX/以 GUI 帧率为唯一目标的优化；再改 Social Force 权重掩盖冻结；扩大人数和地图；同时接入 Arena/HuNav 全栈；在环境未可信前训练新 DRL policy；把照片般真实的动画当成人类行为真实性证据。

证据：[社会导航最终验收](../../runs/scenario_topology_ab/evaluation/FINAL_SOCIAL_NAVIGATION_ACCEPTANCE_20260907.md)、[性能报告](../../runs/isaac_performance/ISAAC_PERFORMANCE_OPTIMIZATION_20260907.md)。

## 2. A/B/C/D 判断及证据链

| 假设 | 证据与反证 | 当前决策 |
|---|---|---|
| A：调用、状态读取或生命周期问题 | 旧 `behavior_agent_interface_validation.py:root_matrix` 读取 `Usd.TimeCode.Default()` 的静态 Xform；主程序 `character_positions()` 已明确使用 BehaviorAgent runtime/Fabric。旧实验在 pause 后 reset；reset 返回 true 未验证位置兑现。旧 task interruption 是“非 running 帧数”，不是抢占事件数。 | 最先检验。旧接口实验不是可靠的引擎缺陷证据。正式 head-on 用了 live getter，不能顺带推翻它。 |
| B：Motion Matching 限制或缺陷 | 正式 head-on 的 non-finite/non-unit 警告是真实失败信号；内部回退旧 pose 可以让外部 quaternion 始终有效。其触发可能受资产、初始状态、目标曲率/刷新与更新节奏影响。 | 保留假设，不凭一条日志断言引擎根因；同时记录 warning、任务状态和实际轨迹。 |
| C：二维速度转换成 character task 的语义差异 | `follow(target)+set_speed` 接收目标和速度标量；heading、NavMesh、加速度与步态仍由引擎决定，没有公开的 desired planar velocity setter。移动目标会触发持续重规划。 | 这是确定存在的执行契约差异；是否足以导致当前冻结要用对照测量。 |
| D：绕过 BehaviorAgent | 如果支持的原生任务通过而有界、可行目标仍无法稳定跟踪，或正确初始化的原生任务跨资产复现故障，则投入替代执行后端有价值。 | 当前是条件后备方案，不是立即重写。 |

官方文档要求 agent 方法在 timeline Play 中使用，并提供 runtime translation/rotation、task status 方法：[Behavior API](https://docs.omniverse.nvidia.com/kit/docs/behavior-simulation/110.1.1/behavior-simulation/api.html)。本机版本以 `omni.anim.behavior.core-110.1.4` 的 `IBehaviorAgent.h` 和本地测试为准；在线文档可能与绑定细节不完全一致。

本地 `tests/follow/test_follow.py` 明确将 follow 定义为不自行终止的任务，也注明移动目标不断重规划。因此需把任务失败、被替换、正常到达及“暂时速度低”分开。停止等待/计数错误不得叫做 Motion Matching freeze。

一个尚未消除的正式链假设：主程序 reset 后推进 3 帧，之后才由 `BehaviorAgentSocialMotion` 取消 IRA patrol。原 patrol 是否在这段窗口覆盖 reset 或替换任务，必须做先接管/后接管的配对实验，不能仅凭调用顺序宣布根因。

## 3. 短期路线：1–2 周

| 时间 | 工作 | 可交付验收 |
|---|---|---|
| 第 1–2 天 | 冻结现有模式；runtime/USD 双通道；单人原生 move_to、静态 follow、移动 follow；reset 前后任务归属对照。 | 每次 clean process；唯一初始 reset；有效 live pose；记录任务状态、帧时间和 warning；失败位置可定位。 |
| 第 3–5 天 | 2 人 reset；分别控制目标刷新、solver dt、animation dt，再做 head-on、crossing、near-wall；同一资产/路径/seed，一次只改一个因素。 | 先验证起点、可达性和持续 motion，再评价社会交互；用 60/15/4/~1.3 Hz 探查响应范围，初始失败不进入后续指标。 |
| 第 6–8 天 | 静止/脚本匀速机器人遭遇；传感器闭环；安全事件真值校准。 | 强制近距遭遇覆盖；人工可控正接触/无接触校准；来源明确的事件及无缺失时间戳。 |
| 第 9–14 天 | 开 DRL-VO 策略闭环；五类场景至少 10 个配对 seed；再跑 20 人回归。 | 感知源隔离、扫描交付、任务完成、交互覆盖和接触真值全部各自报告；没有覆盖即 NOT_COVERED。 |

执行决策门：若 native 和 follow 均通过，保留 BehaviorAgent；优先查正式初始化、ownership 和 update cadence。若 native 通过而 moving follow 失败，先测固定时间更新/有界短轨迹目标并做阶跃、转弯、stop/go 响应；不要在失败期间不断重启 follow。如果原生受支持任务经正确初始化仍在至少两个资产、三个 clean-process 重复中失败，最多花两个工作日形成最小复现和版本证据，然后转替代执行后端。三次无改进修改即停止该分支，不把调参当研究进展。

时序隔离：`--target-period-sec` 只改变 probe 移动目标的 sample-and-hold，不改变 engine animation dt，不启用 Social Force，不能声称复现了完整低 FPS workload。之后再单独回放原始 solver dt/命令序列和控制 app 时序。若需要固定步长社会控制，必须在候选后端中明确状态采样与 command 生效时间，不得用重复处理同一旧状态伪装高频闭环。

替代接口的具体边界：`reset(seed, pose)`、`command(vx, vy, stamp)`、`step(dt)`、`observe(pose, pose_velocity, realized_velocity, applied_command, intervention)`、`events()`。候选用受 NavMesh/扫掠碰撞约束的 character controller 或 capsule 执行二维平移，动画只消费实际移动状态；每一处裁剪/碰撞干预都记录。它是可控的运动学代理，不自动成为具有真实人体接触力学的模型。禁止每帧 teleport、禁止同时让动画 root motion 和 capsule 拥有全局平移权，避免双重执行。

## 4. 最小实验与失败判据

以下阈值是本项目的**预注册工程门限**，并非已得到人类实验支持的“社会规范”。不得在看完结果后为通过而放宽。

通用初始门：Z-up/metres；NavMesh 可用；actor ID/资产/seed 固定；reset 后最后连续 3 帧误差≤0.15 m；quaternion 范数误差≤1e-3；任务调用唯一且未被意外替换；有效样本间隔≤0.25 s。任何 non-finite root warning、无故位置跳变、起点失真，判执行失败。冻结定义为有行走意图、尚未到达、pose-derived speed<0.05 m/s 连续>2 s；合法让行也计入 inclusive freeze，但同时报告其原因，不能简单等同引擎故障。

| 场景 | 目的与控制 | 判失败或未覆盖 |
|---|---|---|
| single | 无其他人/机器人；5 m 直线，随后独立的 90°转弯、stop/go；先 native task 再同目标 follow。 | 15 s 内未进入 0.45 m 终点半径、无意图解释的>2 s freeze、root 无效或任务被替换。只走直线不能算全部 single 路线完成。 |
| head-on | 两人 8 m 相对起步；先 reset 验证，再开共享 kernel。精确对称与固定±0.1 m 偏移分开；不添加特例。 | 初始条件不成立→INVALID_INITIALIZATION；有效交互后未错身/双方未完成路线或长期互锁→FAIL；接触与 clearance 单列。对称决策停住与有命令却不走应分开定位。 |
| crossing | 90°相交；用固定到达时间差 0、±0.5 s 控制让行压力；互换角色。 | 不进入预先定义交互窗→NOT_COVERED；反复互让/任务抢占/越界/接触或无进展→FAIL。 |
| near-wall | 保留现有安全预检路线，让一侧横向避让空间受限；镜像几何；记录 requested/guard/applied target。 | 穿越占据、root 越界、guard 长期返回当前位置并阻止路线完成；保护性裁剪本身不直接判 motion failure。 |
| human–robot | 先静止机器人，再脚本匀速，再 DRL-VO。保持人、机器人碰撞形状和坐标一致；固定迎面/横穿/追越。 | 必须至少覆盖一次预定义交互窗（如 center 距离<3 m 且逼近，或预测 TTC<4 s 持续≥1 s）；未覆盖不能算成功。接触真值缺失→strict safety NOT_COVERED；真接触、超时、无进展分别计数。 |

两人无 Social Force 的 native/follow 运行只是执行定位实验，即使两人完成目标也不是社会错身 PASS。当前独立 probe 只实现每人第一段到目标的执行观测，不替代五类完整交互评估器。

人–人和人–机器人指标需分母明确：全程人秒、交互窗人秒、事件数、持续时间、最大连续时长、p95/p99 clearance/响应尾部；同时给 inclusive freeze 与 execution-only freeze。pose-derived 速度、BehaviorAgent navigation velocity、adapter command 分开保存。jerk 用固定采样/滤波协议，不能直接差分不规则噪声速度。

安全真值按层报告：人体几何 overlap/距离；碰撞代理接触；经校准的 PhysX 接触事件。只有具备 collision/contact 支持的实际形状对才可声称 PhysX 接触覆盖。触碰代理通过不等于人体动力学真实。若角色没有可报告 contact 的形状，需要独立候选、正负接触校准和基线敏感性分析，不能只添加一个计数器便宣称解决。

## 5. 中期路线与论文价值：1–3 个月

推荐可证伪的主问题：**在相同社会决策和感知合同下，行人执行误差与传感延迟是否会系统性改变导航策略的评价，甚至改变策略排序？经执行验证后，这种偏差能否下降？**

第 1 个月：完成上述门限、误差分解、失败复现集和真实交互覆盖；测 command→actual 的幅值误差、方向误差、响应延迟和恢复时间。用回放固定命令识别执行响应，再用真实闭环验证；两者不混为一个实验。无执行缺口时不强行发明新 locomotion 方法。

第 2 个月：在固定场景/资产下配对比较当前 follow、一个有明确执行约束的候选、以及必要的理想化运动学诊断参照；Oracle 与 DR-SPAAM+tracker 的观测源因子分开。至少比较 DRL-VO 和一个传统导航基线，之后才扩大到第三种 planner。保存原始数据/失败结果；报告效应大小、episode 级配对 bootstrap 95% CI 和方法排序的一致性。不能把逐帧几千点当独立统计重复。

最小统计阶段为 5 类场景×10 seeds×2 planners×2 execution modes=200 episodes；感知因素先在最敏感的两类场景验证再扩展。资产和 seed 要交叉控制：相同 seed 若会换人物模型，就不能把效果解释成纯位置扰动。先锁资产再做资产泛化，保留一组未调参场景。

第 3 个月：接真实机器人记录/有同意的受控人机遭遇，比较间距、让行延迟、通行时间和可观察的速度统计；对不匹配行为明确建模边界。只有实机或真实轨迹验证完成后才主张 sim-to-real；传感器像实物或机器人尺寸一致不足以支持迁移结论。

贡献优先级：

1. 有证据的 evaluation validity / execution gap 研究，加可复现基准，是最有价值的路线。
2. 如果新接口降低执行失真、跨资产/场景有效且能改善评价稳定性，它可以成为方法贡献；“换掉一个 API”本身不够。
3. 平台与指标工具是支撑产物，需要清晰的差异化、基线、失败案例和可外部复现性。
4. Social Force transfer 主要是工程复现；不应当作为主创新。

已有工作界限：[SocNavBench](https://arxiv.org/abs/2103.00047) 已提供真实轨迹支撑的场景与统一评价；[HuNavSim](https://arxiv.org/abs/2305.01303) 已提供 ROS 2 人类行为与评价；[HuNavSim 2.0](https://arxiv.org/abs/2507.17317) 已明确涉及 Gazebo 和 Isaac。故“Isaac+社会力+ROS2”不足以区分本项目。这里给出研究定位判断，不声称已经完成全面 novelty survey 或证明方法排序真的发生改变。

从顶级 Robotics/CS PhD 申请角度：这个项目能展示很强的系统能力，但申请材料应围绕一个你独立提出、严谨验证并愿意推翻的研究问题展开。比“20 人、RTX、DRL、照片级渲染”的功能清单更有说服力的是：发现过去 benchmark 结论为什么不可靠，提出检查方法，控制变量，公开反例，并给出真实机器人验证。任何项目或论文方向都不能保证录取。

## 6. 本轮可回退实施

新增独立工具：`behavior_agent_execution_audit.py`、`execution_audit_metrics.py`、`run_execution_audit.py` 和针对指标错误的 9 项单测。原 `legacy/gazebo_social`、共享 kernel、生产主程序、LiDAR、tracker、DRL-VO 和旧接口实验均保留。

runner 使用生产 launcher 的同一 flock，不覆盖已有输出，记录命令、Git HEAD、源码/输入 SHA256、exit code、simulation/wall 时间、warning 和最终 decision。超时时仅清理本次创建的进程组。工具不启动 ROS，不触碰历史 recorder。

备份与初始工作区记录：`runs/research_execution_audit/20260908_093131/`，包含 88 个原始文件、`baseline_sha256.json`、Git diff（含 cached diff）和 status。回退时停用新入口即可；不需要覆盖当前生产文件，也不应使用 reset/checkout/clean。旧文件的 SHA256 检查结果及本轮运行结果见同目录 `RESULTS.md`。

复现单个新实验（输出目录必须不存在）：

```bash
cd /home/user/navigation_project/a_pipeline
python3 isaac_sim/experiments/run_execution_audit.py \
  --output runs/research_execution_audit/reproduce_native_01 \
  --method move_to --ownership before_reset --duration 15
python3 -m pytest -q isaac_sim/tests/test_execution_audit_metrics.py
```

`--reset-mode paused` 仅用于生命周期负对照，不是推荐的生产用法。probe 的 `PASS` 只代表所选第一段 locomotion 工程门限通过；整体决定以 runner 的 `manifest.json:decision` 为准，因为引擎可能回退为有效 pose，同时在日志中报告 Motion Matching 错误。
