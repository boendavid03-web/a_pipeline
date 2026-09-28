# Human Motion Execution：研究决策与实验报告 · 2026-09-08

**决定：继续 Option A，但当前 `follow + set_speed` 映射不能作为已验收的通用二维速度执行器。近期优先解决停止／重启契约及输入刷新对执行的影响；不替换 locomotion 后端，不修改正式 pipeline。**

本轮先在对话中给出 Research Assessment，再冻结 Git 与输入，随后实现隔离基准。未使用子 agent。

## Report 1 · Research Decision

### 真实完成度与研究判断

项目已具备开展研究的工程基础：共享 Social Force kernel、真实传感与导航链、基础 BehaviorAgent locomotion、运行产物与离线评价工具。尚未具备“接近真实机器人社会导航平台已验收”的证据。基础走通、kernel parity、动画可见和机器人到达目标，各自回答不同问题，不能互相替代。

当前最大瓶颈是**决策意图、接口指令和实际运动之间缺少经过验证的执行契约**，以及由此造成的评价误归因。本轮把这个问题从假设推进到了可重复测量：零速度停止段仍发生大幅真实行走，且完全不需要 Motion Matching warning 或任务失败才会出现。

性能结论需精确：已有 physics≈60 Hz、LiDAR≈15 Hz 支持继续研究；此前研究审计报告中的 pedestrian tick≈3.8503 Hz 仍是时序风险。本轮没有重跑完整 workload，不能把最小场景 60 Hz 结果用于证明正式控制时序。这里不继续做 GUI 或 GPU 优化。

停止投入：Social Force 权重与数学调整；扩大人数、spawn、waypoint 的反复优化；新的 DRL 训练；同时迁入多个 benchmark 全栈；以动画外观替代行为真实性验证。继续投入：执行契约、可行输入集合、时序系统辨识、有效局部交互覆盖、接触真值及感知误差对评价结论的影响。

既有证据：[第一阶段研究执行审计](../../runs/research_execution_audit/20260908_093131/RESULTS.md)、[正式社会导航验收边界](../../runs/scenario_topology_ab/evaluation/FINAL_SOCIAL_NAVIGATION_ACCEPTANCE_20260907.md)。这些历史结果本轮仅查阅，未冒充重新执行。

### 根因分析：已证实与未证实

| 判断 | 本轮证据 | 结论强度 |
|---|---|---|
| 当前零速度映射不能可靠表示“停住” | 三次 Stop-Go 的 2 s 零速度段均行走 1.458 m；任务持续 RUNNING；无 invalid root warning | 已证实当前映射在该条件下存在执行误差 |
| “BehaviorAgent 基础 locomotion 坏了” | 横向行走和两种 replay 通过；所有有效运行的 root、任务与采样完整性通过 | 不支持这种笼统归因 |
| 命令刷新越快越好 | 4 Hz 与 60 Hz 方向切换对照，4 Hz 满足本轮门限，60 Hz 未满足 | 单资产、单输入下的反例；不是普遍最优频率 |
| 零向量映射存在目标语义退化 | 现有 pure mapping 在 v=0 时返回当前位置；停止期间实际继续更新目标，角色继续前进并转身 | 已观测现象；是停止失真的候选机制 |
| 引擎忽略 `set_speed(0)`，或某个内部 MM bug 是最终根因 | 本轮记录了调用参数及实际位移，没有独立记录 `get_speed()` 内部状态，也未做停止接口消融 | 未证实，不能由外部轨迹直接判定 |
| 旧 head-on non-finite 已解释 | 本轮没有双人共享 kernel 正式闭环，没有旧初始化状态和 animation dt | 未解释；原失败证据仍有效 |

本机 `IBehaviorAgent.h` 将 speed 描述为 locomotion speed；该注释不足以推导瞬时二维速度／制动保证。本机 follow 测试明确存在持续重规划及不自行终止的任务语义。API 判断以本地 `omni.anim.behavior.core-110.1.4` 为准；在线 API 页面本轮获取失败，未据此声称最新引擎行为。

## Report 2 · Experiment Results

### 冻结实验条件

- 最终协议：[validated_geometry_v2/protocol.json](../../runs/human_motion_execution/20260908_101151/validated_geometry_v2/protocol.json)。五个实验类别；replay 分 raw/adapter；额外一个 4 Hz 对照，共七种条件，各三次独立进程。
- 同一 seed 7、同一 `male_adult_medical_01`、同一 lobby scene、同一初始位置 `[2,4,0]`、同一初始朝向 +X。起点误差约 1.197 mm。
- 每个进程仅一次初始化 reset、一次 persistent follow；无自动恢复，无 task restart。关闭原生人／障碍避让；无 ROS、机器人、live Social Force。
- 直接调用已有 `steering_target_from_velocity`：1 m lookahead、0.02 m target 写入阈值。未重写转换数学。正式上游 adapter、route cursor、free-space guard、emergency 逻辑不在该基准中；因此不能称为完整正式 adapter 等价验证。
- synthetic 命令在 t=0 起步；straight 4 s，lateral 3 s，direction switch 为 2+3 s，Stop-Go 为 2+2+2 s。积分参考从 reset 后实际位置开始，另行报告 reset 误差。
- 命令在 `app.update()` 前写入，速度由随后 live runtime 位置差计算。实测每段 simulation dt=1/60 s。4 Hz 对照改变 target／speed 写入周期，保持相同 app／timeline 推进，不重跑 solver；未单独读取 Motion Matching 内部 animation dt。
- replay 来自旧 head-on 角色 A 的首 6 s，86 个真实原始时序命令；源文件、角色、起始时间和哈希均冻结。保留零阶保持时序，应用到 60 Hz 网格，最大首次交付延迟 16.661 ms。没有重新计算 Social Force。

**门限是预注册工程筛查条件，不是人体真实性标准。** 启动／转向／停止响应：速度向量误差≤0.20 m/s（停止速度≤0.10 m/s）连续 0.20 s，首次达标起点≤1 s。稳态指标排除每个固定指令阶段的前 1 s；其向量 RMSE≤0.20 m/s。停止段总行走距离≤0.30 m；参考路径最大偏差≤0.50 m；有行走意图时连续 freeze≤2 s。未在观察窗达标记为未观测到，不能等同永远不能恢复。

以下数值三次重复一致。固定 seed 的重复说明运行可重复性，不代表不同人的统计分布。raw 逐帧 RMSE 含步态变化；稳态 RMSE 使用 0.2 s 因果速度平均。表中不能将全程误差和稳态误差混用。

| Experiment | Input | Result | Error | Conclusion |
|---|---|---|---|---|
| Single straight | `(1,0)`，4 s | 3/3 FAIL_TRACKING | 全程逐帧 RMSE 0.262 m/s；稳态 0.170 m/s；响应 1.333 s；最大路径偏差 0.165 m | 可以直行，启动响应超过 1 s 门限；没有冻结 |
| Lateral movement | `(0,1)`，3 s；初始朝向 +X | 3/3 PASS | 全程逐帧 RMSE 0.311 m/s；稳态 0.155 m/s；响应 0.583 s；最大偏差 0.354 m | 世界坐标 Y 向行走通过；角色转向，不是朝向锁定的侧移验证 |
| Direction switch | `(1,0)→(0,1)`，60 Hz | 3/3 FAIL_TRACKING | 全程逐帧 RMSE 0.383 m/s；转向后稳态 0.222 m/s；转向响应 0.683 s；最大偏差 0.392 m | 转向发生且任务稳定；启动响应与转向后稳态门限未通过 |
| Stop-Go | `(1,0)→(0,0)→(1,0)` | 3/3 FAIL_TRACKING | 零速度 2 s 内行走 **1.458 m**；停止首次响应 1.233 s；恢复阶段 2 s 内未达到向量响应门限；最大偏差 1.410 m | 当前停止／重启契约不可靠；恢复阶段有移动，不是“永久冻结” |
| Social Force replay — raw | 历史 `(vx,vy,t)`，6 s | 3/3 PASS | 全程逐帧 RMSE 0.235 m/s；因果平均后 0.170 m/s；最大偏差 0.194 m | 此历史命令片段可被执行；不能证明完整交互通过 |
| Social Force replay — adapter | 同片段历史 adapter 输出 | 3/3 PASS | 全程逐帧 RMSE 0.173 m/s；因果平均后 0.179 m/s；最大偏差 0.089 m | 该输入有较小轨迹偏差；不能将两个 replay 的误差相减作为 adapter 因果收益 |
| Direction switch — 4 Hz | 同方向切换，仅控制写入周期变为 0.25 s | 3/3 PASS | 全程逐帧 RMSE 0.363 m/s；转向后稳态 0.190 m/s；转向响应 0.767 s；最大偏差 0.377 m | 通过门限，但转向响应比 60 Hz 慢；不同指标存在取舍 |

所有有效运行均无 task replacement、FAILED task、无效 root、采样间断、NavMesh 采样覆盖失败和 Motion Matching non-finite warning。最大连续 freeze 远小于 2 s。零速度停止段的合法意图不计为 execution freeze，其真实移动单独计入停止误差。

21 次运行的 `get_linear_velocity()` 记录均为零，而 live root 明确移动，进一步支持将 pose-derived velocity 作为本轮真值观测。横向实验结束时身体朝向约 86.98°，因此其 PASS 不能解释为保持 +X 身体朝向侧移。末态朝向是观测量，没有额外锁定 heading。

旧 head-on 的初始条件本身无效。因此其命令可用作真实系统产生的压力信号，不能当作经过验证的正常社会行为标签。本轮 raw-to-adapter 输入向量 RMSE 为 0.320188 m/s；这是该源片段中上游 adapter 的实际改写幅度，不能仅凭幅度说改写是正确或错误。

完整机器可读结果：[analysis.json](../../runs/human_motion_execution/20260908_101151/validated_geometry_v2/analysis.json)。图中预先选用 repeat 1，不按结果优劣挑选重复：

![速度响应与积分轨迹](../../runs/human_motion_execution/20260908_101151/validated_geometry_v2/velocity_and_trajectory.png)

### 实验失败与一次有依据的基准修正

初版直线探针在 0.183 s 被自编几何检查中止；该检查要求规划路径长度最多比直线长 0.05 m。一次只读复核显示：失败处起终点均在 NavMesh 上，位置投影误差为微米以下，而 1 m 请求返回约 1.104899 m 折线路径。这说明“路径不够直”不能直接用作“输入路径被阻挡”的判据。

据此仅修正实验检查为直接线段按≤0.05 m 间距取样，观测到 NavMesh 投影误差≤0.05 m、存在可达路径；规划路径增量另存为诊断量。它仍是离散 NavMesh 覆盖检查，不是连续人体碰撞／接触真值。未改变运动输入、跟踪门限、场景或正式代码。

初版协议、源码、失败 trace 和查询复核保留在父目录；最终 21 次使用新的 v2 协议。复核脚本首次遗漏本地 asset 环境变量后被中止，随后显式设置本地资产路径完成查询；两份日志均保留。没有把技术失败删除，也没有把修正后的结果覆盖到初版目录。

实际 tracking 失败没有触发任何调参修复；完成预定三次重复后停止本轮该分支。

## Report 3 · Architecture Decision

选择 **Option A：Social Force + Adapter + BehaviorAgent**，作为继续研究的受限架构。

理由：基础 locomotion、完整二维方向转换和两个非恒定输入 replay 均可工作；失败首先集中在具体的停止／重启语义与刷新行为。目前没有跨资产、受支持停止操作的对照证据，足以证明必须重写整个 locomotion 后端。直接选 B 会同时更改执行模型、碰撞模型和动画耦合，破坏当前误差定位的可解释性。

同时拒绝一个错误前提：**“调用 follow 并设置 speed，就等同于实现了二维速度执行器”已经被本轮结果否定。** A 的继续使用必须暴露 requested velocity、实际 API 写入、实际速度、heading、目标状态、停止／恢复事件和干预，不允许向评价器隐瞒执行误差。

下一门只做独立的停止／重启契约实验：冻结当前失败基线，分别比较零速度时保持一次性目标与连续将目标更新到当前 root，以及本机支持的停止／恢复调用。每次只改变一种执行机制；补记 `get_speed()`、引擎 target、任务状态。它是通用执行语义验证，不添加角色避让规则，不改 Social Force。

转 B 的条件：经受支持的停止操作、正确初始化和固定时间控制后，至少两个资产、三个独立进程仍不能在明确的速度／曲率／制动可行域内满足停止与恢复契约，或不可避免地反复任务切换／冻结。届时 B 才作为独立可比较的后端设计，并单独承担 NavMesh、人体几何、接触和动画一致性的验证责任。

## Report 4 · Implementation

新增代码／文档：

- [behavior_agent_velocity_benchmark.py](behavior_agent_velocity_benchmark.py)：独立 Isaac runtime、单次 reset／follow、live state 与指令区间记录、几何观测。
- [human_motion_execution_metrics.py](human_motion_execution_metrics.py)：精确零阶保持积分、因果速度误差、阶段响应、停止／冻结与完整性判据。
- [run_human_motion_execution.py](run_human_motion_execution.py)：冻结协议、真实 replay 来源、同正式 launcher 的互斥锁、独立进程 watchdog、manifest／SHA256。
- [analyze_human_motion_execution.py](analyze_human_motion_execution.py)：逐组离线复核、汇总 JSON／Markdown、PNG／PDF 科学图。
- [test_human_motion_execution_metrics.py](../tests/test_human_motion_execution_metrics.py)：延迟、跨帧命令边界、停止／恢复、NaN、FAILED task、缺样、几何误判回归。
- 本报告。

修改已有生产文件：**0**。已有实验代码／报告也未覆盖。所有实际运行源码在各协议的 `source/` 下保存；源文件哈希在协议中，实际 actor／motion library 的 USD composition layer 哈希在每个 runtime metrics 中。

实验产物根目录：`runs/human_motion_execution/20260908_101151/`；有效基准：其 `validated_geometry_v2/`。初始 HEAD：`2872028e653b355e769be21b2c07fbf1af248e44`。初始 Git 工作区干净，开始时保存了 status、log -3、HEAD、完整 tracked／cached diff；保护 408 个已有源码／配置／报告的哈希，备份原 experiments／tests。最终保护检查与 Git 状态另存 `verification.json`。未 stage、commit、push，未使用禁止的恢复／清理命令。

运行命令（输出必须是新目录；prepare 失败时不会覆盖既有协议）：

```bash
cd /home/user/navigation_project/a_pipeline
python3 isaac_sim/experiments/run_human_motion_execution.py \
  --output runs/human_motion_execution/reproduce_unique_name --prepare
python3 isaac_sim/experiments/run_human_motion_execution.py \
  --output runs/human_motion_execution/reproduce_unique_name \
  --case stop_go --repeat 1
```

完整矩阵为 protocol 中七种 `--case`，各 `--repeat 1/2/3`，顺序执行；ERROR／INVALID 配置停止重试。runner 自动使用本地 Isaac 6.0.1、资产根目录及锁，无需启动 ROS。

```bash
python3 -m pytest -q \
  isaac_sim/tests/test_human_motion_execution_metrics.py \
  isaac_sim/tests/test_execution_audit_metrics.py \
  isaac_sim/tests/test_pedestrian_steering.py
python3 isaac_sim/experiments/analyze_human_motion_execution.py \
  --output runs/human_motion_execution/20260908_101151/validated_geometry_v2
```

聚焦测试 37 passed；全部 21 组 runtime trace 的离线重算通过。Kit 与系统 Python 的浮点累加末位存在差异，标量最大差约 6.22e-15；复核允许 1e-10 数值误差，判定、阶段覆盖和失败原因严格一致。这是指标复核，不是重新运行仿真。以 manifest 中研究 status 与 metrics 为准：此 Isaac 关闭路径的 child exit code 即使 tracking 失败也为 0；外层 runner 会据 status 返回非零，不用进程退出码冒充实验通过。

回退：不调用新增实验入口即可；生产代码无须恢复。

## Report 5 · Research Recommendation

最值得研究的问题是：**在社会决策不变时，执行器的可行运动集合、停止误差和时序误差，是否会改变局部交互结果，甚至改变机器人导航算法的相对排名？哪些误差必须显式进入评价与仿真模型？**

不能把“人类决策和 locomotion 有层次差异”当作新发现。Reynolds 已明确分离 steering 与 locomotion，并讨论不同运动能力需要适配；本项目需要证明具体执行失真及其对机器人研究结论的影响。[Steering Behaviors for Autonomous Characters](https://www.red3d.com/cwr/steer/gdc99/)

SocNavBench 已建立基于真实行人数据的可重复场景与评价工具；社会导航评价指南也明确覆盖安全、舒适和多种行为维度。因此“建一个 simulator + 输出几项距离指标”不足以独立构成强论文贡献。上述文献只作为研究定位，本轮不是完整系统性综述，也不宣称该具体问题尚无人研究。[SocNavBench](https://arxiv.org/abs/2103.00047)、[Principles and Guidelines for Evaluating Social Robot Navigation Algorithms](https://arxiv.org/abs/2306.16740)

建议下一阶段依次执行：

1. **先校准执行器**：停止／恢复为首门；接着用具有明确加速度与曲率上限的输入，测量可实现速度域和动态响应。瞬时 90°方向阶跃是系统辨识压力输入，不自动代表真实人会这样走。不能用机器人全向底盘的模型要求人体瞬时侧移。
2. **再隔离时序**：command 交付、target 写入、solver dt、animation dt 分别控制，回放完整 workload 的 jitter；保持同一工作负载做正式回归。当前 4 Hz 单例通过不构成把正式控制器改成 4 Hz 的依据。
3. **再连接局部交互**：先恢复正确初始化的双人共享 kernel head-on，随后 crossing、near-wall、静止与移动机器人近距交互。分别检查决策停住、guard 干预和执行停住；交互未覆盖记 NOT_COVERED。
4. **最后检验研究效应**：冻结社会模型与机器人策略，用配对场景／seed 比较不同已校准执行契约，评价接触、最小距离、让行次序、通行时间及策略排序；同时报告感知输入和延迟。执行更贴合数学命令，不一定意味着更像真人，需真实轨迹或人类评价独立支撑。

推荐的论文贡献是可复现的执行误差数据集、可证伪的执行模型、以及对社会导航评价偏差的因果证据；不是更多 scene、更多 engine 集成或更多补丁。当前交付完成了有界执行诊断，尚未完成停止机制修复、五类正式社会交互矩阵、physical contact truth 或 sim-to-real 验证。
