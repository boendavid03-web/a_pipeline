# Arena TF throttle 隔离验证与正式 Nav2 回归 — 2026-10-01

## 结论

**TF_FIX_STATUS: VERIFIED（局部障碍输入）**。唯一功能变化是 `isaac_utils/graphs/tf.py` 默认 `throttle: 300 → 10`。隔离 Isaac child 现场记录所加载模块、SHA-256 和 `/World/jackal/tf_publisher/on_tick.inputs:framePeriod=10`；原 `default.json + 3 HuNav` 在预先登记的 20–28 仿真秒窗口中，35/35 帧近距扫描通过 local TF filter，74/74 帧 local OccupancyGrid 和 155/155 帧 VoxelGrid 均有占据。正式 Nav2 1.1.20 回归中 short 最终 SUCCEEDED，原 default 的主 action ABORTED。**这不是整个 Arena benchmark READY 或社交安全验收。**

## 修改、隔离加载和原始输入

原 installed TF 基准 SHA-256 为 `a1ba698b321c882226ad7b261c65e88ade709418908dea41b7a7e1212482ea5c`，原 installed Isaac child 为 `b173a069e3603dc33797c8483ae9208b30e9e82b3229e67f2cf898afbdab1809`。单行源码修复见 [已保存补丁](ARENA_LOCAL_COSTMAP_TF_FIX_APPLIED_20261001.patch)。[prepare_runtime.py](scripts/validation/arena_tf_throttle/prepare_runtime.py) 从固定哈希的 factory install 构造完整的 `isaac_utils` 与 `ros2isaacsim` 隔离包；隔离 TF 文件除单行行为变化外只加只读 graph 日志，SHA-256 为 `c8f014b011970a07319ef871b53ebba2ff9b2d8407a3d860b664e569644f97f3`。隔离 child 的 hash 仍为 `b173a069…`。[python.sh](scripts/validation/arena_tf_throttle/python.sh) 和 [run_gui.bash](scripts/validation/arena_tf_throttle/run_gui.bash) 只在明确选择该入口时将隔离包放在 child 的导入路径前面。

`UrdfToUsd.py` 创建 TF graph 时只传 graph path、prim path、prefix，没有显式传 `throttle`。服务用 `URDFParseAndImportFile` 在运行 stage 创建机器人并调用 `tf.tf`；隔离日志中 `created=True`、graph path 为 `/World/jackal/tf_publisher`，现场读取 `OnTick.framePeriod=10`，故没有沿用旧 USD graph。`framePeriod` 是 tick 周期，不等于 Hz；实际发布间隔来自下面的 `/tf` 测量。

前后对照沿用 `map_empty`、原 `default.json`（SHA-256 `4970601dec3f7e56aaa2df1375d09da2f670d486244022c37af37ed5d816ff43`）、HuNav 配置（`1b12b19ba0a9b75bf5b2aba2292e04c96e57b919b784d3e9e80ad8104ec38272`）、live Nav2 YAML（`2c7eea093eca3254d4e5c9c4e08ed044389cfd43bd097791345ae005034d2473`）、NavFn、DWB、`controller_frequency=1.0`、`movement_time_allowance=10.0` 和原 0.25 m / 0.25 rad goal tolerance。场景、LiDAR 几何、voxel、TF filter、controller 均未调参。

## 30 仿真秒同条件诊断

修改前是历史 domain 202 [原始运行](</home/user/arena_local_costmap_diag_20261001/run_20261001/raw_rosbag/metadata.yaml>)。本轮用修正后的 [离线分析脚本](scripts/validation/arena_tf_throttle/analyze_trace.py) 对其做独立重算，输出放在 [baseline_reanalysis](</home/user/arena_tf_throttle_validation_20261001/baseline_reanalysis/trace_analysis.json>)，没有覆盖旧结果。该脚本按本机 `nav2_voxel_grid` 的 `11=marked, 01=unknown, 00=free` 编码计算 unknown，并对三个单 voxel 样例断言通过。Nav2 voxel 算法本身未变。

修改后有效的隔离运行是 domain 214 [原始 bag](</home/user/arena_tf_throttle_validation_20261001/diag_predeclared_214/raw_rosbag/metadata.yaml>)、[launch log](</home/user/arena_tf_throttle_validation_20261001/diag_predeclared_214/launch.log>)、[逐扫描分析](</home/user/arena_tf_throttle_validation_20261001/diag_predeclared_214/trace_analysis.json>)。运行前 [preflight](</home/user/arena_tf_throttle_validation_20261001/diag_predeclared_214/preflight.txt>) 记录稳态窗口 **20 ≤ scan stamp < 28 s**。启动期为 `<20 s`，停止期为 `≥28 s`。运行达到 30.000002 仿真秒，墙钟 62.405 秒；原诊断为 61.738 秒。离线修正后 baseline 仍是 0 marked / 360,000 unknown / 0 free；修复后的最大 marked 为 59、最大 free 为 1,936。

| 指标 | 原 throttle 300 | 隔离 throttle 10 |
|---|---:|---:|
| 发布扫描 / local callback | 57 / 57 | 57 / 57 |
| TF_ACCEPT / DROP / 停止时未决 | 0 / 55 / 2 | 54 / 2 / 1 |
| 预登记 20–28 s 有效近距扫描 TF_ACCEPT / DROP | 0 / 35 | 35 / 0 |
| 末端 TF stamp 中位间隔 | 3.350 s | 0.117 s |
| 末端 TF 收到时 timestamp age 中位 / 最大 | 3.350 / 4.033 s | 0.117 / 0.283 s |
| 最大 marked voxel / local lethal cell | 0 / 0 | 59 / 59 |
| 非零 local grids | 0 / 166 | 109 / 169 |
| 预登记稳态 local grids / voxel grids 有占据 | 0 | 74/74 / 155/155 |

修复后两次 drop 都在启动期（scan stamp 18.967、19.167 s）；稳态窗口没有持续的旧 TF 迟到。停止时 1 帧未决，不记为 drop。`OutTheBack` 文案仍有枚举映射歧义，不据此判定传感器时间戳落后。修复前没有 CPU/GPU 采样，因此不能做受控负载差值结论；修复后 5 秒采样中 GPU 利用率中位/最大约 20%/33%，显存最大 5,705 MiB，Isaac 进程 CPU 中位约 436%，记录见 [resources.log](</home/user/arena_tf_throttle_validation_20261001/diag_predeclared_214/resources.log>)。前后到 30 仿真秒的墙钟时间接近，但不是性能等价证明。

### 一个南墙 scan 的逐层轨迹

另一次隔离 domain 212 的 [bag](</home/user/arena_tf_throttle_validation_20261001/diag_run_20261001_212/raw_rosbag/metadata.yaml>) 和 [插桩日志](</home/user/arena_tf_throttle_validation_20261001/diag_run_20261001_212/launch.log>) 给出 scan stamp `16.433334190 s`（`16433334190` ns）及同一 stamp 的回调、TF_ACCEPT、投影、buffer、voxel cycle。beam 107 的 range 为 1.229165 m，离线投影到 map `(25.5135, 0.0348, 0.2056)` m，离原南墙 0.0348 m。local filter 在 scan callback 后约 0.099 wall s TF_ACCEPT；laser projector 得到 274 点，buffer 经 TF/高度过滤仍有 274 点并存入。下一个 `VOXEL_CYCLE` 的 `scan_ns` 与该扫描完全相同，109 个点通过 range/map、109 次 markVoxel/markCell 调用，marked voxel 从 0 变 27。该墙面 hit 对应 rolling grid cell `(77,63)`，在 scan 时是 0，下一帧 `16.533334195 s` 为 100。插桩只按 scan 计数 mark 调用，没有记录每一束单独的 voxel 写入；beam 到 cell 的归属为同 stamp、几何投影与下一帧 cell 的联合证据。

## 原正式 Nav2 1.1.20 回归

两组从干净 host setup 启动，**不加载诊断 overlay**。[短场景 run](</home/user/arena_tf_throttle_validation_20261001/formal_short_215/preflight.txt>) 和 [default run](</home/user/arena_tf_throttle_validation_20261001/formal_default_216/preflight.txt>) 的 `nav2_costmap_2d` package prefix 均为 `/opt/ros/humble`；controller `/proc/<pid>/maps` 实际加载 `/opt/ros/humble/lib/liblayers.so`（SHA-256 `64d581b8555bd9486fbd5c399791be00f14334b9d8abd5a94cff02958d194e8b`）、`libdwb_core.so`、`libdwb_critics.so`，没有诊断版 `liblayers.so`（SHA-256 `86d14e…`）。三个 deb 包 `nav2_costmap_2d`、`nav2_controller`、`dwb_core` 均为 1.1.20。两个 child 均记录隔离 TF `c8f014…` 与 graph period 10。正式环境无内部 TF_ACCEPT 插桩；以下扫描数是**发布/原始 bag 数**，不冒充 filter 接受数。

| Gate | short no-human（domain 215） | 原 default + 3 HuNav（domain 216） |
|---|---|---|
| INFRASTRUCTURE | **PASS**：306 scans、475/538 local grids 有 lethal cell、最大 60；VoxelGrid 最大 60 marked；TF 末端中位间隔/age 0.117/0.117 s | **PASS**：564 scans、1116/1483 local grids 有 lethal cell、最大 103；VoxelGrid 最大 103 marked；TF 末端中位间隔/age 0.117/0 s；bag 中 3,119 帧 HuNav states，名字为 `D_test_1`、`2`、`3` |
| NAVIGATION | **SUCCEEDED_AFTER_STARTUP_RETRY**：第一个 UUID ABORTED，第二个 UUID `aa5bdfad5bc59829bd05f5742b87f42c` status 4；goal 到结果位移 2.206 m，终点 XY 误差 0.182 m、yaw 误差 0.0023 rad | **ABORTED**：第一个 UUID 启动期 ABORTED；主 UUID `f6952af8bc0700b38bf3608edeb20322` status 6，`Failed to make progress`；goal 到结果位移 28.213 m，终点 XY 误差 0.183 m、yaw 误差 0.566 rad（超过原 0.25 rad 容差） |
| 终态控制 | 终态后 8 条 `cmd_vel` 均为零，最后 `(0,0)` | 终态后 10 条 `cmd_vel` 均为零，最后 `(0,0)` |

short [bag](</home/user/arena_tf_throttle_validation_20261001/formal_short_215/raw_rosbag/metadata.yaml>)、[action/odom/scan 分析](</home/user/arena_tf_throttle_validation_20261001/formal_short_215/formal_analysis.json>)、[实际库 maps](</home/user/arena_tf_throttle_validation_20261001/formal_short_215/controller_libraries_steady.txt>)；default [bag](</home/user/arena_tf_throttle_validation_20261001/formal_default_216/raw_rosbag/metadata.yaml>)、[分析](</home/user/arena_tf_throttle_validation_20261001/formal_default_216/formal_analysis.json>)、[实际库 maps](</home/user/arena_tf_throttle_validation_20261001/formal_default_216/controller_libraries.txt>)。两个 gate 的 effective controller/local/Task Generator parameter dumps 和资源日志各在对应 run 目录。正式 bag 的几何分析表明 default 中有 37,049 个近距南墙候选点；这是离线投影，不等于内部 TF_ACCEPT 直接计数。没有改 near-goal progress checker、DWB critic 或 yaw tolerance。default 的 action 失败是有效的导航结果，不是本轮 TF 基础设施失败。

## 可重建启动和回滚

必要脚本位于 [scripts/validation/arena_tf_throttle](scripts/validation/arena_tf_throttle)。`prepare_runtime.py` 可从原始 SHA-256 `a1ba…` 或其已知单行 patched SHA-256 `d483…` 的 factory 包重建隔离副本，遇到其他 hash 会停止。启动入口只作用于本次进程，不修改旧 wrapper 的默认 `ISAAC_PATH`、shell rc 或正式 Nav2。下面的 domain 与 run ID 仅为命令示例；使用前应重新选空闲 domain 与新 run ID。

```bash
cd /home/user/navigation_project/a_pipeline
python3 scripts/validation/arena_tf_throttle/prepare_runtime.py
timeout 240 scripts/validation/arena_tf_throttle/run_gate.bash diagnostic 220 30 diag_new_220
timeout 420 scripts/validation/arena_tf_throttle/run_gate.bash short 221 80 short_new_221
timeout 650 scripts/validation/arena_tf_throttle/run_gate.bash default 222 170 default_new_222
```

只退出隔离入口时，改用旧 `/home/user/arena_isaac5_host_runtime/run_gui.bash`。但本轮运行期间共享 source/install 被另一并行工作改成 `throttle=10`，所以旧入口目前**不会**自动恢复到 300。要恢复原共享五份 TF 文件，先确认所有 Isaac/Arena 进程已退出，再显式执行 [rollback_shared.py](scripts/validation/arena_tf_throttle/rollback_shared.py)：

```bash
cd /home/user/navigation_project/a_pipeline
python3 scripts/validation/arena_tf_throttle/rollback_shared.py --execute
```

它逐项核对 [原始备份及修复后 provenance](</home/user/arena_local_costmap_tf_fix_20261001/provenance_after.json>) 的 before/after hash，任何文件有新变化就整体拒绝写入；本轮**没有执行回滚**。原 bag、备份和旧 wrapper 都保留。并行工作还将共享修复及 manifest 记录在当前 HEAD `966c22c`；本报告与隔离脚本是额外的正式 Nav2 1.1.20 验证，未对其 commit/push。

## 剩余问题

原 default 的主 goal 仍因 near-goal `Failed to make progress` 退出，yaw 误差 0.566 rad；short 和 default 都有独立的启动期第一个 UUID ABORTED。诊断 overlay 基于 Nav2 1.1.19，正式回归特意使用 1.1.20，因此正式 run 没有直接内部 filter 计数。原因层面，为什么 `throttle=300` 在 default 负载下恰好落后一个完整周期，仍无线程级 trace；本轮只验证了直接 TF 迟到机制和修复效果。原 Arena benchmark、社会安全和其他场景均未宣称 READY。
