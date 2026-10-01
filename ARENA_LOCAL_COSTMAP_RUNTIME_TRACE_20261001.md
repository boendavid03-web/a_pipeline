# Arena local costmap：单次 default 诊断运行轨迹 — 2026-10-01

## 唯一结论

**A1 — TF_MESSAGE_FILTER_FAILURE。** 在这次原样 `map_empty/default.json + 3 HuNav`、30 仿真秒诊断中，LaserScan 发布并到达 local costmap 的订阅回调，但没有一帧通过 TF message filter。55/57 帧在约 0.3 仿真秒的 transform 等待后被丢弃；余下两帧在停止时尚未决议。所有 46 帧带 0.08–2.5 m 返回的扫描都在这 55 帧内。第一失败阶段是 **`jackal/lidar_link → jackal/odom` 的实时 TF 供给与 message filter 等待窗口之间**，不是 ObservationBuffer、voxel 高度/量程过滤或 clearing。

这次重新观察到 166/166 local OccupancyGrid 零占据，和原始 default bag 的 1235/1235 零占据相符。原始 default bag 没有 filter 插桩，所以“它每一帧也因同一机制被丢弃”仍是由同一 runtime、同一结果及其 TF 到达滞后支持的**推断**；精确计数仅属于这次诊断运行。short 对照组原始运行有 585 grids、468 非零、最大 542 cells。near-goal `Failed to make progress` 独立于此问题。

## 运行边界、输入与复现材料

只运行了一次诊断，`ROS_DOMAIN_ID=202`、`ROS_LOCALHOST_ONLY=1`。仿真时钟守卫在 **30.000002 s** 停止，墙钟 61.738 s；没有等待 Nav2 终态。运行前没有该域 ROS 节点，GPU 为 RTX 5090、约 30.9 GiB 空闲。输入为原始 `map_empty`、`default.json`、3 HuNav、NavFn、DWB；[preflight](</home/user/arena_local_costmap_diag_20261001/run_20261001/preflight.txt>) 记录以下 SHA-256：scenario `4970601dec3f7e56aaa2df1375d09da2f670d486244022c37af37ed5d816ff43`，HuNav 配置 `1b12b19ba0a9b75bf5b2aba2292e04c96e57b919b784d3e9e80ad8104ec38272`，live Nav2 YAML `2c7eea093eca3254d4e5c9c4e08ed044389cfd43bd097791345ae005034d2473`，与 frozen manifest 一致。实际 controller dump 为 `controller_frequency=1.0`、`movement_time_allowance=10.0`。

仅在 `/home/user/arena_local_costmap_diag_20261001/` 建立隔离 Nav2 source/build/install overlay。[诊断 diff](ARENA_LOCAL_COSTMAP_DIAGNOSTIC_PATCH_20261001.patch)（SHA-256 `03efe4de75c7ad4b8360d0d918b2f5a3eea9456db65eddff24948497b859fae1`）只增加时间戳、计数和日志，没有改变 filter 条件、队列、TF 超时、marking/clearing、控制或场景参数。隔离 `liblayers.so` SHA-256 `86d14e38244383e8a4f82fc5835c76b4d13465c2b53295fe7a2afec3a449164f`，实际日志出现全部相关插桩标记。隔离副本基于本地 Nav2 1.1.19 源码，现有 `/opt/ros/humble` 是 1.1.20；所用 TF MessageFilter 头文件来自现有安装且与本地源码对应头文件字节一致。此版本差异和 WARN 级插桩负载是诊断结果的限制；原始 default bag 无插桩但显示相同的数秒 TF 滞后和零占据。

[原始 bag](</home/user/arena_local_costmap_diag_20261001/run_20261001/raw_rosbag/metadata.yaml>)、[插桩 launch log](</home/user/arena_local_costmap_diag_20261001/run_20261001/launch.log>)、[逐扫描 JSON](</home/user/arena_local_costmap_diag_20261001/run_20261001/trace_analysis.json>)、[只读分析脚本](</home/user/arena_local_costmap_diag_20261001/analyze_trace.py>)、[时钟守卫](</home/user/arena_local_costmap_diag_20261001/run_20261001/clock_guard.log>) 均保留。三个 runtime param dump 和 endpoint 快照均成功（[capture 状态](</home/user/arena_local_costmap_diag_20261001/run_20261001/capture_status.json>)）：[local](</home/user/arena_local_costmap_diag_20261001/run_20261001/effective_local_costmap_params.yaml>)、[controller](</home/user/arena_local_costmap_diag_20261001/run_20261001/effective_controller_params.yaml>)、[Task Generator](</home/user/arena_local_costmap_diag_20261001/run_20261001/effective_task_generator_params.yaml>)、[local node](</home/user/arena_local_costmap_diag_20261001/run_20261001/local_node_info.txt>)、[lidar endpoint/QoS](</home/user/arena_local_costmap_diag_20261001/run_20261001/lidar_topic_info.txt>)。bag 是 23,904 条消息、可解码的 SQLite/metadata。

## 同一批扫描的逐层计数

| Stage | 诊断实测 |
|---|---:|
| SCAN PUBLISHED | 57 帧，全部 640 束；16,772 个量程内有限返回，4,313 个 0.08–2.5 m 返回，分布在 46 帧 |
| SCAN CALLBACK RECEIVED（local message filter 输入） | 57/57；lidar publisher 1 个，local subscriber 1 个，topic `/task_generator_node/jackal/lidar`，QoS 兼容 |
| TF ACCEPTED | 0 |
| TF DROPPED | 55；全部 `reason_code=1`；剩余 2 帧在停止时未决议，且没有近距返回 |
| TF DROP REASONS | 55 个 `OutTheBack` 文案；没有观察到 queue overflow。见下文对该文案的修正解释 |
| OBSERVATION BUFFER INPUT | 0 次调用；PointCloud projector、buffer TF/height/range 过滤 **未执行**，因此各阶段点数为 N/A，不能把它们称为“过滤后变成 0” |
| MARKING OBSERVATIONS / MARKED VOXELS | 362 个 voxel update cycle 中均为 0 / 0；marking 输入、range/map pass、markVoxel/markCell 调用均为 0 |
| CLEARING OBSERVATIONS / CLEARED VOXELS | 所有 cycle 为 0 / 0；`clearing_endpoints` topic 存在但 0 条消息 |
| VOXEL GRID NONZERO | **NO**；358 帧，标记 voxel 最大 0；每帧 360,000 个 unknown voxel、0 个 free voxel |
| LOCAL COSTMAP NONZERO | **NO**；166 帧均 0 occupied cells，最大 cost 0；362 次局部 layer 合成的 `layer_lethal` 与 `master_after` 均为 0 |

[实际 local 参数](</home/user/arena_local_costmap_diag_20261001/run_20261001/effective_local_costmap_params.yaml>) 明确是 `[voxel_layer,inflation_layer]`、`voxel_layer.enabled=true`、`voxel_layer.observation_sources=lidar`、`lidar.data_type=LaserScan`、`lidar.marking=true`、`lidar.clearing=true`、`lidar.topic=/task_generator_node/jackal/lidar`、`publish_voxel_map=true`。`transform_tolerance=0.3` s；ObstacleLayer 构造 MessageFilter 时将其作为 buffer timeout，并另设 filter 时间容差 **0.05 s**。运行时 local node 的订阅端点存在，故不是 namespace/QoS/未订阅错误；持续的 voxel cycle 与 costmap 发布也排除 layer 完全未激活。local 根节点的 `observation_sources: ''` 不覆盖 `voxel_layer.observation_sources: lidar`。

## 精确 TF 失效点

message filter 在 `stamp` 及 `stamp+0.05 s` 请求 `jackal/lidar_link → jackal/odom`。本次 `/tf` 中 `jackal/odom→jackal/base_link` 是高频的；`jackal/base_link→jackal/chassis_link→jackal/lidar_link` 仅发布 6 组，header stamp 从 5.617 到 26.383 s，中位间隔 **3.350 s**。对 55 个 drop 的 wall timestamp 逐个只查看**当时已收到**的 TF，最新 chassis/lidar 边仍比该 scan header 旧 **3.217–10.400 s，中位 6.167 s**；odom/base 边此时已经覆盖 scan 时间。被卡住的是末端两个边。每次 drop 的等待为 0.300–0.367 仿真秒（中位 0.300），墙钟 0.302–4.443 s（中位 0.547）。

这与事后 bag 可以在两个稀疏 TF stamp 间插值并不矛盾：未来 TF 到来时，MessageFilter 的 0.3 仿真秒等待早已结束。插桩里的 `OutTheBack`/“message earlier than all data”是本机 `tf2_ros::MessageFilter` 对 `future.get()` **任何异常**统一赋的失败枚举，不能据此声称传感器 stamp 真正落在 TF cache 过去端。本次按 bag 到达顺序重建的是**扫描比已到达的 lidar TF 更新**，属于实时供给迟到导致的 forward transform gap；底层 `future.get()` 异常文本没有被单独记录。

[实际执行的 TF graph](</home/user/arena_isaac5_py311_factory/build_ws/humble/isaac_sim_ros_ws/install/ros2isaacsim/lib/python3.11/site-packages/isaac_utils/graphs/tf.py>) 的默认 `throttle=300`，`OnTick.framePeriod=throttle`，并通过非阻塞 `rclpy.spin_once(timeout_sec=0)` 从内部 topic 取 TF、加 `jackal` 前缀再发布 `/tf`。在本次运行，该末端 TF 在收到时的 header 比仿真时钟旧约一个 3.35 s 周期（6 次中 5 次约 3.33–3.35 s，另一次 4.03 s）；前次原始 default bag 同一边中位滞后 **3.35 s**。原始 successful short bag 的这一边中位发布间隔也约 3.35 s，但发布时 stamp 滞后 **0 s**。因此 first divergence 是 **相同稀疏 TF graph 在 default 负载下出现一周期的发布/转发滞后**，而非仅仅 TF 名称或静态几何不同。graph 调度为何在 default 恰好错过这一周期未被线程级 trace 证明；但实时迟到超过 filter timeout 是已测得的直接丢弃条件。

HuNav 的空 parent `/tf` 在本次 bag 有 `robot`、`D_test_1`、`2`、`3` 各 205 条，共 820 条；launch log 有 4,888 行 `TF_NO_FRAME_ID` 相关警告。这些变换首次到达约 wall 1790853144.409，而第一帧墙面 scan 已在 1790853138.923 被 drop。它们可增加负载，但**不是首次 drop 的必要条件**；本轮没有改它们，也不把它们确认为根因。

## 一个南墙 scan 的完整轨迹

| 项 | 值 |
|---|---|
| Scan ID | header stamp `18.066667608` s / `18066667608` ns，`jackal/lidar_link`；wall receipt 1790853138.581069 |
| Publisher / subscription | 640 束、275 个量程内有限返回、111 个 0.08–2.5 m 返回；local MessageFilter `SCAN_CALLBACK` wall 1790853138.581114 |
| 代表墙面 beam | index 129，角度 −1.87514 rad，range 1.34382 m；离线 TF 投影 map hit `(25.8104, 0.0037, 0.2056)` m，离原始南墙线 0.0037 m；扫描后 local rolling grid cell `(80,63)`，值 0 |
| TF decision | wall 1790853138.922998，sim 18.366668 s，`TF_DROP reason_code=1`；无 `TF_ACCEPT` |
| 当时 TF | 最近已到达 chassis/lidar header 12.300001 s；覆盖 18.066668+0.05 s 的下一个 19.000001 s TF 到 wall 1790853151.243 才到达，距 scan 接收约 12.662 wall s |
| 后续链 | Laser projector 未调用；ObservationBuffer 无输入；voxel marking/clearing 均 0；18.467 s 的 local grid cell `(80,63)=0`，voxel grid 0 marked/360,000 unknown |

这个 hit 的距离、Z、墙线和窗口位置在几何上符合 marking 候选；它失效在 TF gate，不能归因于墙面本身不满足 voxel 条件。

## 最小修复提案与验收边界

[单行最小修复提案](ARENA_LOCAL_COSTMAP_MINIMAL_FIX_PROPOSAL_20261001.patch) **尚未应用**：将正式使用的 installed `isaac_utils/graphs/tf.py` 中默认 `throttle` 从 300 降到 10，使一次错过内部 TF 消息造成的典型仿真延迟由约 3.35 s 降至约 0.11 s，低于现有 0.3 s filter timeout。这个 patch 只改变机器人 TF 的发布/转发时效，不改变 TF 数值、LiDAR 几何、HuNav、场景、planner、controller 或 benchmark 阈值。它是针对已测的 TF 基础设施时间问题的**待验证修复假设**：目前没有修复后运行证据，不能声称一定恢复 costmap，且更频繁发布可能增加系统负载。若获准正式应用，先在隔离 runtime 验证 TF stamp age 与 scan filter accept，再按原参数重跑 short no-human 和 default + 3 HuNav 两个 gate，并核对 costmap/voxel 占据、控制与 provenance；不从这一诊断推断 default 导航应成功。

正式 Isaac child 仍是 `/home/user/arena_isaac5_py311_factory/build_ws/humble/isaac_sim_ros_ws/install/ros2isaacsim/lib/python3.11/site-packages/ros2isaacsim/run_isaacsim.py`，SHA-256 `b173a069e3603dc33797c8483ae9208b30e9e82b3229e67f2cf898afbdab1809`。本次未插桩或改写该 module，故 **instrumented module SHA-256 相同**；TF graph 的 installed `tf.py` SHA-256 `a1ba698b321c882226ad7b261c65e88ade709418908dea41b7a7e1212482ea5c`，也未改写。factory source 的 `run_isaacsim.py` 不等于该 installed child，精确 diff 已保存在 [runtime diff](ARENA_ISAAC_INSTALLED_VS_SOURCE_RUNTIME_20261001.patch)。

运行后已停止本次拥有的进程组 66704：孤留 Task Generator PID 66907 对 INT/TERM 无响应，仅对该组使用 KILL。未触碰无关进程；最终该组为空。没有修改正式 Nav2/Isaac/Arena runtime、scenario 或历史 bag。

## 决策

```text
ROOT CAUSE CLASS: A1 — TF_MESSAGE_FILTER_FAILURE
FIRST FAILED STAGE: Local costmap LaserScan MessageFilter 对 jackal/lidar_link → jackal/odom 的实时 TF 等待。
EXACT ROOT CAUSE: default 负载下 chassis/lidar TF 发布到达时约滞后一个 3.35 s graph 周期；55 帧在 0.3 sim s timeout 前拿不到 stamp+0.05 s 的完整 TF，0 帧进入 ObservationBuffer。
IS THIS AN INFRASTRUCTURE BUG: YES
MINIMAL PATCH: 提案把 installed isaac_utils/graphs/tf.py 默认 throttle 300→10；尚未应用或验证。
BENCHMARK SEMANTICS CHANGED BY PATCH: NO（预期只改变 TF 发布时效；仍须实测验证。）
REQUIRES FIX: YES，须先修复 TF 供给时效再尝试冻结 baseline。
REQUIRES SHORT+DEFAULT REGRESSION: YES
BASELINE FREEZE STATUS: NOT_READY
NEXT SINGLE ACTION: 审阅单行 TF throttle 提案；获准后先隔离验证 scan TF_ACCEPT、voxel/local 占据，再重跑 short 与 original default gate。
```
