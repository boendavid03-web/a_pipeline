# Arena frozen local costmap: short versus default — 2026-10-01

## 判定与范围

**D — INSUFFICIENT_EVIDENCE。** 现有 bag 足以否定“default 的近距回波全不符合 marking 条件，所以 1235 帧空栅格合理”。它还不能区分 default 的 scan 在 TF message filter / observation buffer 前被丢弃，还是进入 voxel layer 后被内部过滤或清除。`Failed to make progress` 是另一个 near-goal 问题，本报告不据此解释 local costmap，也不改变 Nav2、DWB、progress checker 或场景。

只读输入是 [short 1 Hz bag](isaac_sim/backends/isaac5/generated/frozen_frequency_control_20261001/1hz/raw_rosbag/metadata.yaml)、[default + 3 HuNav bag](isaac_sim/backends/isaac5/generated/frozen_default_hunav_20261001/raw_rosbag/metadata.yaml)、default 的 [实际 local costmap 参数](isaac_sim/backends/isaac5/generated/frozen_default_hunav_20261001/effective_local_costmap_params.yaml)、两组启动日志及原始 [`walls.yaml`](/home/user/arena_isaac5_host_overlay_ws/src/arena_simulation_setup/worlds/map_empty/map/walls.yaml)。live 和 source `walls.yaml` 的 SHA-256 均为 `a50c3e4f7c6fce855ed7426f349bfb88accde10f8e9441accc512ad63bb3090c`。同一份 [只读分析脚本](ARENA_LOCAL_COSTMAP_AUDIT_20261001.py)处理两组 bag；[完整 JSON](ARENA_LOCAL_COSTMAP_AUDIT_20261001.json)保存每帧 scan 的 TF 判定和十个均匀抽样时刻的几何数据。没有运行新的 ROS/Isaac 场景。

复算时用 ROS Humble 的 `/usr/bin/python3`，并把 `/home/user/arena_full_ws/build_harmonic/hunav_msgs/rosidl_generator_py` 加入 `PYTHONPATH`、该路径下的 `hunav_msgs` 和 `/home/user/arena_full_ws/build_harmonic/hunav_msgs` 加入 `LD_LIBRARY_PATH`；这些路径只用于解码 bag 中的 `hunav_msgs/Agents`，不启动 ROS 节点。分析 JSON 和脚本已通过解析检查，十个 default 样本及 `1235/0`、`22,847` 关键计数通过断言。

## 逐层差分

| 指标 | frozen short | frozen default + 3 HuNav |
|---|---:|---:|
| LaserScan topic | `/task_generator_node/jackal/lidar` | 同左 |
| `frame_id` / 束数 | `jackal/lidar_link` / 640 | 同左 |
| Scan 数 / 有限且在传感器量程内的返回数 | 276 / 117,575 | 496 / 153,449 |
| 0.08–2.5 m 返回数 / 含这种返回的 scan | 5,591 / 45 | 26,173 / 169 |
| scan header 时间跨度（仿真秒） | 20.008–76.367 | 15.400–153.867 |
| bag 接收时间跨度（Unix 秒） | 1790843202.470–1790843255.519 | 1790843504.849–1790843807.872 |
| header 中位间隔 / 接收中位间隔 | 0.200 / 0.189 s | 0.217 / 0.482 s |
| 全链 TF 在**录制完成后**可插值的 scan | 265/276 = 96.0% | 481/496 = 97.0% |
| 几何上适格的近距返回 / 对应 scan | 5,591 / 45 | 22,931 / 154 |
| 其中距原始墙线 ≤0.15 m 的返回 | 5,581 | 22,847 |
| local OccupancyGrid 数 / 有 occupied cell 的帧 | 585 / 468 | 1235 / 0 |
| 单帧 occupied cells 最大值 | 542 | 0 |
| voxel layer 状态 | 占据输出证明有效 marking 曾发生；未保存完整实时参数 | 实际参数 `enabled=true`、`marking=true`；grid 持续发布，未证实 scan callback/voxel 写入 |

两组 `angle_min=-3.14158988`、`angle_max=3.13177252`、`angle_increment=0.009817469` rad，`range_min≈0.08`、`range_max=12.0` m，`scan_time≈0.1` s，正常 `time_increment≈0.00015625` s。short 第一帧、default 前两帧的 `time_increment=inf`；随后分别 275/494 帧为正常有限值，因此这一个异常不能解释 default **全程**为零。default 的 header 中位 scan rate 约 4.62 Hz、接收中位 rate 约 2.08 Hz；short 分别约 5.00/5.30 Hz。接收时间是 rosbag recorder 的时间，不等于 costmap callback 的时间。

几何判定使用每束 `angle_min + index × angle_increment` 和 `range`，通过 `map→jackal/odom→jackal/base_link→jackal/chassis_link→jackal/lidar_link` 变换。候选条件是量程内、`z∈[0,0.8)` m（16 × 0.05 m voxel）、距机器人中心 >0.2 m、落在该 scan 后 0.5 仿真秒内首个 local rolling grid 内。0.2 m 是对实际约 0.1 m footprint 加边距的保守排除，**不是** Nav2 直接输出的完整 raytracing/footprint 决策。候选数量是“若进入有效 observation 则应可 marking 的几何候选”，不能当作实际收到的 observation 数。两组均用相同代码和阈值。

原始墙线包括南边 `y=0`、北边 `y=23`。default 前三个抽样 hit 的 map `y≈0.04–0.06`，与南墙和静态地图 occupied cell 相符；后七个 `y≈22.96–22.97`，与北墙线相距 0.03–0.04 m。北墙样本的静态地图采样 cell 为 0，这只能说明栅格与墙线在该边缘不重合，不能把连续固定的北墙回波判为行人。抽样时最近 HuNav 行人与北墙 hit 至少约 1.66 m，之后距离更大；南墙样本也远离当时的行人。hit 的 `z≈0.205–0.206` m，随墙线稳定，距机器人中心约 1.14–2.43 m；不符合机器人自身或地面伪回波的特征。已知 shelf 位于 `(5,5)`，不对应下表这些墙线 hit。单个 RTX ray 的物体身份未被 bag 直接编码，所以“墙面”是由几何、地图/墙线和时序联合支持的分类，而非 collider ID 证明。

### default 十个时刻：几何上应进入 voxel 4，实际栅格全零

`t` 是 scan header 仿真秒；坐标单位 m；`window` 是跟随 scan 后首个 local grid 的 odom 边界 `[xmin,ymin,xmax,ymax]`；`z4` 是从 `origin_z=0`、`z_resolution=0.05` 得出的 0-index voxel。map→odom 在这十个时刻为零位移，所以表中 map XY 与 odom 网格坐标可直接对照。每行均有完整链 TF 的离线插值，且 grid cell 落在窗口内。

| t | robot map XY | lidar map XYZ | beam: angle, range | hit map XYZ | 墙线距离 | window odom | 预期 voxel / cell | 实际 cell |
|---:|---|---|---|---|---:|---|---|---:|
|15.400|25.292, 1.244|25.292, 1.244, 0.206|81: −2.35, 1.21|25.203, 0.038, 0.206|0.038|17.8, −6.3, 32.8, 8.7|z4 / 74,63|0|
|19.550|25.344, 1.422|25.344, 1.422, 0.206|633: 3.07, 1.36|25.297, 0.062, 0.206|0.062|17.8, −6.1, 32.8, 8.9|z4 / 74,61|0|
|22.967|25.210, 2.007|25.210, 2.007, 0.206|608: 2.83, 1.94|25.233, 0.064, 0.205|0.064|17.8, −5.5, 32.8, 9.5|z4 / 74,55|0|
|128.700|8.219, 20.532|8.219, 20.532, 0.206|208: −1.10, 2.43|8.424, 22.957, 0.206|0.043|0.8, 13.0, 15.8, 28.0|z4 / 76,99|0|
|132.267|7.473, 20.991|7.473, 20.991, 0.206|209: −1.09, 1.98|7.606, 22.971, 0.206|0.029|0.0, 13.5, 15.0, 28.5|z4 / 76,94|0|
|135.683|6.737, 21.435|6.737, 21.435, 0.206|211: −1.07, 1.53|6.790, 22.968, 0.205|0.032|−0.7, 13.9, 14.3, 28.9|z4 / 74,90|0|
|139.033|6.179, 21.762|6.179, 21.762, 0.206|221: −0.97, 1.20|6.265, 22.961, 0.206|0.039|−1.2, 14.2, 13.8, 29.2|z4 / 74,87|0|
|142.667|6.124, 21.819|6.124, 21.819, 0.206|385: 0.64, 1.14|6.164, 22.962, 0.206|0.038|−1.3, 14.3, 13.7, 29.3|z4 / 74,86|0|
|146.667|6.113, 21.819|6.113, 21.819, 0.206|443: 1.21, 1.14|6.080, 22.960, 0.206|0.040|−1.3, 14.3, 13.7, 29.3|z4 / 73,86|0|
|150.467|6.107, 21.815|6.107, 21.815, 0.206|438: 1.16, 1.14|6.198, 22.956, 0.206|0.044|−1.3, 14.3, 13.7, 29.3|z4 / 74,86|0|

同算法分析 short：`t=20.008, 21.042, 22.042, 23.150` 的南墙 hit 也先对应 cell 0；首个有 occupied cell 的 local grid 出现在 `t=24.050`（542 cells），此后 `t=24.133, 24.900, 25.942, 27.000, 27.975` 的对应墙面 cell 为 100。说明 scan 到栅格有实际延迟，不能以某一帧 0 判定 scan 丢失；default 的 1235/1235 帧均 0 是不同性质的结果。short 的完整十行、每帧 TF 标志和坐标均在 JSON 中。

## TF、时间与 observation source

| TF 边 | short 来源 / 数量 | default 来源 / 数量 | 说明 |
|---|---:|---:|---|
| `map→jackal/odom` | `/tf` 6,821 | `/tf` 13,585 | 与 scan 时刻匹配的 age 中位 0 s；无逆向时间跳变 |
| `jackal/odom→jackal/base_link` | `/tf` 6,821；`/tf_static` 1 | `/tf` 13,586；`/tf_static` 1 | 两组均有同一 child 的动态/静态重复边；bag 不含 publisher GID，不能判定 publisher 个数或 TF2 选择结果 |
| `jackal/base_link→jackal/chassis_link` | `/tf` 22 | `/tf` 43 | header 中位间隔约 3.35 仿真秒；default 最大间隔 7.60 s |
| `jackal/chassis_link→jackal/lidar_link` | `/tf` 22 | `/tf` 43 | 变换平移变化 <3 μm，几何近似固定，但发布在动态 `/tf` |

逐 scan 在其 header 时间做**离线**严格插值：short 265/276、default 481/496；两组不满足的 scan 均在最后一条 chassis/lidar TF 之后，未见主链 header 时间倒跳。前一条 chassis/lidar TF 的 age 中位分别 1.675/1.750 仿真秒、最大 3.308/7.550 秒，不能把这个 age 误当作 TF2 已成功执行查询。default 另有 `robot` 和三个人各 2,976 条空 parent 的 HuNav `/tf`，日志有 `TF_NO_FRAME_ID`，TF2 忽略它们；它们没有改写 `jackal/*` 主链，但可能增加处理负载，因果关系未证实。

关键的**运行时到达顺序**：按 recorder 时间，scan 到达时两组都尚无覆盖 `scan stamp + 0.3 s` 的完整 TF；到未来 TF 到齐的等待中位是 short **1.82 wall s**、default **11.70 wall s**（95 分位 3.20/15.20 s）。short 依然出现 occupied cells，因此“扫描发布时 TF 不可立即查询”本身并非充分根因。default 的更长等待值得在下一次诊断中观察 message filter 队列/失败计数；bag 只能看到 recorder 的到达顺序，不能复原 controller_server 内部队列、订阅 callback 或 TF2 buffer 在每一刻的状态。两组 `/clock` 与 scan header 使用仿真秒，bag receipt 是 Unix wall 秒；混用会得到虚假的 extrapolation 结论。

default 的**实际运行参数**来自上述 `ros2 param dump`，不是仅从 YAML 推测：

| 参数 | default 实际值 | short 实际值 |
|---|---|---|
| `plugins`, `rolling_window`, `width/height/resolution`, `global_frame`, `robot_base_frame` | `[voxel_layer,inflation_layer]`; `true`; `15/15/0.1`; `jackal/odom`; `jackal/base_link` | 完整 local param dump 未保存；启动输入 YAML 与 default 解析后相同 |
| `voxel_layer.enabled`, `observation_sources`, `lidar.topic`, `lidar.data_type` | `true`; `lidar`; `/task_generator_node/jackal/lidar`; `LaserScan` | 未保存实时值；short 的占据输出证明至少一个 observation path 有效 |
| `lidar.marking/clearing`, `obstacle_min/max_range`, `raytrace_min/max_range` | `true/true`; `0/2.5`; `0/3.0` m | 未保存实时值；输入模型参数声明 `marking/clearing=true` |
| layer/source `min_obstacle_height/max_obstacle_height`; `origin_z`, `z_resolution`, `z_voxels` | `0/2.0` m；`0`, `0.05`, `16` | 未保存实时值；输入 YAML 的 voxel 几何相同 |
| `unknown_threshold`, `mark_threshold`, `combination_method`, `transform_tolerance` | `15`, `0`, `1`, `0.3` s | 未保存实时值；输入 YAML 及 plugin 默认一致 |
| `lidar.expected_update_rate`, `observation_persistence` | `0`, `0` s | 未保存实时值；plugin 默认相同 |

default 的 local grids 连续发布，controller 有非零导航命令，说明导航生命周期不是完全停机；参数 `enabled=true` 说明 layer 被配置为开启。**这不证明**它在实际运行中订阅到每条 scan、通过 TF message filter、形成非空 `ObservationBuffer`，或成功写入 voxel。两组现存 controller logs 是 warn 级别，没有每帧 observation 的 callback/过滤/marking 计数；bag 未录 `/local_costmap/voxel_grid`、`clearing_endpoints`、订阅端点或参数事件。short 只保存了 controller frequency 和 allowance 的实际读数，没有完整 local costmap param dump。以上是不能在本轮归为 A 或 B 的具体缺口。

参数 dump 的 local costmap 根节点还显示 `observation_sources: ''`；真正由 `VoxelLayer` 读取的是 `voxel_layer.observation_sources: lidar` 和 `voxel_layer.lidar.*`。不能把前者单独解释成 lidar source 未配置。local plugins 中没有 `obstacle_layer`，其源类 `ObstacleLayer` 是 `VoxelLayer` 的实现基础。

## 根因边界与单次诊断设计

**至少一个 theoretically markable observation 存在吗？** 是，按 default 的实际参数，仅最早的 `t=15.400` 南墙扫描就有数百个合法回波；选出的 beam 81 量程 1.21 m、z=0.206 m、墙线距 0.038 m、网格 cell `(74,63)` 在窗口内。后续 22,847 个几何候选同样贴近原始墙线。只有它们确实抵达活跃 voxel layer 且没有随后被 clearing 覆盖时，才能要求 published OccupancyGrid 出现占据。因此本报告确认 **C 错误**，但无法在现有证据中判定是 **A 的 observation/TF 接收故障**还是 **B 的 voxel 内部过滤/覆盖故障**。最早已证实的结果分叉在 scan/TF/几何候选与 voxel/local-grid 输出之间；具体内部子层没有记录。

只设计**一次**最小诊断：隔离域中重放**原样** `default.json + 3 HuNav`，只观察起点附近至约 30 仿真秒（已能看到南墙回波；遇到首个完整的 scan→grid 窗口后停止），保留当前配置和运行模块。仅增加诊断采集：启动时实际 local 参数及订阅端点；`/lidar`、`/tf`、`/tf_static`、`/clock`、`/local_costmap/costmap`、已配置 `publish_voxel_map=true` 的 `/local_costmap/voxel_grid` 与 `/local_costmap/clearing_endpoints`；对该 local costmap 的 TF message filter 接受/丢弃、LaserScan callback、`ObservationBuffer::bufferCloud` 过滤前后点数和 VoxelLayer marking/clearing 数设置**只记录计数的诊断插桩**。对一个匹配墙线的 scan stamp 追踪同一 ID：若未触发 scan callback/TF 接受，定位 A；若进入 buffer 但点数/voxel 消失，定位 B；若 voxel 有标记而最终 grid 为 0，检查该帧 clearing/combination。诊断插桩需要在隔离副本实现并审阅后才运行；本轮未改任何配置、源代码或安装，也未授权现在执行这次 run。

## Runtime/source 附带审计

[精确 unified diff](ARENA_ISAAC_INSTALLED_VS_SOURCE_RUNTIME_20261001.patch) 记录 factory source `run_isaacsim.py`（SHA-256 `c07a7f4683399212522a3d8d6c877a665e648826a6476f7ad3d900c246f0bafc`）和 Isaac child installed `run_isaacsim.py`（SHA-256 `b173a069e3603dc33797c8483ae9208b30e9e82b3229e67f2cf898afbdab1809`）的全部差异。installed 版增加 Arena USD stage 组合/计数和几何审计逻辑；正式 benchmark 的 wrapper/entry point 解析到 `/home/user/arena_isaac5_py311_factory/build_ws/humble/isaac_sim_ros_ws/install/ros2isaacsim/lib/python3.11/site-packages/ros2isaacsim/run_isaacsim.py`，**不是** factory source 文件。只生成 diff，没有覆盖任一模块。

## 决策

```text
SHORT COSTMAP: 585 grids, 468 nonzero, max 542 occupied cells; wall observations demonstrably reached occupancy output.
DEFAULT COSTMAP: 1235 grids, all zero, despite 22,931 geometric marking candidates including 22,847 wall-line hits.
FIRST DIVERGENCE: Between geometrically valid scan/TF data and local voxel/grid output; callback/filter/marking substage unobserved.
ROOT CAUSE CLASS: D — INSUFFICIENT_EVIDENCE
ROOT CAUSE: Existing bag lacks per-scan costmap subscription, TF filter, observation-buffer and voxel marking evidence; short full runtime local parameters were not captured.
MINIMAL FIX: None justified yet; retain frozen inputs and instrument only the single diagnostic gate.
REQUIRES NEW RUN: YES, one bounded same-default diagnostic after instrumentation review.
BASELINE CAN FREEZE: NO
NEXT SINGLE ACTION: Prepare the isolated 30-sim-second default diagnostic that records scan-to-voxel callback/filter/marking counts and voxel_grid, without changing behavior parameters.
```
