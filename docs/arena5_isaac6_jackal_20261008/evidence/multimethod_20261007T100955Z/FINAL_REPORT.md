# Arena5 + Isaac Sim 6.0.0 机器人多方法阶段报告

> 2026-10-07 续报已完成：MPPI quicktest 3/3，并通过桌边、原始 S 弯和 GUI；DWB 固定配置总计 1/5，新增三次超时；DS-RNN 静态障碍输入根因已审计。最新结果见 [CONTINUATION_REPORT_20261007.md](CONTINUATION_REPORT_20261007.md)。下文保留本阶段当时的证据快照。

**结论：BASELINE_COMPLETE / MULTI_METHOD_PARTIAL。** Jackal 在原始 `map_empty/quicktest` 中由 NavFn + RPP 自主规划、发布非零控制、真实移动、绕过原有桌子、到达固定终点并停住；同一配置的 GUI 运行也完成。RPP 还通过派生的桌边绕行场景和原始 `sbend_corridor/sbend`。MPPI 有一次完整配置下的到达与 Arena 2D 零碰撞，DWB 两次完整配置中一次达标、一次超出固定 0.25 m 门槛 0.001 m。Arena 原有非 Nav2 方法尚无避障完成案例，因此不能宣布整个多方法演示完成。

本轮使用 Jackal、0 行人、seed 42、ROS domain 189；`quicktest` 的起点和终点保持原始值，实测终点误差门槛固定为 0.25 m。每个结果可从 `RUNS/<case>/result.json` 复核，原生 ROS/Isaac 日志路径在相应的 `native_case.txt`。`ENVIRONMENT.md` 记录提交版本和场景边界，`METHOD_MATRIX.md` 是全部 18 个 Arena 方法的总账。

## 逐方法结果

| 方法 | 原始 quicktest | 静态桌边场景 | 原始 S 弯 | 到达后停止 | 本轮完整配置重复 | 当前判断 |
|---|---|---|---|---|---|---|
| NavFn + RPP | Headless 0.136 m，GUI 0.232 m；两次均 0 个 Arena 2D 碰撞事件 | 派生 `table_leg_static`：0.149 m，0 事件，真实绕行 | 0.186 m，0 事件 | 三种场景均有终点零车轮目标、低 TF 漂移和 ≥5 仿真秒观察 | 原始 quicktest 2/2；派生静态 1/1；S 弯 1/1 | 机器人任务与 2D 足迹层 PASS；PhysX 接触和终点后持续 `cmd_vel` 发布 INCONCLUSIVE |
| NavFn + MPPI | 0.130 m，0 事件 | 未单独测试 | 未测试 | 零车轮目标，约 7.47 仿真秒保持 | 1/1 | 单次任务 PASS；重复性、PhysX 和持续 `cmd_vel` INCONCLUSIVE |
| NavFn + DWB | 一次 0.240 m、0 事件；另一次 0.251 m、0 事件，严格大于 0.25 m | 未单独测试 | 未测试 | 两次终点零车轮目标 | 1/2 | INCONCLUSIVE：固定门槛下结果不一致；不可写成可靠通过 |
| DS-RNN | 最新碰撞采样运行到 0.168 m，但穿过桌子，47 个 2D 事件；早期三次到达未采碰撞 | 未测试 | 未测试 | 到达后零指令/零车轮目标 | 有碰撞采样的 0/1 | FAIL：终点到达不等于避障成功 |
| CD-SARL | 超时；最近采样误差 0.252 m，结束约 0.295 m | 未测试 | 未测试 | 未达终点 | 0/1 | FAIL：没有放宽终点门槛 |
| CADRL | 超时，贴近南侧墙；结束误差约 6.16 m | 未测试 | 未测试 | 未达终点 | 0/1 | FAIL |
| SICNav | 设置实际 task tolerance 为 0.25 m 后超时；较早的宽松任务成功时外部误差 0.957 m | 未测试 | 未测试 | 未达固定门槛 | 0/1 严格配置 | FAIL |

`RUNS/multimethod_rpp_headless_20261007T111121Z_1665706`、`...rpp_gui_20261007T113311Z_1725050`、`...rpp_headless_20261007T110538Z_1646789` 和 `...rpp_headless_20261007T113619Z_1733689` 分别是 RPP 原始 headless、原始 GUI、派生静态、原始 S 弯的关键证据。MPPI 见 `...mppi_headless_20261007T112611Z_1704637`；DWB 的最终两次见 `...dwb_headless_20261007T112251Z_1696308` 与 `...dwb_headless_20261007T113026Z_1717577`；DS-RNN 见 `...dsrnn_headless_20261007T112859Z_1712245`。所有这些结果均为 Isaac 中的闭环运行，不是离线网络前向或录制轨迹回放。

派生的 `table_leg_static` 成功运行早于地图 `bbox` 与 local static layer 修复；它验证同一桌子的实体绕行，但不计入最终配置的 quicktest 重复次数。S 弯运行使用了后续完整配置。

## 最小修改与上游源码依据

1. 原始 `map_empty` 桌子具有几何碰撞，但其 world YAML 缺少已在模型注释中给出的 `bbox`；ROS 原始地图的桌心 `[12,10]` 曾是空闲值 `0`。只给 `table_0` 增加注释中的边界框后，地图与全局代价地图桌心均为占用值 `100`。实物模型和碰撞形状未改。证据：`logs/map_table_before.txt`、`logs/map_table_after_bbox.txt`、`patches/map_empty_table_bbox.patch`。
2. 原有 Nav2 local costmap 有 `static_layer` 设置却未启用；补进 plugin 列表并指定已有 `nav2_costmap_2d::StaticLayer`。重置后的 local costmap 桌心为 `100`。证据：`logs/local_static_layer_after_reset.txt`、`patches/local_static_layer.patch`。
3. MPPI 只把 `model_dt` 从 0.25 改为与 10 Hz 控制周期相同的 0.1 秒；原配置曾超时。Nav2 Jazzy 的 `Optimizer::setOffset` 源码直接比较控制周期和 `model_dt`，并在相等时启用控制序列平移。证据：`patches/mppi_model_dt.patch`；上游源码 https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_mppi_controller/src/optimizer.cpp 。
4. DWB 将已有的 `BaseObstacle` critic 改为 Nav2 自带的 `ObstacleFootprint`，保持原 scale；运行时参数确认新 critic 加载。上游 `BaseObstacle` 用路径中心点查代价，足迹 critic 检查机器人外形。证据：`patches/dwb_footprint_critic.patch`、`logs/dwb_footprint_runtime_param.txt`；上游源码 https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_dwb_controller/dwb_critics/src/base_obstacle.cpp 。
5. 统一入口 `arena_robot_nav` 复用已有 `configured_a_session.py`、`native_session.py`、`autonomous_nav_probe.py`，增加方法/世界/场景选择、独立结果目录、固定终点参数验证、碰撞与停稳采样、进程清理和退出码。派生的 `table_leg_static` 只新增场景 JSON，没有替换原始 quicktest。其源文件与旧配置副本保存在 `configs/`。

## GUI 和复测

统一入口：`[LOCAL_PATH]`。经实跑的 GUI 命令是：

```bash
[LOCAL_PATH] --method rpp --world map_empty --scenario quicktest --gui
```

Headless 复测只把 `--gui` 换成 `--headless`；MPPI/DWB 用同一入口替换 `--method`。GUI 已看到实际场景、Jackal 和前后位置变化，但起点、目标与路径的 Isaac 窗口叠加标记尚未实现。截图未随公开归档提供；原生 case 的 `gui_view` 子目录记录相机服务调用。

## 验收边界与下一步

- 这里的 0 碰撞表示 Arena 2D 足迹/栅格碰撞事件为 0；未取得逐帧 PhysX 刚体接触证明。不能把它推广为完整物理安全 PASS。
- Nav2 的最后一条观测控制为零，随后车轮目标持续零且 TF 漂移小，但终点后没有新的 `cmd_vel` 消息。未伪造零消息；严格“终点后至少 10 条零 cmd”证据仍是 INCONCLUSIVE。
- DWB 需要在完全相同配置再做固定 seed 多轮回归；MPPI 需要重复和静态/S 弯升级；Arena 非 Nav2 方法尤其 DS-RNN 需要基于真实静态障碍观测修复，不能从单纯到达声称避障。
