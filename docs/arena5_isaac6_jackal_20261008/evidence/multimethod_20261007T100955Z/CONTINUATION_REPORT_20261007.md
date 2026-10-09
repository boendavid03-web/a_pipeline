# Arena5 Jackal 多方法续报（2026-10-07）

**MULTI_METHOD_PROGRESS。** RPP 保持原有基线；MPPI 在最终配置下完成原始 quicktest 3/3、派生桌边 1/1、原始 S 弯 1/1，并新增 GUI quicktest PASS。DWB 固定配置的原始 quicktest 共 1/5 严格达标，新增三次均超时，判为 **FAIL（当前配置的重复性）**。DS-RNN 穿桌的直接输入原因已定位：其策略没有静态家具、激光或栅格输入；全局路径仅通过桥接子目标间接影响网络，最新碰桌案例的逐时子目标尚未留证，因此不能断言间接路径一定正确。

本轮没有修改 Arena/Isaac/Nav2/DS-RNN 源码、模型、世界或控制器参数。所有 GPU 案例由同一负责人串行执行，Jackal、0 行人、seed 42、ROS domain 189、原始 quicktest 起终点和严格 0.25 m 外部终点门槛保持不变。有效导航案例目录含 `result.json`、`native_case.txt` 和原生 ROS/Isaac 日志。以下 PASS 指任务终点、Arena 2D 足迹碰撞和观察到的停稳；PhysX 接触与终点后持续 `cmd_vel` 仍为 INCONCLUSIVE。

## 方法表

| 方法 | 原始 quicktest | 派生桌边 `table_leg_static` | 原始 S 弯 | Arena 2D 碰撞 | 重复性与状态 |
|---|---|---|---|---|---|
| NavFn + RPP | 原有 headless/GUI 2/2，0.136/0.232 m | 原有 0.149 m PASS | 原有 0.186 m PASS | 上述均 0 | 基线 PASS；本轮未重复 |
| NavFn + MPPI | 最终配置 headless 3/3：0.130、0.161、0.124 m；新增 GUI 0.152 m PASS | 0.165 m，距桌心最近 1.663 m，真实绕行 | 0.168 m，行程 25.982 m，PASS | 全部有效运行均 0 | **PASS：当前任务、2D 足迹与所测场景** |
| NavFn + DWB | 最终配置 headless 1/5：0.251 m FAIL、0.240 m PASS、三次超时（结束误差 1.677、4.179、4.536 m）；新增 GUI 超时，结束误差 4.357 m | NOT_TESTED | NOT_TESTED | 六次最终配置运行均 0 | **FAIL：固定配置下导航重复性不足** |
| DS-RNN | 最近一次到达 0.168 m，但穿过原始桌子 | NOT_TESTED | NOT_TESTED | 47 | **FAIL：静态障碍避障** |

MPPI 的新增 headless 运行目录依次为 `RUNS/multimethod_mppi_headless_20261007T155030Z_2048870`、`...155401Z_2057761`、`...155654Z_2065478`、`...160148Z_2079150`；最后两项分别是桌边与 S 弯。GUI 是 `RUNS/multimethod_mppi_gui_20261007T161727Z_2115572`。实测加载的插件均为 `nav2_navfn_planner::NavfnPlanner` 与 `nav2_mppi_controller::MPPIController`。四次新增 headless 的行程分别为 7.733、7.757、6.784、25.982 m，墙钟运行时间分别为 88.579、94.510、81.742、258.245 s；成功案例均有非零控制、全局路径、终点后零车轮目标和低 TF 漂移。

首次 MPPI S 弯尝试 `RUNS/multimethod_mppi_headless_20261007T155934Z_2072900` 在 ROS `goal_tolerance` 参数设置阶段超时，没有生成导航 `result.json`，不计入算法重复统计。精确清理其残留 `ros2 param` 进程后，重试形成上述有效 PASS；原始失败日志保留。

DWB 新增三次 headless 目录为 `RUNS/multimethod_dwb_headless_20261007T160732Z_2090655`、`...161100Z_2099130`、`...161425Z_2107585`；GUI 为 `RUNS/multimethod_dwb_gui_20261007T162106Z_2124507`。均实测加载 `dwb_core::DWBLocalPlanner` 和 NavFn。前两次新超时的 `native_case.txt` 指向的 `env/stdout.log` 记录 `No valid trajectories out of 440`、`ObstacleFootprint/Trajectory Hits Obstacle` 和 progress checker 失败；第三次新超时主要记录 progress checker 失败，不能笼统归因于同一 critic。第一例轨迹在桌边附近长时间停住，后虽恢复运动，仍未在 120 仿真秒内到达。此前 0.251 m 一例在 Nav2 任务结束时约 0.246 m，停稳观察时向外漂约 0.005 m 后跨过固定门槛；这是轨迹支持的边界机制，尚不足以解释每轮起因。没有放宽门槛或调参。

## DS-RNN 静态障碍输入根因

**分类：algorithm limitation（直接静态障碍感知缺失）；间接子目标链的贡献尚未验证。** 当前 `planner.py` 构造 `robot_node`（自身状态、相对子目标、朝向）、`temporal_edges`（世界系速度）和五行 `spatial_edges`（行人相对位置），再输出机器人本体系差速 `[v, omega]`，配置速率为 4 Hz。0 行人时五行都是 `(15,15)`；`planner.yaml` 的 `sensor_needs` 为空，激光、家具和占用栅格不进入策略网络。源码设定隐藏状态 `human_node_rnn` 为 1×1×128、`human_human_edge_rnn` 为 1×6×256，mask 首步为 0、其后为 1；这些状态并未在最新碰桌运行逐帧导出。`SubgoalGenerator` 会结合 NavFn 全局路径和 costmap 选 2 m 前视子目标，故静态地图仍有间接通道。最新碰桌运行 `RUNS/multimethod_dsrnn_headless_20261007T112859Z_1712245/result.json` 记录 47 个 2D 事件，机器人距桌心最近 0.026 m；该运行没有逐时子目标/网络张量留证，不能把子目标生成或跟踪错误排除。

`logs/DSRNN_PRIOR_ZERO_HUMAN_INPUT_SUMMARY.json` 从 **2026-10-06 的较早实跑** `sdk_2474189.jsonl` 提取 149 次实际网络输入：`robot_node` 1×1×7、`temporal_edges` 1×1×2、`spatial_edges` 1×5×2；零行人，所有占位行为 `(15,15)`，输入和动作均无 NaN/Inf。旧的 `DSRNN_ACTUAL_INPUT_CONTRACT.json` 还将 144 次零行人实际张量与 SDK 特征对齐，最大 float32 差约 2.33e-7。这是旧运行的输入契约证据，不能充作最新碰桌时逐帧输入。隐藏状态大小与首步 mask 由当前源码确定，未在最新碰桌运行逐帧记录。

原作者 [观察构造源码](https://github.com/Shuijing725/CrowdNav_DSRNN/blob/main/crowd_sim/envs/crowd_sim_dict.py) 也是机器人速度与人相对位置，没有家具或激光维度；[示例 unicycle 权重配置](https://github.com/Shuijing725/CrowdNav_DSRNN/blob/main/data/example_model_unicycle/configs/config.py) 使用五人、0.1 s 步长，Arena 当前运行是零人占位、4 Hz/0.25 s。上游 [缺席行人状态](https://github.com/Shuijing725/CrowdNav_DSRNN/blob/main/crowd_sim/envs/crowd_sim.py) 的占位是绝对坐标 `(15,15)` 再减机器人位置，而 Arena 当前固定使用相对 `(15,15)`，另有输入契约保真差异；没有证据表明它造成最新碰桌。这些差异不能仅凭源码解释碰桌，也不能未经验证直接塞入 Nav2 costmap 改写网络。当前没有对 DS-RNN 做算法修改。

## GUI 与训练

统一入口的逐方法命令（一次只运行一个）：

```bash
[LOCAL_PATH] --method rpp --world map_empty --scenario quicktest --gui
[LOCAL_PATH] --method mppi --world map_empty --scenario quicktest --gui
[LOCAL_PATH] --method dwb --world map_empty --scenario quicktest --gui
```

RPP 的早期 GUI 与本轮 MPPI GUI 均完成导航；DWB GUI 实际加载、相机服务返回 `success=True`、机器人运行，但任务超时。起点、终点和全局路径的 Isaac 窗口叠加标记仍未实现。GUI 的运行记录不自动证明人工已看清全部可视元素。



## 本轮写入与边界

本轮只写了新增 `RUNS/` 证据、`logs/DSRNN_PRIOR_ZERO_HUMAN_INPUT_SUMMARY.json`、本续报，并给 `FINAL_REPORT.md` 添加续报链接、更新 `METHOD_MATRIX.md` 的 Nav2 总账。原有四个配置补丁未改，Arena 源码/模型未改，[其他项目进度已从公开副本移除。]Arena 2D 零事件不等于 PhysX 零接触；终点后未持续发布零 `cmd_vel` 的案例保持 INCONCLUSIVE。
