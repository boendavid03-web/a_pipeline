# Arena5 + Isaac Sim 6.0.0 Jackal 无行人导航阶段总结

截至 2026-10-08 02:00 UTC。原始 `map_empty/quicktest`、Jackal、0 行人、seed 42、ROS domain 189、固定 0.25 m 外部终点门槛。每个实跑目录的 `result.json` 是终态和轨迹采样总账，`native_case.txt` 指向原生 ROS/Isaac 记录。此报告覆盖 Arena 原有 18 种方法；Nav2 组合另列。较早的 `FINAL_REPORT.md` 和 `CONTINUATION_REPORT_20261007.md` 保留当时快照。

## 18 方法覆盖

| 方法 | 阶段状态 | 原始 quicktest 的决定性证据 |
|---|---|---|
| SICNav | FAIL | 严格任务超时，最近 TF 约 1.008 m，末值 1.180 m；先前宽松任务 0.957 m 仍不达标。见 `continuation_20261007/SICNAV_CADRL_DIAGNOSIS.md`。 |
| HATEB | PASS 2/2 | 真正的 `hateb_local_planner::HATebLocalPlannerROS` 与修复后存活的 CoHAN bridge；0.007/0.027 m，0 Arena 2D 事件。关键案例 `RUNS/multimethod_hateb_headless_20261007T181420Z_2365841`。 |
| DRL-VO | FAIL | 超时、17.492 m、47 桌子 2D 事件。`RUNS/multimethod_drlvo_headless_20261007T172945Z_2252302`。 |
| CrowdNav | BLOCKED | 本地无准确的已训练 SARL checkpoint，`weights.yaml` 空、目标文件缺失；不运行随机权重。`continuation_20261007/CROWDNAV_BLOCKER.json`。 |
| AttnGraph | FAIL | 准确模型与 GST 已加载；超时、4.315 m、0 事件。`RUNS/multimethod_attngraph_headless_20261007T173651Z_2269528`。 |
| KDMA | FAIL | 原生任务结束，但停稳后外部门槛 0.251 m，超出 0.001 m；0 事件。`RUNS/multimethod_kdma_headless_20261007T173948Z_2276981`。 |
| RLRVO | FAIL | 到达 0.178 m，但穿原始桌子，40 个 2D 事件。`RUNS/multimethod_rlrvo_headless_20261007T174220Z_2283724`。 |
| HEIGHT | FAIL | 超时、3.716 m、1,147 个桌子 2D 事件。`RUNS/multimethod_height_headless_20261007T174420Z_2289834`。 |
| SCOPE | FAIL | 超时、11.650 m、45 个桌子 2D 事件。`RUNS/multimethod_scope_headless_20261007T174739Z_2297777`。 |
| CADRL | FAIL | 原始和诊断复测均超时并贴南墙；复测 5.708 m、0 事件。实际 640 束激光及 1,202 组网络输入留证；`RUNS/multimethod_cadrl_headless_20261007T183058Z_2400366`。半径候选未实跑。 |
| CD-SARL | PASS 2/2，修复配置 | 原始约 0.295 m 超时；仅将内部停驶判定与 0.25 m 任务门槛对齐后，0.222/0.231 m、0 事件，终点后 72/69 条零控制。`RUNS/multimethod_cd-sarl_headless_20261007T182419Z_2384958`、`...182820Z_2393477`。 |
| CrowdSurfer | FAIL | 超时、3.711 m、1,097 个桌子 2D 事件。`RUNS/multimethod_crowdsurfer_headless_20261007T175110Z_2305980`。 |
| DS-RNN | FAIL | 到达约 0.165 m 却穿桌，46 个 2D 事件；72 组实际网络输入/动作与逐时路径、代价地图子目标已保存。`continuation_20261007/DSRNN_PATH_INPUT_ACTION_ANALYSIS.json`。 |
| GenSafeNav | FAIL | 超时、4.134 m、1,155 个桌子 2D 事件；非序列化 costmap 未进入策略输入。`RUNS/multimethod_gensafenav_headless_20261007T175438Z_2314278`。 |
| HeR-DRL | FAIL | 超时，终点误差 0.276 m、0 事件。`RUNS/multimethod_her-drl_headless_20261007T175800Z_2324143`。 |
| NaviSTAR | FAIL | 超时，0.513 m、0 事件。`RUNS/multimethod_navistar_headless_20261007T180105Z_2333053`。 |
| NavRep | PASS 1/1 | 原始 quicktest 任务成功、0.124 m、0 事件、68 条终点后零控制、0.0039 m TF 漂移。`RUNS/multimethod_navrep_headless_20261007T180425Z_2341045`。 |
| SoNIC | BLOCKED（准确模型身份） | 当前交付包超时、4.293 m、0 事件；其 `policy.pt` 与 GenSafeNav 的 SHA-256 相同，不能证明为原 SoNIC 权重。`RUNS/multimethod_sonic_headless_20261007T180638Z_2347459`。 |

计数：**PASS 3、FAIL 13、BLOCKED 2、NOT_TESTED 0**，指 18 种方法在这一原始 quicktest 的覆盖和当前证据结论。PASS 只表示所列运行中的机器人任务、固定终点和 Arena 2D 足迹/栅格碰撞层。18 种方法未全面跑桌边、S 弯或 GUI；它们的跨场景泛化与 PhysX 逐帧接触仍未验证。模型加载、离线前向、进程存活及单纯到达均未被计为 PASS。

## Nav2 单独统计

| 组合 | 阶段结论 |
|---|---|
| NavFn + RPP | 基线 PASS：原始 quicktest headless/GUI 2/2、派生桌边 1/1、原始 S 弯 1/1；所有列出运行 0 Arena 2D 事件。 |
| NavFn + MPPI | PASS：最终配置原始 quicktest 3/3、派生桌边、原始 S 弯及 GUI 均达标且 0 Arena 2D 事件。 |
| NavFn + DWB | 固定配置重复性 FAIL：headless 1/5，另 1 次 0.251 m 严格超标、3 次超时；GUI 超时。`continuation_20261007/DWB_FAILURE_DIAGNOSIS.md` 分离了全 440 条轨迹无效和 progress checker 失败。未放宽门槛或盲调参数。 |

## 实际改动、原始失败和依据

- 原阶段的世界桌子 `bbox`、Nav2 local static layer、MPPI `model_dt=0.1` 和 DWB `ObstacleFootprint` 改动及原始证据仍见 `patches/`、`logs/`、`CONTINUATION_REPORT_20261007.md`。本轮没有再改原始场景或终点。
- 本轮扩展 `arena_robot_nav`、`configured_a_session.py`、`native_session.py` 的既有串行入口和方法身份检查；备份与 diff 在 `continuation_20261007/`。GPU/进程/资源门禁、原生 case 和退出清理仍启用。AttnGraph 首次仅因驱动强制 `weights_only` 加载失败；修正驱动后保留失败记录并完成有效闭环。
- HATEB 在 `arena_ws/src/Arena/arena_planners/arena_planners/pyproject.toml` 补登记原有 `cohan_peds_bridge` 入口，并在现有容器内从源码重新 editable 安装。原先桥接崩溃案例和修复后成功案例并存；`continuation_20261007/arena_planners.pyproject.hateb.patch`。
- CD-SARL 在 `arena_ws/src/Arena/arena_planners/planners/cd-sarl/planner.py` 只改内部到达/停驶谓词，保持模型、物理半径 0.3 m 及外部门槛 0.25 m；原始失败与两次修复成功并存。精确候选 hash 门禁在 `configured_a_session.py`，补丁 `continuation_20261007/cd_sarl.goal_stop.patch`。
- DS-RNN 只加只读观测 `dsrnn_path_observer.py` 和 `native_sdk_observer.py` 的实际神经输入记录；方法本体未改。其早期全局路径与子目标绕桌、网络动作、桥接命令和实际轨迹分层记录，不能仅以“网络没有家具输入”解释全部穿桌现象。
- CADRL 的上游障碍簇半径公式与 Arena 包装器有差异；单变量补丁 `continuation_20261007/cadrl.radius_candidate.patch` 已保存，**未通过运行验证，当前生效源码已恢复原 SHA**。原作者 ROS 控制和动作表依据：https://github.com/mit-acl/cadrl_ros/blob/master/scripts/cadrl_node.py 和 https://github.com/mit-acl/cadrl_ros/blob/master/scripts/network.py 。

## 当前阻塞与可接续命令

2026-10-08 02:00 UTC，额外 CADRL 候选的启动被资源门禁拒绝：ROS domain 189 保留 UDP 端口 `54800` 由 `/system.slice/todeskd.service` 占用。该次 `RUNS/multimethod_cadrl_headless_20261008T020005Z_2893608` 没有 `result.json`，**不是导航 FAIL**；GPU 无计算进程、Isaac 已退出，未停止占用端口的系统服务。下一条只读核对命令：

```bash
ss -ulpne | rg '54800|State'
```

资源释放后，先核对 `nvidia-smi --query-compute-apps=pid,process_name,used_gpu_memory --format=csv,noheader` 和运行锁，再按 `cadrl.radius_candidate.patch` 应用并登记候选 SHA-256 `4a7907eeaab8d6de8d04d643772cb26689fc1358bd466f7ccf8540a32cd3dc55` 的精确守卫，只跑一个案例。当前已验证的 CD-SARL 修复源码仍在场，故统一入口的再现命令须带 `A_CDSARL_GOAL_STOP_CANDIDATE=YES`：

```bash
A_CDSARL_GOAL_STOP_CANDIDATE=YES [LOCAL_PATH] --method cd-sarl --world map_empty --scenario quicktest --headless
```

其他方法只替换 `--method`；CADRL 半径补丁只有应用、校验并实跑后才可判断。CrowdNav 需要准确 SARL 权重，SoNIC 需要可核验的原模型身份。原有 Arena 工作树在本轮前已 dirty；未清理、覆盖或提交无关改动。

