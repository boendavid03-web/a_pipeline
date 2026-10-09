# Arena DWB 原始 `map_empty/1.json` 单场记录

采集日期：2026-10-02；run ID：`first_221`；ROS domain：221。**RUN_VALIDITY = VALID；NAVIGATION_RESULT = ABORTED。** 这是冻结 NavFn/DWB 配置下的一次有效导航失败：机器人在起点附近主要收到旋转指令，正式导航区间的 XY 路径长度约 0.019 m，低于 `SimpleProgressChecker` 的 0.5 m/10 s 要求；控制器日志随后报 `Failed to make progress`。目标离终点仍有 28.43 m，所以它与此前 default 的近目标转向 ABORTED 是不同位置的失败。仅此一次运行，不能据此计算成功率。

## 输入、身份与范围

| 项目 | 本场实际值与证据 |
|---|---|
| 场景 | 原始 `map_empty/scenarios/1.json`；source、live overlay、installed share 以及冻结清单 SHA-256 均为 `f1e55f8a874e5ad3f7ad5f6f4f36e62e113386a1eafae50e7e5c4f4f6f48b718`。实际 Task Generator 参数和启动日志均指向 `[LOCAL_PATH]`。 |
| 机器人 | 1 台 Jackal；场景起点 `(24, 20, 0.7)`，终点 `(2, 2, 0)`。运行前真实 odom/map TF 约 `(24.000, 20.000)`；起点位于 global costmap 内。 |
| 行人 | 3 名，名称 `1/2/3`；起点分别 `(14,2)`、`(2,8)`、`(8,2)`；各有原始 JSON 中的两个 waypoint。`waypoint_mode=1`。三人均在 HuNav 首次服务中初始化，Isaac `/World/pedestrians/_1/_2/_3` 均有 READY 日志。 |
| 障碍 | `shelf1/2/3`，位置 `(13,11,-0.5)`、`(12,12,-0.5)`、`(14,10,-0.5)`；三者运行日志各显示 14 个几何体及 14 个 collider。动态障碍即上述三人；interactive 为空。 |
| 冻结运行时 | Isaac Sim `[LOCAL_PATH]`、当前 compatibility adapter、Host ROS Humble Python 3.10、Isaac child Python 3.11。Task Generator 实际模块从 `isaac_sim/arena_ws/build/task_generator/.../robot_manager.py` 加载，解析到本仓库 source，SHA-256 `1b57d1ee62c0a647ebdb1271784cb51ccab8d44a24c347a842f3fc5a7a2969a3`。启动补丁 SHA-256 `46082ee21bf7f9c30d3ad8bf269a4b6025ee61600797856d81ffa6a64bd3a72e`。 |
| TF/Nav2 | 隔离 TF 模块运行日志给出路径、SHA-256 `c8f014b011970a07319ef871b53ebba2ff9b2d8407a3d860b664e569644f97f3`、`framePeriod=10`。Nav2 config SHA-256 `2c7eea093eca3254d4e5c9c4e08ed044389cfd43bd097791345ae005034d2473`；运行参数为 NavFn、DWB、1 Hz、0.5 m/10 s progress、0.25 m/0.25 rad goal。Nav2 package prefix 和已加载 `libdwb_core.so`/`liblayers.so` 均来自 `/opt/ros/humble`，无诊断 overlay。 |
| 其他输入 | map 各文件、HuNav behavior、Jackal control、shelf USD、NavFn/DWB 配置的路径和哈希见原始 `preflight.json`；这些已冻结项全部匹配清单。运行日志中的 `M_Medical_01` 角色 USD/贴图与 Biped Setup/动画共 22 个文件的补充哈希见 `scenario1_asset_hashes_postrun.json`；它们没有历史冻结哈希，且本轮是在运行后计算，不能冒充运行前 pin。保留已登记的 global costmap 覆盖范围偏差和 640 束 LaserScan。源码未暴露或固定本场全部随机种子。根仓库 HEAD `281bc0f4b78203d17738bab842fa615ad82a9930`；原有未跟踪文件与大文件未触碰。 |

本场运行前检查了 GPU、共享进程及 domain；当时没有 Isaac/rosbag/Task Generator 占用，domain 221 没有节点。只启动这一个 episode；没有重试、调整参数或运行后续场景。采集规则在启动前写入 `preflight.json`：以首次本方 NavigateToPose status 为导航计时起点，启动上限 300 s 墙钟、episode 上限 600 s 仿真时间、整次保护 1200 s 墙钟、clock 停滞 45 s；终态后保留 5 s 仿真或 10 s 墙钟收尾。这是本轮采集规则，不是 Arena 官方评测规则；源码任务 timeout 默认 `-1`。

## 结果与数据质量

| 字段 | 本场结果 |
|---|---|
| SCENARIO_ACTUALLY_LOADED | 上述绝对路径 `1.json`，Task Generator 参数与日志一致 |
| SCENARIO_SOURCE_LIVE_HASH_MATCH | YES，三处哈希同冻结清单 |
| ROBOT_AND_HUMAN_COUNTS | 1 Jackal；3 HuNav 人 `1/2/3`；3 shelf 几何与碰撞体成功加载 |
| RUNTIME_AND_CONFIG_MATCH_FROZEN | YES，preflight 无漂移；有效运行参数、TF runtime 标记、正式 Nav2 库一致 |
| HIGH_LEVEL_GOAL_COUNT | 1，本方 UUID `bdacfc72a264406688a7dc717dc54396`，EXECUTING→ABORTED；内部 FollowPath UUID 未计为新 episode；`goal_pose` 消息 0 |
| STARTUP_READY_BEFORE_SEND | YES；Nav2 三个 lifecycle ACTIVE、真实 map TF 和 global costmap 样本早于 `ISAAC_GOAL_SEND`；发送日志时间 `1790934470.4331024` s，接受时间 `1790934470.5316463` s |
| RUN_VALIDITY | VALID：单目标归属、场景/资产、Nav2、TF、scan、odom、HuNav 与 bag 完整性通过 |
| NAVIGATION_RESULT | ABORTED |
| FAILURE_OR_TIMEOUT_REASON | `controller_server: Failed to make progress`；起点附近 XY 进展不足。DWB 在该区间的 23 条 `cmd_vel_nav` 中主要给出负角速度，仅一次很小的正线速度；为何长时间选择旋转还需独立诊断，本场未调参。 |
| DURATION_SIM_S | 10.5667（首次 action 状态至终态，仿真时间） |
| DURATION_WALL_S | 21.6004（同区间墙钟） |
| PATH_LENGTH_M | 0.01860；只积分 action 区间 odom，每 0.1 s 仿真采样，跳变阈值 0.5 m，排除 0 次；起终净位移 0.00613 m。近静止值受 odom 抖动影响。 |
| FINAL_XY_ERROR_M | 28.4314；记录的 `map→jackal/odom` 变换用于将 odom 位姿转至 map 后比较。 |
| FINAL_YAW_ERROR_RAD | 0.55229；同一 map 坐标系下与场景目标朝向比较。 |
| POST_TERMINAL_STOP | YES；终态后 10 条最终 `/cmd_vel` 全为零。 |
| TF_SCAN_COSTMAP_VALIDITY | 有 2170 条 map→odom TF、88 帧 640 束 scan、197 张 local grid、461 张 voxel grid；其中 3 张 local grid 和 5 张 voxel grid 非零。起点周围大多空旷，仅 1 帧 scan 有 2.5 m 内有效回波；零占据比例不单独判无效。正式 Nav2 未插桩，TF_ACCEPT 数量未测。 |
| HUNAV_TRAJECTORIES | `1/2/3` 在 action 区间各 215 个有限位置样本；各自首末位移约 6.424 m。 |
| MIN_HUMAN_DISTANCE_AND_DEFINITION | 19.8054 m，人与机器人中心距代理，发生在人 `1` 附近；HuNav 消息 `frame_id` 为空，按其起点与场景/HuNav 日志一致将位置视作 map；机器人 odom 用录制 TF 变换，按 bag 接收墙钟最近邻同步，容差 0.2 s，缺配 0。它不是外形净空或碰撞结论。 |
| COLLISION_EVIDENCE | NOT_MEASURED：未录到可用于接触判定的 topic，缺完整人体碰撞几何；有 shelf collider 加载证据。 |
| MISSING_METRICS | 接触/碰撞真值、正式 Nav2 TF_ACCEPT 数、Arena Evaluation 官方输出。此表为透明 bag 提取指标。 |
| RAW_EVIDENCE_PATH | `[LOCAL_PATH]` |
| REPRODUCTION_COMMAND | `scripts/validation/arena_dwb_baseline/run_single.bash 221 <new_unique_run_id>`；离线重算先 source host setup 与 HuNav overlay，再运行 `python3 scripts/validation/arena_dwb_baseline/analyze_single.py <run_dir>`。目录已存在会拒绝覆盖。 |
| FILES_CREATED_OR_MODIFIED | 根仓库新增 `scripts/validation/arena_dwb_baseline/{run_single.bash,preflight.py,terminal_guard.py,analyze_single.py}` 和本报告；原始运行目录独立保存。 |
| NEXT_SINGLE_ACTION | 本场记录确认后，以同样规则另一次独立启动原始 `2.json`；本轮未执行。 |

原始 bag 数据库 `raw_rosbag/raw_rosbag_0.db3` 为 **120,209,408 B**，SHA-256 `ee3b78c47463c8bfca2d863e97ff2f0acfa2c315cf1bb1d27c6ad9033ae1da8d`；`metadata.yaml` SHA-256 `b9ad0ac66c20fb7eec8b5268a4409201c72cdc00bf55c3214871362f15d7a689`。`pragma integrity_check = ok`，录包日志有 `Recording stopped`。`/task_generator_node/people` 因同名多消息类型报错、未被此通配录包；本场 HuNav 轨迹使用实际录入的 `/task_generator_node/human_states`（317 条）。`run_summary.json` 包含逐字段机器可读结果，`preflight.json`、`launch.log`、参数 dump、`guard_result.json`、`bag.log` 与 `controller_libraries.txt` 保留原始核查链。

唯一已知运行限制是该条记录只覆盖一个 episode；没有实时连续 reset 证据。结论限定为这条冻结配置下的场景 1 有效 ABORTED 记录。
