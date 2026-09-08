# Isaac Sim Social Navigation 代码库分析

**分析日期：** 2026-09-07  
**范围：** `/home/user/navigation_project/a_pipeline/isaac_sim`  
**方式：** 只读递归盘点、文本搜索和关键源码阅读；未启动 Isaac Sim、ROS 2、Nav2 或仿真，未修改任何代码。

## 0. Executive summary

当前 `isaac_sim` 不是一个已经闭合的单一 benchmark，而是三部分并存：

1. **当前主运行链：** Isaac Sim 6.0.1 standalone + 自有 Mecanum730/XMS5 + IRA 动态行人 + 外部 ROS 2/UDP bridge + 双 LiDAR + DRL-VO/其他 ROS 消费者。
2. **Arena-Rosnav 候选接入树：** `arena_ws/src/arena`、`deps/hunav`、Nav2 和多个 planner 源码存在；但 Evaluation、HuNav 和多个 planner 没有完整 build/install/runtime 闭环。
3. **历史候选快照：** `arena_isaac5_backup` 是 Arena 5.1/Isaac 5.1 候选适配器源码，不在当前 active workspace package 路径中。

最重要的边界：

- 行人不是纯动画。当前 `gazebo_social` 模式使用共享 Gazebo Social Force kernel 计算二维期望速度，再转换为移动目标，交给持久 `IBehaviorAgent.follow(target_prim, distance=0)`；NavMesh、heading、motion matching 和动画仍由 Isaac BehaviorAgent 执行。
- Social Force 是显式 opt-in；默认模式仍可能是 `legacy`。必须从日志和 metadata 确认 `ISAAC_PEDESTRIAN_SOCIAL_MODE=gazebo_social`。
- 当前没有公开的 BehaviorAgent 直接二维速度 setter。横向 Social Force 通过移动 target 间接实现，速度标量通过 `set_speed()` 实现。
- 人-人 Social Force、personal-space 代理、人-机器人 Social Force 和机器人 footprint clearance proxy 已实现；物理接触真值、严格碰撞 ground truth 和完整局部交互矩阵尚未闭合。
- Arena、HuNavSim、IsaacLab 等内容主要是源码、依赖、资产或引用接口；没有证据表明 Arena 5、HuNavSim 已作为当前 Isaac 6 Social Navigation benchmark 正式运行。

## 1. 目录结构

### 1.1 有效源码/配置/评估树

以下树覆盖设计 benchmark 时会直接使用的代码、配置、场景、实验和评估文件。Isaac 安装包、缓存、构建目录和大量运行日志单独说明。

```text
isaac_sim/
├── README.md                                      # Isaac 6.0.1 主运行说明、ROS topic、雷达和行人合同
├── CURRENT_ARENA_SYSTEM_STATE.md                  # Arena/HuNav/Nav2/Planner 只读系统状态审计
├── GAZEBO_PEDESTRIAN_PARITY.md                    # Gazebo–Isaac 行人行为和 Social Force parity 边界
├── SCENARIO_TOPOLOGY_AB_EVALUATION_PLAN.md        # A/B 场景拓扑实验、运行合同和指标
├── config/
│   ├── crowded_tracking_suite_manifest_20260831.json # 多行人跟踪压力套件清单
│   ├── isaac_slam_online_async.yaml                # SLAM Toolbox 在线异步配置
│   └── rtx_lidar/navigation_2d_32k.usda            # 工程自有二维 RTX LiDAR profile
├── scenes/
│   ├── a_pipeline_empty_people.usda                # 空场/行人跟踪实验场景
│   ├── a_pipeline_eng_lobby.usda                   # Engineering Lobby 行人场景
│   ├── eng_lobby_main.usd                          # Lobby 主 USD
│   ├── mecanum_lidar_main.usd                      # Mecanum + LiDAR 主场景
│   └── mecanum_minimal_main.usd                    # 最小机器人场景
├── scripts/
│   ├── run_isaac_6_0_warehouse_people_robot.sh     # Isaac 6 行人+机器人主 launcher
│   ├── show_warehouse_people_robot_6_0.py          # Kit 主脚本：机器人、IRA、BehaviorAgent、Social Force、telemetry
│   ├── run_navigation.py                            # 较早单进程 demo，含简化行人和 root velocity
│   ├── run_custom_people_drlvo_demo.sh              # 自有场景 DRL-VO 一键入口
│   ├── cmd_vel_udp_relay.py                         # 系统 ROS 2 与 Kit 间 UDP/ROS bridge
│   ├── udp_telemetry.py                             # telemetry 编解码、压缩和分片
│   ├── pedestrian_social.py                         # Social Force + Isaac adapter + 指标
│   ├── pedestrian_steering.py                       # patrol polyline、target 生成和 steering
│   ├── pedestrian_free_space_guard.py               # 路径/target 静态 free-space 保护
│   ├── people_route_geometry.py                     # 行人路线和几何辅助
│   ├── physx_lidar_people.py                        # PhysX Raycast、人物腿部 analytic LiDAR
│   ├── rtx_lidar_scan.py                            # RTX returns 到 LaserScan 投影
│   ├── generate_*_people_config.py                  # empty/free-space/crowded/single 配置生成
│   ├── analyze_*_benchmark.py                       # tracking/single-motion 离线分析
│   ├── evaluate_scenario_topology_ab.py             # scenario topology A/B evaluator
│   ├── run_pedestrian_validation_matrix.sh          # 六类行人验证矩阵入口
│   ├── run_single_person_motion_benchmark.sh        # 单人运动 benchmark 入口
│   ├── validate_custom_people_routes.py              # 路线静态预检
│   ├── validate_scenario_topology_baseline.py       # baseline provenance/拓扑校验
│   ├── record_rosbag.sh / check_rosbag.sh           # 录包与 bag 合同验收
│   ├── teleop_robot.sh                              # ROS /cmd_vel 遥控入口
│   ├── ira_people_demo/
│   │   ├── ira_people_demo.yaml                     # IRA 行人 demo 配置
│   │   ├── custom_eng_lobby_people.yaml             # 自有 Lobby 行人配置
│   │   └── patrol_loop.json                         # patrol loop 路线
│   ├── show_ira_people_6_0.py / show_people_demo.py # 行人展示
│   ├── show_usd_skel_walkers_6_0.py                # skeleton walker 展示
│   └── convert_gazebo_boxes_to_usda.py              # Gazebo box 到 USDA 转换
├── tests/
│   ├── test_pedestrian_social.py                    # Social Force 单元测试
│   ├── test_gazebo_social_integration.py            # 共享 kernel integration 回归
│   ├── test_pedestrian_steering.py                  # steering/cursor/target 测试
│   ├── test_pedestrian_free_space_guard.py          # free-space guard 测试
│   ├── test_physx_lidar_people.py                   # analytic/PhysX LiDAR 测试
│   ├── test_single_person_motion_benchmark.py       # 单人 benchmark 合同
│   ├── test_crowded_tracking_*                      # crowd tracking 合同
│   ├── test_evaluate_scenario_topology_ab.py        # A/B evaluator 合同
│   ├── test_scenario_topology_*                     # baseline/split/radius 合同
│   └── test_isaac_*_contract.py                     # actuation、shell、capture 合同
├── experiments/
│   ├── behavior_agent_interface_validation.py       # BehaviorAgent API/lifecycle 探测
│   └── behavior_agent_discovery_debug.py            # agent discovery 调试
├── level3/
│   ├── launch/standalone_level3.launch.py           # standalone Level 3 ROS launch
│   ├── config/nav2_level3.yaml                      # Level 3 Nav2 配置
│   ├── config/test_routes.yaml                      # 测试路线
│   ├── tools/                                       # goal、map、runtime、offline validator
│   └── reports/                                     # 既有 preflight/offline/runtime 报告
├── maps/slam/warehouse_round1.yaml                  # SLAM map 配置
├── bags/                                            # 历史 rosbag
├── captures/                                        # 录制/可视化产物
├── arena_ws/
│   ├── src/arena/arena-rosnav/                      # Arena 总编排、task generator、training、tools
│   ├── src/arena/evaluation/                        # recorder/metrics/plots
│   ├── src/arena/isaac/                             # Arena Isaac 适配器、bridge、messages
│   ├── src/arena/simulation-setup/                  # world/entity/config/launch
│   ├── src/arena/tools/                             # scenario/editor/conversion 工具
│   ├── src/deps/hunav/hunav_sim/                    # HuNav manager/evaluator/messages/RViz
│   ├── src/deps/nav2/navigation2/                   # Nav2 源码依赖
│   ├── src/deps/slam_toolbox/                       # SLAM Toolbox 依赖
│   ├── src/deps/robots/                             # Jackal/iRobot 等依赖
│   ├── src/planners/                                # DRL-VO、CrowdNav、PaS-CrowdNav、SICNav
│   ├── build/                                      # 选择性 build 产物
│   ├── install/                                    # 选择性 install 产物
│   └── log/                                        # colcon 日志
└── arena_isaac5_backup/                             # Arena 5.1/Isaac 5.1 候选快照
    ├── arena_isaac/                                 # services、graphs、pedestrian simulator
    └── isaacsim_msgs/                               # IsaacSim ROS msg/srv
```

### 1.2 不应当误判为 benchmark 源码的内容

- `isaacsim-6.0.1/` 是完整 Isaac 安装、Kit、extensions、examples 和二进制缓存。
- `assets-6.0.1/` 是 NVIDIA/Isaac 资产库；含 IsaacLab 资产不等于使用 IsaacLab task。
- `arena_ws/build`、`install`、`log` 需要按 package 判断，目录存在不代表全 workspace 已闭合。
- `scripts/logs`、`bags`、`captures`、`level3/reports` 是证据产物，不是实现。
- `__pycache__`、Kit cache、shader cache、DerivedDataCache 是派生缓存。

## 2. 关键词搜索结果

搜索排除了 Isaac 安装包、assets、build/install/log/cache、bags 和 Python cache。完整命中数量较大，以下给出可复核的代表性位置和判断。

| 关键词/主题 | 文件与行 | 上下文与判断 |
|---|---|---|
| Arena | `CURRENT_ARENA_SYSTEM_STATE.md:80-136` | 明确说明目录由主 Isaac 线、独立实验线和部分 Arena workspace 组成；当前 launcher source 的是 `workspaces/ros2_ws`，不是 `arena_ws`。 |
| Arena 5.1 | `CURRENT_ARENA_SYSTEM_STATE.md:104-112,330-372` | `arena_isaac5_backup` 是 `arena5-isaac5.1.0` 候选快照；未进入 active package、未 build/install、未接通。 |
| Arena benchmark | `arena_ws/src/arena/arena-rosnav/configs/benchmark/*`；`CURRENT_ARENA_SYSTEM_STATE.md:684-734` | benchmark suites、task generator、evaluation 源码存在；不是当前 Isaac 6 runtime。 |
| HuNavSim | `arena_ws/src/deps/hunav/hunav_sim/README.md`；`CURRENT_ARENA_SYSTEM_STATE.md:684-734` | manager 有行为树/社会力模型和 agent service，另有 evaluator/messages；但 Isaac 分支 continuous update 有断点，且无 build/install/runtime 闭环。 |
| IsaacLab | `assets-6.0.1/Assets/Isaac/6.0/Isaac/IsaacLab/*` | 是安装资产树中的 IsaacLab 内容；主 `isaac_sim/scripts` 未发现 IsaacLab Environment/Manager/Task 接入证据。 |
| NavIsaacLab | 全文未发现同名正式 package/launch/class | 没有当前项目内的 NavIsaacLab 实现证据。 |
| SocNavBench/SocNavGym | 未发现正式 package、下载目录或 launcher | 没有下载或移植证据；现有 social 指标是项目自定义 evaluator/trace。 |
| BARN | 未发现正式 package、地图集或 launcher | 没有下载或接入证据。 |
| navigation benchmark | `SCENARIO_TOPOLOGY_AB_EVALUATION_PLAN.md:8-16,230-272` | 已有自定义 A/B、single motion、crowded tracking、validation matrix；不等同外部标准 benchmark。 |
| evaluation/metric | `scripts/evaluate_scenario_topology_ab.py`、`tests/test_*evaluation*`、`SCENARIO_TOPOLOGY_AB_EVALUATION_PLAN.md:12-16,238-272` | 有 trace、route completion、freeze/fallback、spacing、collision proxy、social quality；physical contact truth 缺失。 |
| Social Force | `scripts/pedestrian_social.py:1-8,77-195,225-411` | 无 Isaac/ROS 依赖；加载唯一 Gazebo kernel，保存 raw 结果，再做 Isaac adapter 和 OBB clearance。 |
| pedestrian/crowd/human | `scripts/show_warehouse_people_robot_6_0.py:1574-1583,1794-1800,2035-2087,2415-2431` | BehaviorAgent persistent follow、live state、target/speed 更新和 metadata 合同。 |
| avoidance | `scripts/show_warehouse_people_robot_6_0.py:500-617,2500-2700` | `off/native/gentle/legacy_dodge` 四档；native 是连续 object avoidance，其他档是条件 dodge。 |

### 2.1 是否已下载、引用还是移植

| 内容 | 结论 |
|---|---|
| Arena-Rosnav | 已存在源码快照/依赖源码和选择性 build/install；不是当前 Isaac 6 主 launcher 的闭合链。 |
| Arena 5.1 Isaac adapter | 已存在隔离 backup 源码，是候选适配器，不是 active package。 |
| HuNavSim | 源码已存在，包含 manager、BT、evaluator、messages、RViz；无安装/runtime 证据，Isaac 分支有接口断点。 |
| Nav2 | 源码/部分依赖和 Level 3 配置存在；不能与 Arena 总链混为一谈。 |
| SocNavBench/SocNavGym/BARN/NavIsaacLab | 未发现下载或移植证据；IsaacLab 仅在官方安装资产树出现。 |
| Social Force | 已做部分移植/共享 kernel 集成：Isaac 绝对路径加载 Gazebo kernel，保留 raw parity，再加 Isaac execution adapter；不是 HuNavSim SFM 直接移植。 |

## 3. 当前 Isaac Sim 导航架构

### 3.1 机器人和传感器

- 主模型是外部工程资产 `robot_related/robots/chassis_arm/motion_wheel_arm_simple_sphere_usd/mecanum730_xms5_default.usd`，即 Mecanum730/XMS5，不是 Arena 默认 Jackal/TurtleBot。
- 主控制合同是全向 `geometry_msgs/Twist` 的 `vx, vy, wz`。较早 `run_navigation.py` 计算四轮速度并直接施加 planar root velocity；主 Isaac 6 脚本通过 UDP 接收 ROS command，再进入 Isaac 控制路径。
- Kit 内不导入系统 `rclpy`。系统 Humble 的 `cmd_vel_udp_relay.py` 做 `/cmd_vel -> UDP -> Kit`，并把 telemetry 转回 `/odom`、`/tf`、`/clock`、`/scan`、`/scan_01`、`/scan_02` 和行人真值。
- 传感器包括双二维 LiDAR（PhysX Raycast 或 RTX）、odom、TF、clock；ROS 侧合并为 `/scan_merged`。PhysX analytic people LiDAR 可补充人物腿部 hit 诊断。
- 行人 GT topic `/pedestrian_ground_truth` 用于评估和轨迹，不应直接当作 DRL-VO 感知输入。DR-SPAAM/S3-Net 等是另一个 ROS 侧感知/策略接入层。

### 3.2 行人

- 主场景通过 IRA/`AgentsManager` 找到 `IRA_Character`，按配置和 patrol loop 生成/初始化；`run_navigation.py` 中另有简化的 `PedestrianManager`。
- IRA 使用 Isaac 人物资产、步行动画/Human Motion Library；BehaviorAgent 负责 NavMesh、heading、motion matching 和最终动画。
- `gazebo_social` 中 Social Force 产生二维期望速度，adapter 产生移动 target 和 scalar speed，由持久 follow 执行；不直接设置 pose，不是每帧 teleport。
- BehaviorAgent native object avoidance 可以开启；`gentle` 在持续接近且 clearance 很小时加低速 dodge；`legacy_dodge` 是历史强干预档。
- `PatrolPolylineCursor`、FreeSpaceMap visibility guard 和 predecessor reattachment 防止 target 穿墙或偏离后无法重接。

### 3.3 数据流

```text
场景 USD / IRA config / patrol_loop
        -> BehaviorAgent live state (position, velocity, facing, NavMesh)
        -> PatrolPolylineCursor base direction
        -> shared Gazebo Social Force kernel
           (human-human + robot-center + personal-space)
        -> Isaac actuator/steering adapter
        -> free-space guard + moving target + set_speed
        -> persistent BehaviorAgent.follow(target, distance=0)
        -> NavMesh + heading + motion matching + animation
        -> live pose/velocity + social JSONL + evaluator

ROS /cmd_vel
        -> system Humble ROS node
        -> localhost UDP
        -> Isaac Mecanum730/XMS5 actuation
        -> robot pose/odom/TF
        -> PhysX/RTX dual LiDAR
        -> UDP telemetry
        -> /scan_01 /scan_02 /scan_merged /clock /odom /tf
        -> Nav2/DRL-VO/SLAM/evaluator
```

| 阶段 | 当前实现 |
|---|---|
| 输入 | 场景/路线/行人 config；机器人目标或 `/cmd_vel`；LiDAR/odom/TF；live human/robot state |
| 感知 | Isaac LiDAR/PhysX/RTX；ROS bridge；行人 telemetry/GT；可选 DR-SPAAM/S3-Net |
| 规划 | 机器人可接 Nav2/DRL-VO；行人是 patrol polyline + Social Force + free-space guard |
| 控制 | 机器人 `Twist(vx,vy,wz)` 经 UDP/底盘；行人 `(vx,vy)` -> target + speed -> BehaviorAgent |
| 输出 | 机器人 pose/odom/TF、双 LiDAR；行人 pose/velocity/animation、social trace、metrics、GT trace |

## 4. Social Navigation 实现判定

1. **只是动画移动吗？** 不是。主 `gazebo_social` 链有路线意图、二维 Social Force、adapter 和 BehaviorAgent motion control；但 `legacy` 或早期 demo 可能更接近预设 patrol，必须检查运行 metadata。
2. **有真实速度/方向控制吗？** 有。controller 输出二维速度，adapter 保存 raw/post-adapter 值，BehaviorAgent 接收移动 target 和 speed。最终实际速度仍受 NavMesh/motion matching 影响。
3. **有 Social Force 吗？** 有。包括期望加速度、行人-行人、行人-机器人中心和 robot personal-space 项；Isaac 限制在 kernel 后执行。
4. **有 personal space 吗？** 有。行人 pair ratio、robot personal-space force、sigma 和 OBB signed clearance 均存在；clearance 是几何 proxy。
5. **有 human-human avoidance 吗？** 有两层：Social Force 人-人斥力/速度调节，以及 BehaviorAgent/native object avoidance；另有 emergency yield/dodge。
6. **有 human-robot avoidance 吗？** 有组合机制：robot-center/personal-space Social Force、robot OBB/free-space clearance、native avoidance 和 dodge。但 physical contact truth 和完整近距覆盖不足，不能直接称为已验证完备。

### 4.1 明确缺失/未闭合

- 没有 BehaviorAgent public desired planar velocity setter；当前 target-follow 是间接执行接口。
- 没有完整 physical contact event ground truth；proxy 不能证明无物理接触。
- 人-机器人近距交互覆盖不足；已有 crowd robot path 可能离人很远。
- crossing、near-wall、head-on、single route completion 等矩阵尚未全部通过；整体验收报告为 `NOT_CONVERGED_STOPPED`。
- HuNavSim SFM/BT 尚未成为当前 Isaac 6 行人后端；源码存在不等于 runtime 生效。
- response time、comfort、interaction distance、physical collision 等指标仍需正式数据合同。

## 5. benchmark 接入可能性

| 环境 | 当前相关程度 | 可借内容 | 接入难度 |
|---|---|---|---|
| Arena 5.0 / Arena-Rosnav | 高 | task generator、地图/场景组织、Nav2 planner plugin、benchmark suite、统一 launch、evaluation recorder | **高**：需统一 reset、sensor/odom、goal/action、pedestrian backend、episode lifecycle；不能直接拷贝 backup。 |
| HuNavSim | 中高 | human manager、behavior tree、社会力行为、human/robot messages、TSV evaluator 设计 | **高**：需决定 HuNav 或 shared Gazebo kernel 的唯一行为真相，修复 Isaac continuous update 断点，建立 state/reset/evaluator bridge，避免 double avoidance。 |
| NavIsaacLab | 低 | 可借 IsaacLab task/env/asset 组织思想 | **高/从零**：未发现 NavIsaacLab task/package，需要重新定义 env、reset、observation/action、人物和 metrics。 |
| SocNavBench / SocNavGym | 低 | scenario taxonomy、social metrics、interaction protocol | **高**：无本地代码，需外部获取、许可证核验并写 Isaac/ROS adapter；指标不能默认等价。 |
| BARN | 低到中 | 静态障碍地图、规划器对比、成功/时间/碰撞报告 | **中高**：偏静态导航，不直接代表 social navigation；需映射 USD/NavMesh 并另加行人指标。 |

推荐先抽象当前 Isaac 6 的 `ScenarioSpec -> reset(seed) -> backend -> observations/actions -> episode events -> metrics/artifact manifest` contract，再接 Arena task generator 或 HuNav evaluator。否则会同时混入 BehaviorAgent lifecycle、ROS bridge、HuNav backend 和 Arena launch 四类变量。

## 6. 下一步建议

### 短期：1–2 周

1. 建立 1–2 个 BehaviorAgent 最小复现，逐帧记录 reset anchor、live root、follow task、target、motion-matching root；non-finite 或 root 偏移立即 fail-fast。
2. 冻结 `legacy/native/gentle/gazebo_social` 行为 backend contract；metadata 保存 backend、kernel source/hash、avoidance mode、seed、scenario 和实际 topic counts。
3. 将 validation matrix 改为 reset/live-root/follow/actual-motion smoke -> 30–60 s interaction -> formal long run；single/head-on 不通过时不进入 crowd 统计。
4. 补齐 human-human/human-robot 的 center distance、body/footprint clearance、contact event、速度/方向、yield/dodge、freeze、fallback、route progress，并给 proxy 字段加 `proxy=true`。
5. 保持共享 kernel 不变，先修 execution lifecycle，不用 head-on 特例掩盖 BehaviorAgent 问题。

### 中期：1 个月

1. 完成 Isaac-native benchmark harness：ScenarioSpec、seed、reset、episode event、action、pedestrian backend、trace、checksum 统一。
2. 实现两个可切换行人 backend：`gazebo_social` parity backend 和 HuNav/BT backend，共享 spawn/route/state/metrics contract。
3. 先做 Arena task/episode/evaluation 最小闭环，不替换当前 Isaac 6 locomotion；验证 goal/reset 与 live root、odom、scan、human state 对应关系。
4. 用多 seed、固定拓扑、统一传感器分辨率和机器人控制合同报告 completion、SPL/时间、clearance、TTC、interaction response、freeze、comfort/jerk、contact。
5. 分离静态 preflight、kernel regression、offline replay、smoke runtime、formal runtime 和 strict acceptance。

### 长期：论文 benchmark 方向

建议定义“执行可验证的多层 Social Navigation benchmark”，而不是只增加人数：

- **层 0：** 空场单人运动和 reset/lifecycle。
- **层 1：** 两人 head-on、crossing、near-wall，测错身方向、response、personal-space、route progress、deadlock。
- **层 2：** 人-机器人迎面/侧向/接近，要求 contact truth 或明确低覆盖拒绝条件。
- **层 3：** 15/25 人 crowd，报告 throughput、completion、freeze/constraint、tail clearance、emergency intervention、多 seed 方差。
- **层 4：** 感知闭环，对比 Oracle、tracked、DR-SPAAM/S3-Net，GT 仅作 evaluator 输入并保持 source isolation。

论文级原则：

1. 将 decision-layer parity、locomotion execution validity、physical safety truth 分成三个 gate。
2. 将 NavMesh/BehaviorAgent 执行差异作为被测系统的一部分。
3. 同时记录 raw Social Force、adapter command、BehaviorAgent velocity、pose-derived actual velocity。
4. 使用 scenario family + multi-seed + paired baseline，不发布单一 15 人单 seed 作为稳定结论。
5. 未覆盖或仅有 proxy 的指标标记 `NOT_COVERED`，不折算成成功率。

## 7. 关键文件索引

| 主题 | 首要文件 |
|---|---|
| 主 launcher | `isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh` |
| Isaac 6 Kit runtime | `isaac_sim/scripts/show_warehouse_people_robot_6_0.py` |
| Social Force/adapter | `isaac_sim/scripts/pedestrian_social.py` |
| 路线/target | `isaac_sim/scripts/pedestrian_steering.py` |
| free-space | `isaac_sim/scripts/pedestrian_free_space_guard.py` |
| 机器人 demo | `isaac_sim/scripts/run_navigation.py` |
| ROS/UDP bridge | `isaac_sim/scripts/cmd_vel_udp_relay.py`、`udp_telemetry.py` |
| LiDAR | `physx_lidar_people.py`、`rtx_lidar_scan.py` |
| benchmark/评估 | `run_pedestrian_validation_matrix.sh`、`evaluate_scenario_topology_ab.py`、`analyze_*_benchmark.py` |
| Arena/HuNav | `isaac_sim/arena_ws/src/arena/`、`isaac_sim/arena_ws/src/deps/hunav/` |
| Isaac 5 候选快照 | `isaac_sim/arena_isaac5_backup/` |

## 8. 最终判断

当前代码具备 benchmark 所需的很多底层部件：真实 Isaac 场景、机器人控制与传感器、动态 IRA 行人、二维 Social Force、NavMesh/BehaviorAgent、路线保护、ROS bridge、trace 和离线指标。但它仍是**主运行链加若干候选集成树**，不是已经通过严格验收的统一 Social Navigation benchmark。

后续最应优先解决 BehaviorAgent reset/follow/motion-matching 生命周期和物理接触真值，而不是继续增加 Social Force 参数、场景特例或直接迁移 Arena/HuNav 全套。解决执行层后，再用统一 ScenarioSpec 接入 Arena/HuNav backend，并从头跑完整矩阵，才能形成可复现、可论文报告的 Isaac Sim Social Navigation benchmark。
