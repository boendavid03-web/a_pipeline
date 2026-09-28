# Isaac Sim 5.1 完整场景迁移报告

## 2026-09-13 Isaac5 configurable social crowd follow-up

The Isaac5 backend now exposes a 1--20 coloured-character interface and a
backend-local stateful social controller. The follow-up result is
**PASS_WITH_LIMITATIONS**. The former 0.428 m short-window overlap and abrupt
free-space hold were fixed with open-polyline ping-pong traversal, bounded
stopping-distance qualification, route-relative predictive passing and a
deterministic Isaac6-style yield rule. Real Isaac5 Stage D GUI and Stage E
DR-SPAAM/tracker passed. A 20-person headless fixed-goal Gate9F also passed at
13.68 Hz wall-clock dual scans and 14.01 Hz DRL-VO inference. The corresponding
GUI run reached the goal and stopped safely but achieved only 11.52 Hz scans,
so Gate9G remains a performance limitation. Three 120-second pure-controller
seed preflights exposed long-term overlap/deadlock on remaining shared route
segments; no Stage G stability acceptance is claimed. Full evidence is in
`isaac5_20_person_social_navigation_report.md`.

更新时间：2026-09-12（Gate 9G 彩色人物、全地图路线与朝向对齐最终回归后）
工作目录：`/home/user/navigation_project/a_pipeline`
当前结论：`PASS_WITH_LIMITATIONS`（2026-09-12 已完成 Isaac5 本地人物资产闭包和通用入口默认切换；许可证记录仅支持本机使用，不作发布授权结论）

Gate 0--10 已按顺序形成当前 Isaac Sim 5.1 的静态、真实 GUI、runtime、ROS 图、时间戳、模型推理、闭环导航和 Isaac 6 回归证据。Isaac 5 active lobby 功能迁移通过；最终保留 `PASS_WITH_LIMITATIONS`：Isaac 5 已复用 Isaac 6 彩色 Character 资产并经 NVIDIA 5.1 retarget core 生成 101-joint 动画，但路线/避障层仍是 backend adapter 而非不可向后兼容的 IRA 1.6 BehaviorAgent；此外 Isaac 6 默认 warehouse 的 RTX 路径在 15 Hz 请求下本轮仅实测 9.68 Hz（改为 10 Hz 后通过）。

## 1. 证据和边界

- `STATIC_PASS` 只表示文件/API/合同审计；不替代 Isaac runtime。
- `RUNTIME_PASS` 必须来自本轮实际启动的 Isaac 5.1 或 host ROS 节点及结构化结果。
- 用户在 Gate 1 画面后回复“基本正常”，故记录 `USER_VISUAL_PASS`。
- DR-SPAAM 检出和 tracker ID 稳定不等价于检测准确率真值。
- 行人的 kinematic capsule 可作为 LiDAR/几何碰撞权威，但不冒充人体网格接触或人体动力学真值。
- Gate 8.3 是合成 LiDAR 近障代理的 safety-veto 证据，不是 PhysX 接触证据。
- Gate 7/8 的控制只写独立 shadow topic；Gate 9 才接入 backend-local Isaac 5 移动底盘 adapter，始终没有占用全局 `/cmd_vel`。
- Gate 9 的人与机器人距离是 capsule/box 中心几何门禁，不等价于人体网格接触真值。
- Gate 9G 在同一 Isaac 5 RTX GUI 中运行 scene、robot visual、8 人、双 LiDAR、DR-SPAAM/tracker 和 DRL-VO；SemanticCNN 是互斥的另一条 policy chain，已由 Gate 7.3/7.4 单独验证，不声称两种 policy 同时控制机器人。
- Gate 10 的 active lobby 通过不消除其中已有的行人重叠/自由空间侵入告警；warehouse 10 Hz smoke 也不证明 15 Hz RTX 合同。

## 2. 固定迁移基线

| 项目 | 当前实测 |
|---|---|
| OS / kernel | Ubuntu 22.04.5 / `6.8.0-136-generic` |
| GPU / driver | NVIDIA GeForce RTX 5090 / `580.173.02` |
| Isaac 5 | `/home/user/isaacsim/5.1.0`，`5.1.0-rc.19+release.26219.9c81211b.gl` |
| Isaac 6 | `isaac_sim/isaacsim-6.0.1`，`6.0.1-rc.7+release.42383.32955d8d.gl` |
| active scene | `isaac_sim/scenes/a_pipeline_eng_lobby.usda` |
| canonical robot | `/home/user/navigation_project/robot_related/robots/chassis_arm/motion_wheel_arm_simple_sphere_usd/mecanum730_xms5_default.usd` |

### 2.1 active scene 的确定证据

Isaac 6 启动器在完全不设置环境变量时仍默认 `warehouse`，但本项目当前工程场景运行合同显式使用 `ISAAC_SCENE=custom`。三类独立证据一致：

1. `run_isaac_6_0_warehouse_people_robot.sh` 的 `custom` 默认路径是 `isaac_sim/scenes/a_pipeline_eng_lobby.usda`；
2. 最近一次 60 秒 GUI 回归日志 `custom_people_robot_6_0_20260912_173250.log` 的 Scene 行和 `WAREHOUSE_PEOPLE_ROBOT_READY.scene_usd` 均为该文件；
3. Isaac 5 Gate 2--9G 的 runtime 结果均记录同一个绝对场景路径和同一 SHA-256。

因此本次迁移主场景是 `a_pipeline_eng_lobby.usda`；`IRA_OBT_Sample_Warehouse.usd` 只作为 Gate 10B 的 Isaac 6 原默认场景回归，不是第二个迁移目标。

哈希：

| 文件 | SHA-256 |
|---|---|
| active scene | `18e012a8d1b9614aefa1517bc3be3ac47775e62cd6fba78240a04b4b6652c1dd` |
| canonical robot USD | `f9dec5e8554d6504c52dc91622338024e035f82697ff331dde0f5c53c1d15f4c` |
| DR-SPAAM | `861ca286ab68c0ab227529435fa62f11a89ce4b20e53cbbdb788c1c545a85dd9` |
| S3-Net | `d7ae12c45a7d2a44ffd6ec07ebc99f70514ad46632c0f5a8f345d98d040d2330` |
| SemanticCNN | `175ee162e1f7ccd23651efdcfc657c512685c439bf98e026a6bfc9c811b9ac26` |
| DRL-VO base | `57cfd10e6f528f96f721480420c1d6f873b0ff53ad4bbd3e997da5d2fcd6c4e2` |

### 2.2 场景和资产依赖闭包

| 资产 | 使用方式 | 闭包结果 |
|---|---|---|
| active lobby | 项目自有 USD，Isaac 5/6 原位引用 | 米制、Z-up、177 个 composed prim、80 个碰撞对象、1 个 PhysicsScene、0 unresolved；场景本体没有外部资产依赖 |
| Mecanum730 完整 USD | Gate 1/3 articulation 验证 | 路径与哈希固定，真实加载成功；源文件未修改 |
| Mecanum730 visual/base layer | Gate 9/9G，沿用 Isaac 6 视觉跟随结构 | `configuration/mecanum730_xms5_default_base.usd` 原位引用；不复制、不启用 arm dynamics |
| 8 个彩色 Character | Isaac 6 IRA 使用的原始人物资产 | `assets-6.0.1/.../People/Characters/` 下施工、警察、医护各人物；每个 101 joints，BaseColor/Normal/ORM 与 Isaac 5 OmniPBR 全部解析，0 unresolved |
| retarget source rig | 仅用于启动期隐藏的 81-joint 数据源 | `.../test_biped/biped_demo_meters.usd`；不作为最终可见人物 |
| walk animation | Isaac 6 data-only 资产 | `assets-6.0.1/.../Animations/stand_walk_loop_in_place.skelanim.usd`；通过 Isaac 5 `omni.anim.retarget.core` 烘焙到每个 101-joint Character |
| DR-SPAAM/S3-Net/SemanticCNN/DRL-VO | 项目既有代码与 checkpoint 原位引用 | launcher 启动前逐路径检查，runtime 记录哈希；权重未修改 |

NVIDIA 自带资产没有盲目复制进 backend；跨版本复用只发生在 USD/SkelAnim 数据层，没有导入 Isaac 6 runtime extension 或 cp312 module。

## 3. Gate 总表

| Gate | 内容 | 状态 | 主要证据 |
|---|---|---|---|
| 0 | 基线、active scene、USD/API 审计 | `STATIC_PASS` | 当前文件、安装与成功 Isaac 6 日志 |
| 1 | robot-only RTX GUI、稳定性、直立机械臂 | `RUNTIME_PASS` + `USER_VISUAL_PASS` | `gate1_robot_upright_580.log/.png` |
| 2 | active lobby scene GUI | `RUNTIME_PASS` | `gate2_lobby_scene_580.log/.png` |
| 3 | scene + Mecanum730 + 有界运动/零速 | `RUNTIME_PASS` | `gate3_lobby_robot_visual_pass_580.log/.png` |
| 4 | ROS common messages + shadow decision | `RUNTIME_PASS` | `gate4_ros_shadow_orchestrator_retry.log` |
| 5 | 双 LiDAR 几何、自滤波、同步与频率 | `RUNTIME_PASS` | `gate5_dual_lidar_self_filter_retry_580.log` |
| 6 | 单个 Isaac 6 彩色 Character + 101-joint retarget | `RUNTIME_PASS` | `gate6_textured_forward_alignment_attempt1_580.log`、`gate6_textured_forward_aligned_gui.png` |
| 6B | 全地图 8 人彩色 crowd adapter | `RUNTIME_PASS` | `gate6b_mapwide_routes_attempt2_boolfix_580.log`、`gate6b_mapwide_crowd_gui.png` |
| 7.1 | DR-SPAAM 单独启用 | `RUNTIME_PASS` | `gate71_drspaam_orchestrator_attempt3_graphfix_580.log` |
| 7.2 | DR-SPAAM + tracker | `RUNTIME_PASS` | `gate72_tracking_orchestrator_580.log` |
| 7.3 | S3-Net + SemanticCNN | `RUNTIME_PASS` | `gate73_semantic_monitor.log`、ROS graph |
| 7.4 | SemanticCNN 同步/freshness | `RUNTIME_PASS` | `gate74_freshness_monitor.log` |
| 8.1 | DR-SPAAM tracks → DRL-VO shadow | `RUNTIME_PASS` | `gate81_drlvo_monitor.log`、ROS graph |
| 8.2 | DRL-VO track freshness gate | `RUNTIME_PASS` | `gate82_drlvo_monitor.log` |
| 8.3 | DRL-VO 近障 safety veto | `RUNTIME_PASS`（几何代理） | `gate83_drlvo_monitor.log` |
| 9A | 完整导航短时闭环 smoke | `RUNTIME_PASS` | `gate9_drlvo_monitor.log`、crowd runtime/graph |
| 9B | 完整导航固定目标 | `RUNTIME_PASS` | `gate9f_drlvo_monitor.log`、crowd runtime/graph |
| 9G | 完整导航固定目标、Isaac 5 RTX GUI、机器人/人群同框 | `RUNTIME_PASS` | `gate9g_drlvo_monitor.log`、`gate9g_crowd_runtime_580.log`、`gate9g_fixed_goal_gui.png` |
| 10A | Isaac 6 active lobby GUI 60 秒回归 | `RUNTIME_PASS` | `custom_people_robot_6_0_20260912_173250.log` |
| 10B | Isaac 6 原 warehouse smoke | `PASS_WITH_LIMITATIONS` | 15 Hz FAIL；10 Hz `warehouse_people_robot_6_0_20260912_174041.log` PASS |

## 4. 机器人、ROS 和 LiDAR

Gate 1/3 使用 canonical Mecanum730 articulation，并证明 `joint1`--`joint6` 的零位是直立姿态。最终 Gate 9 与成功 Isaac 6 运行逻辑一致：加载 `mecanum730_xms5_default_base.usd` 为无动力 visual，由独立动态 collision proxy 驱动底盘；机械臂保持资产 authored upright pose，完全不启用 arm dynamics，因而不会自由摆动。四个导航轮关节可识别，但最终闭环不把 visual wheel 描述成真实轮胎接触。

Gate 4 保持 host Python 3.10 与 Isaac 5 embedded Python 3.11 隔离。cp311 自定义消息 overlay 仅进入 Isaac child process；未污染 active `arena_ws`。

Gate 5 实测：

- 两个原始传感器各 2000 beams，约 15 Hz；
- 原始范围合同为 0.5--50 m；策略节点内部有效范围参数为 0.1--8 m；
- 两路 stamp 同步、严格递增；
- 静态环境有命中，机器人 self-hit 为零；
- `/scan_merged` 仅供 DR-SPAAM，不替代 DRL-VO 的两路原始输入。

## 5. 行人与 Arena 参考

实现保留 Arena Person 合同的稳定 ID、track ID、route、speed 和 collider 分层，并直接复用 Isaac 6 `custom_eng_lobby_people.yaml` 的地图范围与安全巡逻走廊。可见人物不再使用白色测试 Biped，而是来自 Isaac 6 IRA 的 8 个原始彩色 Character：施工、警察和医护人物各自加载 BaseColor、Normal、ORM 与 OmniPBR 材质。

这些 Character 是 101 joints，而旧测试动作源是 81 joints，不能直接绑定。当前启动流程复用资产已有的 `controlRig:retargetTags`、`controlRig:retargetTransforms`、`forwardAxis` 和 `upAxis`，调用 Isaac 5 自带 NVIDIA `omni.anim.retarget.core`，为每个人生成独立 101-joint `Gate6Walk`。单人和 8 人的 dependency、joint order、动画采样、evaluated skeleton pose 均有 runtime PASS。

Isaac 6 Character 的 authored forward axis 是本地 `-Y`。route yaw 以世界 `+X` 为零，因此 visual yaw 显式加 `π/2`；最终 8 人最大“身体正前方—速度方向”误差仅 `8.54e-7°`，消除了横向滑行。

8 条路线不再挤在一个房间：两条近场路线保留机器人感知压力，另外六条分布于东西走廊、北区、书店/咖啡区等 Isaac 6 已验证自由空间。最终 Gate 9G 的初始人群跨度为 `27.149 × 15.343 m`；16.5 秒内每人累计实际前进 `11.868--15.489 m`，最大位移 `1.953--13.311 m`，最小人间中心距 `3.000 m`。所有路线逐段 PhysX ground/radial clearance PASS；近场 `person_01/person_02` 均被双 LiDAR 命中，远处被墙遮挡者不伪称可见。

每人仍有一个随人物根节点运动的 kinematic capsule，作为碰撞与 LiDAR 几何权威。该实现命名为 `isaac5_isaac6_textured_character_retarget_crowd_adapter`；它不是 Isaac 6 IRA 1.6 BehaviorAgent，也不声称有人群社会力避障或人体网格接触真值。

## 6. Gate 7：感知与 SemanticCNN

### 6.1 DR-SPAAM 与 tracking

使用成功 Isaac 6 链中的原节点和 checkpoint。0.95 confidence 在当前 Isaac 5 adapter 输入上输出全空；按单变量原则只把 detector confidence 调到 0.20 后获得非空 runtime 证据。最终 detector 输入为 `/scan_merged` 360 beams、约 15 Hz，输出 stamp 与输入一致。

Tracker 输出 `odom` frame、有限位置/速度、CONFIRMED tracks 和稳定内部 ID。tracker ID 不要求等于 ground-truth ID，结果不声明检测准确率。

### 6.2 S3-Net + SemanticCNN

Gate 7.3 的 10 秒证据窗口：

- aligned scan：150/150 帧，约 15.006 Hz；
- S3-Net：窗口内 150 个 `2×2000`、`16SC1` 标签，全部匹配双雷达 stamp；
- SemanticCNN：143/143 次成功 CUDA 推理，约 15.28 Hz；
- 144 个模型 `ActuationDecision` 均可追溯到输入帧；
- 输出只到 `/isaac5/gate73/semantic_cmd_shadow`。

Gate 7.4 以旁路副本执行“正常 → 两路 stamp 同减 1 秒 → 恢复”：故障前 49 次推理；故障期成功推理为 0，53 次 `stale_or_unsynchronized_scan` 全部零速；恢复后 55 次推理。

## 7. Gate 8：DRL-VO shadow 与安全门禁

### 7.1 正常 shadow 链

链路：raw dual LiDAR → merged scan → DR-SPAAM → tracker → base DRL-VO。crowd ground-truth `/pedestrian_tracks` 被显式禁止，DRL-VO 只使用检测/跟踪结果。

Gate 8.1 的 10 秒窗口：

- raw scans：150/150，约 15.002 Hz；
- 99 个非空 track frames，47 个 CONFIRMED track samples；
- 88 个严格 19,202 维有限 observation；
- 88 次成功 CUDA inference，约 15.41 Hz；
- checkpoint 163 个权重项、2,633,797 参数；
- 原始线速度示例 0.5 m/s 被限制到 0.3 m/s；
- output 为 `/isaac5/gate81/drl_vo_cmd_shadow`，`/cmd_vel` 无发布者。

### 7.2 track freshness

旁路 relay 执行“正常转发 → 超过 0.8 秒不转发 → 恢复”：正常期 46 次推理；stale 期成功推理为 0，65 次安全停机全部零速；恢复后 38 次推理。freshness 使用本地 track receive time，同时仍以 header stamp 做 causal sample selection。

### 7.3 近障 safety veto

给 DRL-VO 专用的双雷达副本短时注入 0.5 m 近障环，DR-SPAAM/tracker 仍读取未修改的原始雷达。结果：正常期 21 个推理决策；近障期 45/45 个推理决策触发 `front_stop` 且最终线速度为零；清障后 83 个决策恢复前进。

这证明输入空间的近障 veto 和恢复逻辑；不证明真实机器人/人体发生过或避免了 PhysX 接触。

## 8. Gate 9：完整导航闭环

Gate 8 的 DRL-VO shadow topic 只接入 Isaac 5 backend-local 移动底盘 adapter。该 adapter 逐项复用成功 Isaac 6 的“canonical base visual 跟随独立动态 collision proxy”结构：

- 可见层是 `configuration/mecanum730_xms5_default_base.usd`，不运行 articulation/arm dynamics；机械臂保持 authored upright pose；
- proxy 尺寸约 `0.625 × 0.487 × 1.761 m`、质量 116.189 kg；
- 线速度限制 `[0, 0.3] m/s`，角速度限制 `[-1, 1] rad/s`；0.5 秒 watchdog 超时即零速；
- 只订阅 backend-local command topic，全局 `/cmd_vel` 发布者为 0；
- 每步测量 visual 最低点，最终全程为 `-4.29e-7--4.65e-7 m`，没有视觉下沉。

最终权威证据是 Gate 9G：真实 Isaac 5 RTX GUI、全地图 8 人彩色 crowd、双 LiDAR、DR-SPAAM/tracker、DRL-VO 与机器人同时运行。固定目标 odom `(3.5, 2.0)`，起点 `(2.0, 2.0)`，容差 `0.35 m`：

- 初始目标距离 `1.500 m`，最小/最终距离 `0.334 m`；
- 双雷达实测 `15.006/15.006 Hz`，每帧各 2000 beams；
- 154 次成功 CUDA inference，约 `15.312 Hz`；
- 170 个带人物的 track frames；
- 93 个 `goal_reached` decision，之后 93 个 command 全为零；
- robot travel `1.170 m`，最终 odom `(3.1689, 2.0449, 0.00921)` 且 twist 为零；
- 人与机器人最小中心距 `0.970 m`；
- 机械臂固定朝上且 robot visual 全程贴地；crowd、路线、朝向、材质、capsule、LiDAR、ROS 和 teardown 全部 PASS。

彩色人物使 60 Hz 同步 GUI 渲染的首轮 LiDAR 墙钟速率降到 `13.14 Hz`。单变量修复只把 viewport 渲染节奏改为 30 Hz；物理仍为 60 Hz、传感器仍为 15 Hz，最终恢复上述约 15.006 Hz。该固定目标场景证明一次有界、可停止的端到端功能闭环，不是长时统计 benchmark，也不证明复杂拥挤路线的成功率。

## 9. Gate 10：Isaac 6 回归

### 9.1 active lobby GUI 60 秒

未修改 Isaac 6 launcher/runtime，使用当前 active lobby、20 名 IRA 行人、dynamic Mecanum proxy、Gazebo Social adapter、ROS 和 PhysX 双 LiDAR 实跑 GUI 60 秒：

- `WAREHOUSE_PEOPLE_ROBOT_RESULT.status=PASS`，按 `duration_reached` 自然结束；
- wall `60.056 s`、timeline `60.011 s`，平均 app `8.44 FPS`；`minFrameRate=10` 不是 GUI FPS 保证；
- 20/20 行人运动，无 runtime reset；机器人保持静止且落地姿态稳定；
- 760 对 `2×2000` LiDAR，仿真频率 `15.0 Hz`，漏采 0；墙钟吞吐约 `12.70 Hz`；
- 未出现 native crash、SceneDB crash、Xid 或残留 Isaac/ROS 进程。

保留现有质量告警：95 个 frame 出现至少一对行人视觉重叠，最小中心距约 `0.069 m`；3 个行人累计 6 次 sustained free-space intrusion。它们没有触发当前 launcher 的总失败条件，且不是本次 Isaac 5 backend 改动造成，但不能据总 `PASS` 宣称社会导航质量通过。

### 9.2 原 warehouse 最小 smoke

第 1 次保持当前默认 `rplidar_s2e / 2000 beams / 15 Hz`：场景、3 名 IRA 行人、机器人、ROS 与 RTX 双雷达均到 READY，3/3 行人运动且 52 对扫描无丢配对，但仿真频率仅 `9.684 Hz`，被原 launcher 的 rate gate 正确判定 FAIL。

第 2 次只将 `ISAAC_LIDAR_RATE_HZ` 改为 10：5 秒 smoke 通过，平均 app `55.64 FPS`，3/3 行人运动，51 对 RTX 扫描实测 `10.0 Hz`、墙钟 `10.22 Hz`、无丢配对，进程自然退出。因此原场景回归结论是 `PASS_WITH_LIMITATIONS`，不是 15 Hz PASS。

## 10. 新增可复现实验入口

- `isaac_sim/backends/isaac5/launch/validate_gate71_drspaam.sh`
- `isaac_sim/backends/isaac5/launch/validate_gate72_tracking.sh`
- `isaac_sim/backends/isaac5/launch/validate_gate73_semantic.sh`
- `isaac_sim/backends/isaac5/launch/validate_gate74_freshness.sh`
- `isaac_sim/backends/isaac5/launch/validate_gate81_drlvo_shadow.sh`
- `isaac_sim/backends/isaac5/launch/validate_gate82_drlvo_freshness.sh`
- `isaac_sim/backends/isaac5/launch/validate_gate83_drlvo_front_veto.sh`
- `isaac_sim/backends/isaac5/launch/validate_gate9_closed_loop_smoke.sh`
- `isaac_sim/backends/isaac5/launch/validate_gate9_fixed_goal.sh`
- `isaac_sim/backends/isaac5/launch/validate_gate9_fixed_goal_gui.sh`

所有 launcher 使用独立 `ROS_DOMAIN_ID`、只清理自身进程组、一次只启动一个 Isaac 实例，并保留 backend-local graph/model hash/monitor/runtime 日志。

## 11. 保留未修改的内容

- `/home/user/isaacsim/5.1.0` 和 `isaac_sim/isaacsim-6.0.1`
- canonical Mecanum730 USD 与其资产包
- Isaac 6 launcher/runtime 及用户现有修改
- active `arena_ws`
- 所有 checkpoint/model 权重
- driver、APT、DKMS、kernel、Vulkan 和系统 shader/cache
- 无 reset、checkout、clean；不覆盖无关 dirty worktree

### 11.1 最终回归与失败账本

最终静态回归：backend contract `6 passed`；Isaac 行人 free-space/social/steering/config/Gazebo integration 相关测试 `79 passed`；所有 runtime Python `compileall`、所有 launcher `bash -n`、`git diff --check` 均通过。结束后没有残留 Isaac、crowd、DR-SPAAM、DRL-VO 或 scan-merger 进程。

保留失败尝试而不覆盖历史：

- 白色人物来自 `biped_demo_meters.usd` 测试模型；改为 8 个 Isaac 6 Character 并用 NVIDIA retarget core 后解决；
- 彩色 crowd 首轮 60 Hz GUI 渲染使 LiDAR 墙钟速率 `13.14 Hz`，低于 13.5 Hz 门限；仅将 GUI render 改为 30 Hz 后恢复约 15 Hz，物理和传感器频率未改；
- 全地图路线首轮运行完成但 `numpy.bool_` 不能 JSON 序列化；转换为原生 `bool` 后同一门禁 PASS；
- 彩色 Character authored forward axis 为 `-Y`，沿用旧 Biped `+X` yaw 造成横移；加入 `+π/2` 后朝向误差门禁 PASS；
- 早期 kinematic capsule 在 `World.reset()` 前切换模式导致 PhysX velocity write 告警；改为 reset 后再切 kinematic；
- 早期闭环用完整 articulation 做 visual 跟随出现视觉下沉；改为复用 Isaac 6 base visual + dynamic proxy，最终最低点误差小于 `5e-7 m`。

## 12. 原迁移结论

当前可以称为“Isaac Sim 5.1 active lobby 的可复现功能迁移完成”：场景、canonical robot visual、固定朝上机械臂、ROS、2×2000 @ 15 Hz、8 名全地图彩色动画 Character、DR-SPAAM/tracker、S3-Net/SemanticCNN、DRL-VO、freshness/safety gates、闭环移动和固定目标均有本轮 runtime 证据，且 Isaac 6 active scene 60 秒回归未被破坏。

不能称为严格版本等价或正式社会导航 benchmark 全通过：Isaac 5 虽复用 Isaac 6 Character 与 NVIDIA retarget core，仍不是 IRA 1.6 BehaviorAgent；人体接触仍是 capsule proxy；当前全地图路线没有人物—人物 Social Force 动态避障；DR-SPAAM 需要 0.20 threshold 才有当前非空检测；Isaac 6 warehouse RTX 15 Hz 本轮失败；active lobby 20 人 Social adapter 仍有重叠/侵入告警。后续若要继续，应建立独立的长时多 seed 评价任务，而不是再扩大这次迁移门禁的结论。

## 13. Isaac6 退役前硬阻塞 A/B 更新（2026-09-12）

### A. 本地人物资产闭包

`isaac_sim/backends/isaac5/assets/people/` 现包含且仅包含运行所需的八个彩色 Character USD、各自 BaseColor/Normal/ORM 纹理、walk `skelanim` 与 81-joint Biped retarget source；无指向 Isaac6 的软链接。`asset_manifest.json`、`SOURCE.md`、`SHA256SUMS`、本地 NVIDIA license 副本与 Isaac5 dependency audit 均已留下。删除了 78 个非运行时 thumbnail 文件（3,023,746 bytes）。本机安装携带的 license 已被记录，但未找到单独的再发布许可；此包只作本地版本化运行，不得由本报告推导出发布授权。

实际 Isaac5 audit 对 10 个 USD 输入报告 `0 unresolved`；所有 Character 的 BaseColor/Normal/ORM 都解析到 backend-local 路径，唯一外部 shader 是 Isaac5 自带 `OmniPBR.mdl`。单人 runtime、8 人 GUI crowd 和完整 Gate9G 均以本地资产 PASS：101-joint 动画、朝向、材质、kinematic capsule、双 2000-beam/15 Hz LiDAR、DR-SPAAM/tracker、DRL-VO、固定目标零速退出与 teardown 均未回退。

### B. 通用入口默认 Isaac5

通用 DRL-VO、DR-SPAAM smoke/tracking/stress、pedestrian validation、single-person benchmark、v7 SemanticCNN demo 和 ROS perception visualization 现在默认 `ISAAC_BACKEND=isaac5`。`run_production_demo.sh` 明确选择互斥的 DRL-VO 或 SemanticCNN chain、拒绝第二个 Isaac 实例并复用 Gate 7--9 已验证的实际场景/crowd/ROS/LiDAR/perception/policy 链。设置 `ISAAC_BACKEND=isaac6` 时各原脚本继续原样执行；名称明确含 `isaac_6_0` 的 launcher 未修改。

默认 `run_custom_people_drlvo_demo.sh` 已完成一次有界 GUI smoke，日志为 `isaac_sim/backends/isaac5/generated/retirement_audit_20260912/default_drlvo_gui_smoke.log`，实际 dispatcher 目标为 Gate9G 并得到 `GATE9g_DRLVO_RESULT=PASS`。该链只启动 DRL-VO（没有 SemanticCNN 驱动），保留 backend-local command topic、独立 ROS domain 与结束时零速门禁。

### 证据边界

runtime/launch 的静态直接 `isaacsim-6.0.1` / `assets-6.0.1` 引用为 0。完整 Gate9G 在无 ptrace 下 PASS。对 headless 8-person Isaac5 crowd 的 `strace -f -e trace=%file` 记录共 233,043 行，未发现任何对两条禁止 Isaac6/Assets6 路径的打开；该 audit 本身亦为 PASS。对 GUI Gate9G 的首个 strace 尝试因 ptrace 干扰 Kit 子进程启动而失败，日志保留但不用于功能失败结论。以上可支持“不删除 Isaac6 文件时，Isaac5 runtime 不读取该安装/资产目录”的结论；不改变 Isaac6 regression 的保留状态，也不等同于资产发布许可、严格版本等价或长期社会导航结论。

## 14. 20 人实时步态与游标修正（2026-09-13）

用户 GUI 观察发现人物虽整体位移，但长时间保持近似同一动作。旧门禁只比较
retarget 动画在两个时间点的骨骼矩阵，不能证明运行时持续换步；同时旧实现按
请求时长预铺动画样本，600 秒、20 人预览会产生不必要的初始化和内存开销。

当前 `RuntimeGaitDriver` 只缓存一轮 80 个 retarget 后姿态，清除全局时间轴样本，
再按每名行人的真实累计位移选择运行时骨骼姿态。慢走会降低步态推进速度，停下
则停止换步，长跑动画内存不再随 duration 增长。单人 6 秒 headless 记录 179 次
姿态变化；20 人 30 秒 RTX GUI 中 20/20 均使用全部 80 个姿态，朝向误差约
`1.21e-6°`，且截图和 teardown 均通过。

路线游标同时复用 Isaac6 `PatrolPolylineCursor` 的 reached-or-passed 与 lookahead
checkpoint 语义，修复锐角处提前朝下一点转弯、却永远进不了旧点 0.38 m 半径的
绕圈问题。新增纯 Python 回归后共 7 项测试通过；真实 8 秒 Isaac5 20 人测试中
20/20 均位移，最小中心距 `0.634 m`、双 LiDAR 调度 `15 Hz`，总体只因部分人物
stall ratio 超 0.50 而保持 FAIL。

这轮还核对了本地 Arena/HuNav：它的多行人场景采用持续目标、Behavior Tree 与
lightSFM，值得复用目标状态和路线相位；其 Gazebo 可视层仍是临时 `walk.dae` 或
简化腿摆，不优于 Isaac Character 骨骼。Isaac6 的 inflated-grid/A* 生成器已生成
并静态通过 20 条、每条约 46--121 m 的候选闭合路线，但离线实验确认只替换路线
仍不能解决共享走廊拥堵。当前剩余问题是明确的路线冲突/让行状态，不再是动画
播放、横向朝向或 waypoint 永不确认。完整证据与边界见
`isaac5_20_person_social_navigation_report.md`。
