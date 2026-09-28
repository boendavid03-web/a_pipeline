# ARENA + ISAAC SIM 5.1 本地只读审计

审计日期：2026-09-28  
审计范围：本机 Arena、Isaac Sim 5.1、相关 overlay/shim、Downloads、历史日志与报告。  
操作边界：本轮未启动 Isaac/Arena/Gazebo/ROS，未安装、构建、解压、切换 Git、修改既有文件或清理进程；仅新建本报告。

证据等级：`CONFIRMED_BY_LOG` 表示有本机运行日志或结构化数据；`REPORTED_NOT_CURRENTLY_VERIFIED` 表示历史报告有结论但本轮未复跑；`SOURCE_EXISTS_ONLY` 表示仅静态源码/配置存在；`UNKNOWN` 表示本轮证据不足。

# 1. EXECUTIVE SUMMARY

1. **最快答案：可以复用既有入口打开一个小规模 Arena + Isaac Sim 5.1 多人场景；它历史上已真实跑通，但本轮按要求没有启动复验。**
2. 已验证链是 Arena 原始 launch → TaskGenerator → `isaacsim_msgs` services → Isaac 5.1 People/NavMesh；不是 HuNav/LightSFM。
3. 历史 `map_empty` 运行证明 GUI、地图/障碍、Jackal、ROS bridge、人物生成及人物连续运动；状态为 `PASS_WITH_LIMITATIONS`。
4. 当前活动 `arena-isaac` checkout 是 detached `a4beefe...`（Isaac 6 系列历史），并非官方 `arena5-isaac5.1.0` 分支；且有一个必须保留的 dirty 文件。
5. 官方 `arena5-isaac5.1.0` 源码完整保存在 Downloads ZIP/backup，但其 pedestrian API 依赖的 `arena_people_msgs` 本机不存在，不能把该分支说成即刻可运行。
6. 本机有大量 Arena 地图和 3/5/7/10/17/30 人 scenario JSON；当前 host runtime overlay 实际只带 `map_empty` 和定制的 `map_highly_social`。
7. 10 人 `map_highly_social` 已配置，但历史续测被 Isaac 4→5 旧传感器 OmniGraph 节点阻塞，不能宣称“一条命令稳定看到 10 人并完成导航”。
8. Isaac 5.1 自带 People/NavMesh 扩展和命令能力，但安装内未找到离线、自包含、可直接打开的完整多人 demo；人物资产依赖项目本地副本或在线资产根。
9. Gazebo Harmonic Hospital 17 人全部运动、Nav2 到达是另一条已验证路线，绝不能当作 Isaac 证据。
10. 当前还有长期存活的 Gazebo/HuNav/Isaac5 进程；下一次启动前应先在获得授权后核对归属并选干净隔离窗口，本审计未杀进程。

# 2. LOCAL COMPONENT INVENTORY

| Component | Path | State | Role / finding |
|---|---|---|---|
| Arena Harmonic workspace | `/home/user/arena_full_ws` | built + historically run | Gazebo Harmonic/HuNav/Nav2；与 Isaac 分开 |
| Active Arena/Isaac source | `/home/user/navigation_project/a_pipeline/isaac_sim/arena_ws` | partial build/install; protected | 活动 Arena、TaskGenerator、`arena-isaac` 源码 |
| Active `arena-isaac` repo | `.../arena_ws/src/arena/isaac` | detached + dirty | HEAD `a4beefe...`; `run_isaacsim.py` modified |
| Host overlay | `/home/user/arena_isaac5_host_overlay_ws` | present | Host Humble CPython 3.10：Arena setup、Jackal、`task_generator_msgs` |
| CPython 3.11 factory | `/home/user/arena_isaac5_py311_factory` | present | Isaac child 所需 cp311 ROS/native packages 与历史 gates |
| Runtime shim | `/home/user/arena_isaac5_runtime_shim` | present + historically verified | 为 Isaac 5.1 child 注入 cp311 overlay；不替代 host overlay |
| Host runtime wrapper | `/home/user/arena_isaac5_host_runtime/run_gui.bash` | custom, source verified | 组织双 ABI 环境并调用原 Arena launch |
| Isaac Sim | `/home/user/isaacsim/5.1.0` | complete runtime | `5.1.0-rc.19+release.26219.9c81211b.gl`, Python 3.11.13 |
| Isaac people assets | `isaac_sim/backends/isaac5/assets/people/characters` | present, 8 variants | 项目本地资产闭包，不属于原始 Isaac 安装 |
| Official 5.1 archive | `/home/user/Downloads/arena-isaac-arena5-isaac5.1.0.zip` | exact source archive | 与 `origin/arena5-isaac5.1.0` 内容匹配 |
| Official 5.1 backup | `isaac_sim/arena_isaac5_backup` | source snapshot | 非 active checkout / 非 active runtime |

活动 `arena_ws/install` 仅有 15 个顶层包目录（包括 `arena_bringup`、`arena_simulation_setup`、`task_generator`、`isaacsim_msgs`、`ros2isaacsim` 和部分 Nav2/Jackal 包），不能把源码中约百个包的存在当成全部已安装。双 overlay 的价值在于明确隔离 Host Humble CPython 3.10 与 Isaac child CPython 3.11；普通 `PYTHONPATH` 不能修复两套 native ROS type support ABI。

# 3. ARENA ROUTES

```text
A. Arena-Rosnav -> Gazebo Harmonic -> HuNav/LightSFM -> Jackal/Nav2
   状态：CONFIRMED_BY_LOG（Hospital 17 人、独立 Nav2 到达）

B. Arena-Rosnav -> arena-isaac -> Isaac 5.1 ROS bridge/services
   -> Isaac People + NavMesh + Animation Graph
   状态：CONFIRMED_BY_LOG（map_empty 小规模人物）；10 人门禁未通过

C. Arena/Isaac 6.x source/runtime experiments
   状态：部分源码/独立运行存在；不是本报告的 Isaac 5.1 路线

D. HuNav standalone -> adapter -> Isaac visualization/native controller
   状态：独立实验路线；不是 Arena 原生 People 路线

E. a_pipeline custom Isaac backend -> local assets/social controllers/UDP or ROS boundary
   状态：已有 10/20 人等实验；不是 Arena benchmark/runtime
```

关键边界：A 的 17 人成功不能证明 B；D/E 的多人成功也不能证明 Arena TaskGenerator、Arena scenario 和 Arena robot task 已闭合。

# 4. ARENA-ISAAC 5.1 VERSION

| Item | Finding |
|---|---|
| Repository | `https://github.com/Arena-Rosnav/arena-isaac.git` |
| Active path | `/home/user/navigation_project/a_pipeline/isaac_sim/arena_ws/src/arena/isaac` |
| Active HEAD | `a4beefe0203ec8a75d65d0ab70496b5e2c400605` (2025-07-10, `full robot localization and transforms`) |
| Active branch | detached HEAD；该 commit 位于本地 `isaac6.0.0` 历史，不是 5.1 branch |
| Dirty state | `M ros2isaacsim/ros2isaacsim/run_isaacsim.py` |
| Dirty change | 使用 `isaacsim.asset.importer.urdf` 替代旧 `omni.importer.urdf`，并移除错误的 `_urdf.ImportConfig()`；本轮未改 |
| Official 5.1 ref | `origin/arena5-isaac5.1.0` = `16b8e3416517d8c3dc1b5038df4fe11b9a6df46c` |
| 5.1 ZIP | `/home/user/Downloads/arena-isaac-arena5-isaac5.1.0.zip`，83,500 bytes |
| ZIP verification | 90 个文件的聚合 SHA-256 与该 Git ref 内容一致 |
| Other refs | `origin/arena5`=`ba0c95f...`; `origin/isaac6.0.0`=`0963896...` |

因此：本机**有官方 5.1 分支源码**，但**活动 checkout 不是它**。活动运行链是用外置 host/cp311 overlay 和兼容性修改把活动代码带到 Isaac 5.1；不能因 ZIP 存在就称当前工作树是官方纯净 5.1 checkout。

# 5. EXISTING WORLDS

活动 Arena simulation-setup 源码中存在：`factory`、`generated`、`hospital`、`house17`、`ignc`、`map_empty`。旧内容还包括 corridor、hawker centre、hospital large/small、NUS COM1/2/3、rooms、warehouse、bookstore、outdoor 等。

| Arena map/world | Isaac USD exists? | Local path/type | Robot-ready | People-ready | Level |
|---|---|---|---|---|---|
| map_empty | 无单一预制 world USD | Arena map/walls/obstacle descriptors，运行时构造 | 历史是 | 历史是 | 4 |
| map_highly_social | 无单一预制 world USD | host overlay 中由 map_empty 派生 | 配置是，完整门禁否 | 10 人 JSON 有，完整门禁否 | 4（未验收） |
| hospital | 未找到 Arena→Isaac 完整 USD | Arena 2D map + scenario；Gazebo route 已验证 | Gazebo 是；Isaac 未验证 | 17 人 scenario 有 | 4（配置）/5（仅 Gazebo） |
| factory | 未找到完整 Isaac world USD | Arena source map/scenarios | source only | 2/5 人 JSON | 4（配置） |
| generated | 未找到完整 Isaac world USD | Arena source map/scenarios | source only | 3/7/10 人 JSON | 4（配置） |
| house17 | 未找到完整 Isaac world USD | Arena source map/scenarios | source only | 0/3/7/10 人 JSON | 4（配置） |
| ignc | 未找到完整 Isaac world USD | Arena source map/scenarios | source only | 默认 3 人 | 4（配置） |
| HuNav wrapper hospital/office/warehouse | 是 | 仅 Downloads ZIP 内的独立 HuNav-Isaac wrapper | 非 Arena robot task | 9/8/5 agents 配置 | 3（archive only） |

这里的 Arena Isaac 架构通常是把 2D Arena map、墙和实体通过服务动态导入/生成，而不是要求每个 Arena world 都已有一个完整 USD。因此“没有 Hospital USD”不等于“没有 Hospital scenario”；但也不能把 Gazebo SDF/2D map 写成可直接打开的 Isaac scene。

# 6. EXISTING MULTI-PERSON SCENARIOS

| Scene/config | Source | Simulator | Human backend | Count | Map | Direct launch now? | Missing / caveat |
|---|---|---|---|---:|---|---|---|
| map_empty random/default | active + host overlay | Isaac 5.1 | Isaac People/NavMesh | 历史小规模（至少 2） | map_empty | **历史可；本轮未复验** | 当前有冲突风险进程；Nav2/RViz 有历史限制 |
| map_highly_social/default | host overlay custom | Isaac 5.1 | Isaac People/NavMesh | 10 | map_empty-derived | 配置可推导，非验收 PASS | 旧 sensor OmniGraph、prim/初始化问题需先复验 |
| factory/default | Arena source | intended multi-sim | selected backend | 2 | factory | 当前 overlay 非直接 | world 未进当前 host runtime overlay |
| factory/default1 | Arena source | intended multi-sim | selected backend | 5 | factory | 当前 overlay 非直接 | 同上 |
| generated/default | Arena source | intended multi-sim | selected backend | 3 | generated | 当前 overlay 非直接 | 同上 |
| generated/blocked_corridors | Arena source | intended multi-sim | selected backend | 7 | generated | 当前 overlay 非直接 | 同上 |
| generated/evacuation, highly_social | Arena source | intended multi-sim | selected backend | 10 | generated | 当前 overlay 非直接 | 同上 |
| hospital/default | Arena source | Gazebo 已验证；Isaac source-compatible | HuNav on Gazebo / People on Isaac selection | 17 | hospital | Gazebo 是；Isaac 否 | 无当前 Isaac runtime overlay/实跑证据 |
| house17 scenarios | Arena source | intended multi-sim | selected backend | 0/3/7/10 | house17 | 当前 overlay 非直接 | 未在 Isaac5 实跑 |
| NUS COM1 scenarios | old Arena content | source only | unknown in current route | up to 30 | arena_nus_com1 | 否 | 旧内容、非当前 overlay |
| floor obs20 | old Arena content | source only | unknown in current route | up to 20 | floor | 否 | 文件名/配置不构成运行证据 |
| HuNav wrapper Hospital | Downloads archive | Isaac wrapper (upstream tested 4.5) | HuNav | 9 | bundled USD | 否 | 未解压/未构建/未验证 5.1，且不是 Arena |
| HuNav wrapper Office | same | same | HuNav | 8 | bundled USD | 否 | same |
| HuNav wrapper Warehouse | same | same | HuNav | 5 | bundled USD | 否 | same |

等级定义：Level 1 只有 map/world；Level 2 加 robot task；Level 3 加 pedestrian config；Level 4 同时有 map+robot+pedestrian+task 配置；Level 5 为实际闭合的 benchmark/runtime。上表多数 Arena JSON 达到**配置意义的 Level 4**，但只有 Gazebo Hospital 有本机 Level 5 证据；Isaac `map_empty` 是实际运行链证据，但完整 Nav2 benchmark 仍未闭合。

问题“启动即看到 5/10/20 人”：

- 5 人：源码有 `factory/default1`，但不在当前最小 host runtime overlay，未确认一条命令直接运行。
- 10 人：`map_highly_social/default.json` 已在 overlay，最接近直接运行；历史 Gate 仍 BLOCKED，不能承诺稳定开箱即用。
- 20 人：Arena 旧 scenario 和 a_pipeline custom backend 有相关材料/结果，但当前 Arena+Isaac5 原生运行链没有已验收的 20 人一键 demo。

# 7. ISAAC PEOPLE / NAVMESH DEMOS

Isaac 5.1 安装确认存在：

- `extscache/omni.anim.people-0.7.9+107.3.3`
- `omni.anim.navigation.core-107.3.8` 及 bundle/UI/schema
- `exts/isaacsim.ros2.bridge`

People 扩展源码支持 command file、dynamic avoidance、NavMesh，以及 `goto`、`idle`、`look_around`、`queue`、`sit`、`talk`、`talkwith` 等命令。这证明**能力存在**，不证明一个完整 demo 已本地封装。

本轮在 Isaac 5.1 安装内未找到同时满足以下条件的现成离线 demo：预置 world + 多人物资产 + NavMesh + command file + 一条本地启动命令。扩展配置中的 character asset root 不是一个自包含本地人物库；runtime shim 还指向在线 Isaac 5.1 asset root。可用的 8 个本地人物 USD 变体位于 a_pipeline 项目资产目录，是以前为当前路线准备的本地闭包。

结论：**不能把“People 扩展已安装”写成“Isaac 原生多人 demo 可离线一键打开”。** 若允许 UI 手动配置或在线资产访问，原生 People 可以搭场景；这不符合本轮寻找“现成直接开”的严格标准。

# 8. PEDESTRIAN CONTROL PIPELINE

活动 Arena/Isaac 调用链（以源码为准）：

```text
arena_bringup/arena.launch.py
  -> Isaac child: $ISAAC_PATH/python.sh + ros2isaacsim/run_isaacsim.py
  -> TaskGenerator 读取 Arena world/scenario
  -> IsaacSimulator / IsaacHumanSimulator
  -> isaacsim_msgs: import_usd, spawn_wall, spawn_pedestrian,
                    move_pedestrians, update/remove entities
  -> ros2isaacsim ROS services
  -> Isaac People Person state machine
  -> NavMesh path + dynamic avoidance
  -> NVIDIA Animation Graph Walk/Action
```

`task_generator/.../human/isaac.py` 选择 character variant，并以 `Person(initial, goal, velocity)` 请求 `isaac/spawn_pedestrian`；`task_generator/.../sim/isaac_simulator.py` 管理 Isaac entity/service clients；Isaac child 启用 People/navigation extensions，构建 NavMesh 并提供服务。

该链没有 HuNav/LightSFM 社会力。人物 pose 的权威 writer 是 Isaac People/NavMesh 状态机；Arena TaskGenerator 负责生成任务和生命周期。后续若研究确实需要 HuNav，应替换/适配 human backend，而不是在第一阶段同时让两套控制器写人物状态。

# 9. arena_people_msgs STATUS

对 `/home/user` 的 source/build/install/archives/Downloads 进行了文件与 ZIP 内容搜索：**未找到 `arena_people_msgs` package、`package.xml` 或其 msg/srv 定义**。找到的只是源码中的导入/文字引用。

官方 `arena5-isaac5.1.0` 分支使用：

- `arena_people_msgs.msg.Pedestrian`
- `SpawnPedestrians`
- `MovePedestrians`
- `UpdatePedestrians`

因此该官方分支的 pedestrian service 路径在本机不能按原样 build/run。当前活动新代码绕开了这个特定缺口，改用现有 `isaacsim_msgs` services；这就是历史实际 Arena+Isaac5 运行能成功生成 People 的原因之一。

不依赖 `arena_people_msgs` 的路径包括：当前活动 `isaacsim_msgs` Arena adapter、Isaac UI/People 原生能力、独立 a_pipeline backend、以及 HuNav wrapper。但只有第一条属于本机已历史验证的 Arena+Isaac5 小规模链。

# 10. HISTORICAL WORK

| Claim | Classification | Evidence / precise boundary |
|---|---|---|
| Arena original entry opened Isaac 5.1 GUI | CONFIRMED_BY_LOG | `arena_isaac5_runtime_report.md`; real 5.1 GUI/Vulkan/window |
| map_empty walls/shelves, Jackal, Isaac people created | CONFIRMED_BY_LOG | same report and Kit/ROS logs |
| Isaac People actually moved | CONFIRMED_BY_LOG | one actor 19.408852 m; delivery actors 18.024 m and 4.589 m; reached Idle/Walk=0 |
| ROS bridge/services/control chain | CONFIRMED_BY_LOG | 11 Isaac services; clock/TF/odom/joint/LiDAR; cmd_vel moved Jackal |
| Full Arena+Isaac autonomous Nav2 success | **not confirmed** | historical `bt_navigator` exit -11; later path/control evidence ended in follow_path timeout |
| 10 people + Nav2 fully accepted in Isaac5 | **not confirmed / blocked** | 2026-09-16 report stops on legacy `omni.isaac.sensor.IsaacReadIMU`; later Gate 22/23 partial-load evidence still not full acceptance |
| Hospital 17 people all move | CONFIRMED_BY_LOG **in Gazebo Harmonic only** | `/home/user/arena_full_ws/ARENA_LONG_RUN_STATE.md` and related evidence |
| Hospital Nav2 goal reached | CONFIRMED_BY_LOG **in separate Gazebo probe only** | `NavigateToPose SUCCEEDED`, not Isaac |
| a_pipeline custom Isaac 20-person runs | CONFIRMED/REPORTED in custom backend scope | useful fallback/reference; not Arena runtime |
| `arena5-isaac5.1.0` archive exists | source-confirmed | exact branch archive; not runtime proof |

本轮发现仍在运行的相关旧进程包括：PGID `2814003` 下多批 Gazebo/Arena TaskGenerator、PGID `2875987` 的 HuNav host adapter，以及 PID `3985214` 的 Isaac5 `validate_crowd.py --headless --pedestrian-count 10`。未检查其业务归属之外的状态，未发送信号。它们意味着现在直接再开 ROS/Arena/Isaac 可能发生资源、domain、GPU 或进程归属冲突。

# 11. EXISTING CUSTOM CODE

| Item | Provenance | Purpose | Validation |
|---|---|---|---|
| active `arena-isaac` repo | official upstream + one local dirty compatibility change | Arena Isaac service runtime | small map_empty historical run |
| `/home/user/arena_isaac5_host_overlay_ws` | local isolated workaround | host cp310 packages/config/world subset | historically used |
| `/home/user/arena_isaac5_py311_factory` | local isolated build factory | cp311 native ROS packages for Isaac child | ABI probes and real run passed |
| `/home/user/arena_isaac5_runtime_shim` | local wrapper | sources cp311 overlay then executes Isaac5 Python | `ISAAC5_SHIM_READY 3.11.13`; real run |
| `/home/user/arena_isaac5_host_runtime/run_gui.bash` | local wrapper | assembles env and calls original Arena launch | source + historical runtime report |
| `map_highly_social` overlay | local scenario packaging | expose 10-person scenario without touching protected tree | configured; full gate blocked |
| `isaac_sim/backends/isaac5/runtime/hunav_isaac_adapter.py` | a_pipeline custom | HuNav host ↔ Isaac backend | separate from Arena |
| a_pipeline UDP/JSON/Isaac backends | custom | Isaac experiments and DRL/social navigation | separate evidence; not Arena |

The host wrapper accepts `ARENA_WORLD`, `ARENA_TM_OBSTACLES`, `ARENA_TM_ROBOTS`, and optional `ARENA_SCENARIO_FILE`; when a scenario is supplied it waits for `/task_generator_node` and sets `task.scenario.file`. This is a prior workaround around launch-argument forwarding, not upstream Arena behavior.

# 12. DOWNLOAD ARCHIVE INVENTORY

| File | Size | Top-level / inferred version | Contents | Current value |
|---|---:|---|---|---|
| `arena-isaac-arena5-isaac5.1.0.zip` | 83,500 B | same-named folder; exact ref `16b8e...` | official 5.1 adapter source | High for provenance; blocked by missing `arena_people_msgs` |
| `arena-simulation-setup*.zip` | 2,031,417,461 B | Arena simulation setup | maps, scenarios, entity assets/USD | High; source/assets archive |
| `arena-rosnav-master.zip` | 855,229 B | Arena-Rosnav master | framework source | Medium; compare only, not active runtime |
| `arena-rosnav-humble*.zip` | 326,508 B each | Humble archive | ROS 2 Arena source | Medium |
| `Hunav_isaac_wrapper-2.0.zip` | 55,425,507 B | robotics-upo wrapper | hospital/office/warehouse USD + 9/8/5-agent YAML | Potentially high, but tested upstream on Isaac 4.5 and not Arena |
| `hunav_sim-2.0.zip` | 78,605,202 B | HuNav 2.0 | HuNav ROS packages/config | Useful backend/unit-test source, not required for first Arena People run |
| other `hunav_sim` archive | 167,131,478 B | HuNav source snapshot | similar human-simulator material | Same |
| `lightsfm-master.zip` | 54,536 B | LightSFM master | social-force library | already useful in Harmonic/standalone, not first Arena Isaac step |
| `hunav_gz_plugin*.zip` | 24,239 B | Gazebo plugin | Gazebo integration | irrelevant to Isaac People path |
| `arena-tools*.zip` | 82,621,722 B | Arena tools | tool assets/source | secondary |
| `arena-evaluation*.zip` | 42,348 B | Arena evaluation | evaluation source | useful later; not scene runtime |
| `CrowdNav*.zip` | 19,098,625 B | CrowdNav | planner/policy research code | not an Isaac scene |
| `Nav2-PaS*.zip` | 16,526,098 B | PaS/Nav2 | navigation/planner code | not a human scene |
| `people*.zip` | 1,101,375 B | ROS people packages | messages/perception | not Isaac character assets; not `arena_people_msgs` |
| Isaac 5 standalone ZIP | 8,768,419,777 B | Isaac Sim 5.x | runtime installer/archive | runtime already installed |
| Isaac 6 assets/standalone archives | ~80 GB / ~13 GB | Isaac 6 | Isaac 6 runtime/assets | preserve separately; do not use for 5.1 closure |

Archive contents were listed in place; none were extracted during this audit.

# 13. TOP 3 DIRECTLY REUSABLE DEMOS

## OPTION 1 — 最容易：历史已验证的 Arena `map_empty` + Isaac 5.1 People

这是“既是 Arena，又是 Isaac 5.1，又真实有人物运动”的最高置信度路径。它通过现有 local wrapper 调用原 Arena launch，不需要 HuNav。

Source-derived wrapper invocation（**VERIFIED FROM SOURCE；本轮未执行**）：

```bash
ARENA_WORLD=map_empty \
ARENA_TM_OBSTACLES=random \
ARENA_TM_ROBOTS=explore \
/home/user/arena_isaac5_host_runtime/run_gui.bash
```

预期：GUI、动态构造 map_empty、Jackal、Isaac People/NavMesh。边界：人物数量由 task 模式/配置决定；它不是保证 5/10/20 人的固定 benchmark；Nav2 完成也未通过历史整体验收。

## OPTION 2 — 最接近固定多人 Arena：`map_highly_social` 10 人

现有 host overlay 已包含 map 与 10-person `default.json`，wrapper 已支持 scenario file。以下是从 wrapper 机制推导的形式（**INFERRED；必须先核对实际 scenario 绝对路径，本轮未执行**）：

```bash
ARENA_WORLD=map_highly_social \
ARENA_TM_OBSTACLES=scenario \
ARENA_TM_ROBOTS=scenario \
ARENA_SCENARIO_FILE=/absolute/path/to/map_highly_social/scenarios/default.json \
/home/user/arena_isaac5_host_runtime/run_gui.bash
```

它是最接近“一启动看到 10 人”的 Arena 方案，但历史结论是 BLOCKED，不是 PASS；不能在不复跑的情况下声称人物和 Nav2 已完整闭合。

## OPTION 3 — 最接近后续社会导航研究：HuNav Isaac wrapper bundled scenes

Downloads 内已有 Hospital/Office/Warehouse USD 和 9/8/5-agent 配置，避免重新建这三张地图。但它只是 archive/source candidate，上游目标版本为 Isaac 4.5，本机尚未解压到独立工作区、构建或验证 5.1，而且它不是 Arena TaskGenerator。它适合作为 Arena 原生 People 不满足社会动力学时的后备，不应抢在 Option 1/2 前面。

补充：a_pipeline custom Isaac5 10/20-person backend已有更强的多人实验材料，但因“不属于 Arena”没有排进前三的 Arena 复用主线；它仍是故障隔离与研究对照的可用 fallback。

# 14. MINIMUM BLOCKERS

| Target | Minimum blocker |
|---|---|
| 先看到小规模 Arena+Isaac5 人物 | 无已知代码缺口；需要一个干净、获授权的运行窗口，并先处理/隔离当前长期存活进程，随后复验现有 wrapper |
| 固定 10 人 Arena+Isaac5 | 复验并确认遗留 sensor OmniGraph/prim/初始化问题是否仍阻塞；不能用 2026-09-16 partial launch 代替新运行证据 |
| 官方 `arena5-isaac5.1.0` 原分支 | 本机缺 `arena_people_msgs` 定义/类型支持；该 checkout 也不是 active branch，且原脚本路径/API需与本机环境核对 |
| Hospital 17 人直接进 Isaac5 | Hospital scenario 有，但当前 host runtime overlay没有该 world，且没有 Arena Hospital→Isaac5 的实跑证据 |
| Isaac 原生离线 demo | 安装内缺自包含的完整 local scene/character/command packaging；目前人物资产闭包来自项目或在线 asset root |
| 20 人 Arena benchmark | 当前 Arena+Isaac5 路线无已验收固定 20-person demo；旧 scenario/custom backend 不能替代 |

# 15. DO NOT REBUILD

- 不要重写 Arena TaskGenerator、scenario parser、Isaac entity services、People state machine、NavMesh 或 Animation Graph。
- 不要重新解决 CPython 3.10/3.11 ABI；host overlay、cp311 factory 和 runtime shim 已存在且历史跑通。
- 不要重新制作 `map_empty`、`map_highly_social` 或本地 8 个 character variants。
- 不要把 Gazebo Hospital 17-person success 复制成 Isaac success，也不要为此重造 Hospital 地图；先复用现有 Arena map/scenario。
- 不要为了第一阶段接入 HuNav/LightSFM/ORCA；Arena People/NavMesh 已足以验证现成 Arena+Isaac 场景链。
- 不要删除 standalone HuNav：保留其 planner/social dynamics/unit-test 基线和独立 backend 价值，但从“先打开 Arena 原生场景”的关键路径移除。
- 不要 checkout/reset/clean active `arena-isaac`；dirty compatibility change 和 protected build/install/log 必须保存。
- 不要用 Isaac 6 assets/runtime 混补 Isaac 5.1。

对 standalone HuNav 的明确取舍：其 YAML planner、LightSFM/ORCA 实验和碰撞指标**不需要带入第一阶段 Arena People demo**；但应保留作为 human-backend 单元测试、社会力对照、以及未来判断 Arena People 社会交互是否不足的证据源。

# 16. RECOMMENDED NEXT ACTION

**唯一下一步：在用户授权执行后，先核对并隔离当前相关长期进程，在干净的 ROS domain/GPU 窗口中仅复跑 Option 1 的既有 `map_empty` Arena+Isaac5 wrapper，并采集人物数量、连续 pose/位移、People/NavMesh 状态和 owned-process teardown 证据。**

不要先修 10 人场景，不要切官方 branch，不要接 HuNav。只有 Option 1 当前复验成功后，再把同一条链切到已有 `map_highly_social/default.json`。

核心问题最终回答：

- **Q1 完整源码？** 有足够完成历史运行的活动源码/overlay；也有官方 5.1 branch archive。但官方原分支单独看不完整可运行，因为 `arena_people_msgs` 缺失，active checkout 也不是该 branch。
- **Q2 现成 Arena+Isaac5 world？** 有可动态构造的 `map_empty` 且历史跑过；有更多 Arena map/scenario source。大多数不是独立预制 USD。
- **Q3 现成多人 scenario？** 有，数量覆盖 2/3/5/7/10/17/30 等；当前最接近直接 Isaac runtime 的是 map_empty 与 10 人 map_highly_social。
- **Q4 一启动看到 5/10/20 人？** 没有三档都已验收的一键 demo。10 人最接近但历史未通过；20 人只在旧配置或非 Arena custom backend 中有材料。
- **Q5 最小缺口？** 小规模只差安全复验；10 人差旧 Isaac5 adapter/runtime 问题的复验闭合；官方 5.1 原分支差 `arena_people_msgs`。
- **Q6 standalone HuNav 怎么办？** 从第一步移除，不删除；保留为 unit test、social backend 与比较基线。
- **Q7 最大化 Arena 复用路线？** 先复跑既有 Arena People/NavMesh map_empty，再复用现有 10-person scenario；仅当原生 People 明确不能满足研究指标时才评估 HuNav backend。
