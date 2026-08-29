审计时间：2026-08-10 至 2026-08-13（Asia/Shanghai）  
审计根目录：`/home/user/navigation_project/a_pipeline`；机器人补充核验根：`/home/user/navigation_project/robot_related`  
审计运行态证据截止：2026-08-13 00:01（21:47 standalone/SLAM 链保存日志无最终 RESULT；23:57 与 00:00 另有无窗口高效 RTX profile 验收 PASS）  
审计方法：静态源码、现有 build/install/log、版本文件、Git 元数据、备份清单、历史 rosbag 与机器人历史验收报告的只读核验；本次增量审计还对 2026-08-12 新 bag 做了离线反序列化，并只读运行项目已有验包器。未以“源码存在”替代“实际运行”，也未把不同仿真器的历史证据混为同一运行链。

> 2026-08-12 后续源码增量：`[CONFIRMED_SOURCE]` 行人/机器人避障已从历史布尔开关扩展为
> `off/native/gentle/legacy_dodge` 四档，旧 `ISAAC_PEDESTRIAN_DODGE=0/1` 继续映射为
> `off/legacy_dodge`。`gentle` 使用原生物体避障，并仅在持续接近 0.25 s、碰撞框净距
> 不超过 0.65 m 时调用 `motion_scale=0.75` 的低速 dodge；历史快速参数原样保留。
> 修改后的 `show_warehouse_people_robot_6_0.py` SHA256 为
> `b0034fba0eed4b46aa8a2ec6e8d89f6a5deb5736b5989a9e8aab10691ec353fa`，启动器 SHA256 为
> `4c512547cc9ee74f0f1d8f69624471eb1dead66cba42e555176523eca2e36fa9`。Python AST、shell
> 语法和模式兼容逻辑已通过；因当时已有用户仿真进程，本增量尚未获得新的 runtime 证据，
> 下文此前记录的 `27a93c...` 源码 hash 及 dodge runtime 结论均属于本次增量之前的 revision。

> 2026-08-12 22:17–22:58 雷达频率与启动安全增量：`[CONFIRMED_SOURCE]` standalone 链新增统一的
> `ISAAC_LIDAR_RATE_HZ`（1–30 整数 Hz）和 `--deterministic`，目标值同时驱动 RTX
> `scanRateBaseHz`、sensor `tickRate`、发布周期、`LaserScan.scan_time` 和验包门槛。双 RTX
> 缓存从每路单一 latest 值改为 64 帧有界队列，按完全相同的 GMO `(frameId,timestampNs)`
> 配对，记录未配对丢帧，并把真实 sensor timestamp/相邻采集周期传给 ROS bridge。新增
> transient-local `/data_collection/sensor_config`，使新 bag 自带雷达参数及 producer、bridge、
> launcher SHA256；录包前会用 scan header 的仿真时间测量两路频率至少 1 s，偏差超过
> `max(0.25 Hz, 15%)` 即拒绝录制。启动器另加入原子单实例锁、已有 Isaac/Kit 检查、
> NVIDIA 响应/僵尸进程/本次启动 fatal kernel signature 检查和最低 16 GiB 可用内存门槛，
> 以防不稳定主机上叠加第二个 Isaac。当前 exact SHA256：producer
> `1d4a42067e0847930081e905a151e1f6a838db9d6e5b690dc8475de77e2afe12`，bridge
> `ea6316167d0e6091713092517f95bb6cb822e15bbf88d64ee3ff582b550e1866`，launcher
> `70c8b8f2b6a3de4d8c49606249f45f08cd848ab93de8a405890a32981781ab07`。Python 编译、shell
> 语法、参数边界、双队列乱序/配对/测频、ROS timestamp/config 发布节流及自描述 bag
> 读取的非仿真测试均 PASS。

> 2026-08-12 22:55 低负载雷达实测：`[CONFIRMED_RUNTIME]` 在启动前确认无既有 Isaac 后，
> 以 `people=0`、`headless+fast`、3 s sim duration、目标 10 Hz 启动一次且未打开窗口。
> `scripts/logs/warehouse_people_robot_6_0_20260812_225514.log` 绑定 producer
> `1d975c22975f3895db6158730da7d41d89c3e3ea09d33254c1a9585301213f50` 与 launcher
> `746e0576fd52d36c183ea76c7a2c1cf0caf6b854d12351986169b689e699b58b`，结果为 PASS：
> 3.0167 sim s 内 30 对双 RTX scan、原生时间戳实测 10.0 Hz、两路未配对丢帧均为 0，
> 360 槽强度非零；应用平均 16.78 FPS。该结果直接证明本机在低负载、非实时快进条件下
> 能生成 10 Hz 仿真时间雷达，因而旧 bag 的 3.195 Hz 不能简单归因为“显卡绝对性能不足”。
> 结束时在 RESULT PASS 之后仍出现两次 teardown GMO invalid-magic warning；随后仅给当前
> producer 增加了关闭标志，使 Replicator/Hydra 销毁期间的迟到 writer callback 不再解析
> 已失效 buffer。该最后一处源码当前只有静态检查，尚未取得当前 exact hash 的运行证据；
> 动态行人、实时 wall-clock 10 Hz 与新版 13-topic rosbag 也仍待安全窗口验收。

> 2026-08-12 23:47 至 2026-08-13 00:01 雷达 profile 根因与修复增量：
> `[CONFIRMED_SOURCE]` 原默认 NVIDIA `Example_Rotary_2D` 并非轻量 360-ray 二维雷达，而是
> 128 channel / 128 emitter、每圈每雷达约 3.6–4.2 万原生回波的密集诊断配置，producer
> 最后才投影为 360 槽。现新增 `ISAAC_RTX_LIDAR_PROFILE=example_dense|rplidar_s2e`，并将
> 本地官方单通道 `RPLIDAR_S2E.usda` 升为导航默认；频率仍由独立的
> `ISAAC_LIDAR_RATE_HZ=1..30` 控制。profile 名、资产路径和资产 SHA256 均进入 READY、
> RESULT 与 transient-local sensor config；launcher/relay/bag reader 会校验 profile 合同。
> `[CONFIRMED_RUNTIME]` 在无既有 Isaac、`people=0`、`headless+fast` 下，23:57 的 10 Hz
> RPLIDAR 测试于 3.0167 sim s 产生 30 对 scan，`sim_hz=10.0`、`wall_hz=16.1763`、
> 97.63 app FPS、两路未配对丢帧 0，360 槽强度非零，RESULT PASS；相同场景的旧密集档
> 只有约 16.16 app FPS 与 2.68 对/s 墙钟吞吐。00:00 又以非默认 7 Hz 验收，产生 20 对，
> `sim_hz=7.000000042`、`wall_hz=10.9564`、95.76 app FPS、零未配对丢帧，RESULT PASS，
> 直接证明自定义频率链不是 10 Hz 特判。7 Hz 运行绑定 producer `40426f5e...ea367`、
> launcher `94e267c4...0801d`、RPLIDAR 资产 `919f5b1d...ec73`。提前销毁 render product
> 的实验会增加 RESULT 之后的 GMO invalid-magic teardown warning，故已恢复更接近 Isaac
> 官方测试的 stop/flush/destroy 顺序；当前 producer `affbde74...db4f`、bridge
> `cb1f2626...cd3709`、launcher `94e267c4...0801d` 已通过 Python/shell 静态检查，最后的
> 生命周期回退本身未再启动第三次 GPU 测试。退出 warning 不影响此前 RESULT/scan，但仍是
> 尚未消除的 Isaac 原生 teardown 噪声。动态行人和新版完整 13-topic rosbag 仍待验收。

# 1. 一句话结论

`[CONFIRMED_RUNTIME]` Isaac Sim 6.0.1 + Mecanum730/XMS5 运动代理 + 3 名 IRA 行人 + 外部 UDP/ROS 2 bridge 的 RTX 实现 lineage 已越过双雷达首帧/warmup 门，并由 2026-08-12 的 12-topic rosbag 证明 360 槽非零强度、TF、odom、遥控运动和行人真值，功能上达到 Level 2；21:47 的后续运行还启动了 Slam Toolbox 并实际收到 live `/map`。但完整 bag 使用旧 `dodge=true` 覆盖项、scan 仅约 3.195 Hz、行人速度一致性略超门槛，长跑曾在约 2333 秒后 crash；最新 SLAM 总检查又因 `/scan_01` publisher=0 而 FAIL，RViz 启动失败且没有持久化地图。`[CONFIRMED_SOURCE]` 当前磁盘源码随后又升级为四档行人避障，因修改晚于 21:47 进程启动，尚无 exact-revision runtime。故当前仍是“功能性 Level 2 + 在线建图前置能力已出现，严格稳定基线与 Level 3 未通过”。Arena、Task Generator、Evaluation、HuNav 与四个 Planner 则没有新增源码/build/install/runtime 闭环。

证据标记在全文中的含义：

- `CONFIRMED_RUNTIME`：现有日志、rosbag 或运行产物直接证明。
- `CONFIRMED_SOURCE`：当前本机源码、配置或清单直接证明；不代表能构建或运行。
- `CONFIRMED_BUILD`：现有构建结果和日志直接证明。
- `CONFIRMED_INSTALL`：现有 install/ament index 或只读包解析直接证明。
- `INFERRED`：由多个已确认事实推导，但缺少直接运行或 revision 绑定证据。
- `UNKNOWN`：当前证据不足，不能下结论。

# 2. 当前工程全景

`[CONFIRMED_SOURCE]` 关键目录图如下，深度限制在五层以内；它同时包含主数据管线、独立 Isaac 实验线和一个部分构建的 Arena workspace，并不是一个单一、已经贯通的 ROS workspace。

```text
a_pipeline/
├── README.md                         # V7 双雷达语义导航数据管线总说明
├── configs/                          # 工程路径与标签等顶层配置
├── environment/                      # 环境、依赖和构建状态检查
├── examples/                         # 小型样例与 smoke-test 输入
├── github_src/                       # 上游 DRL-VO 等参考源码
├── methods/                          # 基线方法和实验实现
├── pipelines/                        # V7 数据处理/验证流水线
├── scripts/                          # 发布、验证与数据操作脚本
├── workspaces/
│   └── ros2_ws/                      # 已构建的 semantic_nav_gazebo ROS 2 workspace
├── runs/                             # 大量历史实验输出
├── build/ install/ log/              # 顶层语义导航 workspace 的历史产物
└── isaac_sim/
    ├── README.md                     # Isaac 5/6 路线及运行记录
    ├── scripts/                      # Isaac 6 主入口、UDP relay、录包、SLAM；亦含 5.1 legacy 脚本
    ├── isaacsim-6.0.1/               # 本地 Isaac Sim 6.0.1 程序
    ├── assets-6.0.1/                 # Isaac 6.0 离线资产
    ├── scenes/                       # 自定义 USD 场景
    ├── config/                       # Isaac SLAM 等配置
    ├── bags/ captures/ maps/         # 历史 bag、采集和地图目标目录
    ├── backups/                      # Arena 集成前备份及校验信息
    ├── arena_isaac5_backup/
    │   ├── arena_isaac/              # Arena 5.1 候选适配器源码
    │   └── isaacsim_msgs/            # 与 active 版本不兼容的接口源码
    └── arena_ws/
        ├── src/
        │   ├── arena/                # arena-rosnav、evaluation、isaac、simulation-setup、tools
        │   ├── deps/                 # Nav2、SLAM Toolbox、HuNav 等依赖源码
        │   ├── planners/             # DRL-VO、CrowdNav、PaS、SICNav 等
        │   └── gazebo/               # Gazebo/TurtleBot 等仿真依赖
        ├── build/                    # 15 个已选包的历史构建产物
        ├── install/                  # 同一批包的 overlay 安装
        └── log/                      # 2026-08-09 构建及后续 list 日志
```

`[CONFIRMED_SOURCE]` 机器人资产在主工程外的同级目录，不受 `a_pipeline` 内的 workspace 或顶层 Git 统一管理：

```text
/home/user/navigation_project/robot_related/
├── robots/chassis_arm/              # 当前 Isaac 5/6 脚本实际引用的完整 URDF/USD 资产
├── Robot_URDF/
│   ├── motion_wheel_arm_simple_sphere_{urdf,usd}/  # 较早副本
│   └── exported_from_usd/       # Gazebo Sim 导航 proxy v1→v7 及历史验收报告
└── roboos/                         # 独立 Git repo；Isaac Lab 4.5 allstar_toilet 移动操作工程
```

`[CONFIRMED_SOURCE]` `robot_related` 中三类内容的用途不同：`robots/chassis_arm` 是当前 standalone Isaac 脚本的资产源，`Robot_URDF/exported_from_usd` 是另一条 Gazebo Sim 导航代理实验线，`roboos` 是 Isaac Lab 机械臂/清洁任务线；三者不是一个可互换的 Arena robot package。

`[CONFIRMED_SOURCE]` 顶层主项目是语义导航数据/训练管线；当前 Isaac 6 启动器实际 source 的是 `workspaces/ros2_ws`，不是 `isaac_sim/arena_ws`。

`[CONFIRMED_SOURCE]` `isaac_sim/arena_ws` 是另一条 Arena 集成候选线；其中源码很多，但仅 15 个包有 build/install，不能把整个目录视为已安装系统。

`[CONFIRMED_SOURCE]` `isaac_sim/arena_isaac5_backup` 是隔离快照，不在 `arena_ws/src` 的 active package 路径中。

`[CONFIRMED_RUNTIME]` `isaac_sim/maps/logs/` 当前已有一次在线 SLAM 启动/检查日志，并直接证明 live `/map` topic；`[CONFIRMED_SOURCE]` `maps/` 仍没有 `.yaml/.pgm`、posegraph 等持久化地图，因此必须区分“在线 map topic 已产出”和“地图文件已保存”。

# 3. 刚才长任务实际做了什么

## 3.1 前一长任务的实际操作

`[CONFIRMED_SOURCE]` 通过归档会话命令记录、备份 manifest、rsync 文件清单、嵌套 Git 状态和 colcon 日志交叉核验，前一长任务的操作如下。

| 执行命令/命令族 | 目的 | 是否修改文件 | 实际修改/产物 | 证据 |
|---|---|---:|---|---|
| `sed -n ... pasted-text-1.txt`（分段读取） | 完整读取 2757 行任务附件 | NO | 无 | `[CONFIRMED_SOURCE]` 归档会话命令记录 |
| goal/plan 与 3 个只读子审计任务 | 拆分 Isaac 6、Isaac 5/build、Arena/Planner 审计 | NO（工程） | 仅会话状态 | `[CONFIRMED_SOURCE]` 归档会话记录 |
| `find`、`rg`、`ls`、`du`、`df`、`pgrep`、`head/tail`、`sed` | 查看结构、源码、大小、日志与进程状态 | NO | 无 | `[CONFIRMED_SOURCE]` 归档会话命令记录 |
| `git status`、`git diff`、`git rev-parse`、`git reflog` | 核验顶层和嵌套 repo 状态 | NO | 无 | `[CONFIRMED_SOURCE]` 归档会话命令记录 |
| `source /opt/ros/humble/setup.bash && colcon list --base-paths src` | 只列 workspace 包 | 日志由 colcon 自动写入 | `arena_ws/log/list_2026-08-10_21-36-07/` 与 `...21-36-27/`；不是 build | `[CONFIRMED_SOURCE]` 两份 `logger_all.log:1` |
| `ros2 pkg prefix ...` / `ros2 interface show ...` | 只读核验包和接口可发现性 | NO | 无 | `[CONFIRMED_INSTALL]` 命令结果与 ament index |
| `rsync --dry-run ... /tmp/arena_backup_dryrun/` | 预演备份范围 | NO（正式工程） | 临时目标检查 | `[CONFIRMED_SOURCE]` 归档会话命令记录 |
| `mkdir -p isaac_sim/backups/arena_pre_integration_20260810_213644` | 建立备份目录 | YES | 新建备份目录 | `[CONFIRMED_SOURCE]` 目录与时间戳 |
| `rsync ... --exclude build/install/log/.git/__pycache__ ...` | 复制所选源码/配置 | YES（只写备份） | 5,826 个普通文件，共 37,615,034 bytes；未覆盖正式源码 | `[CONFIRMED_SOURCE]` `RSYNC_FILE_LIST.log` 与 manifest |
| `tar -czf ...arena_pre_integration_20260810_213644.tar.gz ...` | 创建可移交压缩备份 | YES（只写备份） | 4,079,031-byte tar.gz | `[CONFIRMED_SOURCE]` 归档与 manifest |
| `sha256sum ...tar.gz` | 校验归档 | NO | SHA256 `d15550b242afd165a5cf5e56d5f15bee89875921fa09ded7677d9e36bf6b5fd0` | `[CONFIRMED_SOURCE]` 独立复核一致 |
| `apply_patch` 新增两个备份说明文件 | 记录备份范围和预存 diff | YES（只写备份） | `BACKUP_MANIFEST.md`、`PREEXISTING_CHANGES.patch` | `[CONFIRMED_SOURCE]` 文件内容 |
| 子审计写 `/tmp/current_files_readonly.txt`、`/tmp/backup_files_readonly.txt` | 对当前与备份文件做只读比较 | YES（仅 `/tmp`） | 两个临时清单 | `[CONFIRMED_SOURCE]` 归档会话记录 |

`[CONFIRMED_SOURCE]` `BACKUP_MANIFEST.md` 记录了时间、范围、排除项、文件数/字节数、顶层 Git 不可用、嵌套 repo 状态、tar 路径与 SHA256；它明确说明备份过程未修改或删除正式源码。

`[CONFIRMED_SOURCE]` `PREEXISTING_CHANGES.patch` 只保存嵌套 repo 中早已存在的一处 diff：`arena_ws/src/arena/isaac/ros2isaacsim/ros2isaacsim/run_isaacsim.py` 启用 `isaacsim.asset.importer.urdf` 后导入 `_urdf`，并移除错误的 `extension_path = _urdf.ImportConfig()`。

`[CONFIRMED_SOURCE]` 当前嵌套 Git 仍只有上述一个 modified 文件，且 diff 与备份 patch 完全一致；reflog 显示 clone/checkout 早于前一长任务。顶层 `.git` 是空目录，所以无法以顶层 Git 对所有文件作 tracked-baseline 证明。

`[CONFIRMED_SOURCE]` **正式工程源码：未修改。** 这一定义指前一长任务没有改写 production source/YAML/launch/package/CMake/setup；只新增了指定备份材料。由于顶层 Git 不可用，这一结论由归档命令记录、嵌套 Git 和 21:36 备份后共同文件比较共同支撑，而不是顶层 `git diff` 单独支撑。

## 3.2 前一长任务的 YES / NO 清单

| 行为 | 结果 | 说明 |
|---|---:|---|
| `colcon build` | **NO** | `[CONFIRMED_BUILD]` 21:xx 只有 `colcon list` 日志；现有 build 产物来自 2026-08-09 |
| `apt` / `apt install` | **NO** | `[CONFIRMED_SOURCE]` 归档会话无此命令 |
| `pip` / `pip install` | **NO** | `[CONFIRMED_SOURCE]` 归档会话无此命令 |
| `git clone` | **NO** | `[CONFIRMED_SOURCE]` 归档会话无此命令；嵌套 clone reflog 更早 |
| 启动 Isaac Sim | **NO** | `[CONFIRMED_SOURCE]` 归档会话没有启动命令；21:14–21:16 的 RTX 日志早于 21:34 开始的该长任务 |
| 启动 Gazebo | **NO** | `[CONFIRMED_SOURCE]` 归档会话无此命令 |
| 启动 Nav2 / Arena / ROS 节点 | **NO** | `[CONFIRMED_SOURCE]` 只执行包/接口发现命令，没有 node/launch/run |
| 发送 `cmd_vel` / 启动机器人 / benchmark / 训练 / GPU 测试 | **NO** | `[CONFIRMED_SOURCE]` 归档会话无相应命令 |
| 删除或回滚文件 | **NO** | `[CONFIRMED_SOURCE]` 归档会话无删除/回滚命令 |

## 3.3 本次只读审计的偏差披露

`[CONFIRMED_SOURCE]` 本次审计早期误执行了一次 `colcon version-check`；它没有 build/install、没有修改源码、没有启动节点，但 colcon 自动在顶层创建了 `log/version-check_2026-08-10_23-33-48/logger_all.log`（531 bytes）并更新 `log/latest`、`log/latest_version-check` 两个符号链接。这是对“仅最终报告可写”约束的偏差，已即时向用户披露，且按“禁止删除”要求保留，未作清理或回滚。

`[CONFIRMED_SOURCE]` 除上述自动日志和本报告外，本次审计没有新增/修改正式工程文件；没有再运行 colcon、没有安装、构建、启动仿真或启动 ROS 节点。

## 3.4 2026-08-11 至 2026-08-12 新工作落点

`[CONFIRMED_SOURCE]` 与 2026-08-10 21:36 备份逐文件对比后，近期新增/修改集中在 standalone Isaac 6 分支，而不是 Arena：

| 当前新增/变化 | 类型 | 已证明状态 |
|---|---|---|
| `isaac_sim/scripts/rtx_lidar_scan.py` | 修改 | `[CONFIRMED_SOURCE]` 修正小强度被整数截零问题 |
| `isaac_sim/scripts/show_warehouse_people_robot_6_0.py` | 修改 | `[CONFIRMED_SOURCE]` 重构双 RTX attach/单位/读取/诊断/pose/teardown 生命周期 |
| `isaac_sim/scripts/diagnose_rtx_lidar_6_0.py` | 新增 | `[CONFIRMED_SOURCE]` 最小本地 RTX GMO probe |
| `isaac_sim/scripts/logs/warehouse_people_robot_6_0_20260811_*.log` | 新增运行记录 | `[CONFIRMED_RUNTIME]` 多轮诊断；`012507` 短程 RESULT PASS，`013634` 再次 READY |
| `...20260812_145524.log`、`...150031.log` | 新增运行记录 | `[CONFIRMED_RUNTIME]` 两轮 RTX READY；后者长跑后 native crash |
| `isaac_sim/bags/20260812_150345_isaac_6_teleop/` | 新增 bag | `[CONFIRMED_RUNTIME]` RTX+强度 lineage 的功能性 Level 2 证据，严格总验包未 PASS |
| `show_...py`、Isaac 6 launcher、`isaac_sim/README.md` 的 21:51–21:52 增量 | 修改 | `[CONFIRMED_SOURCE]` 行人避障由旧 boolean 扩展为 `off/native/gentle/legacy_dodge`；当前 exact 文件尚无 runtime |
| `...20260812_214714.log` | 新运行记录 | `[CONFIRMED_RUNTIME]` 中间 revision 再次 RTX first-scan/warmup/READY、收到 cmd、3/3 行人运动；保存日志无 RESULT |
| `maps/logs/20260812_214744_*`、`...214948_*` | 新增 SLAM 日志 | `[CONFIRMED_RUNTIME]` Slam Toolbox 注册雷达并发布 live `/map`；总检查和 RViz 仍失败 |

`[CONFIRMED_SOURCE]` 截止快照时，`show_...py` SHA256 为 `b0034fba...53fa`，launcher 为 `4c512547...fa9`，README 为 `64e99d23...67c5`；当前 launcher 与 8 月 10 日备份相比已是 `+18/-1`，不再能写成“launcher/README 未变化”。`cmd_vel_udp_relay.py` 与录包/验包脚本本身仍无同期修改。

`[CONFIRMED_SOURCE]` 截至本次增量审计，Arena-Rosnav、Evaluation、Simulation Setup、Nav2、HuNav、Planners 与 8 月 10 日备份的可比文件变化均为 0；`arena_ws` 无新增 build/install/test/runtime，`arena_isaac5_backup` 也逐字节未变。机器人资产、Gazebo V7 proxy、`roboos`、Isaac 5、scenes/config 未发现同期正式源码变化；`maps/` 的变化仅是上述运行日志，不是已保存地图。

# 4. 当前 standalone Isaac 6.0.1 主链

`[CONFIRMED_RUNTIME]` RTX+强度实现 lineage 已经修复原先卡在 sensor-ready gate 的问题，并获得功能性 Level 2 日志和 bag；但完整 bag 使用了旧非默认 dodge 覆盖项，10 Hz 严格频率、行人 velocity 一致性与长时间原生插件稳定性也仍未通过。`[CONFIRMED_SOURCE]` 当前磁盘 exact revision 又新增四档行人避障，尚无该 exact bytes 的 runtime。因此“当前主链”表示正在使用的 standalone 路线，不表示最新文件或全部验收项 PASS。

## 4.1 真正入口与源码设计链

`[CONFIRMED_SOURCE]` 真正入口是：

```bash
bash isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh
```

`[CONFIRMED_SOURCE]` 当前入口链如下：

```text
run_isaac_6_0_warehouse_people_robot.sh
├─ isaacsim-6.0.1/python.sh
│  └─ show_warehouse_people_robot_6_0.py
│     ├─ IRA_OBT_Sample_Warehouse.usd
│     ├─ ira_people_demo.yaml → 3 个 native patrol 行人
│     ├─ /World/Robot → mecanum730_xms5_default_base.usd
│     ├─ /World/RobotCollisionProxy
│     └─ 默认 native RTX 双雷达；显式可选 PhysX fallback
├─ /usr/bin/python3 cmd_vel_udp_relay.py
│  └─ localhost UDP 15973/15974 ↔ ROS 2 Humble topics/TF
└─ ros2 run semantic_nav_gazebo v7_dual_laser_scan_merger.py
   └─ /scan_01 + /scan_02 → /scan_merged
```

`[CONFIRMED_INSTALL]` Isaac 路径为 `isaac_sim/isaacsim-6.0.1`，精确版本是 `6.0.1-rc.7+release.42383.32955d8d.gl`；离线资产根为 `isaac_sim/assets-6.0.1/Assets/Isaac/6.0`。

`[CONFIRMED_SOURCE]` 场景为 `Assets/Isaac/6.0/Isaac/Samples/BehaviorTree/IRA_OBT_Sample_Warehouse.usd`；`scripts/ira_people_demo/ira_people_demo.yaml` 配置 3 名 native `patrol` walker、0.8–1.1 m/s 和本地 `HumanMotionLibrary/WalkForward`。当前代码不读取 `patrol_loop.json`。

`[CONFIRMED_SOURCE]` 机器人身份源指向工程外的 `robot_related/robots/chassis_arm/motion_wheel_arm_simple_sphere_usd/mecanum730_xms5_default.usd`，但 stage 实际引用其 `configuration/mecanum730_xms5_default_base.usd` visual/base layer。

`[CONFIRMED_SOURCE]` 顶层 default USD 是 base/physics/sensor 分层包装；当前 standalone Isaac 6 为了避免完整 articulation 在 IRA 厘米/Y-up stage 中降到约 6 FPS，明确绕过 physics/sensor 层，只参考 base layer。default `*_sensor.usd` 只含空的机器人 Xform，双 LiDAR 由 `show_warehouse_people_robot_6_0.py` 在运行时创建，不是机器人 USD/URDF 自带传感器。

`[CONFIRMED_SOURCE]` 控制不是完整轮地接触 articulation：Isaac 端以 20 Hz 对根位姿积分 `(linear.x, linear.y, angular.z)`，另用不可见 kinematic PhysX box 做 sweep/overlap 碰撞阻挡；车轮不做动力学旋转，XMS5 机械臂保持 authored visual pose。

`[CONFIRMED_SOURCE]` Isaac 6 内嵌 Python 3.12 与 ROS Humble 的 Python 3.10 rclpy 被有意隔离；ROS 侧 `/usr/bin/python3` relay 通过 localhost UDP 15973/15974 收发 JSON telemetry/command，ROS domain 为 0，默认 localhost only。

`[CONFIRMED_SOURCE]` 当前默认 `ISAAC_LIDAR_MODE=rtx`；RTX 配置声明为 `Example_Rotary_2D.usda`、10 Hz、360 samples、0.5–50 m，并把 GMO 数据投影为对齐的 ranges/intensities（0–255）。`ISAAC_LIDAR_MODE=physx` 只是显式 fallback，不是默认。这里的 10 Hz 是目标/配置值；最新 bag 的实际输出率约 3.195 Hz。

`[CONFIRMED_SOURCE]` 当前入口默认开启机器人碰撞保护，行人避障默认模式为 `off`；可选 `native` 只使用 BehaviorAgent 连续物体避障，`gentle` 在其上增加低速/近距/持续接近触发的 emergency dodge，`legacy_dodge` 保留历史快速侧闪。旧 `ISAAC_PEDESTRIAN_DODGE=0/1` 兼容映射为 `off/legacy_dodge`。当前默认 ROS topic 集没有 camera、IMU、contact 或独立 collision event topic。

## 4.2 历史真实运行证据

| 运行证据 | 已证明内容 | 结论 |
|---|---|---|
| `isaac_sim/bags/logs/warehouse_people_robot_6_0_20260810_011243.log:377,381` | READY；42.004 s、3739 frames、92.23 FPS、机器人前进 1.7568 m、3/3 行人运动、583 个 cmd，结果 PASS | `[CONFIRMED_RUNTIME]` 历史 Level 2 |
| bag `20260810_011319_isaac_6_teleop` | 17.6785 s、3207 messages、12 topics 全部非零 | `[CONFIRMED_RUNTIME]` sensors/TF/odom/cmd_vel/teleop/rosbag |
| `...111229.log:384` + bag `20260810_111320_isaac_6_teleop` | READY；76.0526 s、14630 messages、12 topics 全部非零；没有保存 RESULT | `[CONFIRMED_RUNTIME]` 第二份历史 Level 2 数据 |
| `...101644.log:380-384` | 临时墙前移动约 0.397 m 后被阻挡，`collision_blocked_count=166`，PASS | `[CONFIRMED_RUNTIME]` 碰撞代理历史验收 |
| `...101944.log:381` | 正角速度产生 `+1.291621 rad` yaw change，PASS | `[CONFIRMED_RUNTIME]` 角速度方向历史验收 |
| `...110732.log:384-393`、`...110536.log:390` | 可选 dodge 下 3 人触发绕避、最小 clearance 约 0.457/0.469 m、PASS | `[CONFIRMED_RUNTIME]` 可选行人绕避历史验收 |

`[CONFIRMED_RUNTIME]` 对两份 bag 以 immutable/read-only SQLite 方式反序列化全部四个 LaserScan topic 后，`011319` 的 `/scan_01,/scan_02,/scan,/scan_merged` 分别为 177/177/177/171 条，`111320` 分别为 752/752/752/1503 条；所有消息的 `intensities` 长度均为 0。

`[CONFIRMED_RUNTIME]` 因此上述 bag 证明双 range scan 与 Level 2 历史链，不证明当前 RTX intensity 合同；两份 bag 均早于当前 relay、launcher、README 和 `show_warehouse_people_robot_6_0.py` 的修改时间，历史 READY JSON 也没有当前 `lidar_mode/lidar_intensity` 字段。

## 4.3 2026-08-11 的 RTX 修复与短程 PASS

`[CONFIRMED_SOURCE]` 相对 2026-08-10 21:36 备份，8 月 11 日的 RTX 修复集中在三个文件：

- `[CONFIRMED_SOURCE]` `scripts/rtx_lidar_scan.py` 不再先把 `scalar×255` 截成 uint8，而是保留 LaserScan float32 精度；否则仓库中小于 `1/255` 的有效回波会全部归零（SHA256 `01181d1e2baf840336e6cbb87db9f48ea9f6dd52eeb339a7af0cee2215f9eea3`）。
- `[CONFIRMED_SOURCE]` `scripts/show_warehouse_people_robot_6_0.py` 将两个 RTX sensor 放在独立 `/World/RtxSensors`，在 timeline stopped 时逐个 attach/同步 Hydra，直接 compose USD reference 避免 centimetre stage 的 MetricAssembler 100/10,000 倍缩放；同时校验 GMO magic、增加 writer+annotator 双读取/诊断、随机器人同步 world pose，并调整 teardown 生命周期。这里的 SHA256 `27a93cb3c8e565350fa9e63e787b8c34bcc962c7ba0af2da1304207104c81b94` 是 8 月 11 日/12 日白天运行所对应的旧源码谱系；当前文件已经变为 `b0034fba0eed4b46aa8a2ec6e8d89f6a5deb5736b5989a9e8aab10691ec353fa`。
- `[CONFIRMED_SOURCE]` 新增 `scripts/diagnose_rtx_lidar_6_0.py`，以本地资产和四个 cube 做最小 GMO writer/direct-reader probe（SHA256 `a94e5b5f37b9110b0732986f16b29081f9607f49308b01a463f0cd9db268c4ed`）。

`[CONFIRMED_RUNTIME]` `warehouse_people_robot_6_0_20260811_012507.log:387-399` 明确记录：双 sensor attach 和 transform scale=1 校验、首个完整双 scan、warmup PASS、带 `lidar_mode=rtx`/`lidar_intensity=true` 的 READY，以及 3.0167 sim s 后 RESULT PASS。该短跑发布 30 对 scan、3/3 行人运动，平均约 13.9 application FPS；它没有收到 cmd_vel，因此单独只验证 RTX/people/bridge ready，不验证遥控。`[INFERRED]` 两个当时核心文件 mtime 均为 2026-08-11 01:24:41，日志在其后生成且行为与该修复一致；但日志没有嵌入 source hash，不能把它绑定到当前 21:51 后的 exact bytes。

`[CONFIRMED_RUNTIME]` 后续 `20260811_013634.log` 与 2026-08-12 两次日志也再次到达 first-scan/warmup/READY，说明原“始终卡在双雷达就绪门”的结论已被新证据推翻。其中 `20260812_145524.log:387-546` 在当时默认 `pedestrian_dodge=false`（现兼容映射为 `off`）下到达 READY、收到命令并运动至 273.8 sim s；它没有 RESULT 或 bag，不能替代完整验包。

## 4.4 2026-08-12 RTX+强度 Level 2 bag

`[CONFIRMED_RUNTIME]` `isaac_sim/bags/20260812_150345_isaac_6_teleop` 是本次最重要的新产物：43.278629458 s、3941 messages、原 12 个 topic 全部非零；db3 SHA256 为 `3f1d4fac2a1cd049dd783f61e624ccf41eb73038f01d68d1ea2fb5e7771aff2a`，metadata SHA256 为 `61da0c53bc0142412eca4a445bcc2ee48a3401008c183c29e1384c96efc18c58`。对应 `150031.log:392-398` 已先通过双 RTX first-scan/warmup/READY，再接收遥控命令。该轮显式使用旧 `pedestrian_dodge=true`，所以它证明当时 RTX 实现与该覆盖配置，不单独证明 launcher 默认环境，更不证明 21:51 后的四档源码。

| 离线核验项 | 结果 | 证据结论 |
|---|---|---|
| 四路 LaserScan | `/scan`、`/scan_01`、`/scan_02`、`/scan_merged` 各 139 帧；每帧 360 ranges + 360 intensities | `[CONFIRMED_RUNTIME]` RTX lineage 完整消息合同 |
| 强度 | 四路每一帧都有正强度；nonzero slots 合计 4726/4726/4303/8936，约 0.000267–0.006535，至少 1024 个 rounded unique values | `[CONFIRMED_RUNTIME]` `lidar_intensity_contract: PASS`，不是固定/伪造常数 |
| 控制 | `/cmd_vel` 867 条、521 条 moving，`linear.x=0..0.5`、`angular.z=-1..1`，最后一条为 0 | `[CONFIRMED_RUNTIME]` `teleop_command_semantics: PASS` |
| 横移 | `linear.y` 全为 0 | `[UNKNOWN]` 这份 bag 仍不能验证 Isaac lateral motion |
| odom | 414 条；首末 XY 净位移约 11.043 m（离线积分路径约 13.04 m） | `[CONFIRMED_RUNTIME]` 机器人确实移动，不只是 command traffic |
| TF | 动态 `odom→base_link`；静态 `base_link→base_scan/base_scan_01/base_scan_02` | `[CONFIRMED_RUNTIME]` TF 合同完整 |
| episode | 一组 start/end，sim duration 26.628994 s | `[CONFIRMED_RUNTIME]` `manual_episode_intervals: PASS` |
| 行人 | 406 条、始终 3 个固定 ID；三人首末位移约 12.11/4.97/6.92 m | `[CONFIRMED_RUNTIME]` typed pedestrian ground truth 非空且人员运动 |
| cmd/time alignment | boundary trim 后可用，scan 全落在 clock 范围且有 prior cmd | `[CONFIRMED_RUNTIME]` 现有 alignment checker PASS |

`[CONFIRMED_RUNTIME]` 因此 RTX 实现 lineage 可确认功能性 Level 2；旧报告中的“当前只到 Level 0”已经失效。该 bag 证明的是 21:51 四档避障修改之前的 RTX/legacy-dodge 谱系；它不验证当前 `b0034fba...` exact revision。

## 4.5 为什么仍不能称为严格稳定基线

`[CONFIRMED_RUNTIME]` 项目自己的 `check_rosbag.sh` 对这份 bag 最终返回非零：第一处失败是 `/scan_01` 仿真时间频率 `3.194760 Hz`，不接近脚本要求的 10 Hz；以实测率只读复核时，scan01/scan02 均为 360 beams、0.5–50 m、约 3.195 Hz。这里必须区分“sensor 内部/源码配置 10 Hz”和“ROS bag 实际输出 3.195 Hz”。

`[CONFIRMED_RUNTIME]` 分项验收中 intensity、cmd、episode、cmd/time alignment 均 PASS；行人 validator 则以 `velocity_error_median=0.511856 m/s` 略超 `0.500 m/s` 门槛而 FAIL（方向 cosine median 0.993644）。所以 README 所述最终 `ISAAC_TELEOP_ROSBAG_CHECK=PASS` 尚未达成。

`[CONFIRMED_RUNTIME]` `warehouse_people_robot_6_0_20260812_150031.log` 在 READY 后持续运行约 2333 s、3/3 行人持续运动，最终于 `:1688-2091` 在 `libomni.anim.behavior.core.plugin.so` 原生栈发生 crash，并记录 `std::out_of_range: no null terminator at count` 后 abort；没有 RESULT。现有证据不能把 crash 归因于 RTX，但长期运行稳定性没有通过。

`[CONFIRMED_RUNTIME]` 8 月 10 日旧 revision 的 GMO invalid magic、300 s timeout 是修复前历史失败，不再描述当前就绪状态；它们仍保留为问题演进证据。

`[CONFIRMED_RUNTIME]` `warehouse_people_robot_6_0_20260811_000909.log:383-439,811+` 证明一次 PhysX fallback 曾到达 READY、接收命令并运动到约 301.6 sim s，但随后在约 333 s 发生 NVIDIA/Vulkan 原生 crash，退出码 139，且无 RESULT/bag。该运行早于当前 RTX producer 最终 mtime；`[UNKNOWN]` 当前精确源码 revision 的 PhysX fallback 是否能完整恢复 Level 2。

## 4.6 当前 exact revision 与 21:47 运行边界

`[CONFIRMED_SOURCE]` 当前 `show_warehouse_people_robot_6_0.py` 相对 8 月 10 日备份为 `+464/-71`，SHA256 `b0034fba0eed4b46aa8a2ec6e8d89f6a5deb5736b5989a9e8aab10691ec353fa`；launcher 为 `+18/-1`，SHA256 `4c512547cc9ee74f0f1d8f69624471eb1dead66cba42e555176523eca2e36fa9`。两者分别在 21:51:03、21:51:17 修改，Python AST 与 shell syntax 的只读检查均 PASS。

`[CONFIRMED_RUNTIME]` `warehouse_people_robot_6_0_20260812_214714.log:388-398` 的进程在 21:47:14/16 已启动，确实再次取得双 RTX 非零强度 first-scan、warmup、READY、3/3 行人运动和 cmd；审计过程中曾观测进程运行，保存日志止于 423.7 sim s，无 RESULT、本次 crash 或退出原因。日志仍输出旧 boolean A/B 字段和旧 READY schema，且进程启动早于 21:51 源码修改，因此它是四档避障提交前的**中间内存 revision**，不能用作当前 exact bytes 的运行证明。

`[CONFIRMED_RUNTIME]` 该轮截至日志 `t=423.7 s`，机器人 XYZ 始终为 `[850,0,600]`，但 yaw 从 0 变化到约 `-3.005 rad`，3/3 行人持续运动；它证明旋转控制，不证明平移探索或地图覆盖质量。

# 5. Isaac 5.1 / arena_isaac5_backup

## 5.1 两个不同的 5.1 概念

`[CONFIRMED_INSTALL]` 本机 native Isaac 5.1 位于 `/home/user/isaacsim/5.1.0`，版本 `5.1.0-rc.19+release.26219.9c81211b.gl`、内嵌 Python 3.11.13；`start_isaac_5_1_595_compat.sh`、`run_navigation.sh/.py`、`validate_robot.sh/.py` 是 legacy/native 5.1 路线。

`[CONFIRMED_SOURCE]` native `run_navigation.py` 使用同一机器人源和 `isaac_sim/scenes/mecanum_lidar_main.usd`，默认 PhysX，并因 RTX 5090 上移动 RTX sensor 的已知崩溃风险而只允许 opt-in RTX；它不是当前 Isaac 6 主入口。

`[INFERRED]` `20260808_205629_isaac_collection` 等 Aug 8 产物与 native 5.1 时间和路径吻合，但日志没有直接写明版本，不能升级为 5.1 `CONFIRMED_RUNTIME`。

`[CONFIRMED_SOURCE]` `arena_isaac5_backup` 则是假设容器内 `/isaac-sim` 的 Arena-native service adapter，与上述 host-native 5.1 脚本不是同一运行链。

## 5.2 backup 身份、包和入口

`[CONFIRMED_SOURCE]` backup 包含两个 ROS 2 package：`arena_isaac`（ament_python, 0.0.0）与另一套 `isaacsim_msgs`（ament_cmake, 0.0.0）。

`[CONFIRMED_SOURCE]` 90/90 文件的 hash 与 active 嵌套 repo 的远端引用 `origin/arena5-isaac5.1.0` commit `16b8e3416517d8c3dc1b5038df4fe11b9a6df46c` 完全一致，因此它是该分支的精确候选源码快照。

`[CONFIRMED_SOURCE]` Python entry point 是 `run_isaacsim=arena_isaac.run_isaacsim:main`；launch 执行 `${ISAAC_PATH}/python.sh`，但 helper 又硬编码容器 `isaac-sim`、`ROS_DOMAIN_ID=1` 和 `/isaac-sim/python.sh`，主程序还硬编码 `/isaac-sim/apps/isaacsim.exp.base/full.kit`。

`[CONFIRMED_INSTALL]` 当前主机不存在 `/isaac-sim`，也没有 `docker` 命令；所以仅设置 `ISAAC_PATH=/home/user/isaacsim/5.1.0` 仍不能满足其 `/isaac-sim/apps/...` 假设。

## 5.3 源码能力与确定缺陷

| 能力 | 源码状态 | 审计结论 |
|---|---|---|
| `SpawnWalls` / `SpawnFloors` / `SpawnCeilings` / `SpawnPrims` | 有服务和实现 | `[CONFIRMED_SOURCE]` 代码存在，未运行 |
| `SpawnPedestrians` / `MovePedestrians` / delete/update | 有实现，但依赖缺失的 `arena_people_msgs` | `[CONFIRMED_SOURCE]` 不能构成当前闭环 |
| `SpawnUrdf` | 含 URDF import、articulation、odom/joint/control/sensor 组装 | `[CONFIRMED_SOURCE]` 相对完整，未运行 |
| `SpawnUsd` | srv 请求无 `prim_path`，实现却访问它；缺 `omni` import；注册 callback 错；callback 不返回 response | `[CONFIRMED_SOURCE]` 确定静态错误 |
| `GetPrims` | srv 请求字段为 `names`，实现读取 `request.prim_paths` | `[CONFIRMED_SOURCE]` 确定 schema/实现不匹配 |
| `ResetWorld` | 只清 walls/doors/floors/elevators | `[CONFIRMED_SOURCE]` 不会完整清 ceilings/robots/pedestrians |
| LiDAR / Camera / IMU / Contact | URDF 解析和 publisher/writer 源码存在 | `[CONFIRMED_SOURCE]` 未运行验证 |
| TF / odom / clock | 源码构图：`map→odom` identity、`odom→base`，odom 无 velocity；clock graph 存在 | `[CONFIRMED_SOURCE]` `base→sensor` 仍委托外部 robot_state_publisher；无明确 tf_static |
| `cmd_vel` | 只是 namespace/enable hook，实际依赖外部 ros2_control/controller manager 再到 joint command | `[CONFIRMED_SOURCE]` 不是一个直接 Twist subscriber，端到端闭环缺失 |

`[CONFIRMED_SOURCE]` 该适配器注册 delete/edit/get/move/reset、walls/floors/ceilings/prims、URDF/USD、pedestrian 等约 14 类业务服务，另有 pause/unpause/step；“注册了服务”不等于请求 schema 与实现均正确。

`[CONFIRMED_SOURCE]` package manifest 没有完整声明代码实际使用的 `arena_bringup`、`arena_simulation_setup`、`std_srvs`、`isaacsim_msgs` 等依赖，消息包也遗漏部分 geometry/std 依赖声明。

## 5.4 Build/install/使用关系

`[CONFIRMED_BUILD]` backup 目录没有 build 产物；`[CONFIRMED_INSTALL]` 没有 install/ament index，source 当前 overlay 后 `ros2 pkg prefix arena_isaac` 返回 Package not found。

`[CONFIRMED_INSTALL]` 当前可发现的是 active `ros2isaacsim` 和 active `isaacsim_msgs`，不是 backup。

`[CONFIRMED_SOURCE]` 两套 `isaacsim_msgs` 同名同版本却 schema 不兼容：backup 有 10 msgs/10 srvs（例如 `SpawnUrdf`、复数 `SpawnWalls`、`SpawnUsd`、`ResetWorld`），active 有 7 msgs/12 srvs（例如 `ImportUrdf`、单数 `SpawnWall`、`ImportUsd`、`DeletePrim`）。同时放入同一 workspace 会产生重复 package name；overlay 也有 type support/schema 遮蔽风险。

`[CONFIRMED_SOURCE]` 当前 Arena launch 调用 `ros2isaacsim/run_isaacsim`，Task Generator 使用 active 的单数/Import 接口，因此当前项目没有使用 backup。

`[INFERRED]` 5.1 backup 在服务式 spawn/reset/world 管理结构上比外部 UDP Isaac 6 链更接近 Arena 原生设计；这不等于它当前更可运行，也不是路线选择结论。

`[CONFIRMED_SOURCE]` **arena_isaac5_backup 当前身份：Arena-Rosnav `arena5-isaac5.1.0` 分支的精确候选源码快照；未进入工作空间、未构建安装、ROS 不可发现、未接通且含确定静态接口缺陷，不是当前运行链。**

# 6. arena_ws

`[CONFIRMED_SOURCE]` workspace 结构及职责如下：

```text
arena_ws/
├── src/
│   ├── arena/
│   │   ├── arena-rosnav/       # 总编排、Task Generator、testing/training/utils
│   │   ├── evaluation/         # Arena Evaluation 源码
│   │   ├── isaac/              # active ros2isaacsim + active isaacsim_msgs
│   │   ├── simulation-setup/   # robot/world/obstacle/Nav2 配置与 launch
│   │   └── tools/              # workspace/开发辅助脚本
│   ├── deps/
│   │   ├── nav2/navigation2/   # Nav2 1.1.19 源码
│   │   ├── hunav/              # HuNavSim 与 evaluator 源码
│   │   └── ...                 # SLAM、消息与通用依赖
│   ├── planners/               # 四个重点 Planner 及依赖
│   └── gazebo/                 # Gazebo/TurtleBot 相关源码
├── build/                      # 15 个包，均有 colcon_build.rc=0
├── install/                    # 同 15 包的 ament overlay
└── log/                        # build/list 的原始证据；无 Arena runtime log
```

`[CONFIRMED_SOURCE]` active Isaac 子仓库位于 `src/arena/isaac`，detached HEAD 为 `a4beefe0203ec8a75d65d0ab70496b5e2c400605`；只有前述 importer 的预存修改。

`[CONFIRMED_SOURCE]` `.repos/isaac.repos` 将 active Isaac 意图 pin 到 `master@a4beefe0`；但顶层 Arena-Rosnav 源码自身没有可用 Git 元数据，所以其实际 branch/commit 是 `UNKNOWN`。

`[CONFIRMED_SOURCE]` 以 2026-08-10 pre-integration snapshot 逐文件复核，Arena-Rosnav、Evaluation、Simulation Setup、Nav2、HuNav、Planners 的可比文件分别为 245/20/1506/1182/126/302 个，变化数全部为 0；backup 范围内 5,826 个文件总体也是 0 DIFF、0 MISSING。少量仅存在于当前目录的文件均是备份按大小/类型排除的旧文件，不是 8 月 11–12 日新增工作。

`[CONFIRMED_BUILD]` 已有 build/install 是选择性构建，不是全 workspace 构建；Arena Evaluation、HuNav 和所有四个学习/优化 Planner 均没有 build 产物。

`[UNKNOWN]` 没有 `arena.launch`、Task Generator、active Isaac adapter 或 Nav2 成功运行日志，不能从 build/install 推断整条 workspace 已启动。

# 7. Arena-Rosnav

## 7.1 包职责与总入口

`[CONFIRMED_SOURCE]` `src/arena/arena-rosnav` 的主要组成是：

- `[CONFIRMED_SOURCE]` `arena_bringup`：总 launch、sim/human 选择器、benchmark/导航配置编排。
- `[CONFIRMED_SOURCE]` `task_generator`：world/entity/robot 管理、起终点、障碍、reset 与 task modes。
- `[CONFIRMED_SOURCE]` `testing`：示例 action/DRL 测试节点；存在不代表测试已执行。
- `[CONFIRMED_SOURCE]` `training`：训练源码存在，但 package 文件被隐藏为 `.package.xml/.setup.py`，当前不是有效 colcon package。
- `[CONFIRMED_SOURCE]` `utils`：mixins 和部分消息/工具；实际已构建的只有 `arena_rclpy_mixins`。
- `[CONFIRMED_SOURCE]` `tools` 位于相邻 `src/arena/tools`，用于 workspace/开发辅助，不是运行证明。

`[CONFIRMED_SOURCE]` 总入口是 `arena_bringup/launch/arena.launch.py`，默认参数如下。

| 参数 | 当前默认 | 证据等级/含义 |
|---|---|---|
| `robot` | `jackal` | `[CONFIRMED_SOURCE]` 不是 Mecanum730 |
| `sim` | `dummy` | `[CONFIRMED_SOURCE]` 要用 Isaac 必须显式选择 |
| `human` | Gazebo/Isaac 时映射为 `hunav`，其他为 dummy | `[CONFIRMED_SOURCE]` 但 HuNav 当前未安装 |
| `world` | `map_empty` | `[CONFIRMED_SOURCE]` |
| `global_planner` | `navfn` | `[CONFIRMED_SOURCE]` |
| `local_planner` | `dwb` | `[CONFIRMED_SOURCE]` |
| `inter_planner` | `navigate_w_replanning_time` | `[CONFIRMED_SOURCE]` |
| `tm_robots` | `explore` | `[CONFIRMED_SOURCE]` |
| `tm_obstacles` | `random` | `[CONFIRMED_SOURCE]` |
| `tm_modules` | `rviz_ui` | `[CONFIRMED_SOURCE]` |
| `record_data_dir` | 空 | `[CONFIRMED_SOURCE]` 默认不启动 recorder |
| `headless` | `0` | `[CONFIRMED_SOURCE]` |
| `use_sim_time` | `true` | `[CONFIRMED_SOURCE]` |
| `env_n`, `env_d` | `1`, `50` | `[CONFIRMED_SOURCE]` 多环境数量与间距 |
| `complexity`, `agent_name` | `1`, `robot` | `[CONFIRMED_SOURCE]` 被声明但没有传给 Task Generator，当前悬空 |

## 7.2 Simulation Setup 现有实体与配置

`[CONFIRMED_SOURCE]` `src/arena/simulation-setup` 的 robot loader 约定每个机器人目录提供 `model_params.yaml`、`mappings.yaml`、`control.yaml` 等；当前顶层机器人目录为 `WLP311D`、`WLP311E`、`boxer`、`dingo`、`husky`、`jackal`、`rbkairos`、`rbrobout`、`rbsummit`、`rbtheron`、`rbvogui`、`ridgeback`、`rskomnidirectional`、`turtlebot`，另有隐藏 `.old`。

`[CONFIRMED_SOURCE]` 障碍 loader 区分 static/dynamic；当前显式 dynamic model 目录包括 `actor1`、`actor2`、`gazebo_actor`、`ugly`，scenario loader 可从 world 文件解析 static/dynamic entities。

`[CONFIRMED_SOURCE]` 当前顶层 worlds 包括 `factory`、`generated`、`hospital`、`house17`、`ignc`、`map_empty`；world 约定使用 `map.yaml`、`obstacles.yaml`、`walls.yaml`、`zones.yaml`，也能解析 scenario JSON/YAML。

`[CONFIRMED_SOURCE]` Nav2 配置按通用 model params、robot-specific model params、controller、global planner、interplanner 五部分合并；现有 controller 配置目录包括 `crowdnav`、`crowdnav_attngraph`、`drlvo`、`dwb`、`graceful`、`mppi`、`regulated_pure_pursuit`、`rotation_shim`、`sicnav`，planner 配置包括 `navfn`、`smac_2d`、`smac_hybrid`、`smac_state_lattice`、`theta_star`。

## 7.3 源码接线

`[CONFIRMED_SOURCE]` 每个环境的 launch 顺序是 human include → `task_generator.launch.py`，并传 sim/human/robot/task modes/planners/world；另行 include physics simulator 并启动 world generator。

`[CONFIRMED_SOURCE]` Task Generator node 构造 `SimulatorRegistry → Human EntityManager → EnvironmentManager → WorldManager → RobotsManager`。RobotManager 通过 EnvironmentManager/Isaac adapter spawn robot，订阅 odom/Nav2 action status，再 include `simulation-setup/launch/robot.launch.py`。

`[CONFIRMED_SOURCE]` `robot.launch.py` 实际 include Nav2，并在 `record_data_dir` 非空时尝试启动 `arena_evaluation record`；`state_publisher.launch.py` 的 include 被注释，因此不能假设 Arena 自动发布目标机器人的完整 robot state/传感器 TF。

`[CONFIRMED_SOURCE]` Nav2 launch 合并通用 model params、机器人 model params、controller、global planner、interplanner 到生成的 nav2 参数，再 include `nav2_bringup`。

`[CONFIRMED_SOURCE]` active Isaac adapter 等待/调用 `isaac/urdf_to_usd`、`import_usd`、`delete_prim`、`get_prim_attributes`、`move_prim`、`spawn_wall`、`import_obstacle`、`spawn_pedestrian`、`move_pedestrians`、`delete_all_pedestrians` 等服务；它与稳定 Isaac 6 的 UDP 协议不是同一接口。

`[CONFIRMED_SOURCE]` Arena 的 Isaac launch 执行 `${ISAAC_PATH}/python.sh` 下的 `ros2isaacsim/run_isaacsim`，不是 `arena_isaac5_backup`。

`[UNKNOWN]` active `ros2isaacsim` 混用较新 `isaacsim.*` 与旧 `omni.isaac.*` API，且当前 importer 有未提交修正；没有 Isaac 6.0.1 runtime 日志证明它能在现有二进制和 ROS ABI 下工作。

`[INFERRED]` 现有 stable Isaac 6 启动时并不 source 或 launch Arena workspace，所以 Arena 当前对那条已跑通历史链不承担运行职责；二者只有潜在 ROS topic/概念层兼容，不是“部分已经接通”的运行链。

# 8. 我的 Mecanum730 + XMS5

## 8.1 资产谱系与当前 Isaac 6 实际用法

| 项目 | 当前事实 | 证据边界 |
|---|---|---|
| 当前机器人名 | `mecanum730_xms5_default` | `[CONFIRMED_SOURCE]` Isaac 5/6 launcher 与 source 常量 |
| 当前完整 USD 包装 | `/home/user/navigation_project/robot_related/robots/chassis_arm/motion_wheel_arm_simple_sphere_usd/mecanum730_xms5_default.usd` | `[CONFIRMED_SOURCE]` 现有 Isaac 脚本全部指向此路径 |
| Isaac 6 实际 stage layer | 同目录 `configuration/mecanum730_xms5_default_base.usd` | `[CONFIRMED_SOURCE]` 稳定启动器不加载完整 physics/sensor 组合 |
| 当前 URDF | `/home/user/navigation_project/robot_related/robots/chassis_arm/motion_wheel_arm_simple_sphere_urdf/mecanum730_xms5_default.urdf` | `[CONFIRMED_SOURCE]` 文件和相对 mesh 齐全；standalone Isaac 6 未使用 |
| 资产变体 | 顶层 20 个 USD 工具变体、21 个 URDF 变体 | `[CONFIRMED_SOURCE]` 包含 brush/gripper/mop/pick/suction 及三合一版；当前导航主链只用 `default` |
| 原始传感器定义 | default URDF 无 sensor/Gazebo/`ros2_control`/transmission；default `*_sensor.usd` 只有空 Xform | `[CONFIRMED_SOURCE]` 双 LiDAR 是 Isaac 脚本运行时创建，不是原机器人资产自带 |
| Isaac 6 底盘控制 | 直接积分 root pose 的 kinematic x/y/yaw + invisible collision proxy | `[CONFIRMED_SOURCE]` 不是轮地接触 articulation |
| 速度通道 | `linear.x`、`linear.y`、`angular.z` 均贯通 teleop→relay→Isaac 积分源码 | `[CONFIRMED_SOURCE]` `continuous_teleop.py`、relay、`show_...py` |
| Isaac 横移运行验收 | 没有单独保存的非零 `linear.y` 验收结果 | `[UNKNOWN]` 不能仅由源码宣称当前 Isaac 横移已实测 |
| Isaac 实测 visual 尺寸 | stage XYZ `[0.58474,1.760708,0.447049] m`；Y 是竖直轴 | `[CONFIRMED_RUNTIME]` 历史 `...110732.log:384` |
| Isaac 实测 proxy 尺寸 | `[0.62474,1.720708,0.487049] m`；平面约 0.625×0.487 m | `[CONFIRMED_RUNTIME]` 同一日志 |
| 车轮/机械臂 | 车轮 joint 和 XMS5 臂 joint 在资产中存在；当前 Isaac 6 不驱动它们 | `[CONFIRMED_SOURCE]` 车轮不做动力学，机械臂保持 authored visual pose |
| Arena/Nav2 footprint | 没有正式 polygon/radius 参数 | `[CONFIRMED_SOURCE]` Gazebo proxy 有简化 box，但 Arena/Simulation Setup 无目标机器人配置 |

`[CONFIRMED_SOURCE]` 当前 default URDF 有 41 links、40 joints（6 个机械臂 revolute、32 个车轮/滚子 continuous、2 个 fixed）和 386 个 collision 元素；其 11 种唯一 mesh 相对路径在当前目录全部可解析。它是较完整的几何/关节描述，但不是一个自带控制器和传感器的可直接导航 ROS robot package。

`[CONFIRMED_SOURCE]` `Robot_URDF/motion_wheel_arm_simple_sphere_usd` 中的 default 顶层 USD 和 base layer 与当前 `robots/chassis_arm` 副本逐字节相同；但 `Robot_URDF/..._urdf/default.urdf` 是 2026-05 的较早版，仍引用 `../../mecanum/...` 旧相对路径，与 2026-06 的当前 URDF 不同。因此当前唯一可依赖的 standalone Isaac 资产入口是 `robots/chassis_arm`，不应在新配置中从两个副本交叉取文件。

## 8.2 `exported_from_usd` 的 Gazebo Sim 导航代理

`[CONFIRMED_SOURCE]` `Robot_URDF/exported_from_usd` 下有从 assembled preview、nav proxy 到 fallback v2–v7 的 11 个模型/备份目录。它们是将高精度移动操作机器人简化为导航代理的独立 Gazebo Sim 实验线，不是 Isaac 6 启动器的 fallback，也未放入 `arena_ws/src/arena/simulation-setup` 的 robot loader 目录。

V7 `mecanum730_xms5_nav_proxy_fallback_v7_teacher_scan01/model.sdf` 的静态合同为：

| 项目 | V7 定义 | 当前判断 |
|---|---|---|
| 底盘 | 50 kg，单一 `0.70×0.62×0.60 m` box collision | `[CONFIRMED_SOURCE]` 是简化代理，不是麦克纳姆轮接触模型 |
| 控制/里程计 | Ignition `VelocityControl` 消费 `/cmd_vel`；`OdometryPublisher` 以 30 Hz 发 `/odom` 和 `/tf` | `[CONFIRMED_SOURCE]` 直接速度代理，不是 ros2_control |
| 双雷达位姿 | scan01 `(0.2,0.13,0.208; 3.14,0,0)`；scan02 `(-0.2,-0.13,0.208; 3.14,0,3.14)` | `[CONFIRMED_SOURCE]` 相对 `base_link` 的 fixed joints |
| 扫描合同 | `/scan_01`、`/scan_02`；10 Hz；360 samples；0.1–50 m | `[CONFIRMED_SOURCE]` SDF 中两个 `gpu_lidar` |
| visual | 带双 LiDAR 凹口的派生 STL，约 261 万 triangles | `[CONFIRMED_SOURCE]` 原精细 mesh 会遮挡两个扫描原点，历史修复只移除 125 triangles |

`[CONFIRMED_RUNTIME]` 2026-06-17 的 V7 验收报告和原始文本记录证明当时在 `gazebo_eng_lobby.world`、`ROS_DOMAIN_ID=79` 中运行过：`/scan_01` 99 条/9.761 Hz，`/scan_02` 98 条/9.752 Hz，每帧 360 ranges；`base_link→base_scan_01/02` TF 与上述位姿一致；`/odom` 由 `(2.0,2.0)` 变为 `(2.0722,1.9829)`，报告记录了 forward/rotate/lateral/stop 命令序列。这是**历史 Gazebo proxy runtime**，不能证明 Isaac 6、Arena、Nav2 或当前路径可运行。

`[CONFIRMED_SOURCE]` V7 当前 `model.sdf`、工具脚本和报告仍硬编码 `/home/suat_wxb/Robot_URDF/...`，而本机实际路径是 `/home/user/navigation_project/robot_related/Robot_URDF/...`，旧绝对路径不存在。因此其历史运行证据有效，但**当前 exact V7 SDF 不是可移植、可直接复跑的模型**；还需修复 mesh URI，再重做 SDF validation 和 runtime 验收。

`[CONFIRMED_SOURCE]` V7 的 `0.70×0.62 m` 碰撞 box 可作为未来 footprint 候选的几何参考，但它不是 Arena/Nav2 已选定的 footprint；它与 Isaac 6 历史实测的约 `0.625×0.487 m` collision proxy 也不相同，必须通过明确的安全余量和实机尺寸决策来统一，不能直接抄值。

## 8.3 `roboos` 是独立的 Isaac Lab 移动操作线

`[CONFIRMED_SOURCE]` `robot_related/roboos` 是独立 Git repo（当前 HEAD `428127e2809219a53b64a33be853b2893a35b7d4`），README 指定 Python 3.10 + Isaac Sim 4.5.0.0 + Isaac Lab + PyTorch 2.5.1 + cuRobo。其主任务是 `allstar_toilet` 移动操作/清洁：不同刷子、吸口、拖把、夹爪的 Gymnasium task，车轮 joint velocity action、XMS5 joint position action、可选 LiDAR observation、cuRobo pose planning、交互录制与 FastAPI tool service。源码注册了 38 个 Mecanum0730/XMS5 task ID，但这些是 Isaac Lab environment，不是 ROS 2/Arena/Nav2 启动项。

`[CONFIRMED_SOURCE]` `roboos` 内的当前 default URDF 与外部 `robots/chassis_arm` 当前 URDF 逐字节相同；其 default USD 是 Git LFS pointer，pointer 中的 oid 与外部真实 default USD 的 SHA256 一致，说明两者有同源关系。但当前 checkout 的 chassis_arm 资产中有 116 个 Git LFS pointer，包括 default USD 的 wrapper/base/physics/sensor 层和多个 mesh；在 `git lfs pull` 或显式改用外部完整资产前，当前 `roboos` checkout 不能仅凭目录存在就视为可运行。

`[CONFIRMED_SOURCE]` 本轮未发现可与当前 commit/revision 绑定的 Isaac Lab 启动日志、HDF5 demo 或验收结果；README 中的 smoke-test 命令是运行说明，不是 `[CONFIRMED_RUNTIME]`。`roboos` 还有未跟踪目录 `2d_lidar/` 和 `baseline/`，因此其当前工作树也不是一个干净、已冻结的可复现 revision。

## 8.4 与 Arena 的真正边界

`[CONFIRMED_SOURCE]` 对 `arena-rosnav` 与 `simulation-setup` 全树不区分大小写搜索 `Mecanum730/XMS5` 为 0 命中；当前 Arena 没有该机器人的正式 model params、URDF/USD/SDF 引用、mapping、control、Nav2 参数或 launch。`robot_related` 中“已有资产”和“已有历史 Gazebo proxy”不会自动填上这个集成缺口。

`[CONFIRMED_SOURCE]` `rbkairos` 和 `rskomnidirectional` 虽标 `is_holonomic: true`，其 `control.yaml` 仍使用 DiffDrive 且只有 x/z；`rbvogui` 是唯一明确含 strafe/linear_y 与 x/y/z 控制限制的现成示例，但它不是 Mecanum730/XMS5。当前 Arena 的 `control.yaml` 也没有形成 active controller 消费链，state publisher include 又被注释，因此不能把其他机器人 YAML 的 `is_holonomic` 当成目标机器人闭环证据。

# 9. ROS topics / TF

## 9.1 当前与历史主链的 12-topic 合同

`[CONFIRMED_RUNTIME]` 两份历史 range-only bag 与 2026-08-12 15:03 RTX+强度 bag 中，下列 12 个 topic 均有消息。后者因而把 relay/merger 的强度合同升级为真实运行合同；但 `/cmd_vel` 来源仍是 teleop，不是 Nav2，且该 bag 早于四档避障源码。

| Topic | ROS type | 发布/消费责任 | 当前证据边界 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | teleop/Nav2 发布，relay 订阅并转 UDP | `[CONFIRMED_RUNTIME]` 当前 867 条；521 条 moving，来源为 teleop |
| `/cmd_vel_stamped` | `geometry_msgs/TwistStamped` | relay 在检测到 recorder raw subscriber 时镜像 | `[CONFIRMED_RUNTIME]` 当前 867 条 |
| `/scan_01` | `sensor_msgs/LaserScan` | Isaac telemetry 经 relay 发布；frame `base_scan_01` | `[CONFIRMED_RUNTIME]` 当前 139×360 ranges/intensities；约 3.195 Hz |
| `/scan_02` | `sensor_msgs/LaserScan` | Isaac telemetry 经 relay 发布；frame `base_scan_02` | `[CONFIRMED_RUNTIME]` 当前 139×360 ranges/intensities；约 3.195 Hz |
| `/scan` | `sensor_msgs/LaserScan` | relay 发布 front/legacy scan；frame `base_scan` | `[CONFIRMED_RUNTIME]` 当前 139×360 ranges/intensities |
| `/scan_merged` | `sensor_msgs/LaserScan` | `semantic_nav_gazebo` merger 合并两 scan；frame `base_link` | `[CONFIRMED_RUNTIME]` 当前 139×360 ranges/intensities |
| `/odom` | `nav_msgs/Odometry` | relay 发布模拟器位姿/速度 | `[CONFIRMED_RUNTIME]` 当前 414 条，机器人实际移动 |
| `/tf` | `tf2_msgs/TFMessage` | relay 动态发布 `odom→base_link` | `[CONFIRMED_RUNTIME]` 当前 414 帧 |
| `/tf_static` | `tf2_msgs/TFMessage` | relay 发布 base 到三 scan frames | `[CONFIRMED_RUNTIME]` 当前 1 帧含 3 个 transform |
| `/clock` | `rosgraph_msgs/Clock` | relay 从 telemetry 发布仿真时间 | `[CONFIRMED_RUNTIME]` 当前 414 条 |
| `/pedestrian_ground_truth` | `semantic_nav_gazebo/msg/PedestrianStateArray` | relay 直接从 UDP JSON 构造 typed message | `[CONFIRMED_RUNTIME]` 当前 406 条、每条 3 人；不是 `/isaac_sim/..._json` |
| `/data_collection/episode_event` | `std_msgs/String` | relay 发布手动 episode start/end | `[CONFIRMED_RUNTIME]` 当前 2 条，组成一个完整手动 episode |

`[CONFIRMED_RUNTIME]` 上表的数量来自已冻结的 15:03 bag，不等于 21:49 的 live graph 仍同时满足全部合同：后续 `isaac_slam_check` 明确看到 `/scan_01` 类型存在但 publisher=0，而 `/scan_02`、`/scan_merged` 仍各有 1 个 publisher。这个新失败不否定旧 bag 内容，但说明当前 live 双雷达发布连续性仍未闭合。

`[CONFIRMED_SOURCE]` `/points_merged_before_dedup` 与 `/points_merged_after_dedup` 只在 high-fidelity probe 打开时发布；默认 `enable_high_fidelity_probe: false`，所以不属于默认 topic 集。

`[CONFIRMED_SOURCE]` 当前没有 ROS collision event publisher；碰撞只存在于 Isaac 内部保护、日志和结果 counter。

`[CONFIRMED_SOURCE]` episode schema 是 `isaac_manual_teleop_episode/v1`：recorder 已订阅时首个非零命令开始，零命令持续 0.5 sim s 后结束；它不是带 navigation goal/success 的 Arena episode。

## 9.2 TF 责任边界

| Transform | 当前默认发布者 | 状态 |
|---|---|---|
| `odom → base_link` | `cmd_vel_udp_relay.py` | `[CONFIRMED_RUNTIME]` 当前 bag 414 帧，仅此一对动态 TF |
| `base_link → base_scan` | relay static broadcaster，translation `(0.2,0.13,0.208)`、yaw 0 | `[CONFIRMED_RUNTIME]` 当前 bag 已反序列化确认 |
| `base_link → base_scan_01` | relay，translation `(0.2,0.13,0.208)`、yaw 0 | `[CONFIRMED_RUNTIME]` 当前 bag 已反序列化确认 |
| `base_link → base_scan_02` | relay，translation `(-0.2,-0.13,0.208)`、yaw π | `[CONFIRMED_RUNTIME]` 当前 bag 已反序列化确认 |
| `map → odom` | 21:47 后启动的 Slam Toolbox 按配置负责 | `[INFERRED]` `/tf` 有 2 个 publisher 且 live `/map` 已确认，但未保存具体 TF lookup/frame 输出 |
| `base_link → wheel/arm links` | standalone 主链没有 robot_state_publisher | `[CONFIRMED_SOURCE]` 当前缺失 |

`[CONFIRMED_SOURCE]` Isaac stage 坐标映射为 ROS `+X → stage +X`、ROS `+Y → stage -Z`；scan frame 名是 `base_scan*`，不是 topic 名 `scan_01/scan_02`。

`[CONFIRMED_SOURCE]` `isaac_slam_online_async.yaml` 配置 `map/odom/base_link`、`/scan_merged` 和 TF publish period 0.05；`[CONFIRMED_RUNTIME]` 21:47 运行已实际启动 async Slam Toolbox、注册雷达并收到 live `/map`。但没有保存具体 `map→odom` lookup，也没有持久化地图文件。

# 10. Nav2

## 10.1 当前源码、build、install 与可发现性

| 层 | 当前状态 |
|---|---|
| Nav2 源码 | `[CONFIRMED_SOURCE]` `arena_ws/src/deps/nav2/navigation2`，package version 1.1.19（Humble） |
| Workspace build | `[CONFIRMED_BUILD]` 只构建 `nav2_common`、`nav2_msgs`、`nav2_util`、`nav2_map_server`，不是完整 Nav2 |
| Workspace install | `[CONFIRMED_INSTALL]` 上述四包从 `arena_ws/install` 可发现 |
| 系统 Nav2 | `[CONFIRMED_INSTALL]` `/opt/ros/humble` 为 1.1.20，controller/planner/AMCL/DWB/MPPI/SMAC/NavFn/Theta/RPP/RotationShim 可发现 |
| Graceful controller | `[CONFIRMED_SOURCE]` 本机 deps 源码存在；`[CONFIRMED_INSTALL]` 当前 ROS 环境不可发现 |
| Online SLAM runtime | `[CONFIRMED_RUNTIME]` Slam Toolbox/Ceres 已启动、注册 `/scan_merged` 传感器并发布 live `/map`；总检查仍 FAIL |
| Nav2 runtime | `[UNKNOWN]` 无现有 `navigate_to_pose` 成功、localization、costmap 或 lifecycle 日志 |

`[CONFIRMED_SOURCE]` 本机源码包含的 global planners：NavFn、SMAC 2D、SMAC Hybrid、SMAC State Lattice、Theta Star。

`[CONFIRMED_SOURCE]` 本机源码包含的 local controllers：DWB、MPPI、Regulated Pure Pursuit、Rotation Shim、Graceful。

`[CONFIRMED_SOURCE]` Arena 还列出 `crowdnav`、`crowdnav_attngraph`、`drlvo`、`sicnav` 等 controller 配置名称，但相应第三方包未 build/install，不能视为当前可用 controller。

`[CONFIRMED_RUNTIME]` 15:03 的 12-topic bag 本身没有 `/map`、`goal_pose`、NavigateToPose action/status、costmap、path 或 lifecycle topic；21:47 的后续 live run 才新增 `/map`。两者都没有 Nav2 goal/action 成功，因此仍不构成 Level 3。

## 10.2 Mecanum/Omni 真实性

`[CONFIRMED_SOURCE]` 当前 Arena DWB 配置的 `min_vel_y=0`、`max_vel_y=0`、`acc_lim_y=0`、`decel_lim_y=0`，虽有 `vy_samples=5`，实际仍按 DiffDrive 禁止横移。

`[CONFIRMED_SOURCE]` 当前 Arena MPPI 配置虽写 `vy_std=0.2`、`vy_max=0.5`、`ay_max=3`，却明确 `motion_model: "DiffDrive"`，这些 y 参数不构成 Omni 运行配置。

`[CONFIRMED_SOURCE]` 本机 Nav2 1.1.19 MPPI 源码真实支持 `DiffDrive`、`Omni`、`Ackermann` 三种 motion model；Omni model 会处理 `vx/vy/wz`。所以问题不是“当前版本不支持 Omni”，而是 Arena 配置没有选择 Omni。

`[CONFIRMED_SOURCE]` 当前 AMCL 参数为 `DifferentialMotionModel`；目标 Mecanum730 又没有专属 Nav2 model params、速度/加速度限制和 footprint，因此完整全向 localization/controller 配置不存在。

`[CONFIRMED_SOURCE]` 通用 `model_params.yaml` 仍使用 `/front/scan`，且 footprint 只是约 `0.2×0.2 m` 的 placeholder；它既不匹配当前 `/scan_merged` 合同，也不匹配 Mecanum730 的实测/代理尺寸。

`[UNKNOWN]` 即使改为 Omni，当前机器人实际横移运动学、footprint、costmap、goal behavior 与外部 UDP relay 是否能共同通过 Nav2 验收，尚无运行证据。

## 10.3 2026-08-12 21:47 在线 SLAM 实测

| 检查项 | 当前证据 | 结论 |
|---|---|---|
| Slam Toolbox 启动 | `maps/logs/20260812_214744_isaac_slam_toolbox.log:1-4`：40M stack、Ceres/SCHUR_JACOBI、`Registering sensor: [Custom Described Lidar]` | `[CONFIRMED_RUNTIME]` 节点真实运行 |
| live map | `...214948_isaac_slam_check.log:9,12,15-17`：`/map` 1 publisher，live `/scan_merged`、`/clock`、`/map` 均 PASS | `[CONFIRMED_RUNTIME]` 在线 OccupancyGrid 已产出 |
| 双雷达合同 | `/scan_02`、merged、odom、TF、clock、map PASS；`/scan_01` 类型正确但 publisher=0 | `[CONFIRMED_RUNTIME]` 总检查 `ISAAC_SLAM_CHECK=FAIL failures=1` |
| RViz | `...214744_isaac_slam_rviz.log:1-2`：Snap core20 `libpthread` 缺 `__libc_pthread_init@GLIBC_PRIVATE`，exit 127 | `[CONFIRMED_RUNTIME]` 可视化未启动 |
| 地图保存 | `maps/` 仅有三份日志，无 `.yaml/.pgm` 或 posegraph | `[CONFIRMED_SOURCE]` 未持久化 |
| 运动覆盖 | 关联 Isaac 日志只有 yaw 变化，XYZ 固定 | `[CONFIRMED_RUNTIME]` 只证明旋转式建图，不证明平移探索/覆盖质量 |

`[INFERRED]` `/tf` 在检查时有两个 publisher，结合配置的 `transform_publish_period=0.05`，第二个 publisher 很可能正在发布 `map→odom`；由于本轮没有保存 `tf2_echo` 或具体 TF message frame 对，此项不能升级为直接确认。

`[CONFIRMED_RUNTIME]` 审计快照时没有 planner server、controller server、BT navigator、AMCL、goal/action 或导航成功产物。在线 `/map` 是 Level 3 的重要前置能力，但不是 Nav2 自动导航本身。

# 11. Arena Task Generator

## 11.1 模式不是一个单一枚举

| 类别 | 当前源码中的模式 | 状态 |
|---|---|---|
| Robot task mode | `guided`、`explore`、`random`、`scenario` | `[CONFIRMED_SOURCE]` 均已注册 |
| Obstacle task mode | `parametrized`、`random`、`scenario`、`environment` | `[CONFIRMED_SOURCE]` 均已注册 |
| Task modules | `staged`、`dynamic_map`、`clear_forbidden_zones`、`rviz_ui`、`benchmark` | `[CONFIRMED_SOURCE]` 均已注册 |

`[CONFIRMED_SOURCE]` 因此用户点名的 Random、Scenario、Staged、Parametrized、Benchmark 都存在，但分别属于 robot/obstacle/module 三个正交维度，不能当成同一个 mode 互斥表。

## 11.2 start/goal/reset/success/timeout/实体责任

| 责任 | 当前实现 | 状态 |
|---|---|---|
| start | Random 从 occupancy map 取安全点；Scenario 读 world scenario；RobotManager 调 simulator 移动机器人 | `[CONFIRMED_SOURCE]` |
| goal | guided/explore/random/scenario 选择；RobotManager 发布 map-frame `goal_pose`，每 3 s 重发，最长 60 s | `[CONFIRMED_SOURCE]` |
| success | 只看 `navigate_to_pose/_action/status` 最新状态是否 `STATUS_SUCCEEDED` | `[CONFIRMED_SOURCE]`；goal tolerance 参数读取后未使用 |
| timeout | robot task 用 `/clock.sec-last_reset` 比较 timeout；默认 `-1` 被解释为 infinity | `[CONFIRMED_SOURCE]` 默认没有 timeout |
| reset | 0.5 s 周期检查 `task.is_done` 或 `reset_task` service；modules-before → start/goal → obstacles → respawn → modules-after | `[CONFIRMED_SOURCE]` |
| static/dynamic obstacles | obstacle mode 生成，EnvironmentManager 经 simulator adapter spawn | `[CONFIRMED_SOURCE]` |
| pedestrians | human manager 建模，再委托 physics backend；Isaac 路径另有特殊分支 | `[CONFIRMED_SOURCE]` |
| collision termination | Task Generator 没有 contact/bumper/collision subscriber | `[CONFIRMED_SOURCE]` 碰撞不会直接结束 task |

`[CONFIRMED_SOURCE]` 默认 `tm_robots=explore` 在成功时直接换 goal、超时时换 start，并始终返回 false，因此默认 explore 不产生“episode done → 自动 reset”。`_auto_reset` 被读取但没有实际检查；`finished` publisher 被创建但没有 publish。

`[CONFIRMED_SOURCE]` staged 仍混用 ROS 1 `rospy`/`rosrun dynamic_reconfigure`；benchmark 存在不可达代码；这些模式的注册不等于在 ROS 2 当前环境可运行。

`[CONFIRMED_SOURCE]` Task Generator 发布 namespaced `task_reset`；Arena Evaluation recorder 订阅 `/scenario_reset`，仓库内没有找到 bridge/publisher 补上该名称差异。

`[CONFIRMED_BUILD]` `task_generator` 已 build；`[CONFIRMED_INSTALL]` overlay 可发现；`[UNKNOWN]` 没有完成一次自动 start→goal→success/timeout→reset 的运行日志。

# 12. Arena Evaluation

## 12.1 能记录什么

`[CONFIRMED_SOURCE]` `setup.py` 的 `record` entry point 调用 `data_recorder_node:main`，而 main 实例化的是 `BagRecorder`；它将 scan、`scenario_reset`、odom、cmd_vel、human_states 写入 rosbag。

`[CONFIRMED_SOURCE]` 旧 CSV DataCollector 源码仍能写 scan/odom/cmd_vel/episode/start_goal，但当前 `record` main 不实例化它。

## 12.2 源码实际列出的指标

`[CONFIRMED_SOURCE]` 基础 metrics 返回：`curvature`、`normalized_curvature`、`roughness`、`path_length_values`、`path_length`、`acceleration`、`jerk`、`velocity`、`collision_amount`、`collisions`、`path`、`angle_over_length`、`time_diff`、`time`、`episode`、`result`、`goal`、`start`；`cmd_vel` 和 `action_type` 已被注释，不是当前输出。

`[CONFIRMED_SOURCE]` Pedsim 扩展 metrics 返回：`num_pedestrians`、`avg_velocity_in_personal_space`、`total_time_in_personal_space`、`time_in_personal_space`、`total_time_looking_at_pedestrians`、`time_looking_at_pedestrians`、`total_time_looked_at_by_pedestrians`、`time_looked_at_by_pedestrians`。

`[CONFIRMED_SOURCE]` 绘图源码包含 result count、单 episode line/distribution、跨 episode aggregate line/distribution、categorical/distribution、按 namespace 的 path、best path，以及 strip/swarm/box/violin/boxen/point/bar/count 等 seaborn 图。

## 12.3 不能工作的连接点

- `[CONFIRMED_SOURCE]` 当前 recorder 输出 rosbag，而 metrics 仍硬读 `odom.csv`、`scan.csv`、`episode.csv`、`start_goal.csv`；recorder→metrics 格式断链。
- `[CONFIRMED_SOURCE]` Task Generator 发布 `task_reset`，recorder 订阅 `/scenario_reset`；事件名称断链。
- `[CONFIRMED_SOURCE]` metrics 将 position 和 velocity 都读成 `frame["position"]`，因此 velocity/acceleration/jerk 计算输入错误。
- `[CONFIRMED_SOURCE]` robot model 参数被硬编码到不存在的顶层 `waffle/model_params.yaml`；当前 waffle 只在隐藏 `.old` 中。
- `[CONFIRMED_SOURCE]` result 不是 Nav2 action 的真实结果，而是按 180 s timeout 或 3 次 lidar 阈值 collision 推断。
- `[CONFIRMED_SOURCE]` `setup.py` 只安装顶层 package、`create_plots.py` 又导入不存在的 `arena_evaluation.utils`，另有旧路径/ament 声明问题。

`[CONFIRMED_BUILD]` `arena_evaluation` 与 `arena_evaluation_msgs` 均无 build；`[CONFIRMED_INSTALL]` overlay 不可发现；`[UNKNOWN]` 没有 recorder、metrics 或 plot 的成功 runtime/output 数据。

`[CONFIRMED_SOURCE]` Evaluation 的 data/plots 输出目录当前仍只有 `.gitkeep`；最新 standalone bag 也没有 `/scenario_reset`、`human_states` 或 evaluation result topic，不能当作 Arena Evaluation 产物。

# 13. HuNav

## 13.1 HuNavSim 与 evaluator 的职责

`[CONFIRMED_SOURCE]` HuNavSim 的 `hunav_agent_manager` 用行为树/社会力模型更新 agent，提供 compute_agents、compute_agent、move_agent、reset_agents 服务，并发布 human_states、robot_states、people。

`[CONFIRMED_SOURCE]` `hunav_evaluator` 是独立节点，订阅 human_states、robot_states、goal_pose，支持 service/automatic 记录方式，并写 aggregate summary TSV 与逐步 TSV；Arena 的 HuNav launch 只声明启动 manager，没有启动 evaluator。

## 13.2 实际注册的 29 项 social-navigation metrics

`[CONFIRMED_SOURCE]` 本机 registry 的全部 29 项为：

1. `time_to_reach_goal`
2. `path_length`
3. `cumulative_heading_changes`
4. `avg_distance_to_closest_person`
5. `minimum_distance_to_people`
6. `maximum_distance_to_people`
7. `intimate_space_intrusions`
8. `personal_space_intrusions`
9. `social_space_intrusions`
10. `group_intimate_space_intrusions`
11. `group_personal_space_intrusions`
12. `group_social_space_intrusions`
13. `completed`
14. `minimum_distance_to_target`
15. `final_distance_to_target`
16. `robot_on_person_collision`
17. `person_on_robot_collision`
18. `time_not_moving`
19. `avg_robot_linear_speed`
20. `avg_robot_angular_speed`
21. `avg_acceleration`
22. `avg_overacceleration`
23. `avg_pedestrian_velocity`
24. `avg_closest_pedestrian_velocity`
25. `social_force_on_agents`
26. `social_force_on_robot`
27. `social_work`
28. `obstacle_force_on_robot`
29. `obstacle_force_on_agents`

`[CONFIRMED_SOURCE]` proxemic 阈值为 intimate `<0.45 m`、personal `0.45–1.2 m`、social `1.2–3.6 m`、public `≥3.6 m`。`path_irregularity`、`path_efficiency`、`static_obs_collision` 只有 TODO/pass，未注册，不能列作已实现指标。

## 13.3 当前断链和缺陷

`[CONFIRMED_SOURCE]` 当 `sim=isaac` 时总 launch 默认 `human=hunav`，但 HuNav wrapper 的 non-Gazebo 分支在循环内提前 `return obstacles`，后面的 `ComputeAgents` 注册和 10 Hz update timer 不可达；随后 BaseHumanSimulator 把动态障碍交给 IsaacSimulator，由后者调用 `spawn_pedestrian`/`move_pedestrians`。所以默认 Isaac 行人走 Isaac MovePed，不是 HuNav SFM/BT 持续控制。

`[CONFIRMED_SOURCE]` evaluator 配置写 `use_navgoal_to_start`，节点读取 `use_nav_goal_to_start`；semi-auto stop 使用比较 `self.recording == False` 而非赋值；behavior 过滤还按不存在的扁平字段访问，而消息实际是 `behavior.type/state`。

`[CONFIRMED_BUILD]` `hunav_agent_manager`、`hunav_evaluator`、`hunav_msgs`、`hunav_sim`、`hunav_rviz2_panel` 均无 build；`[CONFIRMED_INSTALL]` overlay 不可发现；`[UNKNOWN]` 没有 HuNav manager/evaluator runtime 或指标输出。

# 14. DRL-VO

| 字段 | 审计结果 |
|---|---|
| 算法 | `[CONFIRMED_SOURCE]` DRL-VO（PPO policy + scan/path/robot state 的 Nav2 controller wrapper） |
| 源码路径 | `[CONFIRMED_SOURCE]` `isaac_sim/arena_ws/src/planners/Drl_vo` |
| ROS package | `[CONFIRMED_SOURCE]` `nav2py_drl_vo_controller`；支持消息包 `cnn_msgs` |
| 权重 | `[CONFIRMED_SOURCE]` `Drl_vo/nav2py_drl_vo_controller/model/drl_vo/policy.pth`（10,637,750 bytes）、`policy.optimizer.pth`（21,126,037 bytes）、`pytorch_variables.pth`（431 bytes） |
| 输入 | `[CONFIRMED_SOURCE]` C++ 从 local costmap 获得 LaserScan、robot pose/twist、global path；Python ControllerV2 最多使用 10 帧、resize 到 720 点，并创建 80×80×2 pedestrian map |
| 人群输入真实性 | `[CONFIRMED_SOURCE]` 集成路径的 pedestrian map 是 zero-filled，不消费 live human state；旧 standalone `cnn_data` 源码不等于当前 integrated path |
| 输出 | `[CONFIRMED_SOURCE]` `linear.x`、`angular.z` |
| `linear.y` | `[CONFIRMED_SOURCE]` 不支持/不输出 |
| Source | `[CONFIRMED_SOURCE]` 存在 |
| Build | `[CONFIRMED_BUILD]` 无 build 目录/rc |
| Install/ROS discoverable | `[CONFIRMED_INSTALL]` 无 install；`ros2 pkg prefix nav2py_drl_vo_controller` 与 `cnn_msgs` 均 NOT_FOUND |
| 依赖 | `[CONFIRMED_SOURCE]` Python 3.8.5、Stable-Baselines3 1.1.0、Torch 1.7.1+cu110、Pandas 1.2.1、protobuf 3.20 等旧 pin |
| Runtime | `[UNKNOWN]` 无 controller load、inference 或 cmd_vel 成功日志；与系统 Python 3.10/当前 GPU 的兼容性未验证 |

# 15. CrowdNav

`[CONFIRMED_SOURCE]` 本节指 `CrowdNav Prediction AttnGraph`，不是泛称所有 crowd controller。

| 字段 | 审计结果 |
|---|---|
| 算法 | `[CONFIRMED_SOURCE]` CrowdNav Prediction AttnGraph / GST predictor |
| 源码路径 | `[CONFIRMED_SOURCE]` `isaac_sim/arena_ws/src/planners/CrowdNav_Prediction_AttnGraph` |
| ROS package | `[CONFIRMED_SOURCE]` `nav2py_crowdnav_attngraph_controller`；另有 `crowdnav_base` 与 `dr_spaam_ros` 源码 |
| 权重 | `[CONFIRMED_SOURCE]` `src/nav2py_crowdnav_attngraph_controller/nav2py_crowdnav_attngraph_controller/nav2py_crowdnav_attngraph_controller/GST_predictor_rand/checkpoints/41665.pt`（10,027,146 bytes） |
| 输入 | `[CONFIRMED_SOURCE]` C++ bridge 发送 robot pose、当前 Twist、global plan；没有把 live scan/person state 送进 Python policy |
| 人群输入真实性 | `[CONFIRMED_SOURCE]` Python 侧创建内部 crowd simulation environment，不能视为感知当前 Isaac 行人 |
| 输出 | `[CONFIRMED_SOURCE]` `linear.x`、`angular.z` |
| `linear.y` | `[CONFIRMED_SOURCE]` 不支持/不输出 |
| Source | `[CONFIRMED_SOURCE]` 存在 |
| Build | `[CONFIRMED_BUILD]` 无 build 目录/rc |
| Install/ROS discoverable | `[CONFIRMED_INSTALL]` `nav2py_crowdnav_attngraph_controller`、`crowdnav_base` 均 NOT_FOUND |
| 依赖 | `[CONFIRMED_SOURCE]` Python 3.10、Torch 1.12.1+cu116、Gym 0.15.7、tensorflow-gpu 2.11、Pandas 1.5.2；相对 `MODEL_DIR` 还有 install layout 风险 |
| Runtime | `[UNKNOWN]` 无 Nav2 plugin load/inference/cmd_vel 日志 |

# 16. PaS-CrowdNav

| 字段 | 审计结果 |
|---|---|
| 算法 | `[CONFIRMED_SOURCE]` PaS-CrowdNav（VAE + policy，基于时序 local costmap） |
| 源码路径 | `[CONFIRMED_SOURCE]` `isaac_sim/arena_ws/src/planners/PaS_CrowdNav` |
| ROS package | `[CONFIRMED_SOURCE]` `nav2py_pas_crowdnav_controller` |
| 权重 | `[CONFIRMED_SOURCE]` `models/policy.pt`（5,895,531 bytes）、`models/vae.pth`（3,370,171 bytes） |
| 输入 | `[CONFIRMED_SOURCE]` C++ 序列化完整 local costmap、pose/twist、变换后的 global path；Python 使用 96×96 时序图、robot/goal |
| 输出 | `[CONFIRMED_SOURCE]` 默认 unicycle action，Nav2 bridge 最终只发 `linear.x`、`angular.z` |
| `linear.y` | `[CONFIRMED_SOURCE]` 没有 ROS 输出；内部 holonomic 分支也把 vx/vy 转成标量速度+heading |
| Source | `[CONFIRMED_SOURCE]` 存在 |
| Build | `[CONFIRMED_BUILD]` 无 build 目录/rc |
| Install/ROS discoverable | `[CONFIRMED_INSTALL]` package NOT_FOUND |
| 依赖 | `[CONFIRMED_SOURCE]` Python 3.8.x、Torch/Torchvision、NumPy 1.23.x、Gym 等 |
| Runtime | `[UNKNOWN]` 无 VAE/policy load、Nav2 controller 或 cmd_vel 成功日志 |

# 17. SICNav

| 字段 | 审计结果 |
|---|---|
| 算法 | `[CONFIRMED_SOURCE]` SICNav / CAMPC，使用 CasADi、IPOPT、RVO2 的在线优化，不是 checkpoint policy |
| 源码路径 | `[CONFIRMED_SOURCE]` `isaac_sim/arena_ws/src/planners/SICNav` |
| ROS package | `[CONFIRMED_SOURCE]` `nav2py_sicnav_controller` |
| 权重 | `[CONFIRMED_SOURCE]` 无 `.pt/.pth/.onnx/.ckpt`；算法不依赖训练权重 |
| 输入 | `[CONFIRMED_SOURCE]` global path、LaserScan（costmap 或 `/scan`）、odom；默认 odom topic 硬编码 `/task_generator_node/jackal/odom`，并把 scan cluster 当 stationary humans |
| 输出 | `[CONFIRMED_SOURCE]` 优化得到 `v,r`，bridge 发 `linear.x`、`angular.z` |
| `linear.y` | `[CONFIRMED_SOURCE]` 不支持/不输出 |
| Source | `[CONFIRMED_SOURCE]` 存在 |
| Build | `[CONFIRMED_BUILD]` 无 build 目录/rc |
| Install/ROS discoverable | `[CONFIRMED_INSTALL]` package NOT_FOUND |
| 依赖 | `[CONFIRMED_SOURCE]` Python 3.8.13、CasADi 3.6.5、NumPy 1.24.4、Torch 1.13.1、Torchvision 0.14.1、RVO2/IPOPT 等本地/pip 依赖 |
| Runtime | `[UNKNOWN]` 无求解器初始化、Nav2 plugin load 或 cmd_vel 成功日志 |

`[CONFIRMED_SOURCE]` Planner 相关权重的完整本机清单就是：DRL-VO 的 3 个 `.pth`、PaS 的 `policy.pt` 与 `vae.pth`、CrowdNav 的 `41665.pt`；未发现 Planner 相关 `.onnx` 或 `.ckpt`，SICNav 无权重。

`[CONFIRMED_SOURCE]` 现有 Arena benchmark 配置也没有把这四个重点 Planner 组成可运行 suite：默认 simulator 是 Gazebo、basic contest 使用 TEB，`allplanners` 只列旧的 TEB/DWA/Dragon/Rosnav/APPLR/LFLH/Trail/CoHAN，且多处 world 引用隐藏 `.old` 目录；其中对 DRL-VO、CrowdNav、PaS、SICNav、Mecanum730 均为 0 命中。因此“源码/权重存在”之外，还缺正式 benchmark 配置。

`[CONFIRMED_INSTALL]` 当前系统 Python 3.10.12 未发现 Planner 专用 venv/conda；对声明的关键 Python 依赖逐项查询时，仅见 NumPy 1.26.4 与 protobuf 3.12.4，未发现 torch、torchvision、Stable-Baselines3、gym、tensorflow、pandas、casadi 或 rvo2。现有权重因此还没有一个就绪的共同推理环境。

# 18. Build / Install 状态

## 18.1 arena_ws 实际成功的 15 个包

`[CONFIRMED_BUILD]` `arena_ws/build` 恰有下列 15 个包，每个 `colcon_build.rc=0`；`[CONFIRMED_INSTALL]` 每个都有 ament package marker，source `/opt/ros/humble` 与 `arena_ws/install/setup.bash` 后均解析到 workspace install。

| # | Package | BUILD | INSTALL / ROS discoverable |
|---:|---|---|---|
| 1 | `arena_bringup` | `[CONFIRMED_BUILD]` rc=0 | `[CONFIRMED_INSTALL]` workspace |
| 2 | `arena_rclpy_mixins` | `[CONFIRMED_BUILD]` rc=0 | `[CONFIRMED_INSTALL]` workspace |
| 3 | `arena_simulation_setup` | `[CONFIRMED_BUILD]` rc=0 | `[CONFIRMED_INSTALL]` workspace |
| 4 | `bond` | `[CONFIRMED_BUILD]` rc=0 | `[CONFIRMED_INSTALL]` workspace |
| 5 | `bondcpp` | `[CONFIRMED_BUILD]` rc=0 | `[CONFIRMED_INSTALL]` workspace |
| 6 | `isaacsim_msgs` | `[CONFIRMED_BUILD]` rc=0 | `[CONFIRMED_INSTALL]` active schema, workspace |
| 7 | `jackal_description` | `[CONFIRMED_BUILD]` rc=0 | `[CONFIRMED_INSTALL]` workspace |
| 8 | `nav2_common` | `[CONFIRMED_BUILD]` rc=0 | `[CONFIRMED_INSTALL]` workspace |
| 9 | `nav2_map_server` | `[CONFIRMED_BUILD]` rc=0 | `[CONFIRMED_INSTALL]` workspace |
| 10 | `nav2_msgs` | `[CONFIRMED_BUILD]` rc=0 | `[CONFIRMED_INSTALL]` workspace |
| 11 | `nav2_util` | `[CONFIRMED_BUILD]` rc=0 | `[CONFIRMED_INSTALL]` workspace |
| 12 | `ros2isaacsim` | `[CONFIRMED_BUILD]` rc=0 | `[CONFIRMED_INSTALL]` workspace |
| 13 | `rviz_utils` | `[CONFIRMED_BUILD]` rc=0 | `[CONFIRMED_INSTALL]` workspace |
| 14 | `smclib` | `[CONFIRMED_BUILD]` rc=0 | `[CONFIRMED_INSTALL]` workspace |
| 15 | `task_generator` | `[CONFIRMED_BUILD]` rc=0 | `[CONFIRMED_INSTALL]` workspace |

`[CONFIRMED_BUILD]` 主构建命令记录在 `log/build_2026-08-09_13-21-30/logger_all.log:1`：`colcon build --symlink-install --packages-up-to isaacsim_msgs ros2isaacsim arena_simulation_setup task_generator arena_bringup`；之后另有 rviz_utils、arena_bringup、ros2isaacsim 的成功重建日志。

`[CONFIRMED_BUILD]` 抽查总 launch、Task Generator node、Nav2 launch 和 robot 配置的 source/build 副本 hash 一致；Simulation Setup build 的 stderr 仍有缺失 namespace `__init__.py` 警告。不存在 `colcon_test.rc`/test log，所以不能声称测试通过。

## 18.2 明确未构建/未安装的部分

| 范围 | BUILD | INSTALL/discoverable | RUNTIME |
|---|---|---|---|
| `arena_isaac5_backup` 两包 | `[CONFIRMED_BUILD]` 无 | `[CONFIRMED_INSTALL]` `arena_isaac` NOT_FOUND | `[UNKNOWN]` |
| Arena Evaluation/messages | `[CONFIRMED_BUILD]` 无 | `[CONFIRMED_INSTALL]` NOT_FOUND | `[UNKNOWN]` |
| HuNav manager/evaluator/messages/meta/panel | `[CONFIRMED_BUILD]` 无 | `[CONFIRMED_INSTALL]` NOT_FOUND | `[UNKNOWN]` |
| DRL-VO/CNN msgs | `[CONFIRMED_BUILD]` 无 | `[CONFIRMED_INSTALL]` NOT_FOUND | `[UNKNOWN]` |
| CrowdNav/AttnGraph | `[CONFIRMED_BUILD]` 无 | `[CONFIRMED_INSTALL]` NOT_FOUND | `[UNKNOWN]` |
| PaS-CrowdNav | `[CONFIRMED_BUILD]` 无 | `[CONFIRMED_INSTALL]` NOT_FOUND | `[UNKNOWN]` |
| SICNav | `[CONFIRMED_BUILD]` 无 | `[CONFIRMED_INSTALL]` NOT_FOUND | `[UNKNOWN]` |
| Nav2 controller/planner workspace packages | `[CONFIRMED_BUILD]` 无 | `[CONFIRMED_INSTALL]` 多数由 `/opt/ros/humble` 1.1.20 提供，而非 workspace | `[UNKNOWN]` |

`[CONFIRMED_BUILD]` 前一长任务与本次审计均没有执行 `colcon build`；本表描述的是预先存在的 Aug 9 产物。

`[CONFIRMED_SOURCE]` 顶层 `build/install/log` 主要属于 `semantic_nav_gazebo` 数据管线，不应与 `isaac_sim/arena_ws/build/install/log` 混为一谈。

`[CONFIRMED_BUILD]`/`[CONFIRMED_INSTALL]` 8 月 12 日复核时，`arena_ws/build`、`install` 仍恰好是上述 15 包，mtime 均停在 8 月 9 日，最新 workspace log 仍是 8 月 10 日的 `colcon list`；这直接确认近期 RTX 工作没有同步产生 Arena build/install。

# 19. 版本关系

| 组件 | 当前实际版本/身份 | 关系或风险 |
|---|---|---|
| OS/kernel | `[CONFIRMED_INSTALL]` Ubuntu 22.04.5 LTS Jammy，kernel 6.8.0-136-generic | ROS Humble 的宿主环境 |
| ROS 2 | `[CONFIRMED_INSTALL]` Humble，`ROS_VERSION=2` | 系统 Python 3.10.12 |
| Isaac 6 | `[CONFIRMED_INSTALL]` `isaacsim-6.0.1` = `6.0.1-rc.7+release.42383.32955d8d.gl`，Python 3.12.13 | UDP 隔离 rclpy ABI；RTX lineage 已有功能性 Level 2/在线 `/map`，但当前四档避障 exact source 尚无 runtime，严格验收未过 |
| Isaac 5 | `[CONFIRMED_INSTALL]` `/home/user/isaacsim/5.1.0` = `5.1.0-rc.19+release.26219.9c81211b.gl`，Python 3.11.13 | native legacy 与 container-style backup 是两条不同路径 |
| `/isaac-sim` / Docker | `[CONFIRMED_INSTALL]` 路径不存在；docker command 不存在 | backup 的硬编码前提不满足 |
| Arena-Rosnav core | `[UNKNOWN]` checkout 无 Git metadata | `.repos` pin 只能说明意图，不能证明 core exact commit |
| active Isaac adapter | `[CONFIRMED_SOURCE]` detached `a4beefe0`，另有 1 个预存 importer diff | `.repos` 指 `master@a4beefe0`；精确 runtime 未证实 |
| 5.1 backup | `[CONFIRMED_SOURCE]` 90/90 文件等同 `arena5-isaac5.1.0@16b8e341` | 未 build/install，且含静态 bug |
| active `isaacsim_msgs` | `[CONFIRMED_INSTALL]` active 7-msg/12-srv schema | 与 backup 同名 0.0.0、schema 不兼容 |
| Gazebo V7 robot proxy | `[CONFIRMED_RUNTIME]` 2026-06-17 历史验收；Ignition VelocityControl/OdometryPublisher + 双 GPU LiDAR | 当前 mesh URI 仍指向不存在的 `/home/suat_wxb/...`，不与 Arena/Isaac 共享 runtime |
| `roboos` | `[CONFIRMED_SOURCE]` Isaac Sim 4.5.0.0 / Python 3.10 / Torch 2.5.1 / cuRobo 的文档约束 | 独立 Isaac Lab 移动操作工程；关键资产仍是 Git LFS pointer，无当前 runtime 证据 |
| Nav2 source | `[CONFIRMED_SOURCE]` workspace 1.1.19 | 只 overlay 四个已构建包 |
| Nav2 system | `[CONFIRMED_INSTALL]` `/opt/ros/humble` 1.1.20 | 形成 1.1.19/1.1.20 混合 overlay，ABI/行为需运行验证 |
| Arena installer/tool defaults | `[CONFIRMED_SOURCE]` installer/source helper 仍指 Isaac 4.2 / `~/isaacsim-4.2.0` | 与本机 5.1/6.0.1 路线漂移 |
| Planner Python/CUDA | `[CONFIRMED_SOURCE]` 各自 pin Python 3.8/3.10、Torch 1.7/1.12/1.13、CUDA 11.0/11.6 等 | 系统 Python 3.10.12 未发现专用 venv/conda；已查的 torch/torchvision/SB3/gym/tensorflow/pandas/casadi/rvo2 均未安装，当前不是 Planner 运行环境 |

`[CONFIRMED_SOURCE]` stable Isaac 6 外部 UDP 方案的优势只是绕开 Python ABI，不代表它已实现 Arena service/spawn/reset API。

`[UNKNOWN]` 顶层 `.git` 是空目录，历史 Level 2 成功无法绑定到 commit/source hash；新日志也没有嵌入 source hash。当前只能以已记录的文件 SHA256、mtime、日志 schema 和行为签名建立强关联，不能声称每次运行都已 bit-for-bit 绑定 revision。

# 20. 当前能力等级

能力等级严格采用任务给定定义，而不是重新定义：Level 0=模型只能显示；Level 1=能遥控；Level 2=传感器/TF/odom/cmd_vel 完整；Level 3=Nav2 自动导航；Level 4=Arena 自动 start/goal/reset；Level 5=Arena Evaluation 自动记录；Level 6=多算法 Benchmark。

| Level | 所需能力 | 历史 Isaac 6 pre-intensity | 当前 RTX 实现 lineage | Arena 集成线 |
|---:|---|---|---|---|
| 0 | 模型显示 | `[CONFIRMED_RUNTIME]` warehouse、robot、3 pedestrians | `[CONFIRMED_RUNTIME]` warehouse、robot、3 pedestrians、RTX sensors | `[UNKNOWN]` 无 Arena launch runtime |
| 1 | 能遥控 | `[CONFIRMED_RUNTIME]` 583 cmd、机器人移动及 teleop bag | `[CONFIRMED_RUNTIME]` 旧 off/dodge lineage 收到 cmd 并运动；新 bag 另有 521 条 moving cmd | `[UNKNOWN]` |
| 2 | sensors/TF/odom/cmd_vel 完整 | `[CONFIRMED_RUNTIME]` 两份 12-topic bag；scan intensity 为空 | `[CONFIRMED_RUNTIME]` **功能性完成**：12 topics、360 ranges/intensities、TF/odom/teleop；严格 10 Hz/行人门槛失败 | `[UNKNOWN]` |
| 3 | Nav2 自动导航 | `[UNKNOWN]` 无 goal/action/localization 成功证据 | `[CONFIRMED_RUNTIME]` live `/map` 前置能力已出现；**仍无 Nav2 goal/action/planner/controller 成功** | `[CONFIRMED_SOURCE]` launch/config 存在；无 Arena runtime |
| 4 | Arena 自动 start/goal/reset | `[UNKNOWN]` | `[UNKNOWN]` | `[CONFIRMED_SOURCE]` Task 逻辑存在且包已 build/install；无完整 episode runtime |
| 5 | Arena Evaluation 自动记录 | `[UNKNOWN]` | `[UNKNOWN]` | `[CONFIRMED_SOURCE]` recorder 源码存在但未 build/install且 topic/格式断链 |
| 6 | 多算法 Benchmark | `[UNKNOWN]` | `[UNKNOWN]` | `[CONFIRMED_SOURCE]` benchmark/Planner 源码存在但均未 build/install/runtime |

`[CONFIRMED_RUNTIME]` **历史 pre-intensity 与当前 RTX 实现 lineage 的最高确认能力都到 Level 2；当前 lineage 额外证明了非零强度。**

`[CONFIRMED_RUNTIME]` **Level 2 是已运行 lineage 的功能能力结论，不是严格质量门全 PASS，也不是当前 exact source 的验收。** 完整 bag 使用旧 `dodge=true` override；旧 off 模式有短程 sensor PASS、READY、cmd 和运动证据。当前 `off/native/gentle/legacy_dodge` exact source 于 21:51 才落盘，晚于最新进程启动，四档均尚无该 bytes 的完整 bag。

`[CONFIRMED_RUNTIME]` 若单独按 topics/TF/odom/cmd_vel 标准评级，Gazebo V7 robot proxy 有一次历史 Level 2 验收；该证据不是 Arena Level 4+ 证据，也不改变“当前 exact 文件因绝对路径无法原样复跑”的结论。

`[UNKNOWN]` 整个当前系统没有 Level 3–6 的真实运行证明；源码、build 或权重文件不能提高 runtime 等级。

# 21. 已确认事实

## CONFIRMED_RUNTIME

- `[CONFIRMED_RUNTIME]` 历史 Isaac 6 pre-intensity 链运行过 warehouse、Mecanum visual/kinematic proxy、3 名行人、遥控、双 range scan、odom、TF、clock、typed pedestrian truth、episode event 和 rosbag，达到 Level 2。
- `[CONFIRMED_RUNTIME]` 两份历史 bag 的所有 LaserScan intensities 均为空，不能作为 RTX+强度成功证据。
- `[CONFIRMED_RUNTIME]` 当前 RTX lineage 已多次出现 first-scan/warmup/READY；`012507` 有短程 RESULT PASS，`150345` bag 有完整 12-topic、四路 360 槽非零强度、TF、odom、teleop、3 人真值与 episode，功能性达到 Level 2。
- `[CONFIRMED_RUNTIME]` 最新 bag 的 raw scan 仅约 3.195 Hz，行人 velocity median error 为 0.511856 m/s；对应长跑又在约 2333 秒后发生 native anim plugin crash，因此严格稳定验收仍未通过。
- `[CONFIRMED_RUNTIME]` 21:47 关联运行已启动 Slam Toolbox/Ceres、注册 `/scan_merged` 并收到 live `/map`；但 `/scan_01` publisher=0 令总检查 FAIL，RViz exit 127，且地图未持久化。
- `[CONFIRMED_RUNTIME]` 历史碰撞阻挡、正 yaw 和可选 pedestrian dodge 分别有 PASS；这些不绑定当前 RTX revision。
- `[CONFIRMED_RUNTIME]` `robot_related/Robot_URDF/exported_from_usd` 中的 Gazebo V7 proxy 历史上实测过双 10 Hz/360-sample LaserScan、TF、odom 和混合运动；这是与 Isaac/Arena 独立的证据链。

## CONFIRMED_SOURCE

- `[CONFIRMED_SOURCE]` stable 入口使用 Isaac 6.0.1、外部 UDP relay 和 `workspaces/ros2_ws` merger，与 `arena_ws` 运行上独立。
- `[CONFIRMED_SOURCE]` 当前 standalone Isaac 只从 `robot_related/robots/chassis_arm` 取 default USD 的 base visual layer；原 URDF/USD 不自带导航传感器或 ROS controller。
- `[CONFIRMED_SOURCE]` Mecanum 控制源码处理 x/y/yaw，但没有目标机器人 Arena 配置、Nav2 footprint 或 Isaac 独立 lateral runtime 验收。
- `[CONFIRMED_SOURCE]` Gazebo V7 proxy 有简化底盘/双雷达合同，但存在旧绝对 mesh URI；`roboos` 是另一个 Isaac Lab 4.5 移动操作工程且关键 LFS 资产未展开。
- `[CONFIRMED_SOURCE]` Arena 总编排、Task Generator、Nav2 合并 launch、Evaluation、HuNav 和四个 Planner 源码均存在。
- `[CONFIRMED_SOURCE]` 8 月 10 日 snapshot 后，Arena/Nav2/Evaluation/HuNav/Planner 可比源码变化均为 0；新增工作集中在 standalone RTX producer。
- `[CONFIRMED_SOURCE]` 当前 standalone exact source 已加入 `off/native/gentle/legacy_dodge` 四档行人避障并保持旧 boolean 兼容；Python AST/shell syntax PASS，但尚无 exact-revision runtime。
- `[CONFIRMED_SOURCE]` Arena 当前默认 DWB/MPPI/AMCL 都按 differential 配置；本机 MPPI 源码本身支持 Omni。
- `[CONFIRMED_SOURCE]` Evaluation recorder→metrics、Task reset→recorder、Isaac→HuNav control 等连接存在确定断点。
- `[CONFIRMED_SOURCE]` 5.1 backup 是分支精确快照，但依赖/路径/schema/实现有确定问题，当前 launch 不使用它。

## CONFIRMED_BUILD / CONFIRMED_INSTALL

- `[CONFIRMED_BUILD]` arena_ws 只有列出的 15 个包有 rc=0 build；没有 tests 通过证据。
- `[CONFIRMED_INSTALL]` 同 15 包在 overlay 可发现；Evaluation、HuNav、四个 Planner 和 backup `arena_isaac` 均不可发现。
- `[CONFIRMED_INSTALL]` Isaac 6.0.1、Isaac 5.1、ROS Humble/System Nav2、Slam Toolbox 和 semantic_nav_gazebo merger 均存在于本机。

# 22. 尚未确认的问题

- `[UNKNOWN]` RTX 配置声明 10 Hz 而最新 bag 实际约 3.195 Hz 的瓶颈位于 sensor、render/app FPS、telemetry、relay 还是 recorder 哪一层。
- `[UNKNOWN]` 行人真值速度与位移差分产生 0.511856 m/s median error 的具体原因，以及应修 producer 还是验收容差。
- `[UNKNOWN]` 约 2333 秒后的 `omni.anim.behavior` 原生 crash 根因、可重复性及其与 dodge/行人生命周期的关系；现有证据不足以归因 RTX。
- `[UNKNOWN]` 当前 exact `off/native/gentle/legacy_dodge` 四档分别能否生成同等完整且严格通过的 Level 2 产物；特别是推荐 `gentle` 的安全距离、自然性与长期稳定性尚未运行验收。
- `[UNKNOWN]` 当前 exact revision 选择 PhysX fallback 能否重新达到 Level 2。
- `[UNKNOWN]` 历史成功对应的精确 source commit/hash；顶层 Git 缺失使其无法追溯绑定。
- `[UNKNOWN]` `linear.y` 是否在当前 Isaac 机器人和碰撞 proxy 下经过独立运行验收；Gazebo V7 报告的混合命令序列不是这条链的证据。
- `[UNKNOWN]` 修复 Gazebo V7 的旧绝对 mesh URI 后，当前机器上是否仍能复现其双雷达/TF/odom/运动验收。
- `[UNKNOWN]` `roboos` 完成 Git LFS 展开和环境恢复后，当前 commit 的 Gym/cuRobo/录制任务能否通过 smoke test。
- `[UNKNOWN]` 21:47 run 的具体 `map→odom` TF 内容、在线地图质量和持久化保存能否通过；AMCL/localization 与 Nav2 goal 仍从未有成功证据。
- `[UNKNOWN]` active `ros2isaacsim@a4beefe0+diff` 是否能在 Isaac 6.0.1/ROS Humble 下完整启动并提供 Task Generator 所需全部服务。
- `[UNKNOWN]` Arena-Rosnav core 的 exact branch/commit。
- `[UNKNOWN]` Arena default Isaac+HuNav launch 在缺少 HuNav install 时的实际首个失败点；本轮按限制未启动验证。
- `[UNKNOWN]` Evaluation 修复前的任何自动记录/指标/绘图输出；当前没有产物。
- `[UNKNOWN]` 四个 Planner 在相互冲突的 Python/Torch/CUDA 依赖下能否构建、load、实时推理或满足控制周期。
- `[UNKNOWN]` native Isaac 5.1 legacy 路线是否有可直接绑定版本的完整 runtime 记录。

# 23. 当前真正缺失的环节

1. `[CONFIRMED_RUNTIME]` **当前质量基线缺口**：RTX+强度功能性 Level 2 已有旧谱系 bag，但当前四档避障 exact source 的完整 bag、10 Hz 实际输出、行人 velocity 一致性和长跑无 crash 尚未同时通过。
2. `[CONFIRMED_SOURCE]` **协议接线缺口**：stable Isaac 6 只提供 UDP/topic 合同，Arena Task Generator 需要 native service 式 spawn/move/delete/reset；两者之间没有已实现、已运行的 adapter。
3. `[CONFIRMED_SOURCE]` **机器人接入缺口**：外部已有完整几何/关节资产和历史 Gazebo proxy，但 Arena 没有唯一权威资产入口、model package、mapping/control、robot state/TF、Nav2 Omni 参数和正式 footprint；Gazebo V7 还有旧绝对路径。
4. `[CONFIRMED_RUNTIME]` **定位导航缺口**：standalone 已有一次 live `/map`，但双雷达检查失败、RViz 失败、地图未保存、`map→odom` 未直接抓取；仍无 AMCL/Nav2 action runtime，DWB/MPPI/AMCL 配置也仍是 differential。
5. `[CONFIRMED_BUILD]` **可执行包缺口**：Evaluation、HuNav、四个 Planner 及其消息/依赖均未 build；`arena_isaac5_backup` 也未 build。
6. `[CONFIRMED_SOURCE]` **任务/行人缺口**：默认 explore 不形成正常 episode reset；Isaac non-Gazebo 分支绕过 HuNav continuous ComputeAgents；碰撞不终止 Task。
7. `[CONFIRMED_SOURCE]` **评估闭环缺口**：`task_reset/scenario_reset`、rosbag/CSV、错误 velocity 字段、硬编码 waffle 和非 Nav2 result 使 recorder→metrics→plots 不闭合。
8. `[CONFIRMED_SOURCE]` **多算法 benchmark 缺口**：四个 Planner 不输出 `linear.y`，部分不消费真实行人，依赖环境互相冲突，也没有 install/runtime；现有 benchmark suite 还只列旧 Planner/`.old` worlds，并未纳入这四套算法。

`[INFERRED]` 因此现在缺的不是“再多放一些源码”，而是一条按 revision 冻结、接口一致、逐级有证据的执行链。

# 24. 下一步建议（只给3～5步）

1. `[INFERRED]` 先把当前 `b0034fba...` show.py、`4c512547...` launcher 与运行参数写入每次日志/bag；分别验收 `off/native/gentle/legacy_dodge`，并修到 raw scan 实际 10 Hz、行人 velocity validator PASS、完整录包和长跑无 crash。
2. `[INFERRED]` 在不混用协议的前提下，分别验证一次最小 Isaac 6 UDP 路线和 5.1/Arena service 路线的启动前提，再决定 Arena 接哪条；不因 5.1 更像原生接口就预先选定它。
3. `[INFERRED]` 路线确定后，先将 `robots/chassis_arm` 冻结为唯一权威资产源，移除或 recipe 化机器绝对路径；再建立唯一的 Mecanum730+XMS5 Arena robot package，明确 URDF/USD/SDF 转换、`linear.y`、robot state/TF、footprint、MPPI Omni/AMCL 运动模型，并单独验收 Level 1→3。
4. `[INFERRED]` 隔离构建 HuNav、Evaluation 和每个 Planner 的依赖环境，先逐包通过 discover/load，再修正 task/reset、human-state 和 recorder/metrics 数据合同。
5. `[INFERRED]` 按 Level 2→3→4→5→6 设置运行门槛，每一级保存 source identity、完整日志、topic/TF 快照、bag 与结果；上一级未通过时不启动多算法 benchmark。

```text
机器人外部材料（robot_related）
├─ robots/chassis_arm        → 当前 standalone Isaac 5/6 资产源
├─ Robot_URDF/exported...   → Gazebo V7 历史 Level 2；当前绝对路径待修
└─ roboos                   → Isaac Lab 4.5 移动操作；关键 LFS 资产未展开
              └─ 三者均未被注册为 Arena 的 Mecanum730/XMS5 robot package
```

```text
                                      当前系统
                                         │
             ┌───────────────────────────┼───────────────────────────┐
             │                           │                           │
             ▼                           ▼                           ▼
  已经真实跑通过（历史 revision）     当前 RTX 实现 lineage          已有源码/部分构建，但未接通
  [CONFIRMED_RUNTIME]                 [CONFIRMED_RUNTIME]           [SOURCE/BUILD/INSTALL ≠ RUNTIME]
             │                           │                           │
  Isaac Sim 6.0.1 warehouse          Isaac Sim 6.0.1              Arena-Rosnav 总 launch
  + Mecanum visual/kinematic         + 3 IRA people               ├─ Task Generator（已 build/install）
  + 3 IRA people                     + native RTX lidar           ├─ Nav2（混合 1.1.19/1.1.20，无 runtime）
  + 双 range scan（无 intensity）     first-scan / warmup PASS      ├─ Arena Mecanum 配置（缺失；外部资产已有）
             │                           │                         ├─ Evaluation（source only/断链）
  localhost UDP relay             12 topics + TF + intensity      ├─ HuNav（source only/Isaac 分支断链）
             │                           │                         └─ DRL-VO / CrowdNav / PaS / SICNav
  ROS Humble 12 topics + TF       teleop + bag，功能性 Level 2        （source+weights，未 build/install）
             │                           │                                   │
  teleop + rosbag，历史 Level 2     3.195 Hz / ped validator FAIL              │
                                         │
                              live /map（SLAM 前置能力）
                                         │
                          scan01 check/RViz/map-save 未通过
                                         │
                      当前四档避障 exact source 尚无 runtime
             └───────────────────────────┬───────────────────────────────────────┘
                                         │
                       当前没有运行连线：stable UDP topics
                       ≠ Arena 所需 spawn/move/delete/reset services
                                         │
                         ┌───────────────┴────────────────┐
                         │                                │
                         ▼                                ▼
          Isaac 6 UDP 路线：历史较可运行       Isaac 5.1 arena_isaac5_backup
          但缺 Arena service adapter           更接近 Arena 原生 service 结构
                                               但未 build/install、路径/依赖/
                                               schema/实现均有缺口
                         └───────────────┬────────────────┘
                                         ▼
                      下一步只应选择并验证一条接口路线，
                      再接 Mecanum→Nav2→Task→Evaluation→Benchmark
```
