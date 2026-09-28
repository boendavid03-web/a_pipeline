# Arena-Rosnav + Isaac Sim 5.1 GUI 首跑报告

执行时间：2026-09-13--2026-09-14 CST  
工作目录：`/home/user/navigation_project/a_pipeline`  
最终状态：`PASS_WITH_LIMITATIONS`

## 1. 结论

Arena-Rosnav 已通过原始入口在本机 Isaac Sim 5.1 GUI 中完成真实首跑：

- Isaac Sim 5.1 Full GUI、Vulkan renderer 和 2880x1800 X11 窗口持续运行；
- Task Generator 已连接 11 个 `/isaac/*` services 并完成一次 `Task Reset`；
- `map_empty` 的墙体/货架、Jackal 和 Isaac 人物均已创建；
- `/clock`、TF、odom、joint states、640 点 LaserScan 与 cmd_vel 控制链均有运行证据；
- 人物 NavMesh 路径覆盖真实 Arena 坐标；既有验收人物连续前进 19.409 m，本次交付重启中的两名人物又分别前进 18.024 m 和 4.589 m，朝向与移动方向一致，到站后均切换 `Idle` 且 `Walk=0`；
- 最终 GUI/launch 进程按用户要求保留运行。

根据用户后续要求，本轮最终验收不截图、不操纵摄像机。GUI 证据改为 X11 窗口元数据、非 headless 启动参数、Vulkan/app-ready 日志和场景/ROS 运行数据。

`PASS_WITH_LIMITATIONS` 而不是无条件 `PASS`，因为 Nav2 的 `bt_navigator` 仍以 exit `-11` 退出，RViz 又受 `/snap/core20` GLIBC 冲突影响；Arena 自主导航尚未闭合。此外，Gate 0 与最终完整目录摘要不能逐字复现，最终审计发现 active install 下曾新增一个空的 `shelf/usd` 目录（见第 8 节）。Jackal 的 Isaac cmd_vel 订阅和物理运动已经用手动短指令验证，因此这些限制不否定 GUI、场景、传感器和人物首跑结果，但禁止把受保护工作区审计描述成无条件 PASS。

## 2. Gate 汇总

| Gate | 状态 | 真实证据 |
| --- | --- | --- |
| 0：冻结/隔离 | PASS_WITH_LIMITATIONS | active dirty source 得到保留，构建和运行日志位于外置目录；最终发现 active install 下有一个空目录变更，完整摘要与冻结值不一致 |
| 1：host CPython 3.10 | PASS | `task_generator_msgs` cp310 type-support 与 `task_generator.node` 实际导入 |
| 2：Isaac CPython 3.11 | PASS | Docker factory：ROS 基础层 158 packages，Arena/Isaac 层完成；85 个 cp311 `.so`、0 个 cp310 `.so` |
| 3：runtime shim | PASS | Isaac 3.11.13 实际加载 cp311 `rclpy` 与 `isaacsim_msgs` |
| 4：launch 静态预检 | PASS | host 3.10、Pydantic 2 overlay、Arena launch graph 和双 ABI 路径解析成功 |
| 5：SimulationApp smoke | PASS | RTX 5090/Vulkan 启动，11 个 Isaac services，`ISAAC5_SIMULATIONAPP_SMOKE_READY` |
| 6：真实 GUI 全启动 | PASS_WITH_LIMITATIONS | GUI、world、Jackal、人物、ROS 数据链通过；Nav2/RViz 限制仍在 |

## 3. ABI 隔离

Host 侧：

```text
Python 3.10.12
pydantic 2.10.6:
  /home/user/arena_isaac5_host_overlay_ws/python_deps/pydantic/__init__.py
task_generator_msgs:
  /home/user/arena_isaac5_host_overlay_ws/install/task_generator_msgs/local/lib/python3.10/dist-packages/task_generator_msgs/__init__.py
isaacsim_msgs:
  /home/user/navigation_project/a_pipeline/isaac_sim/arena_ws/install/isaacsim_msgs/local/lib/python3.10/dist-packages/isaacsim_msgs/__init__.py
```

Isaac child 侧：

```text
ISAAC5_SHIM_READY 3.11.13
rclpy:
  /home/user/arena_isaac5_py311_factory/build_ws/humble/humble_ws/install/local/lib/python3.11/dist-packages/rclpy/__init__.py
isaacsim_msgs:
  /home/user/arena_isaac5_py311_factory/build_ws/humble/isaac_sim_ros_ws/install/isaacsim_msgs/local/lib/python3.11/dist-packages/isaacsim_msgs/__init__.py
native extension count: cp311=85, cp310=0
```

运行 shim：`/home/user/arena_isaac5_runtime_shim/python.sh`。

## 4. 最终启动命令

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH COLCON_CURRENT_PREFIX
source /opt/ros/humble/setup.bash
source /home/user/navigation_project/a_pipeline/isaac_sim/arena_ws/install/setup.bash
source /home/user/arena_isaac5_host_overlay_ws/install/local_setup.bash
export PYTHONPATH=/home/user/arena_isaac5_host_overlay_ws/python_deps${PYTHONPATH:+:$PYTHONPATH}
export ISAAC_PATH=/home/user/arena_isaac5_runtime_shim
export DISPLAY=:1
export ROS_DOMAIN_ID=69
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/home/user/arena_isaac5_py311_factory/gate6_ros_logs_delivery_20260914
export OMNI_KIT_ACCEPT_EULA=YES
ros2 launch arena_bringup arena.launch.py \
  sim:=isaac human:=isaac robot:=jackal world:=map_empty \
  local_planner:=dwb global_planner:=navfn env_n:=1 headless:=0
```

最终交付重启的 launch PID 为 `2090241`，Isaac Kit Python PID 为 `2090401`，ROS domain 为 `69`。交付时进程仍在运行。正常停止可执行：

```bash
kill -INT 2090241
```

## 5. GUI 与场景证据（无截图）

最终 Kit 日志：

```text
/home/user/isaacsim/5.1.0/kit/logs/Kit/Isaac-Sim Python/5.1/kit_20260914_211939.log
```

关键运行身份：

```text
Isaac Sim 5.1.0
Driver 580.173.02
Graphics API Vulkan
GPU NVIDIA GeForce RTX 5090
app ready
Simulation App Startup Complete
```

X11 窗口数据：

```text
"Isaac Sim Python 5.1.0"
class: IsaacSim
geometry: 2880x1800
Map State: IsViewable
```

外层日志已记录 Task Generator `Task Reset!`，同时记录 `map_empty` 世界、货架 USD、Jackal URDF→USD 和人物资产加载。最终外层日志：

```text
/home/user/arena_isaac5_py311_factory/gate6_full_gui_delivery_20260914.log
```

对应 X11 元数据保存在 `/home/user/arena_isaac5_py311_factory/gate6_delivery_xwininfo.txt`。

## 6. ROS 图与机器人数据链

快照：

- nodes：`/home/user/arena_isaac5_py311_factory/gate6_delivery_nodes.txt`，21 个；
- services：`/home/user/arena_isaac5_py311_factory/gate6_delivery_services.txt`，其中 `/isaac/*` 11 个；
- topics：`/home/user/arena_isaac5_py311_factory/gate6_delivery_topics.txt`，55 个；
- 当前消息样本：`gate6_delivery_{clock,tf,odom,lidar}.txt`，均来自 ROS domain 69 的存活交付进程。

关键 topics：

```text
/clock
/tf
/tf_static
/task_generator_node/jackal/odom
/task_generator_node/jackal/joint_states
/task_generator_node/jackal/lidar
/task_generator_node/jackal/lidar/points
/task_generator_node/jackal/cmd_vel
/task_generator_node/jackal/cmd_vel_nav
```

实际消息：

- `/clock`：收到仿真时钟；
- TF：`jackal/odom -> jackal/base_link`；
- odom：收到 Jackal 世界位姿；
- LaserScan：640 ranges，`angle_min=-pi`，`scan_time=0.1 s`，`range_min=0.08 m`，`range_max=12 m`；
- cmd_vel：Isaac 控制图有 2 个实际 subscriber。发送 12 帧 `linear.x=0.2 m/s` 后再发送零速，odom 从 `(15.6599, 1.8986)` 变为 `(16.2561, 2.4132)`，证明 topic→OmniGraph→Jackal 物理运动链真实工作。

对应数据文件以 `gate6_navmesh_final_{clock,tf,odom,lidar_full,cmd_*}.txt` 保存在 factory 根目录。

## 7. 人物移动、朝向和停止动画

Isaac 5.1 的 `CreateNavMeshVolumeCommand` 默认只创建 10x10 m 体积，先前把地图中所有远端坐标投影到边界 `(5,5)`。隔离副本现使用：

```text
path=/NavMeshVolume
center=[15.0,12.5,0.0]
size=[50.0,45.0,10.0]
agentRadius=0.35 m
```

本轮真实人物合同：

```text
initial=[9.8,16.75,0.1]
goal=[30.4,18.95,0.0]
NavMesh path points=219
first path point=[9.8000,16.7500]
last path point=[30.4000,18.9500]
walking speed=0.407306 m/s
SkelRoot=/World/pedestrians/D_gazebo_actor_1/ManRoot/male_adult_construction_05
```

40 组内置状态采样的量化结果：

```text
walking displacement=19.408852 m
orientation convention=character forward yaw-pi/2
orientation intervals=25
median direction error=1.285 deg
P95 direction error=5.812 deg
final path index=219/219
Idle + graph Walk=0 samples=15
Idle first-to-last drift=0.009255 m
```

最后一个状态为 `commanded_action=Idle`、`commanded_walk=0.0`、`graph_walk=[0.0]`。通过 `/isaac/get_prim_attributes` 对实际 SkelRoot 间隔 5 秒采样，位置从 `(29.92275,18.82978)` 到 `(29.92608,18.83291)`，位移约 4.6 mm，与停止状态一致。

交付重启又产生了独立的 80 行状态证据，保存在 `gate6_delivery_pedestrian_states.txt`，量化结果保存在 `gate6_delivery_pedestrian_metrics.txt`：

```text
D_gazebo_actor_1: 40 samples, 18.023776 m, 23 个朝向区间，
  median error=1.875 deg, P95=4.454 deg,
  final path=201/201, Idle, Walk=0, idle drift=0.000258 m
D_gazebo_actor_2: 40 samples, 4.589063 m, 7 个朝向区间，
  median error=2.748 deg, P95=6.624 deg,
  final path=68/68, Idle, Walk=0, idle drift=0.000713 m
```

人物运动由 Arena pedestrian service 下发起点/目标/速度；Isaac NavMesh 生成可行路径，Arena `Person` 状态机逐 waypoint 驱动 NVIDIA Animation Graph 的 `Walk/Action` 变量，Isaac dynamic avoidance 负责局部避让。它不是 Gazebo HuNav/lightSFM：当前没有 HuNav social-force 状态或个体社会参数接入 Isaac。若后续移植 HuNav，需要新增 HuNav 状态输入、坐标/时间同步、单一权威 movement writer，以及把速度/朝向命令映射到 Animation Graph。

## 8. Isaac 6 与受保护工作区审计

Gate 5 文件轨迹 275,334 行、完整 GUI 文件轨迹 774,006 行。匹配到的 `isaac6` 字样只来自仓库文档名 `isaac6_to_isaac5_migration_analysis.md`；对 `/home/user/isaacsim/6*` 或 Isaac 6 kit/exts/apps/assets 的匹配为 0。

最终进程的 `/proc/2090401/{environ,cmdline,maps,fd}` 审计也没有 Isaac 6 runtime/assets 引用；实际 mappings 中 CPython 3.11/native 项为 245，映射的 CPython 3.10 native binary 为 0。证据为 `gate6_delivery_proc_audit.txt` 和 `gate6_delivery_audit_summary.txt`。此前尝试短时附加 `strace` 时被系统 `kernel.yama.ptrace_scope` 拒绝；没有修改 sysctl 或绕过系统安全设置。

所有新增编译、安装和 ROS 日志均位于：

```text
/home/user/arena_isaac5_py311_factory
/home/user/arena_isaac5_host_overlay_ws
/home/user/arena_isaac5_runtime_shim
```

没有对 active `isaac_sim/arena_ws/{build,install,log}` 执行 build、install、clean、reset，最终 `ROS_LOG_DIR` 也明确指向外置 factory。受保护目录根 mtime 仍为：

```text
build   2026-08-09 13:25:12 +0800
install 2026-08-09 13:25:12 +0800
log     2026-09-10 20:54:27 +0800
```

但是完整元数据摘要的最终值为：

```text
build   e6a61525273852d2e38a8c64065bf13af111128318e4c39236244fca721d4ab4
install dccfb706fb94ef7f9b5a44dbb8c26ee5f1e358e2b715e7f2fdbc2a6d11d797f8
log     9b9eda26250e90d330c7bdb0b976f70ae63000c68e5872381a85a43f882302e0
```

这些值与 Gate 0 记录的完整摘要不一致。定点检查发现 `install/arena_simulation_setup/share/arena_simulation_setup/entities/obstacles/static/shelf/usd` 是空目录，mtime 为 `2026-09-13 19:25:36 +0800`，其父目录 mtime 同时改变；`build` 和 `log` 内没有 2026-09-13 之后的新条目。没有删除该目录或伪造 mtime，因为这会继续修改受保护路径。因此本报告只声称“没有重建或覆盖 active 产物”，不再声称目录逐项零变化。

## 9. 已知限制

1. `bt_navigator` 启动后 exit `-11`，lifecycle manager 随后关闭相关 Nav2 nodes；本报告不声称自主导航成功。
2. RViz 因 `/snap/core20/current/.../libpthread.so.0` 与宿主 GLIBC 冲突而 exit `127`；Isaac GUI 不受影响。
3. 场景重置时 Isaac PhysX tensor 会报告 simulation-view invalidation 警告；当前 odom、TF、LiDAR 和手动 cmd_vel 实测仍工作。
4. `biped_demo_meters.usd` 在 Isaac 5.1 下没有可用 SkelRoot，并曾触发 native animation plugin crash；隔离适配层将该请求回退到本轮验证过的 `F_Medical_01`，且在进入 animation plugin 前执行 SkelRoot fail-fast 校验。
5. 当前是 Arena 原生 Isaac/NavMesh 人物链，不代表 HuNav/lightSFM 已移植。
6. 受保护 install 的空目录变更导致 Gate 0/最终完整摘要不一致；这是隔离审计限制，已在第 8 节明确披露。

最终状态：`PASS_WITH_LIMITATIONS`。

## 10. 2026-09-16 Arena 高社会性多人续测（取代本节之前的最终状态）

本次只使用外置 host overlay、CPython 3.11 factory 和 runtime shim；没有 build、clean 或写入
`isaac_sim/arena_ws/{build,install,log}`。Host ABI 复核中
`nav2_bt_navigator`、`nav2_planner`、`nav2_msgs`、`nav2_util`、`bond` 和 `bondcpp` 都解析为
`/opt/ros/humble`。

已落实的修复包括：扩大全局 costmap 至地图范围、行人批量创建后只 reset 一次、幂等删除不存在的
`/pedestrians` prim、完整地图 NavMesh volume、移除会造成启动风暴的 map_empty shelf、移除远端
地面 MDL，以及让工厂源码优先使用 `ISAAC_ASSETS_ROOT` 而不进行同步 Nucleus 根目录探测。相关可重建
源码位于 `/home/user/arena_isaac5_py311_factory/humble_ws/src/arena_local/ros2isaacsim`；factory build
命令 `sg docker -c './build_ros.sh -d humble -v 22.04'` 每次均完成。

最后一次隔离 GUI 启动日志为
`/home/user/arena_isaac5_py311_factory/gate20_highly_social_wheeled_graph_20260916/launch.log`。
它确认了真实 Isaac 5.1 GUI、ROS2 bridge、CPython 3.11 factory、`isaacsim.core.nodes` 和
`isaacsim.robot.wheeled_robots` 已实际加载；进程组仅在验证失败后精确停止。

本轮**没有**达到“10 名真实行走行人 + Nav2 实际驱动机器人”的验收：URDF 导入随后在遗留传感器图的
`omni.isaac.sensor.IsaacReadIMU` 节点名处失败。此前已依次迁移旧 core、ROS2 bridge、wheeled-robot
图节点；这构成同一 Isaac 4→5 OmniGraph API 兼容性问题的三次差异化阻塞，因此按停止规则未继续盲目
扩展到 IMU/LiDAR 图迁移、场景参数注入或多人路线门。Host `ros2` CLI 在此轮仍会以
`RuntimeError: !rclpy.ok()` 失败，故也没有伪造参数设置或导航动作结果。

2026-09-16 的当前状态：`BLOCKED — 需要完成遗留传感器 OmniGraph 的 Isaac 5.1 命名空间迁移后，重新执行高社会性 10 人 GUI 与 Nav2 运行验收`。
