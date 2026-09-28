# Isaac Sim 6.0.1 → Isaac Sim 5.1 迁移分析

审计日期：2026-09-11  
审计范围：`/home/user/navigation_project/a_pipeline/isaac_sim` 及其实际引用的机器人 USD  
执行边界：只读检查源码、脚本、USD 元数据和本机 Isaac 安装；未修改业务代码，未启动 Isaac Sim、ROS 2 节点或仿真，未安装依赖。本文件是本次唯一新增文件。

## 1. 结论

**可以迁移到 Isaac Sim 5.1，但不能把当前 Isaac 6.0.1 完整主入口原样降级运行。**

需要把“迁移”分成两种目标：

| 目标 | 可行性 | 判断 |
|---|---|---|
| 基础机器人、场景、控制、里程计/TF、双距离雷达、ROS 2 Humble | 高 | 项目已经保留面向本机 Isaac 5.1 的 `run_navigation.py/.sh`，核心 Core/USD/PhysX/ROS API 在本机 5.1 中存在；仍需真实 runtime smoke 才能确认 |
| 当前 6.0.1 完整主链：IRA 1.6 行人、BehaviorAgent Social Force、两种双 LiDAR 后端、6.0 时序/性能合同、现有实验工具 | 中低 | 存在多个确定的 6.0 专用接口和 IRA 0.x/1.x 架构断层，需要重写行人层、传感器层和仿真生命周期适配，不能只改启动路径 |

因此建议不要“整体替换 6.0.1”。更合理的工程边界是保留两个版本化 backend：

```text
共同的纯 Python / ROS 合同层
    ├── Isaac 6 backend：保留现有完整研究主链
    └── Isaac 5 backend：以现有 run_navigation.py 为最小基线逐项补能力
```

若最终要求与当前 6.0.1 功能等价，迁移工作量估计为 **15–28 个工程人日**，另需 **5–10 个工程人日**做确定性、频率、碰撞和长跑回归；若只要求基础机器人导航 demo，约 **2–5 个工程人日**（已有 5.1 路线可复用）。

## 2. 审计对象与版本事实

### 2.1 当前两套 runtime

| 项目 | 本机事实 |
|---|---|
| Isaac 6 | `isaac_sim/isaacsim-6.0.1/VERSION` = `6.0.1-rc.7+release.42383.32955d8d.gl`；embedded Python 3.12.13 |
| Isaac 5 | `/home/user/isaacsim/5.1.0/VERSION` = `5.1.0-rc.19+release.26219.9c81211b.gl`；embedded Python 3.11.13 |
| Isaac 6 local assets | `isaac_sim/assets-6.0.1/Assets/Isaac/6.0` |
| Isaac 5 local assets | 审计范围内未发现等价的本地 `Assets/Isaac/5.1` 树；已有 5.1 USD 中存在远端 5.1 asset URL |

Python ABI 从 3.12 降为 3.11 会影响所有进入 Kit 进程的二进制 Python 扩展和自定义 ROS type support。纯 Python 模块仍需检查依赖版本，不能直接把 6.0 runtime 的 `site-packages` 或 extension binary 放进 5.1。

### 2.2 实际入口不是一条链

当前目录至少有三类入口：

1. **Isaac 6 主链**：`scripts/run_isaac_6_0_warehouse_people_robot.sh` → `scripts/show_warehouse_people_robot_6_0.py`，使用 IRA 行人、自有机器人、双 RTX/PhysX LiDAR、UDP 遥测。
2. **原生 Isaac 5 基线**：`scripts/run_navigation.sh` → `scripts/run_navigation.py`，脚本默认 `ISAAC_SIM_ROOT=/home/user/isaacsim/5.1.0`，直接使用 5.1 ROS 2 Humble bridge/internal libraries。
3. **Arena 5.1 service backend 候选**：`arena_isaac5_backup` 和 active `arena_ws` 内的 `ros2isaacsim`。它是另一条 `/isaac/*` service 数据面，不是当前 Isaac 6 UDP 主链，也不应被当作本报告中“当前主链已可迁移”的证据。

## 3. 当前项目依赖的 Isaac API

### 3.1 通用且 5.1 中可找到对应实现的 API

| API/扩展 | 当前用途 | 5.1 静态结论 |
|---|---|---|
| `isaacsim.SimulationApp` | standalone Kit 生命周期 | 可用 |
| `isaacsim.core.api.World` | 5.1 基线 world/physics stepping | 可用 |
| `isaacsim.core.api.robots.Robot` | 机器人 articulation | 可用 |
| `isaacsim.core.api.objects.DynamicCapsule`, `FixedCuboid` | 基础行人代理和场景物体 | 可用 |
| `isaacsim.core.prims.XFormPrim`, `SingleRigidPrim` | 6.0 主链中的运动学根和动态碰撞代理 | 5.1 有同名实现，但方法签名/生命周期仍需逐项 smoke |
| `isaacsim.core.utils.stage` | 新建/打开 stage、添加 reference、保存 USD | 可用 |
| `isaacsim.core.utils.extensions.enable_extension` | 启用 ROS、RTX、IRA 等扩展 | 可用 |
| `isaacsim.core.utils.viewports.set_camera_view` | viewport 相机 | 可用 |
| `omni.usd`, `omni.timeline`, `omni.physx` | stage、timeline、scene query | Kit 107/110 均有；二进制接口行为需验证 |
| `pxr.Gf/Sdf/Usd/UsdGeom/UsdPhysics/UsdShade/UsdSkel/PhysxSchema` | USD authoring、变换、材质、骨骼、PhysX schema | 基本可用；6.0 生成资产的 schema 闭包不能据此推定可回读 |
| `isaacsim.sensors.rtx`、`IsaacSensorCreateRtxLidar` | legacy RTX LiDAR 创建 | 5.1 可用，现有 `run_navigation.py` 已采用此路径 |
| `isaacsim.ros2.bridge` | 5.1 基线内嵌 `rclpy`/ROS 通信 | 5.1 可用 |

### 3.2 当前主入口调用的重要 API

`show_warehouse_people_robot_6_0.py` 不是薄封装，而是 7616 行的集成 runtime。与版本绑定最紧的调用包括：

- `SimulationManager.setup_simulation(dt=..., device=...)`
- `SimulationManager.get_physics_scenes()`
- `SimulationManager.register_callback(... PRE_PHYSICS_STEP/POST_PHYSICS_STEP ...)`
- `isaacsim.sensors.experimental.physics.Raycast.create(...)`
- `isaacsim.sensors.experimental.physics.RaycastSensor`
- `isaacsim.sensors.experimental.rtx` runtime 对象和 GMO buffer 读取
- `isaacsim.replicator.agent.core.api`
- `isaacsim.replicator.agent.core.character.IRA_Character`
- `omni.anim.behavior.core.acquire_interface()` 及 Behavior task/status API
- `SimulationApp` 的 multi-tick/per-sensor TLAS 启动设置

此外，机器人碰撞、行人避障、LiDAR 捕获和评测依赖 `omni.physx.get_physx_scene_query_interface()`；这类低层 scene query 在 5.1 存在，但应以 5.1 的 callback 和 PhysX 生命周期重新验证。

## 4. 确定的 Isaac 6.0 特有或不向后兼容接口

### 4.1 Physics RaycastSensor：确定不兼容

当前 PhysX 双雷达直接导入：

```python
from isaacsim.sensors.experimental.physics import Raycast, RaycastSensor
```

本机 5.1 没有 `isaacsim.sensors.experimental.physics` 扩展；其 `isaacsim.sensors.physics` 只有 contact/effort/IMU 等旧 API，也没有同名 `Raycast`/`RaycastSensor`。NVIDIA 的 6.0 迁移文档明确说明，新 experimental physics API 在 6.0 中替换旧 physics API，并新增 RaycastSensor。[NVIDIA Physics Sensors migration guide](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/sensors_physics_to_experimental_physics.html)

后果：当前 `PhysxDualLidarScheduler` 无法通过改 import 在 5.1 工作。候选方案只有：

1. 复用现有 5.1 `run_navigation.py` 的逐束/scene-query 距离雷达；或
2. 改用 5.1 `isaacsim.sensors.physx` 的 legacy PhysX SDK LiDAR；或
3. 只保留 5.1 RTX LiDAR。

三者都需要重新验证 2000 束、15 Hz、双雷达同步、hit path/self-filter 和仿真时间合同。

### 4.2 Experimental RTX runtime：确定命名空间不兼容

当前 6.0 主链的 RTX 路径使用：

```python
from isaacsim.sensors.experimental.rtx import ...
import isaacsim.sensors.experimental.rtx.generic_model_output as gmo_utils
```

本机 5.1 没有 `isaacsim.sensors.experimental.rtx`，但有稳定命名空间 `isaacsim.sensors.rtx`、`LidarRtx`、`IsaacSensorCreateRtxLidar` 和 `isaacsim.sensors.rtx.generic_model_output`。这说明 RTX 能力存在，但当前 producer 对象、初始化和读取代码需要回迁到 5.1 API。

特别是当前 `navigation_2d_32k.usda` 及 6.0 的 multi-tick 调度不能仅凭 USD 字段相同就视为 5.1 已支持。

### 4.3 SimulationManager：存在确定的 API/回调签名断层

本机静态对比结果：

| 调用 | 6.0.1 | 5.1 |
|---|---|---|
| `setup_simulation(dt, device)` | 有 | 无 |
| `get_physics_scenes()` | 有 | 无 |
| pre/post physics callback | `callback(step_dt, context)` | `callback(step_dt)` |
| `get_simulation_time()` / `get_num_physics_steps()` | 有 | 有 |
| `register_callback` / `deregister_callback` | 有 | 有，但行为和异常合同不同 |

当前动态 robot controller 和 PhysX 双雷达 callback 都声明两个参数；直接运行在 5.1 会发生调用签名错误。GPU PhysX 设置也依赖 6.0 `PhysicsScene` wrapper，需改为 5.1 schema/manager 路径。

### 4.4 IRA/行人系统：完整架构不兼容

当前配置是 `isaacsim.replicator.agent` **1.6.0** schema，采用：

- `environment.base_stage_asset_path`
- `character.groups`
- `routines: patrol`
- `path_points` / `speed_range`

本机 6.0 extension 是 IRA 1.6.8；本机 5.1 extension 是 IRA 0.7.28。5.1 默认 schema 使用 `global.simulation_length`、`scene.asset_path`、扁平 `character.asset_path/command_file/num`，没有 1.x group/routine 配置。

NVIDIA 明确将 IRA 1.0 定义为 complete architectural overhaul，并指出 0.x 配置和代码不能不经修改直接工作。[NVIDIA IRA 6.0 migration guide](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/ext_isaacsim_replicator_agent_migration_guide.html)

当前代码还直接使用 6.0 的：

- `from isaacsim.replicator.agent.core import api as ira`
- `from isaacsim.replicator.agent.core.character import IRA_Character`
- `omni.anim.behavior.core` 的 `IBehaviorAgent`/task/status 对象

本机 5.1 的 `isaacsim.replicator.agent.core` 没有 `api.py` 和 `character/IRA_Character` 模块；0.7 行人主要通过 `omni.anim.people` behavior script/command file 控制。因此当前 Social Force adapter 不能保留原对象层，只能重新设计为 5.1 command injection、旧 behavior script 扩展，或放弃 IRA、改用独立的 capsule/USD-skeleton pedestrian backend。

### 4.5 6.0 multi-tick RTX 时序：不能下放为相同保证

当前入口在创建 `SimulationApp` 时设置：

- `/rtx/hydra/supportMultiTickRate=true`
- `/rtx/rendering/perSensorTickTlas=true`
- `/ExternalSimulationTime`
- `SimulationManager.setup_simulation(...)`

NVIDIA 6.0 release notes把“由物理仿真时间驱动相机和 RTX LiDAR 的 multi-tick rendering”列为 6.0 改进。[Isaac Sim 6.0 release notes](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/overview/release_notes.html)

5.1 可以生成 RTX LiDAR 数据，但不能假定它拥有与当前 6.0 主链相同的多 tick 调度、相位、catch-up 和频率语义。迁移验收必须重新测量，不可沿用 6.0 的 15 Hz PASS 阈值结果。

## 5. Robot / pedestrian / sensor / world 兼容性

### 5.1 Robot

| 项目 | 结论 | 依据与风险 |
|---|---|---|
| 自有 Mecanum730/XMS5 USD | **有条件兼容** | `run_navigation.py` 已在 5.1 路径引用同一 `mecanum730_xms5_default.usd`；文件是 USD crate 0.8.0，顶层由 base/physics/sensor sublayer 组成。但本次未启动 5.1，不能确认 articulation、joint drive、材质和传感器 schema 均能实例化 |
| 当前 6.0 主链的 visual-only robot | **较高兼容** | 只 reference `configuration/mecanum730_xms5_default_base.usd`，用 `XFormPrim` 驱动根，绕开完整 articulation；5.1 有对应 prim API |
| 动态碰撞代理 | **需适配** | USD/PhysX schema 可复用，但 `SingleRigidPrim` 生命周期、callback 参数和 GPU PhysX scene 设置不同 |
| 麦克纳姆控制数学 | **兼容** | `clamp_twist`、轮速映射、运动学积分是普通 Python/NumPy，不依赖 6.0 API |

结论：robot 本体不是迁移主阻塞项；主要风险是 physics/control wrapper 与实际 USD schema/runtime，而不是网格资产。

### 5.2 Pedestrian

| 子系统 | 结论 |
|---|---|
| 6.0 IRA 1.6 config | 不兼容 5.1 IRA 0.7 |
| `IRA_Character` 对象发现 | 5.1 无对应模块 |
| `IBehaviorAgent.follow/moveTo/dodge/setSpeed` 集成 | 不能视为 5.1 等价接口 |
| Social Force 纯数学内核 | 可复用；`pedestrian_social.py` 和大部分 steering/geometry 逻辑不依赖 Isaac |
| Social Force → Isaac motion adapter | 必须重写 |
| USD skeleton-only walkers | 可能作为降级展示路径，但不等价于 NavMesh、避障和 Social Force 行人 |

因此 pedestrian 是完整迁移的最大风险和最大工作包。若验收要求 20 人循环巡逻、速度控制、互让、机器人避障、free-space boundary 和重启/liveness 合同，则不能用“5.1 能显示人物动画”替代。

### 5.3 Sensor

| Sensor 路径 | 5.1 兼容性 | 工作量 |
|---|---|---|
| 现有 5.1 `run_navigation.py` PhysX/scene-query scan | 高；已是 5.1 定向源码 | 低，重点是 runtime 验证 |
| 6.0 experimental Physics RaycastSensor | 不兼容 | 中高，需替换 producer |
| 6.0 experimental RTX + GMO | API 不兼容，但 5.1 有 stable RTX/GMO | 中，需重写初始化、attach、readback 和 teardown |
| 自定义 `navigation_2d_32k.usda` | 未确认 | 中，需在 5.1 校验 schema、频率和 raw return |
| 双 LiDAR ROS 投影、合并、UDP framing | 大部分兼容 | 纯 Python/外部 ROS，可保留；上游 native frame/time 仍需重新验收 |

建议 5.1 首版以已有 distance-only PhysX/scene-query 路线为准，不把 RTX intensity 作为首个 gate。

### 5.4 World / assets

| World 类型 | 结论 |
|---|---|
| `a_pipeline_eng_lobby.usda`、`a_pipeline_empty_people.usda` | **较高静态兼容性**：项目自有、Z-up/metre、基础 USD/PhysX/NavMesh authoring；仍需 5.1 打开、NavMesh bake/query 和碰撞 smoke |
| `mecanum_lidar_main.usd`、`mecanum_minimal_main.usd` | **面向 5.1 的历史场景**：文件内直接引用 5.1 asset URL，优先作为最小验证输入 |
| `assets-6.0.1/Assets/Isaac/6.0/...` 的 warehouse/hospital/people/sensor USD | **不能作为 5.1 兼容资产使用**：路径和依赖闭包是 6.0 版本化内容；应获取对应 5.1 assets 或用项目自有场景 |
| 6.0 IRA warehouse + HumanMotionLibrary/WalkForward | **未兼容**：既受资产版本影响，也受 IRA schema/behavior 架构影响 |

USD 文件格式通常可读不代表 composed stage 可用。真正的兼容门应检查 missing reference、unknown schema、材质/骨骼 binding、NavMesh、collision 和 sensor prim，而不是只检查 `Usd.Stage.Open()`。

## 6. ROS 2 bridge 差异

### 6.1 当前 6.0 主链实际没有使用 Isaac ROS bridge

`show_warehouse_people_robot_6_0.py` 明确不在 Kit 内导入 `rclpy`。实际链路是：

```text
Isaac 6 Python 3.12
    ⇄ localhost UDP
system ROS 2 Humble / Python 3.10 cmd_vel_udp_relay.py
    ├── publish /clock /odom /tf /tf_static
    ├── publish /scan /scan_01 /scan_02
    ├── publish /pedestrian_ground_truth /isaac/actuation_state /isaac/reset_event
    └── subscribe /cmd_vel /isaac/reset_pose
```

因此，如果 5.1 backend 继续保留 UDP 边界，ROS topic/QoS/消息层几乎不需要因 Isaac 版本而改变；主要改动在 Isaac 侧 producer。这个方案也避免 Python 3.11 Kit 与系统 Humble Python 3.10 的 ABI 混装。

### 6.2 直接使用 5.1 ROS bridge 时

本机 5.1 有 monolithic `isaacsim.ros2.bridge` 4.12.4；6.0 同名 extension 是 5.1.2，并在内部拆分出 `isaacsim.ros2.core/nodes/ui/examples`。当前项目没有直接依赖这些 6.0 新拆分 extension ID，因此模块拆分不是主阻塞。

5.1 官方支持 Ubuntu 22.04 + ROS 2 Humble，但 embedded Python 是 3.11。使用 common interfaces 时可用内置 Humble libraries；若 Isaac 进程必须 import 自定义消息，则自定义包也要构建 cp311 版本，而外部 Ubuntu Humble 节点仍使用 cp310 版本。[Isaac Sim 5.1 ROS 2 installation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html)

现有 `run_navigation.sh` 已显式：

- 设置 `ROS_DISTRO=humble`
- 默认 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- 将 `5.1.0/exts/isaacsim.ros2.bridge/humble/lib` 加入 `LD_LIBRARY_PATH`
- 用 5.1 `python.sh` 运行内嵌 `rclpy`

这是合理的 5.1 common-message 路线。若迁移目标还包含 Arena `isaacsim_msgs` services，则必须采用 cp311/cp310 双构建和子进程环境隔离，不能把系统 `/opt/ros/humble` 的 cp310 `rclpy` 直接 source 给 Isaac 5.1。

## 7. 可复用与必须重写的边界

| 模块 | 迁移策略 |
|---|---|
| `pedestrian_social.py`、`pedestrian_steering.py`、route/free-space 几何、评测纯函数 | 尽量原样复用，并保持离线单元测试 |
| `cmd_vel_udp_relay.py`、UDP framing/telemetry、外部 ROS merger | 保留，必要时只改 node/runtime 标识，不改消息合同 |
| `run_navigation.py/.sh` | 作为 5.1 最小 backend 起点，不从 7616 行 6.0 主入口直接做机械降级 |
| robot visual reference、kinematic integration、USD 基础场景 | 可复用，需 5.1 runtime gate |
| `PhysxDualLidarScheduler` | 替换 6.0 RaycastSensor authoring/runtime，重验 callback/time/self-filter |
| `RtxDualLidar` | 回迁到 5.1 stable RTX API，或第二阶段再实现 |
| IRA config generator | 输出独立的 5.1 0.7 config/command-file schema；不要让一份 YAML 同时服务 0.x/1.x |
| `BehaviorAgentSocialMotion`、yield/restart adapter | 针对 5.1 `omni.anim.people` 重写，或建立新的 5.1 pedestrian backend |
| `SimulationManager.setup_simulation`、GPU scene wrapper、two-argument callbacks | 建立版本适配层或改用 5.1 World/physics callback 生命周期 |
| 6.0 assets 路径 | 改为 5.1 assets manifest；项目自有场景单独做 compatibility manifest |

## 8. 迁移工作量估计

估计以一名熟悉 Isaac/ROS/USD 的工程师计算，不包含下载时间；范围包含实现、静态测试和一次短 runtime gate，不把长期实验等待算作纯编码时间。

### 8.1 最小机器人导航 demo

| 工作包 | 人日 |
|---|---:|
| 固定 5.1 runtime/assets/driver shim，核对启动环境 | 0.5–1 |
| 验证现有 `run_navigation.py` 的 world、robot、control | 0.5–1 |
| 验证/修复 distance-only 双 scan、odom、TF、clock、cmd_vel | 1–2 |
| Nav2/rosbag smoke 与文档 | 0.5–1 |
| **合计** | **2–5** |

### 8.2 保留当前主链核心能力（不含完整 IRA/Social Force 等价）

| 工作包 | 人日 |
|---|---:|
| versioned backend/launcher 和 5.1 SimulationManager 适配 | 2–4 |
| 机器人 visual/physics/collision proxy 适配 | 1–3 |
| 双 PhysX/scene-query LiDAR、时序与 self-filter | 3–5 |
| 5.1 RTX/GMO 可选路径 | 2–4 |
| world/assets manifest 和场景 smoke | 1–2 |
| ROS/UDP 合同回归 | 1–2 |
| **合计** | **10–20** |

### 8.3 完整行人和研究合同等价

在上表基础上增加：

| 工作包 | 人日 |
|---|---:|
| IRA 1.6 groups/routines → 5.1 0.7 config/commands | 2–4 |
| 5.1 character discovery/control adapter | 3–5 |
| Social Force、yield、stop/restart、NavMesh lifecycle 接回 | 4–7 |
| 20 人 liveness、clearance、频率、确定性、长跑回归 | 5–10 |
| **完整总计** | **20–38** |

风险储备建议至少 25%，因为行人 native extension 和 RTX teardown/driver 行为无法靠静态检查封顶。

## 9. 推荐的迁移判定路线

### Gate A：先证明 5.1 最小 backend

只使用现有 `run_navigation.py/.sh` 和面向 5.1 的 `mecanum_minimal_main.usd`/`mecanum_lidar_main.usd`，检查：

1. `SimulationApp` 创建、stage compose 无 missing reference/unknown schema。
2. robot articulation 或 visual-only root 可创建并响应有限时长命令。
3. `/clock`、`/odom`、`/tf`、`/scan_01`、`/scan_02`、`/cmd_vel` 合同成立。
4. 记录实际 scan sim-time/wall-time rate，不以 authored frequency 代替观测频率。
5. 正常 teardown，无 Vulkan/RTX/PhysX crash。

### Gate B：再迁移项目自有 world 和 robot collision

分别验证 Z-up/metre、NavMesh、79 个静态箱体、机器人 bounds/collision。不要在同一轮加入 IRA 和 RTX。

### Gate C：选择 5.1 pedestrian 实现

在下列方案中只选一条做实验：

- **C1：IRA 0.7 适配**：最接近 5.1 官方组件，但现有 1.6 schema和对象控制层重写最多。
- **C2：独立 skeleton/capsule backend**：运动控制更可控，但需自行承担动画、NavMesh、碰撞和 character semantics。
- **C3：5.1 只做 robot/sensor backend，不追求行人等价**：工作量最低，也最符合“只为 Arena 5.1 demo”的目标。

### Gate D：最后恢复高风险能力

按顺序加入 Social Force adapter、双 2000 束/15 Hz、RTX intensity、20 人压力、长跑。每层失败时停在第一处，不同时改资产、middleware、sensor 和行人 API。

## 10. 最终判断

| 问题 | 回答 |
|---|---|
| 当前项目能否迁移到 5.1？ | **能，但要限定目标。基础导航高可行；完整 6.0 主链不是直接兼容。** |
| 是否使用 6.0 特有接口？ | **是。** experimental Physics RaycastSensor、experimental RTX、`SimulationManager.setup_simulation/get_physics_scenes`、双参数 physics callback、IRA 1.x/`IRA_Character`/新版 behavior 对象层、multi-tick RTX 合同均是关键差异。 |
| robot 是否兼容？ | **大体兼容，需 runtime 验证。** 项目已有同一 robot 的 5.1 定向入口。 |
| pedestrian 是否兼容？ | **不兼容当前实现。** 纯 Social Force 数学可复用，Isaac adapter 必须重写。 |
| sensor 是否兼容？ | **能力可替代，API 不兼容。** 现有 5.1 legacy RTX/scene-query 路径可作为替代。 |
| world 是否兼容？ | **项目自有简单 USD 较可能兼容；6.0 版本化官方资产不可直接沿用。** |
| ROS 2 bridge 是否阻塞？ | **不是当前 UDP 主链的主要阻塞。** 直接 bridge 路线要处理 5.1 Python 3.11 和 custom message 双 ABI。 |
| 是否建议彻底放弃 6.0？ | **不建议。** 保留 6.0 研究主链，新增受控的 5.1 backend，比把现有完整入口整体降级更安全、可回滚且更容易验证。 |

本报告的“可用/存在”均为静态源码或本机安装证据；除项目历史材料外，本次没有产生新的 5.1 runtime 证据。因此当前最高结论是 **migration feasible with redesign**，不是 “Isaac 5.1 demo 已跑通”。
