# Arena-Rosnav 与本机 Isaac Sim 4.x/5.x 兼容性报告

审计日期：2026-09-10  
审计范围：`/home/user/isaacsim`、`/home/user/navigation_project/a_pipeline/isaac_sim/arena_ws`、相关 Isaac assets/extension/cache、ROS 2 Humble 与 Arena Isaac 源码  
执行边界：只读检查目录、版本、源码、扩展描述、ROS 包索引和不创建 `SimulationApp` 的 Python import/type-support 探针；**未安装依赖、未编译、未启动 Isaac Sim、未启动 ROS 节点、未运行仿真**。除本报告外未修改项目文件。

## 1. 结论

### 1.1 最终判断

本机确实有一套完整、可执行的 **Isaac Sim 5.1.0**：

```text
/home/user/isaacsim/5.1.0
VERSION = 5.1.0-rc.19+release.26219.9c81211b.gl18G
embedded Python = 3.11.13
size = 18 GB
```

但当前没有完整的 Isaac Sim 4.x runtime。发现的 `isaacsim_4_5` 只有约 72 KB，是 RoboOS/Isaac Lab 的 `.kit` experience 配置集合，不含 `python.sh`、`isaac-sim.sh`、Kit runtime 或 `VERSION`，不能用于 Arena launch。

对当前 active `arena_ws` 的结论是：

- **Isaac 5.1 API 兼容性：有较高的兼容可能，但依赖 deprecated compatibility layer。** 当前 `ros2isaacsim` 大量使用 `omni.isaac.*`；Isaac 5.1 同时提供新的 `isaacsim.*` API 和 `extsDeprecated/omni.isaac.*` 转接扩展，所以不是“所有旧 API 已删除”。
- **当前 install space：不能直接运行。** 决定性阻塞不是 Isaac executable，而是 Python ABI：Isaac 5.1 使用 Python 3.11；当前 `isaacsim_msgs` 是 ROS Humble/system Python 3.10 构建产物。实际强制加载消息类型支持时失败为 `UnsupportedTypeSupport: Could not import 'rosidl_typesupport_c' for package 'isaacsim_msgs'`。
- **ROS 2 Humble 本身受 Isaac 5.1 支持。** 5.1 安装内包含 Humble 与 Jazzy bridge libraries，Humble 的 `rclpy` 是 CPython 3.11 build。当前失败是 Arena 自定义消息 overlay 与 Isaac embedded Python 不一致，不是“Humble 完全不支持”。
- **本地缺少 5.1 对应的完整 assets pack。** 5.1 默认指向 NVIDIA 5.1 cloud assets；本次对 Arena 所需 `Isaac/People/Characters/Biped_Setup.usd` 做 HTTP HEAD 返回 200，因此联网条件下资产路径当前可达。项目内 98 GB 的 `assets-6.0.1` 是 6.0 资产，不能未经验证当作 5.1 的等价替代。
- **完整 Arena 首跑还有 workspace 层阻塞。** `task_generator_msgs` 未安装；若使用默认 `human:=hunav`，HuNav 包也未安装。Isaac 首次尝试应显式用 `human:=isaac`，但仍需先闭合 Task Generator 与 Python 3.11 custom-message build。

因此，答案不是“现在直接执行一条 launch 命令即可”，而是：**5.1 是本机最值得保留和继续验证的 Isaac 候选；无需先改完所有 `omni.isaac.*`，但必须先建立 Python 3.11 兼容的 Arena Isaac 消息/bridge overlay，并补齐 Arena core build。** 在不允许启动仿真的本次审计中，不能把 deprecated API 的静态存在写成运行成功。

### 1.2 候选优先级

| 优先级 | 候选 | 当前状态 | 与 active Arena 的判断 |
|---:|---|---|---|
| 1 | `/home/user/isaacsim/5.1.0` | 完整 18 GB runtime，5.1.0-rc.19，Python 3.11 | 最合理候选；API 有兼容层，但现有 ROS custom messages ABI 不兼容 |
| 2 | Isaac Sim 4.2.0 | 当前未安装 | 本地 Arena installer 写死的原始目标版本；理论匹配更直接，但需要重新下载安装 |
| 3 | Isaac Sim 4.5.0 | 没有完整 runtime | 只有 `.kit` 文件，不可启动 |
| 4 | 项目内 Isaac Sim 6.0.1 | 完整 34 GB，Python 3.12；另有 98 GB assets | 不是本报告的首选；Python/API 迁移面比 5.1 更大 |

## 2. 当前所有相关 Isaac 安装与候选目录

### 2.1 完整 runtime

| 路径 | 版本证据 | 大小 | 可执行入口 | 判断 |
|---|---|---:|---|---|
| `/home/user/isaacsim/5.1.0` | `VERSION` 为 `5.1.0-rc.19+release.26219.9c81211b.gl18G` | 18 GB | `isaac-sim.sh`、`python.sh` 均存在 | 完整安装 |
| `/home/user/navigation_project/a_pipeline/isaac_sim/isaacsim-6.0.1` | `VERSION` 为 `6.0.1-rc.7+release.42383.32955d8d.gl34G` | 34 GB | `isaac-sim.sh`、`python.sh` 均存在 | 完整安装，但不是 4/5 候选 |

### 2.2 不是完整 runtime 的目录

| 路径 | 实际内容 | 判断 |
|---|---|---|
| `/home/user/navigation_project/robot_related/roboos/apps/isaacsim_4_5` | 约 72 KB；Isaac Lab 2.3.2 的 `.kit` experience，配置中声明 `app.version=4.5.0` | 只是配置，不是 Isaac 4.5 安装 |
| `/home/user/navigation_project/a_pipeline/isaac_sim/arena_isaac5_backup` | Arena Isaac 5.1 candidate 源码快照 | 不是 simulator runtime，也未接入 active install |
| `/home/user/.cache/isaacsim_*` 与 `~/.cache/ov/...` | cache、generated OGN、历史运行状态 | 不能作为独立安装 |

### 2.3 已下载但不是额外安装

- `/home/user/Downloads/isaac-sim-standalone-5.1.0-linux-x86_64.zip`：约 8.77 GB，是 5.1 standalone 压缩包。
- `/home/user/Downloads/arena-isaac-arena5-isaac5.1.0.zip`：约 83.5 KB，是 Arena Isaac 5.1 源码包。
- 没有发现 4.2/4.5 的完整 standalone archive 或可执行解压目录。

## 3. Arena `ros2isaacsim` 实际依赖的 Isaac API

### 3.1 active checkout

active 源码位于：

```text
isaac_sim/arena_ws/src/arena/isaac
commit: a4beefe0203ec8a75d65d0ab70496b5e2c400605
commit date: 2025-07-10
HEAD: detached
local change: ros2isaacsim/ros2isaacsim/run_isaacsim.py
```

该本地修改把旧的 `omni.importer.urdf._urdf` 改为先启用 `isaacsim.asset.importer.urdf`，再从 `isaacsim.asset.importer.urdf` import `_urdf`。这项修改提高了 4.5/5.x 兼容性，但它是用户现有未提交修改，本报告没有改动它。

主运行文件的依赖可分为：

| 功能 | 当前使用的 API/extension |
|---|---|
| App bootstrap | `from isaacsim import SimulationApp` |
| Core/World/prim/stage | `omni.isaac.core.*` |
| URDF importer | `isaacsim.asset.importer.urdf` |
| ROS bridge | enable `omni.isaac.ros2_bridge`；OmniGraph 也大量写旧 node ID |
| Sensors | `omni.isaac.sensor`、`omni.isaac.core_nodes` |
| People/NavMesh | `omni.anim.people`、`omni.anim.navigation.*`、`omni.anim.graph.*` |
| Assets | `omni.isaac.nucleus`，目标路径含 `Isaac/People/Characters` |
| ROS client/service | `rclpy`、`isaacsim_msgs` |
| General Python | NumPy、PyYAML |

启动顺序在源码中是正确的大方向：先创建 `SimulationApp`，随后 import Kit/Isaac 模块并启用 extensions。因本次禁止启动仿真，没有执行 `SimulationApp` 级 import 验证。

### 3.2 `omni.isaac.*` 与 `isaacsim.*` 的 5.1 兼容性

Isaac 5.1 安装同时包含两层：

| Arena 用法 | Isaac 5.1 当前证据 | 判断 |
|---|---|---|
| `omni.isaac.core` | `extsDeprecated/omni.isaac.core` 存在，声明由 `isaacsim.core.api`、`isaacsim.core.utils` 替代 | 可通过 deprecated adapter；不宜作为长期接口 |
| `omni.isaac.sensor` | deprecated extension 存在，转向 `isaacsim.sensors.camera/physics/physx/rtx` | 可兼容的可能性高 |
| `omni.isaac.nucleus` | deprecated extension 存在，转向 `isaacsim.storage.native` | 可兼容的可能性高 |
| `omni.isaac.ros2_bridge` | deprecated extension 存在，依赖 `isaacsim.ros2.bridge` | extension 可被解析，但应迁移新名称 |
| `omni.isaac.core_nodes` | deprecated extension 存在，依赖 `isaacsim.core.nodes` | node 名由 deprecation manager 映射 |
| `isaacsim.asset.importer.urdf` | 5.1 active extension 存在 | 当前本地修改采用了正确的新命名 |
| `isaacsim.ros2.bridge` | 5.1 active extension 4.12.4 存在 | 可用，且包含 Humble/Jazzy libraries |
| `omni.anim.people/navigation` | 5.1 `extscache` 中相关 extensions 均存在 | 静态可发现；未做运行期加载/NavMesh bake |

注意：deprecation manager 能映射 extension settings 和许多 OmniGraph node IDs，但这不等于每个函数签名、数据属性、传感器行为和 NavMesh 命令都未经运行即可保证兼容。当前最合理策略是先保留兼容层做最小 backend smoke，再根据第一处真实 traceback 迁移，而不是在没有运行证据时大面积重写。

### 3.3 Arena Isaac 5.1 backup 不是 drop-in replacement

`isaac_sim/arena_isaac5_backup` 更系统地采用了 `isaacsim.core.*`、`isaacsim.sensors.*` 和 `isaacsim.ros2.bridge`，看起来是面向 Arena 5 / Isaac 5.1 的候选实现；但它：

- ROS 包名从 active 的 `ros2isaacsim` 变为 `arena_isaac`；
- `isaacsim_msgs` 的消息和 service 集合与 active Task Generator 所期待的接口明显不同；
- 没有构建、没有安装，也没有接入 active `arena_bringup` launch；
- 来源 zip 记录的对象 ID 为 `16b8e3416517d8c3dc1b5038df4fe11b9a6df46c`，但目录本身没有 git provenance 可供本次进一步确认。

所以不能把 backup 文件复制进 active tree 后直接启动；那会是一次明确的接口迁移项目，不是环境变量修复。

## 4. Arena Isaac launch 入口

### 4.1 调用链

```text
arena_bringup/launch/arena.launch.py
  -> launch/simulator/sim/sim.launch.py   (sim:=isaac)
    -> launch/simulator/sim/isaac/isaac.launch.py
      -> $ISAAC_PATH/python.sh
      -> installed executable: ros2isaacsim/run_isaacsim
      -> ros2isaacsim.run_isaacsim:main
```

具体入口：

- `isaac_sim/arena_ws/src/arena/arena-rosnav/arena_bringup/launch/arena.launch.py`
- `isaac_sim/arena_ws/src/arena/arena-rosnav/arena_bringup/launch/simulator/sim/sim.launch.py`
- `isaac_sim/arena_ws/src/arena/arena-rosnav/arena_bringup/launch/simulator/sim/isaac/isaac.launch.py`
- `isaac_sim/arena_ws/src/arena/isaac/ros2isaacsim/ros2isaacsim/run_isaacsim.py`

当前 `arena_bringup`、`ros2isaacsim`、`isaacsim_msgs` 均可由已 source 的 package index 找到。

### 4.2 launch 层已发现的问题

- `isaac.launch.py` 强制依赖 `ISAAC_PATH/python.sh`，当前 shell 中 `ISAAC_PATH` 为空。
- `tools/source.bash` 只认识 installer 标记，并写死 source `~/isaacsim-4.2.0/setup.bash`；它不会自动发现 `/home/user/isaacsim/5.1.0`。
- `sim.launch.py` 没有把 `headless` 传给 Isaac launch；`run_isaacsim.py` 又硬编码 `headless=False`，所以主 launch 的 `headless` 选择对 Isaac backend 不生效。
- active runner 固定 `renderer="Wireframe"`。
- `run_isaacsim.py` 在循环中捕获 `BaseException` 后继续运行，真实 runtime 错误可能只被记录为 warning，增加首跑诊断难度。
- full Arena 对 Isaac 默认仍选择 `human:=hunav`。当前 HuNav 没安装；若目标是使用 Arena 自带 Isaac pedestrian services，首次尝试应显式设置 `human:=isaac`。

## 5. ROS 2 Humble 与 Python ABI

### 5.1 支持情况

NVIDIA Isaac Sim 5.1 文档明确支持 ROS 2 Humble 与 Jazzy，并推荐在 Ubuntu 22.04 使用随 Isaac Sim 提供的 internal Humble libraries。当前安装确有：

```text
/home/user/isaacsim/5.1.0/exts/isaacsim.ros2.bridge/humble
/home/user/isaacsim/5.1.0/exts/isaacsim.ros2.bridge/jazzy
```

Humble 目录包含 CPython 3.11 的 `rclpy`、Fast DDS 与 Cyclone DDS libraries。系统 ROS 2 Humble 则是 Ubuntu 22.04/Python 3.10 build。

官方参考：[Isaac Sim 5.1 ROS 2 installation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html)。

### 5.2 本次实际探针

1. 直接 source `/opt/ros/humble` 与当前 `arena_ws/install` 后调用 Isaac 5.1 `python.sh`：

```text
Python 3.11 尝试加载 /opt/ros/humble 的 Python 3.10 rclpy
结果：ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
```

2. 把 Isaac 5.1 internal Humble `rclpy` 和 libraries 放在搜索路径最前：

```text
rclpy import: PASS
isaacsim_msgs Python class import: PASS
```

3. 强制加载 `ImportUsd` 的 ROS type support：

```text
UnsupportedTypeSupport:
Could not import 'rosidl_typesupport_c' for package 'isaacsim_msgs'
```

安装目录中的扩展模块文件明确是：

```text
isaacsim_msgs_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so
```

Isaac 5.1 需要 CPython 3.11 对应产物。仅调整 `PYTHONPATH` 能解决 `rclpy` 优先级，不能把 cp310 自定义消息二进制变成 cp311。

### 5.3 结论

**ROS 2 Humble compatible；当前 Arena overlay incompatible。** 要在 Isaac 5.1 embedded Python 中创建 `isaacsim_msgs` service server，必须按 NVIDIA 的 custom ROS workspace 指南生成 Python 3.11 兼容的消息类型支持，或采用一个架构上隔离 Python ABI 的 bridge。当前 install 不能直接复用。

## 6. Assets、extensions、Python packages 与环境变量

### 6.1 Assets

| 项目 | 当前状态 | 判断 |
|---|---|---|
| Isaac 5.1 local assets pack | 未发现 | 离线运行缺失 |
| Isaac 5.1 default cloud root | `https://.../Assets/Isaac/5.1` | 配置存在 |
| `Biped_Setup.usd` cloud probe | HTTP 200，约 20 KB | 联网时当前可达 |
| Isaac 6 local assets | 98 GB，路径为 `Assets/Isaac/6.0` | 版本不匹配，不应默认复用 |
| Arena own world/robot assets | simulation setup 中存在 | 仍需 runtime 通过 services 完成导入 |

若首跑联网，5.1 assets 不一定是启动前的硬阻塞；若要求完全离线，则需安装/配置 5.1 对应 assets，并验证 People、Materials、Robots 的引用闭包。

### 6.2 Extensions

当前 5.1 安装中已发现：

- `isaacsim.asset.importer.urdf`
- `isaacsim.ros2.bridge`
- `isaacsim.core.api/utils/nodes`
- `isaacsim.sensors.camera/physics/physx/rtx`
- `omni.anim.people`
- `omni.anim.navigation.bundle/core/ui`
- `omni.anim.graph.*`、`omni.anim.retarget.*`、`omni.anim.timeline`
- active Arena 使用的 `omni.isaac.core/sensor/nucleus/ros2_bridge/core_nodes` deprecated adapters

所以没有发现“关键 extension 目录完全缺失”的静态阻塞。是否能在同一 experience 中成功 enable，只能在获准的 SimulationApp smoke 中验证。

### 6.3 Python packages/build tools

Isaac 5.1 embedded Python 已确认：

- `isaacsim`：PASS
- `SimulationApp` symbol：PASS（未实例化）
- NumPy 1.26.0：PASS
- PyYAML 6.0.2：PASS
- internal Humble `rclpy`：调整路径后 PASS

但 embedded Python 当前缺：

- `colcon_core`
- `empy` (`em`)
- `catkin_pkg`

这意味着“用 Isaac embedded Python 直接重建 custom ROS messages”的工具链当前也未闭合。安装/构建方案需单独设计和授权，本报告没有执行。

### 6.4 当前缺失/未设置的环境变量

当前 IDE shell 实测为空：

```text
ISAAC_PATH
ROS_DISTRO
ROS_DOMAIN_ID
RMW_IMPLEMENTATION
VIRTUAL_ENV
CONDA_PREFIX
```

Arena/Isaac 5.1 首跑至少应一致设置：

```bash
export ISAAC_PATH=/home/user/isaacsim/5.1.0
export ROS_DISTRO=humble
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="$ISAAC_PATH/exts/isaacsim.ros2.bridge/humble/lib:${LD_LIBRARY_PATH:-}"
```

对于 Python service runner，还必须确保 Isaac 3.11 `rclpy` 和未来的 cp311 Arena message overlay 排在 `/opt/ros/humble` 的 Python 3.10 packages 之前。不要同时让两个 ABI 的 `rclpy` 随机竞争 import 顺序。

## 7. A：Isaac 5.1 第一次运行步骤

### A0. 当前是否可以直接执行

**不可以。** 现在执行 full Arena launch，至少会遇到 cp310/cp311 `isaacsim_msgs` type-support 阻塞和 `task_generator_msgs` 缺失。下面步骤是修复依赖闭包后的首次运行手册，**本次未执行**。

### A1. 修复后先做不启动仿真的 preflight

新终端中：

```bash
cd /home/user/navigation_project/a_pipeline/isaac_sim/arena_ws

export ISAAC_PATH=/home/user/isaacsim/5.1.0
export ROS_DISTRO=humble
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="$ISAAC_PATH/exts/isaacsim.ros2.bridge/humble/lib:${LD_LIBRARY_PATH:-}"

# ROS CLI/launch 仍来自系统 Humble；随后叠加经验证的 Python 3.11 custom-message overlay
source /opt/ros/humble/setup.bash
source /path/to/arena_isaac_py311_overlay/install/setup.bash

# 确保 Isaac 5.1 自带的 CPython 3.11 rclpy 优先于系统 CPython 3.10 rclpy
export PYTHONPATH="$ISAAC_PATH/exts/isaacsim.ros2.bridge/humble/rclpy:${PYTHONPATH:-}"

ros2 pkg prefix arena_bringup
ros2 pkg prefix task_generator
ros2 pkg prefix task_generator_msgs
ros2 pkg prefix ros2isaacsim
ros2 pkg prefix isaacsim_msgs

"$ISAAC_PATH/python.sh" -c \
  'from isaacsim import SimulationApp; from isaacsim_msgs.srv import ImportUsd; ImportUsd.Request.__class__.__import_type_support__(); print("ISAAC5_ARENA_TYPESUPPORT_READY")'
```

只有最后出现 `ISAAC5_ARENA_TYPESUPPORT_READY`，才进入启动阶段。

### A2. 首次 full Arena 命令

终端 1：按 A1 设置相同的 domain、RMW、Isaac path 与 overlay，确认 preflight PASS 后执行：

```bash
ros2 launch arena_bringup arena.launch.py \
  sim:=isaac \
  human:=isaac \
  robot:=jackal \
  world:=map_empty \
  local_planner:=dwb \
  global_planner:=navfn \
  env_n:=1 \
  headless:=0
```

注意：当前源码即使传 `headless:=0/1`，Isaac runner 仍硬编码 GUI；该参数问题未修复前不能依赖 headless 行为。

终端 2：使用相同 `ROS_DOMAIN_ID=1` 与 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`，等待终端 1 出现 controller/service ready 迹象后检查：

```bash
ros2 node list
ros2 service list | grep '^/isaac/'
ros2 topic list | grep -E 'clock|scan|odom|cmd_vel|tf'
```

预期至少出现 Arena Isaac services（如 `isaac/import_usd`、`isaac/spawn_wall`）、`/clock`、机器人 odom/TF/LiDAR/cmd_vel 链，并在 UI 中实际生成 world 与 Jackal。Ctrl-C 从终端 1 结束。

### A3. 分段 gate

首次获准运行时建议按以下顺序停止在第一处失败，不同时修改多个因素：

1. `SimulationApp` 创建且 required extensions enable。
2. `rclpy` 和 `isaacsim_msgs` service server 创建。
3. Arena Task Generator 连上 `/isaac/*` services。
4. `map_empty`/wall 导入。
5. Jackal import、control graph、TF/odom。
6. LiDAR 与 `cmd_vel`。
7. Isaac pedestrians/NavMesh。

## 8. B：需要修改或补齐的地方

### B1. 启动前必须补齐，不属于源码 API 重写

1. 为 Isaac 5.1/Python 3.11 构建独立的 `isaacsim_msgs` type-support overlay；不要覆盖当前 cp310 install。
2. 使 `ros2isaacsim` 使用同一 cp311 overlay，并完成其实际依赖闭包。
3. 构建/安装 `task_generator_msgs` 及 full launch 所需 Arena core packages。
4. 配置 `ISAAC_PATH=/home/user/isaacsim/5.1.0`；不要依赖写死 4.2 的 `tools/source.bash` 自动发现。
5. 固定 internal Humble `rclpy`/library 搜索顺序，避免系统 cp310 `rclpy` 被 Isaac Python 3.11 先加载。
6. 首次使用 `human:=isaac`，否则默认 `human:=hunav` 会触发当前未安装的 HuNav stack。
7. 决定联网 cloud assets 或安装 5.1 local asset pack；不要把 6.0 assets 静默冒充 5.1 assets。

### B2. 建议在第一次真实 traceback 后再做的源码迁移

这些是长期维护项，但不能仅凭静态检查断言全部是首跑前硬阻塞：

- `omni.isaac.core.*` -> `isaacsim.core.api/utils/prims` 对应接口。
- `omni.isaac.sensor` -> `isaacsim.sensors.camera/physics/physx/rtx`。
- `omni.isaac.nucleus` -> `isaacsim.storage.native`。
- `omni.isaac.ros2_bridge` extension 与 node IDs -> `isaacsim.ros2.bridge`。
- `omni.isaac.core_nodes` -> `isaacsim.core.nodes`。
- 把 `headless`/renderer 从 hard-coded CONFIG 接到 Arena launch arguments。
- 不再吞掉所有 `BaseException`；首跑至少应让 fatal initialization error 明确终止并保留 traceback。
- 在 `ros2isaacsim/package.xml`/Python packaging 中声明实际使用的 `isaacsim_msgs`、NumPy、PyYAML 等依赖。
- 明确 5.1 asset root 配置和缺失资产失败策略。

### B3. 4.2/4.5 路线

- **4.2：** Arena installer 的原始目标是 4.2.0，API 匹配可能比 5.1 更直接；但本机没有该 runtime，选择它意味着新增约大体积下载安装，而且仍不能解决当前 Arena workspace 未完整构建的问题。
- **4.5：** 本机没有完整 4.5 runtime。现有 `.kit` 配置不能作为 `$ISAAC_PATH`。不应尝试把该 72 KB 目录传给 Arena launch。

## 9. C：Isaac 失败后的 Gazebo 方案

切换条件建议明确为以下任一项：

- cp311 `isaacsim_msgs` overlay 无法稳定构建/加载；
- deprecated Isaac API 在 SimulationApp gate 中出现多处不相干失败；
- ROS services 已建立但 world/robot/sensor graph 仍不闭合；
- People/NavMesh/assets 需要超出“第一次体验”范围的大面积迁移。

此时不要继续在 active Isaac checkout 上叠加补丁，回到已审计的隔离 Gazebo 路线：

1. 在 `/home/user/navigation_project/a_pipeline` 之外建立官方 Arena workspace。
2. installer 只选择 Gazebo feature，不选 Isaac、planners、training。
3. 完成 package preflight 后运行：

```bash
ros2 launch arena_bringup arena.launch.py \
  sim:=gazebo \
  human:=hunav \
  robot:=jackal \
  world:=map_empty \
  local_planner:=dwb \
  global_planner:=navfn \
  env_n:=1 \
  headless:=0
```

完整 Gazebo 环境审计和 installer 步骤见 `arena_first_run_environment_report.md`。

## 10. 推荐决策

```text
先保留 Isaac 5.1 runtime
        |
        +-- 建立 cp311 Arena Isaac message/bridge overlay
        |       |
        |       +-- type-support preflight PASS
        |       |       -> 获准后做一次分段 Isaac runtime smoke
        |       |
        |       +-- type-support/build 不闭合
        |               -> 停止 Isaac 首跑路线
        |
        +-- 若 runtime smoke 暴露少量集中 API 问题
        |       -> 基于真实 traceback 做最小 5.1 migration
        |
        +-- 若暴露多处 API/NavMesh/assets/service contract 问题
                -> 转官方隔离 Gazebo workspace
```

当前推荐：**先不要下载 4.x，也不要覆盖 active `arena_ws`。把 5.1 作为唯一 Isaac 首跑候选，先解决可独立验证的 cp311 custom-message gate；只有该 gate PASS 后才值得启动 SimulationApp。若该 gate 或第一次分段 smoke 不闭合，直接转 Gazebo。**

## 11. 本地证据索引

- Isaac 5.1 version：`/home/user/isaacsim/5.1.0/VERSION`
- Isaac 5.1 entrypoints：`/home/user/isaacsim/5.1.0/python.sh`、`isaac-sim.sh`
- Isaac 5.1 ROS bridge：`/home/user/isaacsim/5.1.0/exts/isaacsim.ros2.bridge/`
- Deprecated adapters：`/home/user/isaacsim/5.1.0/extsDeprecated/omni.isaac.*`
- Isaac 5.1 assets setting：`/home/user/isaacsim/5.1.0/exts/isaacsim.storage.native/config/extension.toml`
- Active Arena Isaac：`isaac_sim/arena_ws/src/arena/isaac/`
- Active runner：`isaac_sim/arena_ws/src/arena/isaac/ros2isaacsim/ros2isaacsim/run_isaacsim.py`
- Arena Isaac launch：`isaac_sim/arena_ws/src/arena/arena-rosnav/arena_bringup/launch/simulator/sim/isaac/isaac.launch.py`
- Arena environment：`isaac_sim/arena_ws/src/arena/arena-rosnav/tools/source.bash`
- Arena Isaac installer：`isaac_sim/arena_ws/src/arena/arena-rosnav/installers/2_isaac.sh`
- Isaac 5.1 backup candidate：`isaac_sim/arena_isaac5_backup/`
- Non-runtime 4.5 kit files：`/home/user/navigation_project/robot_related/roboos/apps/isaacsim_4_5/`
- Isaac 6 assets：`isaac_sim/assets-6.0.1/Assets/Isaac/6.0/`
