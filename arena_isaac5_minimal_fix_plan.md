# Arena + Isaac Sim 5.1 最小修复执行计划

规划日期：2026-09-11  
目标：在不修改 active Arena/Isaac 源码、不污染当前 `arena_ws`、不影响 Isaac Sim 6 项目的前提下，为 Arena + Isaac Sim 5.1 第一个 demo 建立可验证的最小运行环境。  
本次边界：只读分析并生成本计划；**未安装 Docker/依赖、未复制源码、未创建 overlay、未编译、未启动 ROS 节点或 Isaac Sim**。除本计划外未修改项目文件。

## 1. 执行结论

需要新建两个互相隔离的 overlay，以及一个只负责进程环境隔离的 runtime shim：

```text
/home/user/arena_isaac5_py311_factory/
    NVIDIA 官方 IsaacSim-ros_workspaces，固定 tag IsaacSim-5.1.0
    内含 Python 3.11 ROS 2 Humble build
    额外构建 active isaacsim_msgs + active ros2isaacsim
    只供 Isaac 5.1 子进程使用

/home/user/arena_isaac5_host_overlay_ws/
    系统 ROS 2 Humble / Python 3.10 overlay
    只构建 task_generator_msgs
    供 ros2 launch、Task Generator 等宿主机进程使用

/home/user/arena_isaac5_runtime_shim/
    仅含 python.sh wrapper
    Isaac 子进程启动前 source Python 3.11 ROS/消息 overlay
    再 exec /home/user/isaacsim/5.1.0/python.sh
```

核心原因是 `isaacsim_msgs` 同时位于两个 Python ABI 域：

| 进程 | Python | `isaacsim_msgs` 用途 | 应加载的构建 |
|---|---:|---|---|
| Arena launch / Task Generator | 3.10 | ROS service client、request/response | 保留当前 cp310 build |
| Isaac 5.1 `ros2isaacsim` | 3.11 | ROS service server | 新建 cp311 build |

不能用一个 overlay 同时覆盖二者，也不能把 cp311 overlay source 到外层系统 `ros2 launch` 终端。最小方案是保留当前 cp310 `isaacsim_msgs`，只把 cp311 版本注入 Isaac 子进程。

## 2. `isaacsim_msgs` 应从哪个源码构建

### 2.1 必须使用的源码

使用当前 active Arena Isaac checkout 中的：

```text
/home/user/navigation_project/a_pipeline/isaac_sim/arena_ws/src/arena/isaac/isaacsim_msgs
```

来源与状态：

```text
repository: https://github.com/Arena-Rosnav/arena-isaac.git
active checkout commit: a4beefe0203ec8a75d65d0ab70496b5e2c400605
branch state: detached HEAD
isaacsim_msgs itself: clean
```

该接口集合与当前 Task Generator 的 `task_generator/simulators/sim/isaac_simulator.py`、`simulators/human/isaac.py` 直接匹配，包括：

- `ImportUsd`
- `ImportYaml`
- `ImportObstacles`
- `DeletePrim`
- `GetPrimAttributes`
- `MovePrim`
- `SpawnWall`
- `UrdfToUsd`
- `Pedestrian`
- `MovePed`
- `Person`、`NavPed`

### 2.2 不应单独采用的源码

不要单独从以下目录重建消息，然后与 active Task Generator 混用：

```text
/home/user/navigation_project/a_pipeline/isaac_sim/arena_isaac5_backup/isaacsim_msgs
```

该 backup 对应远端 `arena5-isaac5.1.0` 分支对象 `16b8e3416517d8c3dc1b5038df4fe11b9a6df46c`，使用 `DeletePrims`、`SpawnPrims`、`ResetWorld` 等另一套接口合同。它只有在 `arena_isaac` backend 与对应 Arena core 一起迁移时才适用，不是当前 `ros2isaacsim` 的最小修复来源。

### 2.3 为什么还要复制 `ros2isaacsim`

`ros2isaacsim` 是纯 Python package。实际探针表明 Isaac Python 3.11 可以读取当前 cp310 install 中的 distribution metadata，但为了让 Isaac 子进程完全落在同一个 cp311 workspace，并避免依赖 cp310 `egg-link`/搜索路径，建议将以下 package 与 `isaacsim_msgs` 一起复制到官方 Python 3.11 build 输入：

```text
/home/user/navigation_project/a_pipeline/isaac_sim/arena_ws/src/arena/isaac/ros2isaacsim
```

这会复制当前工作树，其中 `run_isaacsim.py` 已有用户保留的 URDF importer 修改。执行前必须保存 commit、diff 和 manifest hashes，不能静默把它当成干净 upstream。

## 3. `task_generator_msgs` 在哪里

源码位置：

```text
/home/user/navigation_project/a_pipeline/isaac_sim/arena_ws/src/arena/arena-rosnav/utils/msgs/task_generator_msgs
```

它包含七个 service：

- `GetEnvironments.srv`
- `GetParametrizeds.srv`
- `GetRandoms.srv`
- `GetRobotScenarios.srv`
- `GetRobots.srv`
- `GetScenarios.srv`
- `GetWorlds.srv`

`task_generator.node` 无条件 import `task_generator_msgs.srv`。当前 `arena_ws` 没有其 build/install 目录，因此当前 import 的第一处失败就是：

```text
ModuleNotFoundError: No module named 'task_generator_msgs'
```

该包由外部系统 ROS 2 Humble/Python 3.10 Task Generator 使用，不需要放进 Isaac Python 3.11 workspace。它应单独在 `/home/user/arena_isaac5_host_overlay_ws` 构建。

系统已确认存在它的构建依赖：

- `ament_cmake_auto`
- `rosidl_default_generators`
- `std_msgs`

## 4. 为什么必须新建 overlay workspace

### 4.1 必须新建

是。原因不是目录整洁，而是 ABI 和回滚边界：

1. 当前 `arena_ws/build`、`install` 是 Python 3.10 的历史局部构建，不能原地切换到 Python 3.11。
2. 同一 package `isaacsim_msgs` 必须保留 cp310 与 cp311 两套产物。
3. 原地重建可能覆盖当前可供 Task Generator 使用的 cp310 type support。
4. 当前 `arena_ws` 有用户改动与未跟踪文件；不能 clean、reset 或改变其 build provenance。
5. Isaac 6 项目位于 `a_pipeline/isaac_sim`，所有新增构建应放在 `/home/user` 下独立目录，避免其环境、assets 和运行脚本被继承。

### 4.2 不允许的做法

- 不在 `isaac_sim/arena_ws` 内执行 `colcon build`。
- 不删除或覆盖 `arena_ws/build`、`arena_ws/install`、`arena_ws/log`。
- 不修改 `/home/user/isaacsim/5.1.0` 内的 Python packages。
- 不把 pip/colcon 安装进 Isaac 5.1 embedded Python。
- 不 source Isaac 6 的 `setup_python_env.sh`、assets root 或自研 UDP bridge 环境。
- 不 checkout/reset active `arena-isaac` 的 detached/dirty worktree。
- 不用 `arena_isaac5_backup/isaacsim_msgs` 替换 active message definitions。

## 5. Python 3.11 构建方法选择

### 5.1 推荐且受 NVIDIA 5.1 文档支持的路径

使用 NVIDIA 官方 [IsaacSim-ros_workspaces](https://github.com/isaac-sim/IsaacSim-ros_workspaces)，并固定：

```text
tag: IsaacSim-5.1.0
tag object: 50de00358f220d790d17050c6368cfe9a9cb9f51
Dockerfile: ubuntu_22_humble_python_311_minimal.dockerfile
```

不能使用该仓库当前 `main`：截至本次检查，`main` 的 build script 已切到 Python 3.12 Dockerfile，适配更新的 Isaac 版本，不符合 Isaac Sim 5.1 的 Python 3.11 合同。

官方 5.1 文档要求 Ubuntu 22.04 上供 Isaac 使用的 custom ROS packages 以 Python 3.11 构建，并给出：

```bash
./build_ros.sh -d humble -v 22.04
```

官方参考：[Isaac Sim 5.1 - Enabling rclpy and custom ROS workspaces with Python 3.11](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html#enabling-rclpy-custom-ros-2-packages-and-workspaces-with-python-3-11)。

### 5.2 当前机器缺少的执行前提

当前实测：

```text
docker: missing
vcs: missing on host
/usr/bin/python3.11: missing
Isaac embedded Python: 3.11 exists, but lacks colcon_core/empy/catkin_pkg
disk free: about 1.1 TB
```

推荐 Docker 路线不要求宿主机自行安装 Python 3.11 或 `vcs`；Dockerfile 会在容器内准备它们。宿主机必须先安装并启用 Docker。Docker 安装是系统变更，执行时需要单独授权。

### 5.3 不推荐作为第一选择的宿主机捷径

理论上可以用 Isaac embedded Python 建 venv，再安装 colcon/empy/catkin_pkg，并用系统 ROS CMake packages 生成 cp311 type support；但这会混合：

- Ubuntu Humble Python 3.10 underlay；
- Isaac embedded Python 3.11；
- 手工安装的 ROS build tools；
- Isaac internal Humble libraries。

其可复现性和动态库闭包低于 NVIDIA 官方 Dockerfile。本计划不把该捷径列为首轮执行方案。

## 6. 明确执行步骤

以下所有命令均为**待授权后执行**；本次没有运行。

### Phase 0：冻结路径与执行边界

固定变量，避免使用宽泛路径或误操作当前项目：

```bash
export ARENA_CURRENT_WS=/home/user/navigation_project/a_pipeline/isaac_sim/arena_ws
export ISAAC5_REAL=/home/user/isaacsim/5.1.0
export ISAAC5_PY311_FACTORY=/home/user/arena_isaac5_py311_factory
export ISAAC5_HOST_OVERLAY=/home/user/arena_isaac5_host_overlay_ws
export ISAAC5_SHIM=/home/user/arena_isaac5_runtime_shim

test -x "$ISAAC5_REAL/python.sh"
grep '^5\.1\.0' "$ISAAC5_REAL/VERSION"
test ! -e "$ISAAC5_PY311_FACTORY"
test ! -e "$ISAAC5_HOST_OVERLAY"
test ! -e "$ISAAC5_SHIM"
```

如果任一目标目录已经存在，停止并检查，不要覆盖或删除。

记录 active 输入状态：

```bash
git -C "$ARENA_CURRENT_WS/src/arena/isaac" rev-parse HEAD
git -C "$ARENA_CURRENT_WS/src/arena/isaac" status --short
git -C "$ARENA_CURRENT_WS/src/arena/isaac" diff -- \
  ros2isaacsim/ros2isaacsim/run_isaacsim.py

sha256sum \
  "$ARENA_CURRENT_WS/src/arena/isaac/isaacsim_msgs/package.xml" \
  "$ARENA_CURRENT_WS/src/arena/isaac/isaacsim_msgs/CMakeLists.txt" \
  "$ARENA_CURRENT_WS/src/arena/arena-rosnav/utils/msgs/task_generator_msgs/package.xml" \
  "$ARENA_CURRENT_WS/src/arena/arena-rosnav/utils/msgs/task_generator_msgs/CMakeLists.txt"
```

当前已记录的四个 hash 应分别为：

```text
0e1f84fb2dc7324f03ce432901c40414abde653b6d47e2bb3460a51d59b355e3
0990f5e24677e31e0a2fc23eb3f346adfc1145db63bc5e04dc29006ed4e4566f
7e1764a38048f13bc189ba8d2cc73c394bfc710a1987e59ea95d965985c79aa3
04ce006cf28cb87a770a6f0564c22025efe1510eeb2a264f41924393e31c89d8
```

若 hash 改变，说明源码已漂移，应重新审计接口后再构建。

### Phase 1：安装 Docker（需要明确授权）

Ubuntu 22.04 的最小系统包方案：

```bash
sudo apt-get update
sudo apt-get install -y docker.io
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"
```

退出当前登录会话并重新登录，使 `docker` group 生效，然后验证：

```bash
docker --version
docker info
docker run --rm hello-world
```

不要用 `sudo ./build_ros.sh` 绕过 group 配置，否则输出文件可能全部归 root 所有。

### Phase 2：建立固定版本的 Python 3.11 ROS factory

```bash
cd /home/user
git clone \
  --branch IsaacSim-5.1.0 \
  --depth 1 \
  --recurse-submodules \
  https://github.com/isaac-sim/IsaacSim-ros_workspaces.git \
  "$ISAAC5_PY311_FACTORY"

git -C "$ISAAC5_PY311_FACTORY" rev-parse HEAD
git -C "$ISAAC5_PY311_FACTORY" describe --tags --exact-match
```

预期 tag 为 `IsaacSim-5.1.0`，commit/object 应解析到 `50de00358f220d790d17050c6368cfe9a9cb9f51`。

把 active package **复制**到隔离 factory；不要使用指向 project 的外部 symlink，因为 Docker `COPY` 对 build context 外的链接不可靠：

```bash
mkdir -p "$ISAAC5_PY311_FACTORY/humble_ws/src/arena_local"

cp -a \
  "$ARENA_CURRENT_WS/src/arena/isaac/isaacsim_msgs" \
  "$ISAAC5_PY311_FACTORY/humble_ws/src/arena_local/isaacsim_msgs"

cp -a \
  "$ARENA_CURRENT_WS/src/arena/isaac/ros2isaacsim" \
  "$ISAAC5_PY311_FACTORY/humble_ws/src/arena_local/ros2isaacsim"
```

保存本地修改证据：

```bash
git -C "$ARENA_CURRENT_WS/src/arena/isaac" diff --binary -- \
  ros2isaacsim/ros2isaacsim/run_isaacsim.py \
  > "$ISAAC5_PY311_FACTORY/ARENA_ACTIVE_RUNNER.patch"

git -C "$ARENA_CURRENT_WS/src/arena/isaac" rev-parse HEAD \
  > "$ISAAC5_PY311_FACTORY/ARENA_ACTIVE_SOURCE_COMMIT.txt"

find "$ISAAC5_PY311_FACTORY/humble_ws/src/arena_local" \
  -type f -print0 | sort -z | xargs -0 sha256sum \
  > "$ISAAC5_PY311_FACTORY/ARENA_LOCAL_SOURCE_SHA256SUMS"
```

确认 factory 采用 3.11 Dockerfile：

```bash
grep 'ubuntu_22_humble_python_311_minimal.dockerfile' \
  "$ISAAC5_PY311_FACTORY/build_ros.sh"
```

### Phase 3：在官方容器中构建 cp311 ROS 与 Arena Isaac packages

```bash
cd "$ISAAC5_PY311_FACTORY"
./build_ros.sh -d humble -v 22.04
```

该脚本会在**这个隔离 clone 内**重建 `build_ws/humble`。它包含删除该 clone 下旧 `build_ws/humble` 的逻辑，所以 Phase 0 必须保证 factory 是新目录；不要把 `ISAAC5_PY311_FACTORY` 指向当前项目或已有数据目录。

预期输出：

```text
$ISAAC5_PY311_FACTORY/build_ws/humble/humble_ws/install/
$ISAAC5_PY311_FACTORY/build_ws/humble/isaac_sim_ros_ws/install/
```

构建完成后先检查 ABI，不启动 Isaac：

```bash
find "$ISAAC5_PY311_FACTORY/build_ws/humble/isaac_sim_ros_ws/install" \
  -type f -o -type l | grep 'isaacsim_msgs.*cpython-311.*\.so'

if find "$ISAAC5_PY311_FACTORY/build_ws/humble/isaac_sim_ros_ws/install" \
  -type f -o -type l | grep -q 'isaacsim_msgs.*cpython-310.*\.so'; then
  echo 'FAIL: cp310 artifact leaked into Isaac overlay'
  false
fi
```

在干净 shell 中验证 Python 3.11 type support：

```bash
env -i \
  HOME="$HOME" \
  USER="$USER" \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  ISAAC5_REAL="$ISAAC5_REAL" \
  ISAAC5_PY311_FACTORY="$ISAAC5_PY311_FACTORY" \
  bash --noprofile --norc -c '
    set +u
    source "$ISAAC5_PY311_FACTORY/build_ws/humble/humble_ws/install/local_setup.bash"
    source "$ISAAC5_PY311_FACTORY/build_ws/humble/isaac_sim_ros_ws/install/local_setup.bash"
    set -u
    export ROS_DISTRO=humble
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export PYTHONNOUSERSITE=1
    "$ISAAC5_REAL/python.sh" -c '\''
from importlib.metadata import distribution
from isaacsim_msgs.srv import ImportUsd
ImportUsd.Request.__class__.__import_type_support__()
print("isaacsim_msgs type support:", bool(ImportUsd.Request.__class__._TYPE_SUPPORT))
print("ros2isaacsim distribution:", distribution("ros2isaacsim").version)
print("ISAAC5_CP311_OVERLAY_READY")
'\''
  '
```

接受条件：必须出现 `ISAAC5_CP311_OVERLAY_READY`，且不能出现 `UnsupportedTypeSupport`、cp310 suffix 或 system `/opt/ros/humble` 的 `rclpy` 路径。

### Phase 4：建立宿主机 cp310 `task_generator_msgs` overlay

新终端：

```bash
export ARENA_CURRENT_WS=/home/user/navigation_project/a_pipeline/isaac_sim/arena_ws
export ISAAC5_HOST_OVERLAY=/home/user/arena_isaac5_host_overlay_ws

test ! -e "$ISAAC5_HOST_OVERLAY"
mkdir -p "$ISAAC5_HOST_OVERLAY/src"

cp -a \
  "$ARENA_CURRENT_WS/src/arena/arena-rosnav/utils/msgs/task_generator_msgs" \
  "$ISAAC5_HOST_OVERLAY/src/task_generator_msgs"

set +u
source /opt/ros/humble/setup.bash
source "$ARENA_CURRENT_WS/install/local_setup.bash"
set -u

cd "$ISAAC5_HOST_OVERLAY"
colcon build \
  --base-paths src \
  --packages-select task_generator_msgs \
  --event-handlers console_direct+
```

这一步只写 `/home/user/arena_isaac5_host_overlay_ws/{build,install,log}`，不会写当前 `arena_ws`。

验证宿主机消息与 Task Generator import：

```bash
set +u
source /opt/ros/humble/setup.bash
source "$ARENA_CURRENT_WS/install/local_setup.bash"
source "$ISAAC5_HOST_OVERLAY/install/local_setup.bash"
set -u

python3 - <<'PY'
from isaacsim_msgs.srv import ImportUsd
from task_generator_msgs.srv import GetWorlds

ImportUsd.Request.__class__.__import_type_support__()
GetWorlds.Request.__class__.__import_type_support__()

print("host isaacsim_msgs:", bool(ImportUsd.Request.__class__._TYPE_SUPPORT))
print("host task_generator_msgs:", bool(GetWorlds.Request.__class__._TYPE_SUPPORT))

import task_generator.node
print("ARENA_HOST_CP310_READY")
PY
```

接受条件：出现 `ARENA_HOST_CP310_READY`。如果出现新的 Python dependency error，停止并记录完整 traceback；不要向 system Python 随机 pip install。应将新的缺失项加入单独的 Arena host dependency closure，再决定是否需要第三个隔离 venv/rebuild。

### Phase 5：建立只作用于 Isaac 子进程的 runtime shim

为什么需要 shim：外层 `ros2 launch` 必须继续使用 system Humble/cp310；Isaac 子进程必须使用 Python 3.11 ROS build。当前 Arena launch 只能调用 `$ISAAC_PATH/python.sh`，没有对子进程单独设置环境的接口。

待执行时创建：

```text
/home/user/arena_isaac5_runtime_shim/python.sh
```

内容应为：

```bash
#!/usr/bin/env bash
set -eo pipefail

ISAAC5_REAL=/home/user/isaacsim/5.1.0
ISAAC5_PY311_FACTORY=/home/user/arena_isaac5_py311_factory

export ROS_DISTRO=humble
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONNOUSERSITE=1

set +u
source "$ISAAC5_PY311_FACTORY/build_ws/humble/humble_ws/install/local_setup.bash"
source "$ISAAC5_PY311_FACTORY/build_ws/humble/isaac_sim_ros_ws/install/local_setup.bash"
set -u

exec "$ISAAC5_REAL/python.sh" "$@"
```

给 wrapper 执行权限：

```bash
chmod 0755 /home/user/arena_isaac5_runtime_shim/python.sh
```

这里将 `ISAAC_PATH` 指向 shim，而不是改 Arena launch。全 workspace 搜索表明 active runtime 只在 Isaac launch 中用 `ISAAC_PATH` 拼接 `python.sh`；源码其他运行部分不依赖它寻找 assets/extensions。

先用 shim 做不创建 `SimulationApp` 的验证：

```bash
export ROS_DOMAIN_ID=1
/home/user/arena_isaac5_runtime_shim/python.sh -c '
from isaacsim_msgs.srv import ImportUsd
ImportUsd.Request.__class__.__import_type_support__()
print("ISAAC5_SHIM_READY")
'
```

### Phase 6：完整 launch 前的只读 preflight

外层终端只 source system/cp310 环境：

```bash
export ARENA_CURRENT_WS=/home/user/navigation_project/a_pipeline/isaac_sim/arena_ws
export ISAAC5_HOST_OVERLAY=/home/user/arena_isaac5_host_overlay_ws
export ISAAC_PATH=/home/user/arena_isaac5_runtime_shim
export ROS_DISTRO=humble
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

set +u
source /opt/ros/humble/setup.bash
source "$ARENA_CURRENT_WS/install/local_setup.bash"
source "$ISAAC5_HOST_OVERLAY/install/local_setup.bash"
set -u

python3 --version
ros2 pkg prefix arena_bringup
ros2 pkg prefix arena_simulation_setup
ros2 pkg prefix task_generator
ros2 pkg prefix task_generator_msgs
ros2 pkg prefix ros2isaacsim
ros2 pkg prefix isaacsim_msgs

python3 -c 'import task_generator.node; print("TASK_GENERATOR_IMPORT_READY")'
"$ISAAC_PATH/python.sh" -c \
  'from isaacsim_msgs.srv import ImportUsd; ImportUsd.Request.__class__.__import_type_support__(); print("ISAAC_CHILD_TYPESUPPORT_READY")'

ros2 launch arena_bringup arena.launch.py --show-args
```

应确认：

- 外层 `python3` 是 3.10；
- 外层 `isaacsim_msgs` 仍指向当前 Arena cp310 package；
- `task_generator_msgs` 指向 host overlay；
- shim 内 `isaacsim_msgs` type support 是 cp311；
- 没有 source Isaac 6 环境；
- `--show-args` 完成且未启动节点。

### Phase 7：获准后的第一次 demo

本阶段会真正启动仿真，本次没有执行。只有 Phase 3、4、5、6 全部 PASS 后才运行：

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

必须显式 `human:=isaac`，避免当前未安装的 HuNav backend。

第一次运行按以下 gate 逐层判断，并在第一处失败停止：

1. shim 进入 Python 3.11 overlay。
2. `SimulationApp` 创建。
3. deprecated/new Isaac extensions enable。
4. `ros2isaacsim` controller 创建 `/isaac/*` services。
5. Task Generator 连接 services。
6. `map_empty`/walls 生成。
7. Jackal、TF、odom、LiDAR、cmd_vel 出现。
8. Isaac pedestrians/NavMesh 工作。

不要在一次失败后同时迁移 API、修改 assets、改 ROS middleware 和改 Task Generator。

## 7. 环境隔离与不影响现有项目的验证

### 7.1 当前 Arena workspace

执行前后分别运行：

```bash
git -C /home/user/navigation_project/a_pipeline status --short
git -C /home/user/navigation_project/a_pipeline/isaac_sim/arena_ws/src/arena/isaac status --short
```

允许新增的项目内文件只有本计划；后续所有 factory/overlay/shim 均在 `/home/user/arena_isaac5_*`。不得出现对以下目录的新写入：

```text
/home/user/navigation_project/a_pipeline/isaac_sim/arena_ws/build
/home/user/navigation_project/a_pipeline/isaac_sim/arena_ws/install
/home/user/navigation_project/a_pipeline/isaac_sim/arena_ws/log
```

可以在执行前后记录这些目录的 mtime 或文件清单作为证据。

### 7.2 Isaac 5.1 runtime

不向 `/home/user/isaacsim/5.1.0` 执行 pip install，不编辑其 extensions/config。它只作为只读 runtime 被 shim `exec`。

### 7.3 Isaac 6 项目

执行终端中不得出现：

```text
/home/user/navigation_project/a_pipeline/isaac_sim/isaacsim-6.0.1
/home/user/navigation_project/a_pipeline/isaac_sim/assets-6.0.1
ISAAC_PEDESTRIAN_*
自研 UDP bridge 的 PYTHONPATH/LD_LIBRARY_PATH
```

预检命令：

```bash
env | grep -E 'isaacsim-6\.0\.1|assets-6\.0\.1|ISAAC_PEDESTRIAN|UDP' && {
  echo 'FAIL: Isaac 6 environment leaked into Arena 5.1 terminal'
  false
} || true
```

## 8. 停止条件与回滚

### 8.1 停止条件

- Docker build 使用的不是 `IsaacSim-5.1.0` tag/Python 3.11 Dockerfile。
- cp311 overlay 出现 `cpython-310` type-support artifact。
- host overlay 覆盖了现有 cp310 `isaacsim_msgs`。
- `task_generator.node` 在补齐消息后仍出现新的依赖错误。
- shim 内 import 到 `/opt/ros/humble` 的 cp310 `rclpy`。
- 需要修改 active source 才能通过尚未运行的静态 gate。
- 任何命令准备写入 Isaac 6 runtime/assets 或当前 `arena_ws/build/install/log`。

### 8.2 回滚

本方案不修改 active source/runtime。若执行失败，停止 source 这些 overlay 并开新终端即可恢复原环境。三个新目录可先保留供取证：

```text
/home/user/arena_isaac5_py311_factory
/home/user/arena_isaac5_host_overlay_ws
/home/user/arena_isaac5_runtime_shim
```

删除它们属于后续独立的破坏性操作，本计划不自动执行。

## 9. 最小成功标准

在真正启动 demo 前，必须全部满足：

| Gate | 成功判据 |
|---|---|
| Source contract | cp311 与 cp310 均来自 active `isaacsim_msgs` definitions |
| Python 3.11 | `ImportUsd` type support 在 Isaac 5.1 `python.sh` 下加载成功 |
| Python 3.10 | `ImportUsd` 与 `GetWorlds` type support 在 system Python 下加载成功 |
| Task Generator | `import task_generator.node` 成功 |
| Isolation | 当前 `arena_ws/build/install/log` 无写入；Isaac 6 环境无泄漏 |
| Launch parse | `arena.launch.py --show-args` 成功 |
| Runtime gate | 获准后 `/isaac/*` services、world、Jackal、TF/odom/LiDAR/cmd_vel 逐层出现 |

前六项只证明“可以开始 runtime smoke”，不证明 demo 已跑通。只有最后的真实运行证据才可将 Arena + Isaac 5.1 标为成功。

## 10. 建议执行顺序

```text
先授权 Docker 系统安装
        |
        v
固定 IsaacSim-ros_workspaces tag 5.1.0
        |
        v
构建 cp311 isaacsim_msgs + ros2isaacsim
        |
        +-- type support FAIL -> 停止，不启动 Isaac
        |
        v
独立构建 cp310 task_generator_msgs
        |
        +-- task_generator import FAIL -> 只分析下一缺失依赖
        |
        v
创建并验证 Isaac 子进程 shim
        |
        v
外层 launch 只读 preflight
        |
        v
用户再次确认后启动第一次 demo
```

下一次最合理的执行授权范围是：**只安装/验证 Docker，并创建 `/home/user/arena_isaac5_py311_factory` 和 `/home/user/arena_isaac5_host_overlay_ws`；仍不启动仿真。**

