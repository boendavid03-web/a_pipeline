# Arena-Rosnav 第一次运行环境检查报告

审计日期：2026-09-10  
审计范围：`/home/user/navigation_project/a_pipeline/isaac_sim/arena_ws` 及与其直接相关的系统、ROS 2、Gazebo、Isaac Sim 安装  
执行边界：只读检查源码、目录、版本、包索引、Python import 和历史 colcon 日志；**未安装依赖、未编译、未启动 Gazebo/Isaac/ROS 节点、未运行训练**。除本报告外未修改项目文件。

## 1. 当前状态总结

### 1.1 结论

当前 `arena_ws` 是一个**源码较多、但只做过局部构建的 Arena 工作区**，不能按现状直接跑通完整官方 demo。

- 源码侧：`colcon list --base-paths src` 能识别 **107 个包**；Arena core、Task Generator、simulation setup、Arena Evaluation、旧式 Arena Isaac bridge、HuNav、Nav2、机器人描述和若干 planners 源码均有不同程度的存在。
- 构建侧：`build/` 与 `install/` 各只有 **15 个包目录**。已安装的核心包包括 `arena_bringup`、`task_generator`、`arena_simulation_setup`、`isaacsim_msgs`、`ros2isaacsim`；但 `task_generator_msgs`、HuNav、Arena Evaluation 等没有进入 install space。
- 环境侧：Ubuntu 22.04.5、ROS 2 Humble、Python 3.10.12 符合本地 Arena 源码的基本平台方向；但当前 shell 未 source ROS、未激活 conda/venv，而且本机没有 `poetry`、`vcs` 或 Arena 自己的 `.venv`。
- Gazebo 侧：Gazebo Classic 11.10.2 与 Gazebo Sim 6.18.0 均已安装，ROS Humble 的 `ros_gz_sim`、`ros_gz_bridge`、`ros_gz_interfaces` 也存在；但这份 Arena 源码默认声明 `GAZEBO_VERSION=harmonic`，当前 Gazebo Sim 6.18 属于 Fortress 代际，不能未经验证就视为版本匹配。
- Isaac 侧：本地有完整 Isaac Sim `6.0.1-rc.7` 程序（约 34 GB）与 `assets-6.0.1`（约 98 GB）；但是 Arena Isaac checkout 是 2025-07-10 的提交，工作树已有用户修改，入口还大量使用 `omni.isaac.*` 兼容/旧 API，且本地 Arena installer 仍写死 Isaac 4.2.0。它不是当前最稳妥的第一次体验入口。
- `rosnav-rl` 与新的 `Arena-Training` **不存在于当前 active source/install**。`arena-rosnav/training/` 只是旧代码，并且其 `package.xml`、`setup.py`、`CMakeLists.txt` 均用点文件禁用。

因此，推荐选择：**方案 C（官方自动安装流程）+ Gazebo simulator**，并安装到当前项目之外的独立 workspace。它同时是“最接近原版”和“最不容易破坏现有 Isaac 项目”的路径。

### 1.2 审计方法与证据等级

- **已确认可发现**：文件、目录、包索引或可执行文件当前存在。
- **历史构建成功**：colcon 日志中相应 job 返回 0；不等于完整栈当前可运行。
- **当前缺失/冲突**：通过 `ros2 pkg prefix`、Python import、CLI 查找或版本检查直接确认。
- **未验证运行**：本次没有启动仿真，因此不把静态条件满足写成 demo 已成功。

## 2. 第一部分：现有 Arena 安装状态

### 2.1 系统与运行环境

| 项目 | 当前实测 | 判断 |
|---|---|---|
| Ubuntu | 22.04.5 LTS (Jammy) | 符合官方文档支持的平台 |
| Kernel | 6.8.0-136-generic | 已记录；不是当前阻塞项 |
| ROS 2 | `/opt/ros/humble`；`ros-humble-ros2cli 0.18.19` | Humble 已安装；当前 IDE shell 未 source |
| Python | `/usr/bin/python3` 3.10.12 | 版本方向符合本地 `pyproject.toml` 的 `^3.10` |
| conda | 未找到 conda executable 或 conda installation | Arena 本地方案使用 Poetry，不要求 conda |
| venv | `VIRTUAL_ENV` 为空；Arena `.venv` 不存在 | 官方安装器应创建隔离环境 |
| Poetry | 缺失 | 当前 `tools/source.bash` 会调用它，属于直接阻塞 |
| vcstool (`vcs`) | 缺失 | 无法按 `.repos` 补齐源码 |
| GPU | RTX 5090，driver 595.84，32 GB VRAM | 对 Isaac 足够，但不是 Gazebo 首跑必需条件 |
| 磁盘 | 根分区约 1.1 TB 可用 | 足以建立独立 Arena workspace |

重要冲突：系统 Python 当前优先加载用户目录中的 NumPy 1.26.4，而 `/opt/ros/humble` 的 `tf_transformations` 依赖旧 `transforms3d 0.3.1`；实际 import 因 `np.float` 已移除而失败。使用官方安装器创建的隔离 Poetry 环境比直接在当前系统 Python 上补包更可靠。

### 2.2 `arena_ws` 结构与完整度

```text
arena_ws/
├── src/       4.4 GB；colcon 可识别 107 个包
│   ├── arena/    Arena core、evaluation、Isaac、simulation setup、tools
│   ├── deps/     HuNav、Nav2、robots、slam_toolbox 等
│   ├── gazebo/   仅 turtlebot4_simulator；manifest 中另外四个仓库未落盘
│   └── planners/ DRL-VO、CrowdNav、PaS、SICNav 及辅助依赖
├── build/     149 MB；15 个包
├── install/    61 MB；15 个包
└── log/        18 MB；存在 4 次 build 日志和若干 list 日志
```

最近四次 build 都发生于 2026-08-09：第一次只构建 `--packages-up-to isaacsim_msgs ros2isaacsim arena_simulation_setup task_generator arena_bringup`，随后分别单独构建 `rviz_utils`、`arena_bringup`、`ros2isaacsim`。日志没有非零 job return code；`arena_simulation_setup` 有若干缺 `__init__.py` 的 setuptools warning。它们证明“这 15 个包曾局部构建”，不证明 107 包全量 build 完成。

install 内共有 9239 个符号链接，检查时没有 broken symlink；但**包覆盖率不完整**才是主要问题。

### 2.3 Arena 组件矩阵

| 组件 | src | build/install | 当前判断 |
|---|---:|---:|---|
| `arena_bringup` | 有 | 有 | 主 launch 存在 |
| `task_generator` | 有 | 有 | executable 存在，但运行期依赖未闭合 |
| `task_generator_msgs` | 有 | 无 | 必须先构建；`task_generator.node` 无条件 import 它 |
| `arena_simulation_setup` | 有 | 有 | world、robot、Gazebo model 资产丰富 |
| `arena_rclpy_mixins` / `rviz_utils` | 有 | 有 | 已局部安装 |
| `rl_utils` / `task_generator_gui` | 有 | 无 | 非最小首跑核心，但完整 workspace 未构建 |
| `arena_evaluation` / `_msgs` | 有 | 无 | 首次导航可暂缓；评测前必须构建 |
| `arena_tools` | 有 | 无 | 首次 demo 可暂缓 |
| `isaacsim_msgs` / `ros2isaacsim` | 有 | 有 | 历史安装存在；当前系统 Python 无 `isaacsim`，必须由 Isaac `python.sh` 启动 |
| HuNav (`hunav_msgs`, `hunav_agent_manager`, `hunav_sim`) | 有 | 无 | 默认 `human:=hunav` 的直接阻塞项 |
| Nav2 | 大量源码存在；系统也有 binaries | 主要由 `/opt/ros/humble` 提供 | DWB/NavFn 首跑基础已在系统中 |
| `rosnav-rl` | 无 | 无 | 当前不存在 |
| `Arena-Training` | 无 | 无 | 当前不存在；首跑不需要 |

### 2.4 现有 world、robot 与 models

- active world 目录有：`.generated`、`factory`、`generated`、`hospital`、`house17`、`ignc`、`map_empty`。
- active 与 `.old` 合计找到 299 个 `.world`/`.sdf`/地图/场景类文件；`gazebo_models/` 顶层约 380 个 model 目录。
- `map_empty` 有 map、scenario、walls/zones 和 mesh；Gazebo launch 在找不到 `<world>/worlds/<world>.world` 时回退到 `arena_bringup/configs/gazebo/empty.sdf`，再由 world generator/task generator 工作。
- 默认机器人 `jackal` 的 URDF、mesh、Nav2 config、control config 和 Gazebo topic mappings 均存在。

所以不是“没有 world/model/assets”，而是**运行环境、精确 Gazebo 版本、HuNav/plugin、消息包和构建层没有闭合**。

## 3. 第二部分：三种运行方式评估

### 3.1 方案 A：Arena + Gazebo

当前条件：

- Gazebo Classic：11.10.2；已安装，但本地 Arena 主 launch 不走 Classic。
- Gazebo Sim：6.18.0；这是 Fortress 代际。
- ROS bridge：`ros_gz_sim`、`ros_gz_bridge`、`ros_gz_interfaces` 已由 `/opt/ros/humble` 提供。
- Arena 本地源码：`tools/source.bash` 默认 `GAZEBO_VERSION=harmonic`；Gazebo installer 安装 `gz-${GAZEBO_VERSION}`，并准备对应的 `ros_gz`、`sdformat_urdf`、HuNav Gazebo plugin 路线。
- 缺失的 manifest 仓库目录：`src/gazebo/ros_gz`、`src/gazebo/sdformat_urdf`、`src/deps/robots/turtlebot4`、`src/gazebo/hunav_gz_plugin`。
- 缺失的 runtime 包：`hunav_agent_manager`、`hunav_msgs`、`hunav_sim`；未发现 HuNav Gazebo `.so` plugin。

判断：**最适合第一次体验，但不能直接用当前 install 启动。** 要么让官方 installer 在隔离 workspace 安装它指定的 Gazebo feature，要么在以后明确授权后补齐当前 workspace 并重建。不要假设系统已有 Fortress 可无缝替代源码声明的 Harmonic。

### 3.2 方案 B：Arena + Isaac Sim

当前条件：

- Isaac Sim：`6.0.1-rc.7+release.42383...`，`python.sh` 可执行，embedded Python 是 3.12.13；`isaacsim` 基础 import 成功。
- assets：本地 `assets-6.0.1` 约 98 GB，不需要为当前自研 Isaac 链重新下载。
- Arena Isaac checkout：commit `a4beefe`，detached HEAD，分支名显示 `isaac6.0.0`，但 `run_isaacsim.py` 当前已有未提交修改。
- 兼容风险：代码仍大量 import `omni.isaac.core`、`omni.isaac.ros2_bridge` 等旧/兼容 API；未执行 SimulationApp 级启动验证。
- 环境入口不闭合：Arena `isaac.launch.py` 要求 `ISAAC_PATH/python.sh`；Arena `source.bash` 却在 `.installed` 标记中寻找 installer 后再 source `~/isaacsim-4.2.0/setup.bash`。当前 `.installed` 与该 setup 文件均不存在。
- active 自研 Isaac 6 UDP bridge 与 `ros2isaacsim` 的 Arena service bridge 是两条不同链，不能互换。

判断：**不适合作为第一次官方体验路线。** 它需要先冻结/审计已有修改、统一 Isaac 版本入口、验证旧 API、ROS 2 bridge 和 Arena entity services。直接尝试的故障面远大于 Gazebo。

### 3.3 方案 C：官方当前入门方式

官方文档给出的最小启动命令是：

```bash
ros2 launch arena_bringup arena.launch.py sim:=gazebo
```

官方安装指南支持 Ubuntu 22.04，并推荐自动 installer；Gazebo、Isaac 和 planners 是可选 feature。参见：

- [官方 Installation](https://arena-rosnav.readthedocs.io/en/latest/tutorials/installation/)
- [官方 Getting Started / Usage](https://arena-rosnav.readthedocs.io/en/latest/tutorials/usage/)
- [官方 Arena-Rosnav GitHub](https://github.com/Arena-Rosnav/arena-rosnav)
- [Arena 5.0 组织主页与组件说明](https://github.com/Arena-Rosnav)

判断：官方文档页面标题仍显示 Arena 4.0，但组织主页标注 Arena 5.0；两者对第一次启动的共同稳定路径仍是 **Ubuntu 22.04 + 官方 installer + Gazebo + `arena.launch.py`**。本报告不把第三方 fork 当成官方推荐依据。

### 3.4 最终推荐

选择 **C，具体 simulator 选 Gazebo**。

理由：

1. 这是官方文档直接给出的入门命令。
2. 当前机器的 OS、ROS 方向、显卡和磁盘都满足；不需要碰现有 Isaac 6 runtime。
3. Gazebo 的调试面较小，Jackal/world/model 已是 Arena 原生资产。
4. 在项目外创建独立 workspace 可以避开当前 `arena_ws` 的部分 install、系统 Python 冲突和 Arena Isaac 未提交修改。
5. `rosnav-rl`、训练模型和 Arena Evaluation 都不是第一次看到机器人在 Arena 中导航的前置条件。

## 4. 第三部分：官方 demo 入口与第一次启动步骤

### 4.1 当前源码中的入口

| 类型 | 路径 | 作用 |
|---|---|---|
| 主入口 | `src/arena/arena-rosnav/arena_bringup/launch/arena.launch.py` | 创建 task generators、simulator、world generator |
| simulator selector | `arena_bringup/launch/simulator/sim/sim.launch.py` | 当前 active choices：`dummy`、`gazebo`、`isaac` |
| Gazebo | `arena_bringup/launch/simulator/sim/gazebo/gazebo.launch.py` | 启动 `ros_gz_sim`、clock bridge、model staging |
| Isaac | `arena_bringup/launch/simulator/sim/isaac/isaac.launch.py` | 用 `$ISAAC_PATH/python.sh` 启动 `ros2isaacsim` |
| Task Generator | `task_generator/launch/task_generator.launch.py` | map server、RViz、task generator node |
| 官方本地说明 | `src/arena/arena-rosnav/README.md` | 指向官方自动安装文档 |
| 环境脚本 | `src/arena/arena-rosnav/tools/source.bash` | 激活 Poetry、ROS、overlay，并设 Arena 环境变量 |

`.old/` 下的 launch 与 `arena_evaluation/README.md` 中 ROS 1 `roslaunch` 示例不是当前 ROS 2 首跑入口。

### 4.2 推荐的完整命令（**本次未执行**）

为了不改动 `/home/user/navigation_project/a_pipeline/isaac_sim`，建议开一个全新终端，并安装到项目外，例如 `/home/user/arena_first_run_ws`。

```bash
# 新终端：不要预先 source ROS，不要激活 conda/venv
env | grep -E '^(ROS_|AMENT_|COLCON_|CONDA_|VIRTUAL_ENV)' || true

cd /home/user
curl https://raw.githubusercontent.com/Arena-Rosnav/arena-rosnav/humble/installers/install.sh \
  -o arena_rosnav_install.sh

# installer 询问 workspace 时输入：/home/user/arena_first_run_ws
# optional feature 只选择 Gazebo；第一次体验不选 Isaac、planners、training
bash arena_rosnav_install.sh
```

安装结束且明确出现 installer 的完成提示后，开一个新终端：

```bash
cd /home/user/arena_first_run_ws
. arena.bash

# 先做轻量确认
ros2 pkg prefix arena_bringup
ros2 pkg prefix task_generator
ros2 pkg prefix hunav_agent_manager
ros2 pkg prefix ros_gz_sim

# 官方最小 demo；Ctrl-C 结束
ros2 launch arena_bringup arena.launch.py \
  sim:=gazebo \
  robot:=jackal \
  world:=map_empty \
  local_planner:=dwb \
  global_planner:=navfn \
  human:=hunav \
  env_n:=1 \
  headless:=0
```

预期现象：Gazebo 与 RViz 启动；Task Generator 生成 Jackal、任务与障碍物；Nav2/DWB 接收目标并发布速度。若 RViz 没有自动任务，可依照界面使用 2D/Nav2 Goal。成功判据至少包括 `/clock`、机器人 namespaced `odom`/`cmd_vel`、LiDAR、TF 存在且机器人实际移动。

注意：以上是**安装后应执行的命令**，不是当前 `arena_ws` 已经具备的可运行证明。

### 4.3 当前 `arena_ws` 为什么不能直接执行同一命令

即使手工执行：

```bash
source /opt/ros/humble/setup.bash
source /home/user/navigation_project/a_pipeline/isaac_sim/arena_ws/install/setup.bash
ros2 launch arena_bringup arena.launch.py sim:=gazebo
```

仍有以下已确认阻塞：

- `task_generator_msgs` 未构建/未安装；
- `hunav_agent_manager`、`hunav_msgs`、`hunav_sim` 未构建/未安装；
- Arena Poetry 环境不存在，`poetry` CLI 缺失；
- 当前 system Python 与 Arena 声明依赖存在多项缺失/版本不符，并已有 `tf_transformations` import failure；
- 本地代码声明 Harmonic，而现有 `gz sim` 是 6.18/Fortress；
- Gazebo/HuNav feature manifest 中的仓库没有全部落盘；
- root 下没有 installer 创建的 `arena.bash`、`.installed` 等完成标记。

## 5. 第四部分：缺失依赖

### 5.1 第一次 Gazebo demo 必须补齐

#### Arena/ROS packages

- 构建并安装：`task_generator_msgs`。
- 构建并安装：`hunav_msgs`、`hunav_agent_manager`、`hunav_sim`；默认 `human:=hunav` 会启动其中组件。
- 补齐与选定 Gazebo 版本一致的 `ros_gz`/`sdformat_urdf`/HuNav Gazebo plugin；本地 `.repos/gazebo.repos` 声明的相关目录目前缺失。
- 全量重建 Arena core 的依赖闭包，而不是只依赖现有 15 包 install。

#### Python/环境工具

- `poetry` 和 Arena in-project `.venv`。
- `vcs`/vcstool。
- 当前缺失的 core Python 包至少包括：`rospkg`、`filelock`、`defusedxml`、`watchdog`、`rosros`。
- 当前已装但不符合本地 `pyproject.toml` 约束的主要包包括：PyYAML 5.4.1（要求 `^6.0`）、SciPy 1.8.0（要求 `^1.8.1`）、lxml 4.8.0（要求 `^4.9.1`）、transforms3d 0.3.1（要求 `^0.4.2`）、Pydantic 1.10.26（要求 `^2.11.5`）、Pillow 9.0.1（要求 `^11.2.1`）、requests 2.25.1（要求 `^2.32.4`）。
- `opencv-python` 的 pip distribution metadata 不存在，但系统 `cv2 4.5.4` 可 import；是否满足应由隔离环境安装器决定，不应在 system Python 上混装猜测。

#### Simulator/system

- 使用这份源码时应按 installer 安装其声明的 Gazebo Harmonic feature，以及匹配的 SDFormat/ROS bridge/plugin。
- 当前已有 Classic 11 与 Fortress，不能把“有 gazebo executable”当成版本匹配。

### 5.2 第一次 demo 不需要，后续再装

- `rosnav-rl`、`Arena-Training`、Stable-Baselines3、Gym、SB3-Contrib、Torch、TensorBoard、W&B：只在 DRL inference/training 阶段需要。
- `arena_evaluation`、`arena_evaluation_msgs`、pandas、seaborn、scikit-learn：只在官方记录/统计/绘图阶段需要；当前均未安装或未构建。
- DRL-VO/CrowdNav/PaS/SICNav planners 及其模型权重：不需要用于 DWB/NavFn 首跑。
- Isaac Sim、Isaac assets、`ros2isaacsim`：Gazebo 首跑不需要。
- TurtleBot4 feature：默认使用 Jackal 时不是必需；当前已有 Jackal description/config。
- hospital Fuel 下载工具的 `docopt`：选择 `map_empty` 首跑时不是核心依赖。

## 6. 第五部分：是否需要重新下载

如果目标只是“第一次体验 Arena 机器人导航”：

| 项目 | 是否需要 | 结论 |
|---|---:|---|
| 手工重新 clone 当前目录 | 否 | 不建议覆盖或删除当前 `arena_ws` |
| 新建隔离 Arena workspace | **建议** | 官方 installer 会取得一致的 Arena source/deps，并生成环境 wrapper |
| 重新安装/下载 Gazebo | **需要补齐官方所选版本** | 当前 Fortress 与本地源码声明的 Harmonic 不一致；让 installer 管理最稳妥 |
| 下载 Isaac Sim | 否 | 首跑选择 Gazebo；现有 6.0.1 也无需重复下载 |
| 下载 Isaac assets | 否 | Gazebo 首跑不需要；现有 assets 已约 98 GB |
| 下载训练模型 | 否 | DWB/NavFn 是经典 planner，不需要神经网络权重 |
| 下载 Arena world/models | 由新 workspace installer 获取 | 当前已有大量资产，但为保证官方版本一致，隔离 workspace 应使用其自己的 simulation setup |

最小下载策略：**只运行官方 core installer，并只选 Gazebo feature；不选 Isaac、planners、training。** 这仍可能下载/编译较多 ROS/Arena 依赖，因为官方安装器按自己的环境隔离策略工作，但避免了 Isaac 和训练模型的大体积下载。

若以后只追求“最少网络流量”而接受不是全新官方环境，可以在明确授权后补齐当前 workspace 缺失仓库、Poetry 环境和包并重建；但当前 core 目录缺少独立 git provenance、已有部分 build/install 和 Python 冲突，这条路的排错时间可能高于重新建立隔离环境。

## 7. 第六部分：与当前 SemanticCNN / DRL-VO 项目的关系

未来可以形成：

```text
Arena task/world + Gazebo or Isaac
        │
        ├── LaserScan ──> SemanticCNN / pedestrian perception（可选）
        ├── goal + odom/TF ───────────────────────────────┐
        └── pedestrian GT（仅 reward/evaluation，不泄漏给部署 policy）
                                                         ▼
                                         DRL-VO / rosnav-rl policy
                                                         │
                                                         ▼
                                              geometry_msgs/Twist
                                                         │
                                                         ▼
                                                     cmd_vel
```

最小接口合同：

| 方向 | 接口 | 建议 |
|---|---|---|
| 输入 | `sensor_msgs/LaserScan` | 明确选 Arena robot 的 `lidar`/`scan` topic，固定 beams、FOV、range、frame、QoS |
| 输入 | goal（通常 `geometry_msgs/PoseStamped`） | 明确 map/odom/base frame，并可派生 local goal 的距离与方位 |
| 输入 | `nav_msgs/Odometry` + TF | 提供 robot pose、linear/angular velocity；统一 `use_sim_time` |
| 可选输入 | pedestrian tracks | SemanticCNN/DR-SPAAM 输出需做消息 adapter；GT 只用于 reward/evaluation |
| 输出 | `geometry_msgs/Twist` `cmd_vel` | 保证只有一个控制 publisher；声明 differential/holonomic action 约束 |
| 生命周期 | reset/episode event | 清理 DRL-VO scan history、tracker、policy recurrent state，并等待新 observation |

建议分两步：

1. 先用原版 Arena + Gazebo + DWB/NavFn 跑通，记录其 namespace、topic、TF、goal 与 reset 语义。
2. 再做只读接口映射，随后在单独 adapter/package 中接 SemanticCNN 与 DRL-VO；不要改 Arena simulator 核心，也不要复用当前自研 Isaac UDP bridge 来冒充 Arena `ros2isaacsim`。

## 8. 最终回答

1. 当前已经有 Arena core、Task Generator、simulation setup、world/models、Arena Evaluation 源码、HuNav/Nav2 源码、Arena Isaac bridge 源码和部分 install。
2. 当前缺少完整 Python/Poetry 环境、vcstool、Task Generator messages install、HuNav install/plugin、与源码匹配的 Gazebo feature，以及完整 colcon build；`rosnav-rl`/Arena-Training 完全不在 active workspace。
3. 最快且可靠的第一次“原版”体验是：在项目外用官方 installer 建立隔离 workspace，只选 Gazebo，然后运行 `ros2 launch arena_bringup arena.launch.py sim:=gazebo`。
4. 不推荐第一次走 Isaac；它的版本/API/bridge/工作树状态都增加了不必要的不确定性。Flatland/Unity 也不是当前 active launch 的可用首选。
5. 不需要删除或覆盖任何现有环境，不需要重新下载 Isaac 或训练模型；需要为隔离 Arena workspace 下载其 core dependencies 和匹配的 Gazebo feature。
6. 当前报告只完成静态与环境检查。官方 demo 仍是**待安装、待启动、待运行验证**，不能写成已经跑通。

## 9. 本地证据索引

- Arena root README：`isaac_sim/arena_ws/src/arena/arena-rosnav/README.md`
- 主 launch：`isaac_sim/arena_ws/src/arena/arena-rosnav/arena_bringup/launch/arena.launch.py`
- Gazebo launch：`isaac_sim/arena_ws/src/arena/arena-rosnav/arena_bringup/launch/simulator/sim/gazebo/gazebo.launch.py`
- Isaac launch：`isaac_sim/arena_ws/src/arena/arena-rosnav/arena_bringup/launch/simulator/sim/isaac/isaac.launch.py`
- Task Generator：`isaac_sim/arena_ws/src/arena/arena-rosnav/task_generator/`
- Arena environment script：`isaac_sim/arena_ws/src/arena/arena-rosnav/tools/source.bash`
- Arena Python constraints：`isaac_sim/arena_ws/src/arena/arena-rosnav/pyproject.toml`
- Gazebo installer：`isaac_sim/arena_ws/src/arena/arena-rosnav/installers/1_gazebo.sh`
- Isaac installer：`isaac_sim/arena_ws/src/arena/arena-rosnav/installers/2_isaac.sh`
- Repository manifests：`isaac_sim/arena_ws/src/arena/arena-rosnav/.repos/`
- Simulation assets：`isaac_sim/arena_ws/src/arena/simulation-setup/`
- Arena Isaac candidate：`isaac_sim/arena_ws/src/arena/isaac/`
- Build history：`isaac_sim/arena_ws/log/build_2026-08-09_*`
- Isaac version：`isaac_sim/isaacsim-6.0.1/VERSION`
