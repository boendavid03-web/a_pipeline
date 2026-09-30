# Arena + Isaac Sim 5.1 工厂 10 人实跑记录

日期：2026-09-29。运行入口：`/home/user/arena_isaac5_host_runtime/run_gui.bash`。原有 Gazebo/HuNav 进程未处理。下方第一轮验证使用的是单个 workcell USD；本日续测已加载完整的 Arena `factory.world`，结果见后文。

## 本轮修复

- 上次仅把工厂 USD 加载代码写入活动 `arena-isaac` 源码；Isaac child 实际使用的 CPython 3.11 安装副本没有该代码。本轮把最小加载逻辑同步到 `/home/user/arena_isaac5_py311_factory/build_ws/humble/isaac_sim_ros_ws/{src,build,install}/ros2isaacsim` 的对应 `run_isaacsim.py`，并在 stage 中核验可视网格与碰撞 prim。
- 新建 `isaac_sim/backends/isaac5/config/arena_factory_10_people.json`：复用工厂 `scenario1.json` 的机器人任务，将 10 人起点和短巡逻路线放在工厂地图已知的可通行区域。原先直接使用 `map_highly_social/default.json` 的机器人目标与工厂不匹配。
- 为同一 CPython 3.11 overlay 的 `isaac_utils/graphs/odom.py` 增加 `ROS2PublishOdometry`，使 `/task_generator_node/jackal/odom` 使用现有 odom→base TF 图的同一位姿和 `IsaacComputeOdometry` 的速度。

## 运行证据

| 门禁 | 实际结果 |
|---|---|
| 单 workcell USD 进入 Isaac stage | `visual_valid=True collision_valid=True mesh_count=1 collision_count=42`。该 USD 只覆盖 Arena 原工厂 world 的一个模型。日志：`/home/user/arena_isaac5_py311_factory/test_20260929_factory_aligned_10p_final/console.log`。 |
| 固定 10 人与路线 | 10/10 `ARENA_PEDESTRIAN_READY`，连续 `ARENA_PEDESTRIAN_WAYPOINT_BATCH`，无 `WAYPOINT_BATCH_FAILED`。 |
| 人物运动 | 7 次服务位姿采样横跨 29.0667 秒仿真时间；10 人各累计移动 5.6706–9.0209 m。采样中的最小两人中心距 1.4043 m。原始世界坐标：`/home/user/arena_isaac5_py311_factory/test_20260929_factory_aligned_10p_final/world_positions.json`。世界坐标由父 prim 位姿加 SkeletonRoot 局部位姿得到。 |
| Jackal 里程计 | 修复后 `/task_generator_node/jackal/odom` 有 1 个发布者且收到 Odometry 消息。最终复跑同时采样的 `/odom` 与 `jackal/odom → jackal/base_link` TF 均为 `(-1.547, 4.705)`。日志：`/home/user/arena_isaac5_py311_factory/test_20260929_factory_odom_tf_10p/console.log`。 |

以上通过的是工厂几何加载、10 人创建/持续运动及里程计发布。1.4043 m 是 **7 个离散采样时刻**中的最小距离，不代表连续时间的碰撞验收。工厂 USD/人物资源目前是本机文件，重现依赖这些本地资产。

## 尚未通过的机器人导航门禁

同一最终复跑中，Nav2 曾报 `GridBased: failed to create plan`，之后又报 `Timed out while waiting for action server to acknowledge goal request for follow_path` 和 `Goal failed`。RTX LiDAR 仍有 `Render product not attached to RTX Lidar` 警告。**不能把本轮结果写成 Jackal 已到达目标或完整社交导航 benchmark PASS。**

## 重现命令

先选择一个未占用的 `ROS_DOMAIN_ID`，并确认没有其他本机 Isaac GUI 实例。随后执行：

```bash
ROS_DOMAIN_ID=225 \
ARENA_WORLD=factory \
ARENA_WORLD_USD=/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/arena_worlds/factory/factory_full_scene.usda \
ARENA_TM_OBSTACLES=scenario \
ARENA_TM_ROBOTS=scenario \
ARENA_SCENARIO_FILE=/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/config/arena_factory_10_people.json \
ARENA_PEDESTRIAN_WAYPOINT_PERIOD=12.0 \
/home/user/arena_isaac5_host_runtime/run_gui.bash
```

当前 wrapper 在 `ARENA_WORLD=factory` 时默认选用完整的 `factory_full_scene.usda`，所以可以省略上面的 `ARENA_WORLD_USD` 行。

## Arena 原始 factory.world 的完整场景续测

本次直接读取 Arena 原有的 `isaac_sim/arena_ws/src/arena/simulation-setup/worlds/factory/worlds/factory.world`，扩展已有的 `isaac_sim/backends/isaac5/runtime/build_arena_factory_scene.py`，按模型、link、visual/collision 的 SDF 位姿与 mesh scale 生成 `isaac_sim/backends/isaac5/generated/arena_worlds/factory/factory_full_scene.usda`。复用本机 `gazebo_models` 中的网格和已有 workcell 贴图 USD。`disk_part/meshes/disk.dae` 被 Kit 资产转换器拒绝，因此用脚本中的 COLLADA 几何解析后备路径生成其网格；这个零件没有随源文件提供单独贴图。

| 验证项 | 实测 |
|---|---|
| 原始 Arena 模型闭合 | 原 SDF 有 66 个顶层模型、175 个 visual、209 个带几何的 collision；生成日志给出 `ARENA_FACTORY_WORLD_PASS models=66 visuals=175 collisions=209`。原先统计的 211 个 `<collision>` 标签包括 contact sensor 内的 2 个碰撞体名称引用。 |
| USD 静态组合 | Isaac USD stage 可打开，得到 66 个模型、175 个 visual、209 个 collision、124 个 mesh，引用了 15 个 USD layer。`/home/user/arena_isaac5_py311_factory/factory_full_check.log`。 |
| Arena GUI 运行 | 现有 Arena launch 以 `headless:=0` 打开 `Isaac Sim Python 5.1.0` 窗口；运行时日志输出 `ARENA_ENVIRONMENT_STAGE models=66 visuals=175 collisions=209`。`/home/user/arena_isaac5_py311_factory/test_20260929_factory_full_world/console.log`。 |
| 10 人创建与运动 | 10/10 `ARENA_PEDESTRIAN_READY`；连续路线批次。通过 `/isaac/get_prim_attributes` 对 10 人采样 3 次，首末相隔 10.84 秒，10 人净位移为 0.7521–1.4800 m。结构化原始数据：`/home/user/arena_isaac5_py311_factory/test_20260929_factory_full_world/full_world_positions.json`。 |

这说明原 Arena 工厂的**静态模型几何和位姿**已经在 Isaac 5.1 的 Arena 入口呈现，且场景内 10 人实际移动。Gazebo 专有插件、传感器、物理材料与脚本材质没有逐项复刻；它们不属于静态场景等价性结论。此次 Nav2 仍出现 `Goal failed`，不能宣称机器人导航到达或完整 benchmark 通过。行人材质有 `poseEstimationMale` 贴图缺失警告；人物几何、服务和运动仍可用。

完整场景生成命令：

```bash
/home/user/isaacsim/5.1.0/python.sh isaac_sim/backends/isaac5/runtime/build_arena_factory_scene.py \
  --world isaac_sim/arena_ws/src/arena/simulation-setup/worlds/factory/worlds/factory.world \
  --models-root isaac_sim/arena_ws/src/arena/simulation-setup/gazebo_models \
  --workcell-usd isaac_sim/backends/isaac5/generated/arena_worlds/factory/workcell_mesh_textured.usd \
  --output isaac_sim/backends/isaac5/generated/arena_worlds/factory/factory_full_scene.usda
```

本次续测的 GUI 仍在 PGID `413886` 中运行，供本机直接查看；此 PGID 仅对应本次 Arena/Isaac 启动。原有 Gazebo/HuNav 进程未发送信号。此前单 workcell 验证的启动实例已结束。
