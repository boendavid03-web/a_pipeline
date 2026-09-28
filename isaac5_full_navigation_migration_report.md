# Isaac5 Migration Status

审计日期：2026-09-11  
执行策略：严格按 Stage 1 → Stage 8 顺序；Stage 2-D 在 Stage 2-E 修复 typesupport、Stage 2-F 修复 callback 生命周期后通过；Stage 3–7 及 Stage 8 相关 benchmark gate 已完成。原生 Arena 集成未加载、未宣称通过。  
保护边界：未修改 Isaac6 主链，未删除旧 Isaac5 backend，未修改 Isaac 安装或系统 NVIDIA driver，未引入 RTX LiDAR。

## Stage 状态

| Stage | Status | Evidence |
|---|---|---|
| 1 Policy Adapter | **PASS** | `stage1_policy_adapter_run.log:104-105,180` |
| 2-A Dual LiDAR backend | **PASS** | 两路 2000 beams、时间戳差 0、约 10 Hz |
| 2-B Observation Adapter | **PASS** | 12 次 `(10,720)` → `(19202,)` |
| 2-C Human observation contract | **PASS** | 非零 `(2,80,80)` map、track 1 写入 |
| 2-D DRL-VO inference | **PASS** | 51 次 successful inference；51/6.033 s ≈ 8.45 Hz |
| 2-E ROS custom message compatibility | **PASS** | 独立 cp311 overlay；两类 custom message publisher smoke PASS |
| 2-F Inference callback lifecycle fix | **PASS** | track callback、scan callback、model forward、action、cmd_vel、teardown 均通过 |
| 3 SemanticCNN | **PASS** | 17/17 inference、goal tolerance、cmd_vel、teardown 通过 |
| 4 Episode Environment | **PASS** | 3/3 goals、reset、obstacle compose、collision proxy、teardown 通过 |
| 5 Pedestrian | **PASS** | minimal kinematic human、relative observation、DynamicCapsule、teardown 通过 |
| 6 Social Navigation | **PASS** | 最小 Social Force、personal space 和避让 runtime gate 通过 |
| 7 DR-SPAAM / Human perception | **PASS** | 60 scan samples、DR-SPAAM detection/tracking、DRL-VO 51/51、cmd_vel、teardown 通过 |
| 8 Arena / Benchmark | **PASS（相关 benchmark）** | 3/3 episode、reset、collision proxy、teardown 通过；原生 Arena 未接入 |

## Stage 1：Navigation Policy Adapter

### 修改内容

新增：

- [`policy_adapter.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/policy_adapter.py)

最小修改：

- [`navigation_interface.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/navigation_interface.py)：为 `NavigationObservation` 增加 `goal` 字段。
- [`basic_navigation.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/basic_navigation.py)：manual-goal phase 改由 `PolicyAdapter` 调用，不修改任何模型内部。

Adapter 合同：

```text
Isaac5 observation:
    scan, odom, tf, goal
        ↓
    PolicyAdapter
        ↓
    finite (vx, vy, omega)
        ↓
    Mecanum controller
```

### 验证

纯接口验证：

```text
PYTHONPATH=isaac_sim/backends/isaac5/runtime python3 <policy adapter contract smoke>
STAGE1_POLICY_ADAPTER_CONTRACT=PASS
```

Isaac5 runtime 验证：

```text
./isaac_sim/backends/isaac5/launch/run_basic_navigation.sh \
  --phase navigation --duration 4 --fast --goal 0.5 0.0 0.0
```

证据：[`stage1_policy_adapter_run.log`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/stage1_policy_adapter_run.log:104)

结果：

- custom `minimal_core_physx_no_rtx.kit` 启动；
- 240 physics steps；
- manual policy 通过 adapter 生成 cmd_vel；
- 机器人 `Δx=+0.42060 m`；
- `goal_reached=true`；
- 360-beam scene-query scan、odom、TF 被装入 observation；
- teardown PASS，见日志第 180 行。

Stage 1 判定：**PASS**。

## Stage 2-A：双 LiDAR backend

新增并验证：

- [`lidar_sensor.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/lidar_sensor.py)：两个独立 PhysX scene-query raycaster，默认各 2000 beams、10 Hz。
- [`stage2a_dual_lidar_probe.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/stage2a_dual_lidar_probe.py)：使用 `minimal_core_physx_no_rtx.kit`，不加载 Arena、IRA、RTX LiDAR 或 DRL-VO。
- [`ros_bridge.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/ros_bridge.py)：增加可选 `/scan_01`、`/scan_02` publisher 和对应静态 TF；默认 `/scan` 行为保持不变。

证据：[`stage2a_dual_lidar_run.log`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/stage2a_dual_lidar_run.log:1)

结果：两路各接收 20 个 2000-beam 样本，frame_id 为 `base_scan_01`/`base_scan_02`，最大成对时间戳差 `0 ns`，约 `9.75 Hz`，ROS/world/Kit teardown 均 PASS。

Stage 2-A 判定：**PASS**。

## Stage 2-B：Observation Adapter

新增：

- [`drlvo_observation_adapter.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/drlvo_observation_adapter.py)
- [`stage2b_observation_probe.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/stage2b_observation_probe.py)

adapter 不导入 torch，不改变网络或 checkpoint，并直接复用现有 legacy `observation_adapter.py` 的 front-scan 投影和 history compression 合同。它实现双 2000-beam scan → `(720,)` front scan、10 帧 `(10,720)` history、local `base_link`/final `odom` goal timestamp contract、19202 维 observation，以及 legacy 2-vector normalized action → `(vx, vy, omega)`（原模型没有横向 action，`vy=0`）。

证据：[`stage2b_observation_run.log`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/stage2b_observation_run.log:1)

12 次实际 custom-experience 采样全部通过，时间戳严格递增，teardown PASS。

Stage 2-B 判定：**PASS**。

## Stage 2-C：Pedestrian observation contract

新增：

- [`minimal_human_backend.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/minimal_human_backend.py)
- [`stage2c_human_map_probe.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/stage2c_human_map_probe.py)

该 backend 是单纯的有限状态运动学 source：输出 human position、velocity、goal、track state 和 confidence，并在 custom experience 中用一个 `DynamicCapsule` 表示。它不使用 IRA、BehaviorAgent、Social Force、DR-SPAAM 或 Arena。

证据：[`stage2c_human_map_run.log`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/stage2c_human_map_run.log:1)

8 个采样通过：track 1 被 legacy converter 写入，pedestrian map 为非零 `(2,80,80)`，DRL-VO observation 为 `(19202,)`，teardown PASS。这里验证的是 map/input contract，不是行人感知精度或社会导航效果。

Stage 2-C 判定：**PASS**。

## Stage 2-D：真实 DRL-VO inference

本阶段使用现有 `drl_vo_fixed_dual_inference_node.py` 类和现有 `base_bc_best.pt`，未修改模型结构、checkpoint 或 node 源码。初次运行使用 Isaac5 embedded Python 3.11；当时没有安装依赖或重编译原 workspace，因而暴露出 cp310 custom typesupport 缺口。后续修复限定在独立 Stage 2-E overlay。

### 现有 DRL-VO 合同

源文件：

- [`drlvo_model.py`](/home/user/navigation_project/a_pipeline/sim_to_real/robot/comparison_models/runtime_code/methods/experiments/drl_vo_ros2_offline/drlvo_model.py:152)
- [`observation_adapter.py`](/home/user/navigation_project/a_pipeline/sim_to_real/robot/comparison_models/runtime_code/methods/experiments/drl_vo_ros2_offline/observation_adapter.py:11)
- [`drl_vo_fixed_dual_inference_node.py`](/home/user/navigation_project/a_pipeline/sim_to_real/robot/comparison_models/ros2_ws/src/semantic_nav_runtime/scripts/drl_vo_fixed_dual_inference_node.py:986)

确定的输入差异：

| 项目 | 当前 Isaac5 backend | 现有 DRL-VO |
|---|---|---|
| LaserScan topic | 单路 `/scan` | 同步 `/scan_01` 与 `/scan_02` |
| beam 数 | 360 | legacy adapter 要求每帧 720；predicted pedestrian mode 要求两路各 2000 |
| 历史 | 当前 runtime 可产生单路 360 scan | 明确要求 10 帧 `(10,720)` scan history |
| observation | `scan + odom + tf + goal` envelope | 19202 维：`2×80×80` pedestrian map、scan image、2 维 goal |
| goal | 内部 `(x,y,yaw)` | ROS `PointStamped` local subgoal/final goal，带 timestamp/frame contract |
| pedestrian input | 当前阶段明确不接行人 | model observation 固定包含 pedestrian map；node 还包含 pedestrian source 分支 |
| output | `(vx,vy,omega)` | model 先输出 normalized 2-vector，再转换为 physical action |

证据：

- DRL-VO observation size 为 `19202`、pedestrian map 为 `(2,80,80)`、scan history 为 `10`：`observation_adapter.py:11-17`。
- scan history 函数拒绝非 `(10,720)` 输入：`observation_adapter.py:476-484`。
- model forward 将前 `12800` 元素解释为 pedestrian map、接着 `6400` 元素解释为 scan、最后 2 元素解释为 goal：`drlvo_model.py:152-164`。
- node 对 `/scan_01` 和 `/scan_02` 建立 ApproximateTimeSynchronizer：`drl_vo_fixed_dual_inference_node.py:986-1003`。
- predicted pedestrian mode 明确拒绝非 `2000+2000` fixed-slot LiDAR：`drl_vo_fixed_dual_inference_node.py:1915-1923`。

验证命令：

```text
/home/user/isaacsim/5.1.0/python.sh \
  isaac_sim/backends/isaac5/runtime/stage2d_drlvo_inference_probe.py
```

初次运行证据：[`stage2d_drlvo_run.log`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/stage2d_drlvo_run.log:1)。初次失败为：

```text
UnsupportedTypeSupport(
  "Could not import 'rosidl_typesupport_c' for package 'navigation_evaluation_msgs'"
)
```

该初次失败由 Stage 2-E 的独立 cp311 overlay 处理；本阶段没有修改模型、checkpoint 或 inference node。

## Stage 2-E：ROS custom message compatibility

### 原 workspace 审计

原 workspace：

```text
sim_to_real/robot/comparison_models/ros2_ws
```

其 `build/` 和 `install/` 中的 custom Python extension 为 `cp310`，并链接系统 Python 3.10，例如：

```text
navigation_evaluation_msgs_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so
semantic_nav_runtime_s__rosidl_typesupport_c.cpython-310-x86_64-linux-gnu.so
```

这与 Isaac5 embedded Python `3.11.13` 不兼容；消息定义本身未发现需要修改的地方。

### 独立 cp311 overlay

使用原消息源码的 symlink，构建输出完全放在：

[`ros2_ws_cp311_overlay`](/home/user/navigation_project/a_pipeline/sim_to_real/robot/comparison_models/ros2_ws_cp311_overlay)

没有写入原 `ros2_ws/build`、`ros2_ws/install`、`ros2_ws/log`，没有修改 `/home/user/isaacsim/5.1.0` 或 `/opt/ros/humble`。构建明确指定 Isaac5 Python 3.11 executable、headers、library；CMake cache 记录 `PYTHON_SOABI=cpython-311-x86_64-linux-gnu`。

构建命令：

```text
source /opt/ros/humble/setup.bash
colcon --log-base ros2_ws_cp311_overlay/log build \
  --base-paths ros2_ws_cp311_overlay/src \
  --build-base ros2_ws_cp311_overlay/build \
  --install-base ros2_ws_cp311_overlay/install \
  --packages-select navigation_evaluation_msgs semantic_nav_runtime \
  --symlink-install \
  --cmake-args \
    -DPython3_EXECUTABLE=/home/user/isaacsim/5.1.0/kit/python/bin/python3 \
    -DPython3_INCLUDE_DIR=/home/user/isaacsim/5.1.0/kit/python/include/python3.11 \
    -DPython3_LIBRARY=/home/user/isaacsim/5.1.0/kit/python/lib/libpython3.11.so
```

构建结果：两个 package 均成功，生成并安装了 `cpython-311-x86_64-linux-gnu` custom typesupport extension；ROS Humble 消息定义未改变。

### Isaac5 Python 3.11 publisher smoke

在 custom `minimal_core_physx_no_rtx.kit` experience 中，使用现有 `RosControlBridge` 创建 node 后，以下检查通过：

```text
PYTHON_VERSION=3.11.13
IMPORT_NAVIGATION_EVALUATION_MSGS=PASS
IMPORT_SEMANTIC_NAV_RUNTIME=PASS
PUBLISHER_INFERENCE_METRICS=PASS
PUBLISHER_TRACKED_PEDESTRIAN_ARRAY=PASS
STAGE2E_PUBLISHER_SMOKE=PASS
```

本 smoke 未加载 robot、LiDAR、Arena、IRA 或 DRL-VO；它只验证 Isaac5 Python 3.11 下 custom message import 和 publisher type support。

Stage 2-E 判定：**PASS**。

### Stage 2-D overlay 重跑（Stage 2-F 修复前）

重跑使用 cp311 overlay、原 `drl_vo_fixed_dual_inference_node.py` 和原 `base_bc_best.pt`。仅对 probe 增加了 `ISAAC5_ROS_CP311_OVERLAY_INSTALL` 路径选择；inference node 源码、网络结构和 checkpoint 均未修改。

重跑已越过原 `UnsupportedTypeSupport`，并观察到：

- existing node 启动；
- 原 checkpoint 被读取，device 为 `cpu`；
- custom ROS publisher 创建成功；
- custom experience app ready，teardown 字段为 `PASS`。

随后在现有 node 的 `pedestrian_tracks_callback()` 中失败：

```text
NameError("name 'receive_ns' is not defined")
```

源代码在 [`drl_vo_fixed_dual_inference_node.py:1681-1718`](/home/user/navigation_project/a_pipeline/sim_to_real/robot/comparison_models/ros2_ws/src/semantic_nav_runtime/scripts/drl_vo_fixed_dual_inference_node.py:1681) 调用了 `self._observe_clock()`，但随后把未定义的 `receive_ns` 写入 `self.pedestrian_track_receive_ns`。因此本次没有形成有效的 DRL-VO inference metrics/action/cmd_vel/trajectory 证据。

该次重跑判定：**FAIL**。这个结果触发了 Stage 2-F；失败不是 cp311 custom typesupport，而是现有 inference node callback 的变量生命周期错误。

## Stage 2-F：Inference callback lifecycle fix

### 根因分析

`_observe_clock()` 在 [`drl_vo_fixed_dual_inference_node.py:1129-1138`](/home/user/navigation_project/a_pipeline/sim_to_real/robot/comparison_models/ros2_ws/src/semantic_nav_runtime/scripts/drl_vo_fixed_dual_inference_node.py:1129) 中返回当前 ROS simulation clock 的 `now_ns`，并执行 clock rollback 检查。

`odom_callback()` 已按原设计使用：

```python
receive_ns = self._observe_clock()
```

`pedestrian_tracks_callback()` 原先只调用 `_observe_clock()`，随后却使用未定义的 `receive_ns`。最小修复为恢复同一 receive-time 来源：

```python
receive_ns = self._observe_clock()
```

message header stamp 仍只用于 `pedestrian_track_stamp_ns`；没有改变 track timestamp、history、observation 或 inference 逻辑。

### 修改范围

- 修改 [`drl_vo_fixed_dual_inference_node.py`](/home/user/navigation_project/a_pipeline/sim_to_real/robot/comparison_models/ros2_ws/src/semantic_nav_runtime/scripts/drl_vo_fixed_dual_inference_node.py:1684)：仅补充 `receive_ns` 局部变量赋值。
- 修改 [`stage2d_drlvo_inference_probe.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/stage2d_drlvo_inference_probe.py:279)：将 `numpy.bool_` 转为 Python `bool`，修复 probe 结果 JSON 序列化；不影响 node 或模型。

未修改网络结构、checkpoint、observation 输入格式、双 LiDAR、模型 forward 或 action conversion。

### Stage 2-D 验证结果

验证使用原 node、原 checkpoint、Isaac5 custom Core/PhysX experience 和 Stage 2-E cp311 overlay：

```text
/home/user/isaacsim/5.1.0/python.sh -u \
  isaac_sim/backends/isaac5/runtime/stage2d_drlvo_inference_probe.py
```

关键运行结果：

```json
{
  "status": "PASS",
  "sim_time_s": 6.033333647996187,
  "wall_time_s": 2.6931181650143117,
  "inference_count": 51,
  "successful_inference_count": 51,
  "ros_received_cmd_count": 60,
  "delta_position": [0.19183343648910522, -0.001609591068699956, -0.040437523275613785],
  "latency_ms": {
    "min": 12.56000804901123,
    "max": 20.382938385009766,
    "mean": 13.2539
  },
  "checks": {
    "checkpoint_loaded": true,
    "inference_metrics_received": true,
    "successful_inferences": true,
    "cmd_vel_received": true,
    "finite_commands": true,
    "robot_pose_finite": true,
    "physics_duration": true
  },
  "teardown": {"status": "PASS"}
}
```

推理频率按 simulation time 计算为 `51 / 6.0333 = 8.45 Hz`；60 个 `/cmd_vel` 接收对应约 `9.94 Hz`。model action sample 包括：

```text
[0.23803555965423584, -0.018435867503285408]
[0.23803552985191345, -0.018435845151543617]
[0.23455464839935303, -0.019328491762280464]
```

运行日志还确认 external pedestrian track callback 正常收到并写入 `tracks=1, written=1`。51 个 successful inference 证明 synchronized dual-scan callback、observation assembly、model forward、action output 和 `/cmd_vel` publish 均已执行。

Stage 2-F 判定：**PASS**。

## Stage 3：SemanticCNN 接入

### 接口分析

现有 SemanticCNN node 使用固定训练合同：

- 双路同步 2000-beam LaserScan；
- 10 帧 scan/semantic history；
- 80×80 scan map 和 semantic map；
- `base_link` local subgoal；
- checkpoint 输出二维 `(linear.x, angular.z)` action。

本阶段复用现有 node、model code 和 checkpoint，没有修改网络结构、权重或输入预处理。Isaac5 侧使用 PhysX scene-query 双雷达、`/odom`、静态 TF 和现有静态 semantic label map；没有启动 S3-Net、行人、Arena 或 RTX LiDAR。

### 验证

模型纯 forward smoke：

```text
PYTHON_VERSION=3.11.13
FORWARD_SHAPE=(1, 2)
FORWARD_FINITE=True
SEMANTIC_CNN_MODEL_SMOKE=PASS
```

Isaac5 runtime 验证：

```text
/home/user/isaacsim/5.1.0/python.sh -u \
  isaac_sim/backends/isaac5/runtime/stage3_semantic_cnn_probe.py
```

证据：[`stage3_semantic_cnn_run.log`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/stage3_semantic_cnn_run.log:1)

结果：

- custom `minimal_core_physx_no_rtx.kit` 启动；
- static-map SemanticCNN node 和原 checkpoint 加载成功；
- 17/17 inference 成功；
- 41 次 actuation decision 和 `/cmd_vel` 接收；
- 机器人位移 `Δx=+0.20942 m`；
- 最终目标距离 `0.29058 m`，满足 node 的 `0.35 m` goal tolerance；
- 平均 inference latency `1174.6 ms`，约 `2.82 Hz`；
- 所有 command 有限，physics duration 和 teardown PASS。

Stage 3 判定：**PASS（功能闭环通过，CPU 推理频率受限）**。

## Stage 4：Episode Environment

### 实现

新增 [`episode_environment.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/episode_environment.py)，提供：

- episode reset；
- 多目标队列；
- goal tolerance、timeout、success/failure 状态；
- 机器人 footprint 与障碍物 AABB 的几何碰撞检查。

Isaac5 custom runtime 使用 local floor、四周边界和内部 box obstacle。碰撞检查是 episode termination 的几何 proxy，不等同于物理接触或连续碰撞验收。

### 验证

```text
PYTHONPATH=isaac_sim/backends/isaac5/runtime python3 \
  <episode environment contract smoke>
STAGE4_EPISODE_CONTRACT=PASS
```

Isaac5 runtime 验证：

```text
/home/user/isaacsim/5.1.0/python.sh -u \
  isaac_sim/backends/isaac5/runtime/stage4_episode_probe.py
```

证据：[`stage4_episode_run.log`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/stage4_episode_run.log:1)

结果：

- 3 个目标：`(0.5,0.0)`、`(-0.5,0.0)`、`(0.0,0.5)`；
- 3/3 episode `SUCCESS`；
- 每个 episode 约 `1.933 s`，最终 goal distance 约 `0.0794 m`；
- 3/3 reset position error 为 `0`；
- 5 个障碍物 prim 成功 compose；
- 无几何碰撞记录；
- world stop 和 app close PASS。

Stage 4 判定：**PASS**。

## Stage 5：Pedestrian backend

### 实现与边界

复用并扩展 [`minimal_human_backend.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/minimal_human_backend.py)，增加 base-link 相对位置/速度观察。该 backend 是有限状态运动学 source，并在 Isaac5 stage 中用 `DynamicCapsule` 表示一个 human；不使用 IRA、BehaviorAgent、Social Force、DR-SPAAM 或 Arena。

### 验证

```text
PYTHONPATH=isaac_sim/backends/isaac5/runtime python3 \
  <minimal human backend contract smoke>
STAGE5_PED_BACKEND_CONTRACT=PASS
```

Isaac5 runtime 验证：

```text
/home/user/isaacsim/5.1.0/python.sh -u \
  isaac_sim/backends/isaac5/runtime/stage5_pedestrian_probe.py
```

证据：[`stage5_pedestrian_run.log`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/stage5_pedestrian_run.log:1)

结果：10 个采样中 human position、velocity、goal 和相对观察均有限；human prim compose 成功；360-beam scene-query scan 持续存在；world/app teardown PASS。

这里的“观察”是 ground-truth relative observation，scan 没有被宣称为已经完成 pedestrian detection/tracking。

Stage 5 判定：**PASS（最小 backend）**。

## Stage 6：Social Navigation

### 实现

新增 [`social_navigation.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/social_navigation.py)，包含：

- human goal-seeking motion；
- robot/human personal-space repulsion；
- robot goal controller 的短程避让项。

这是一个最小 deterministic pairwise Social Force 风格适配器，不是 IRA、BehaviorAgent 或完整社会导航 benchmark。

### 验证

```text
PYTHONPATH=isaac_sim/backends/isaac5/runtime python3 \
  <social force contract smoke>
STAGE6_SOCIAL_FORCE_CONTRACT=PASS
```

Isaac5 runtime 验证：

```text
/home/user/isaacsim/5.1.0/python.sh -u \
  isaac_sim/backends/isaac5/runtime/stage6_social_navigation_probe.py
```

证据：[`stage6_social_navigation_run.log`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/stage6_social_navigation_run.log:1)

结果：

- human 位移约 `1.803 m`；
- robot 最大横向偏移约 `0.454 m`，说明发生避让；
- 最小人机中心距离约 `0.652 m`；
- 几何接触阈值为 `0.36+0.24=0.60 m`，未发生直接碰撞；
- human/robot motion、physics stepping 和 teardown PASS；
- robot 最终距 goal 约 `0.923 m`，未把该实验宣称为 goal-complete navigation。

Stage 6 判定：**PASS（交互/避让 gate；非完整社会导航验收）**。

## Stage 7：DR-SPAAM / Human perception

### 实现与验证边界

本阶段复用了现有 DR-SPAAM checkpoint、third-party Detector 和 `PointCVKalmanTracker`，没有修改 detector、tracker、DRL-VO 网络或 checkpoint。runtime 链路为：

```text
Isaac5 PhysX scene-query 2000-beam scan
    → existing DR-SPAAM Detector
    → existing PointCVKalmanTracker
    → TrackedPedestrianArray
    → existing DRL-VO node
    → /cmd_vel
```

没有启动 S3-Net、IRA、Arena 或 RTX LiDAR。

### 结果

```text
/home/user/isaacsim/5.1.0/python.sh -u \
  isaac_sim/backends/isaac5/runtime/stage7_drspaam_probe.py
```

证据：[`stage7_drspaam_run.log`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/stage7_drspaam_run.log:1)

结果：

- 60 个 scene-query scan samples；
- DR-SPAAM 产生 detection，20 个 sample 与模拟 human 位置匹配；
- tracker 产生 track，37 次与模拟 human 位置匹配；
- 原 DRL-VO node 收到 external tracks，51/51 inference 成功；
- `/cmd_vel` 接收 60 次；
- robot pose/commands finite，teardown PASS。

Stage 7 判定：**PASS（动态感知链路 gate）**。这不是大规模人群检测精度、长跑稳定性或社会导航 acceptance。

## Stage 8：Arena / Benchmark

### 实现边界

本阶段没有把现有 Arena source、历史 workspace 或 `ros2isaacsim` 强行接入 custom Core/PhysX experience。新增的是独立的 Isaac5 相关 benchmark harness：复用 Stage 4 的 episode/reset/goal/collision-proxy 合同，在不加载 ROS、Arena、行人或 RTX LiDAR 的条件下，对多个目标场景执行可重复 episode。

因此本阶段的 PASS 表示 **related benchmark harness PASS**，不表示原生 Arena task generator、Arena service backend、标准 benchmark message 或训练环境已经完成。

### 验证

纯 Python 合同 smoke：

```text
STAGE8_BENCHMARK_CONTRACT=PASS
```

Isaac5 runtime 验证：

```text
/home/user/isaacsim/5.1.0/python.sh -u \
  isaac_sim/backends/isaac5/runtime/stage8_benchmark_probe.py
```

证据：[`stage8_benchmark_run.log`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/stage8_benchmark_run.log:1)

结果：

- `goal_forward`、`goal_reverse`、`goal_lateral` 共 `3` 个场景；
- `3/3` episode `SUCCESS`，success rate `1.0`；
- `3/3` reset position error 为 `0`；
- `collision_count=0`、`timeout_count=0`；
- mean final goal distance `0.07938 m`；
- world stop 和 app close PASS；
- probe 明确记录 `arena_native_integration=false`。

Stage 8 判定：**PASS（Isaac5 相关 benchmark gate）**。原生 Arena 集成仍是未完成项。

## 当前可运行系统

当前已确认可以运行：

```text
Isaac5 custom Core/PhysX experience
    └── Mecanum730 articulation
        ├── wheel velocity + planar motion control
        ├── ROS2 bundled Humble rclpy
        │   ├── /cmd_vel
        │   ├── /clock
        │   ├── /odom
        │   ├── /tf
        │   └── /tf_static
        ├── PhysX scene-query 2D LaserScan
        │   ├── /scan, 360 beams, approximately 10 Hz
        │   └── /scan_01 + /scan_02, 2000 beams each, approximately 10 Hz
        ├── Stage 1 manual policy adapter
        │   └── goal → cmd_vel → robot motion
        ├── Stage 2-B/C DRL-VO input preparation
        │   ├── dual scan → 10×720 history
        │   ├── minimal human track → (2,80,80) map
        │   └── observation → 19202 dimensions
        └── Stage 2-D/F real DRL-VO inference
            ├── original checkpoint, CPU inference
            ├── 51/51 successful inference metrics
            ├── 60 `/cmd_vel` receives
            └── robot motion and teardown PASS
        ├── Stage 3 SemanticCNN
        │   └── static semantic map → original checkpoint → /cmd_vel
        ├── Stage 4 episode environment
        │   └── reset → multi-goal run → success/failure → collision proxy
        ├── Stage 5/6 pedestrian and social interaction
        │   └── minimal human → relative observation → Social Force-style avoidance
        ├── Stage 7 dynamic perception
        │   └── DR-SPAAM → tracker → TrackedPedestrianArray → DRL-VO → /cmd_vel
        └── Stage 8 related benchmark harness
            └── 3 scenarios → episode metrics → teardown PASS
```

此前 Gate1、Gate2、Gate3、Gate4、Gate5 和 manual-goal smoke 的证据仍有效；本轮新增并验证了 Stage 3–8。当前可运行闭环覆盖 robot、双非 RTX LiDAR、ROS2、SemanticCNN、DRL-VO、最小 pedestrian/social adapter、DR-SPAAM/tracker 和相关 benchmark harness。

当前仍有明确限制：SemanticCNN CPU 平均推理约 `1174.6 ms`（约 `2.82 Hz`）；Stage 6 是最小交互/避让 gate，实验中的机器人未到达最终目标；Stage 4/8 的 collision 是几何 proxy，不是物理接触真值；尚未完成大规模人群、长时间稳定性、严格社会导航 acceptance 或原生 Arena task/benchmark integration。

## 修改文件

本次修改：

- [`policy_adapter.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/policy_adapter.py)
- [`navigation_interface.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/navigation_interface.py)
- [`basic_navigation.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/basic_navigation.py)
- [`lidar_sensor.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/lidar_sensor.py)
- [`ros_bridge.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/ros_bridge.py)
- [`drlvo_observation_adapter.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/drlvo_observation_adapter.py)
- [`minimal_human_backend.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/minimal_human_backend.py)
- [`stage2d_drlvo_inference_probe.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/stage2d_drlvo_inference_probe.py)：增加独立 cp311 overlay install 路径选择
- [`stage3_semantic_cnn_probe.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/stage3_semantic_cnn_probe.py)：SemanticCNN Isaac5 custom-experience probe
- [`stage3_semantic_cnn_run.log`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/stage3_semantic_cnn_run.log)：Stage 3 runtime evidence
- [`episode_environment.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/episode_environment.py)：Stage 4 episode/reset/collision contract
- [`stage4_episode_probe.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/stage4_episode_probe.py)：Stage 4 runtime probe
- [`stage4_episode_run.log`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/stage4_episode_run.log)：Stage 4 runtime evidence
- [`stage5_pedestrian_probe.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/stage5_pedestrian_probe.py)：Stage 5 pedestrian runtime probe
- [`stage5_pedestrian_run.log`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/stage5_pedestrian_run.log)：Stage 5 runtime evidence
- [`social_navigation.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/social_navigation.py)：Stage 6 minimal Social Force adapter
- [`stage6_social_navigation_probe.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/stage6_social_navigation_probe.py)：Stage 6 runtime probe
- [`stage6_social_navigation_run.log`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/stage6_social_navigation_run.log)：Stage 6 runtime evidence
- [`stage7_drspaam_probe.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/stage7_drspaam_probe.py)：Stage 7 DR-SPAAM/tracker/DRL-VO probe
- [`stage7_drspaam_run.log`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/stage7_drspaam_run.log)：Stage 7 runtime evidence
- [`benchmark_runner.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/benchmark_runner.py)：Stage 8 相关 benchmark metrics contract
- [`stage8_benchmark_probe.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/stage8_benchmark_probe.py)：Stage 8 custom-experience benchmark probe
- [`stage8_benchmark_run.log`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/stage8_benchmark_run.log)：Stage 8 runtime evidence
- [`drl_vo_fixed_dual_inference_node.py`](/home/user/navigation_project/a_pipeline/sim_to_real/robot/comparison_models/ros2_ws/src/semantic_nav_runtime/scripts/drl_vo_fixed_dual_inference_node.py)：Stage 2-F 仅补充 `receive_ns = self._observe_clock()`
- Stage 2 probe files under `isaac_sim/backends/isaac5/runtime/stage2*.py`
- 独立 `ros2_ws_cp311_overlay` build/install/log 产物

未修改：

- Isaac6 主链；
- 现有 Isaac5 `run_navigation.py`；
- DRL-VO、SemanticCNN 模型和 checkpoint；
- inference node 的网络结构、observation、model forward 和 action conversion；
- Isaac Sim 安装、系统 driver；
- Arena、行人、Social Force 或 DR-SPAAM runtime。

## 当前阶段边界

Stage 1、Stage 2-A/B/C/D/E/F、Stage 3、Stage 4、Stage 5、Stage 6、Stage 7 和 Stage 8 相关 benchmark 均已通过各自 gate。

Stage 8 的范围必须保持清晰：本次没有启动或接入原生 Arena；因此 Arena task generator、Arena service backend、标准 benchmark lifecycle、训练和正式 Arena metrics 仍未通过。

## 下一步建议

1. 保持当前 cp311 overlay、custom Core/PhysX experience 和已验证的 model/checkpoint contract；不要复用原 workspace 的 cp310 Python extension。
2. 若要继续做研究级验收，应先分别补做 SemanticCNN CPU 性能、长跑/多场景 episode、物理接触真值和多行人压力测试。
3. 若目标明确要求 Arena，应另立一个原生 Arena integration 阶段，先审计 active package/service/episode contract，再决定是否建立隔离 backend；不能把本报告的 related benchmark PASS 解释为 Arena 已接通。

本次没有修改 Isaac6、删除旧 Isaac5 backend、修改 Isaac Sim 安装或系统 driver，也没有通过复制 `.pyc`、注入 cp310 `rclpy` 或填 zero pedestrian map 绕过接口合同。
