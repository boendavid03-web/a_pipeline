# Arena 5 / Isaac 5.1 启动 goal 与终点附近 abort 根因 — 2026-10-01

## 结论和边界

**STARTUP_CLASS = S3，带限定。** 原 default 的第一个 goal 在真实机器人位姿进入 `map→jackal/odom→jackal/base_link` TF 链之前发出；NavFn 立即用占位位姿 `(-10,-10)` 判断起点超出已经有效的 global costmap。这是场景 reset 与导航发送的生命周期竞态，属于 S1。short 的首个 goal 则在 TF 刚转到真实位姿时发出，前两次 NavFn 规划成功且 DWB 发出了速度，第三次重规划失败后 action abort；现有 WARN 日志不能再细分第三次规划失败的 NavFn 内部原因。RobotManager 还存在独立的 S2 缺陷：用 `status_list[-1]` 推断本轮结果，没有持有或匹配 UUID。本次两个 bag 的列表恰好按旧、新 goal 排列，没有观察到旧状态污染主 goal；代码仍不能保证未来 episode 的归属。

**NEARGOAL_CLASS = N1。** default 主 UUID 的最后一次平移 progress baseline 代理位姿到终点约走了 0.449 m，历时约 10.75 仿真秒；同时 yaw 净变化约 2.05 rad。正式 Nav2 1.1.20 `SimpleProgressChecker` 只测 XY，配置要求 `>0.5 m` / `10 s`；controller 先检查 progress，再算 DWB 命令，最后检查 goal。实测 DWB 有连续角速度命令、最终速度链有输出、odom 有旋转响应。终止时 XY 误差 0.183 m 满足 0.25 m，但 yaw 误差 0.566 rad 超过 0.25 rad。`Failed to make progress` 是当前原参数对终点转向的可解释 baseline 失败；本轮不调参。

**INFRASTRUCTURE_BASELINE_READY = NO；READY_FOR_ARENA_1_TO_5 = NO。** TF、local costmap、控制执行链通过既有 gate；但首 goal 竞态和无 UUID 归属的 Task Generator 状态机尚未修复和回归。这个结论不要求 default 导航成功。已有 default 终点 abort 可作为当前配置的有效失败观察，但不能把带启动竞态的自动化平台标成正式可重复 baseline。

## 证据和时间定义

- 正式 short：[bag](</home/user/arena_tf_throttle_validation_20261001/formal_short_215/raw_rosbag/metadata.yaml>)、[launch.log](</home/user/arena_tf_throttle_validation_20261001/formal_short_215/launch.log>)、[action 汇总](</home/user/arena_tf_throttle_validation_20261001/formal_short_215/formal_analysis.json>)、[逐事件重算](</home/user/arena_tf_throttle_validation_20261001/formal_short_215/root_cause_trace.json>)。
- 正式 default：[bag](</home/user/arena_tf_throttle_validation_20261001/formal_default_216/raw_rosbag/metadata.yaml>)、[launch.log](</home/user/arena_tf_throttle_validation_20261001/formal_default_216/launch.log>)、[action 汇总](</home/user/arena_tf_throttle_validation_20261001/formal_default_216/formal_analysis.json>)、[逐事件重算](</home/user/arena_tf_throttle_validation_20261001/formal_default_216/root_cause_trace.json>)。
- 两个 run 的 effective controller/local costmap YAML 和实际库 maps 在各自目录。实际 Nav2 deb 为 1.1.20，`liblayers.so` 来自 `/opt/ros/humble`，SHA-256 `64d581b8555bd9486fbd5c399791be00f14334b9d8abd5a94cff02958d194e8b`。[先前 TF 正式验证](ARENA_TF_THROTTLE_FIX_VALIDATION_20261001.md)列出完整环境 provenance；[冻结清单](ARENA_ISAAC_FROZEN_BASELINE_MANIFEST_20261001.yaml)继续标为未 ready。
- `wall` 是 bag 接收纳秒时间戳按 Unix 秒换算；`sim` 是该 wall 时刻前最近一条 `/clock`。`/odom` 的 sim 时间来自消息 header。两时钟仅在对应事件处对齐，不把运行时间比直接当作 ROS 时间。Task Generator 发出的 `goal_pose.header.stamp` 在这些 bag 里为 epoch wall 值，故下表采用 bag wall 和 `/clock`，不拿该 header 当 sim 时间。
- 新增的只读提取器是 [analyze_root_causes.py](scripts/validation/arena_tf_throttle/analyze_root_causes.py)。它不连接运行中的 ROS 域；报告没有新跑场景、改 Nav2 参数、改 TF 或修改原 bag。

## A. 首个 goal 的发送者、时序和失败

场景路径为 [TM_Scenario.reset](isaac_sim/arena_ws/src/arena/arena-rosnav/task_generator/task_generator/tasks/robots/scenario.py) → [RobotManager.reset](isaac_sim/arena_ws/src/arena/arena-rosnav/task_generator/task_generator/manager/robot_manager/robot_manager.py) 的 `move_robot_to_pos()` 和 `_publish_goal()`。RobotManager 首发 `.../jackal/goal_pose`，随后 3 秒 timer 重发；[Nav2 1.1.20 的 NavigateToPoseNavigator](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_bt_navigator/src/navigators/navigate_to_pose.cpp) 订阅 `goal_pose` 并由自己的 action client 为每条消息生成新的 NavigateToPose UUID。因此两个 UUID 不是两个人工命令，也不是同一个 action 的重试状态。Isaac 模型初始占位位姿见 short 启动日志的 `(-10,-10)`；实际场景 start 为 `(25.3,1.25,0.7)`。

| 阶段 | short：wall / sim s | default：wall / sim s | 现场证据 |
|---|---|---|---|
| T0 Arena bringup 进行中 | `1790870425.597` / `/clock` 尚未对齐 | `1790870570.606` / `/clock` 尚未对齐 | Task Generator 最早日志；不是完整 launch 起始时刻 |
| T1 初始占位机器人仍在 TF | `1790870455.687` / `17.883` | `1790870602.666` / `15.200` | `jackal/odom→jackal/base_link ≈(-10,-10)`，`map→jackal/odom=(0,0)` |
| T2 Nav2 lifecycle active | controller/local `1790870447.528` / `10.317`；planner/global `0447.987` / `10.783`；BT `0448.207` / `10.983` | controller/local `0595.991` / `9.833`；planner/global `0596.448` / `10.233`；BT `0596.660` / `10.400` | bag transition events |
| T3 global map/grid 有效 | `1790870447.936` / `10.733` | `1790870596.400` / `10.183` | 已发布 `626×481`、0.05 m、origin `(0,0)`，20,816 个占据 cell；start/goal 坐标在格内且其 cell 为 free |
| T4 首条 goal_pose 和首个 UUID EXECUTING | `1790870456.685903` / `18.333`；`3012f132…` at `0456.686221` | `1790870603.663699` / `15.567`；`ff4f1692…` at `0603.663895` | 两条均来自 RobotManager reset 首发 |
| T5 首个 goal 的规划/控制 | 首次 NavFn 成功 `0456.6899`，第一条 `cmd_vel_nav=(0.26,-0.053)` at `0456.8963`；第二次规划仍成功；第三次 `0457.6879` ABORTED | 首次 NavFn at `0603.6649` ABORTED，明确 `start position is off the global costmap`；没有 path/cmd | short 的第三次失败日志只写 `failed to generate a valid path` |
| T6 首个 NavigateToPose 终态 | `1790870457.987304` / `19.033` ABORTED | `1790870603.964581` / `15.800` ABORTED | status code 6，各自独立 UUID |
| T7 第二条 goal_pose 和主 UUID EXECUTING | `1790870459.686460` / `20.467`；`aa5bdfad…` | `1790870609.833452` / `17.133`；`f6952af8…` | short 恰为首发后约 3 s；default reset 阶段直到 `0609.831751` 才完成，timer 回调随后发出 |
| T8 第二条的结果 | `1790870469.486865` / 约 `29.93` SUCCEEDED | `1790870876.533757` / `149.183` ABORTED | short 最终 XY 误差 0.182 m、yaw 0.0023 rad；default 见下文 |

**default 的具体竞态：** 首 goal 前最近 `jackal/odom→jackal/base_link` 是 wall `1790870603.648034`、sim `15.566667`、位姿约 `(-10.010,-9.999)`；真实位姿的第一条 TF 在 wall `0603.674498`、sim `15.583334` 才到。首 goal wall `0603.663699`，NavFn 的 off-costmap 错误 wall `0603.664715`，都在真实 TF 之前。global map 的边界为 `x∈[0,31.3)`、`y∈[0,24.05)`；`(-10,-10)` 真在边界外，而真实 `(25.3,1.25)` 在边界内。map 已发布、planner/BT 已 active，故这里不是 global costmap 尚未初始化/resize，也不是地图尺寸不足。`map→odom` 已存在且为零变换；缺的是在发送前等待机器人 **当前真实 pose** 被 Nav2 使用。

**short 的限定：** 首 goal 前约 0.000084 wall s 才出现真实 TF；第一条 odom 真位姿甚至在首 goal 之后约 0.00035 wall s。前两次规划的 path 都从 `(25.3,1.25)` 到目标，首个 controller 命令和 local plan 已有，因此不能写成“首 goal 因 Nav2 尚未 active / TF 始终为占位位姿而直接失败”。第三次重规划时 NavFn 返回失败，继而 FollowPath 和 NavigateToPose abort；bag 中同一 global costmap 的 start/goal cell 仍是 free，已有 WARN 日志没有给出更深层的 NavFn 失败条件。它发生在 robot reset 刚结束后的首轮执行，第二 UUID 的同一路径成功；**短场景第三次规划失败的内部原因保持未证实**，需要在生命周期修复回归中检查是否重现。

### Task Generator 状态归属

[RobotManager 第 234–247 行](isaac_sim/arena_ws/src/arena/arena-rosnav/task_generator/task_generator/manager/robot_manager/robot_manager.py) reset 时清空 `_is_goal_reached` / `_goal_action_running`，并先移动后发 topic goal；第 257–310 行在未成功、也未看到“当前 action 正在执行”时每 3 秒重发，最多到 60 s。第 375–381 行直接取 `status_list` 最后一个条目；没有 active UUID，也没有验证 status 属于本轮 goal。`is_done` 只由最后条目 `SUCCEEDED` 决定。失败不会把 episode 标成成功，timer 才促成第二次发送。由于发布的是 topic，真正的 action UUID 在 Nav2 自身 client 创建，RobotManager 目前并不知道它。

两个 bag 的实际 status array 在第二 goal 执行时都是 `[旧 UUID:ABORTED, 新 UUID:EXECUTING]`；终态时新 UUID 为最后项。本次**没有观察到**旧 UUID 污染主 goal 或第二次误重发，但实现依赖数组顺序，跨 episode 或其他客户端目标可错误地置 `_is_goal_reached` / `_goal_action_running`。因此 `GOAL_LIFECYCLE_TRUSTWORTHY=NO`。单纯在现有 topic publisher 旁边加一个“保存最近 UUID”变量无法知道 Nav2 自己创建的 UUID；最小可靠修复应让 Isaac 的 RobotManager 用 action client 直接发送，保存 `goal_handle.goal_id`，只处理该 UUID 的 accepted/executing/terminal，并在 terminal 后清理。发送前需通过 `map→odom→base_link` 的真实 pose 与 global costmap 边界、planner/controller/BT active gate；不要以“节点存在”或占位 `(-10,-10)` 当 ready。保留原场景坐标、NavFn/DWB/频率/容差，回归时核对首次 goal 不再发生启动 abort、主 UUID 唯一归属。

## B. default 主 UUID 的最后 20 个仿真秒

主 goal 为 `(6.0,21.8,0)`，UUID `f6952af8bc0700b38bf3608edeb20322`。下表抽取 [逐事件重算](</home/user/arena_tf_throttle_validation_20261001/formal_default_216/root_cause_trace.json>) 中的每秒行；完整文件还保留各行 wall 时间、机器人 x/y/yaw、两级速度和 odom twist。NavigateToPose 在区间内始终是主 UUID 执行中，直到终态 status code 6。`cmd_vel_nav` 为 controller 输出，`cmd_vel` 为下游最终速度；表中的瞬时值是该 sim 时刻之前最近一条消息。`xy` 和 `yaw` 为相对原 goal 的误差。progress baseline 和 goal checker 内部没有逐次日志，下面的 baseline 是按正式源码规则和 controller 输出时刻重建的代理值。

| sim s | XY误差 m | yaw误差 rad | `cmd_vel_nav` vx/wz | 最终 `cmd_vel` vx/wz | odom vx/wz | 说明 |
|---:|---:|---:|---:|---:|---:|---|
| 130 | 2.472 | 2.586 | 0.233 / +0.158 | 0.233 / +0.158 | 0.219 / +0.090 | 接近目标，仍平移 |
| 135 | 1.281 | 2.603 | 0.246 / +0.053 | 0.233 / +0.263 | 0.218 / +0.050 | 平移继续 |
| 139 | 0.359 | 2.618 | 0.178 / +0.263 | 0.178 / +0.263 | 0.210 / +0.084 | 最后平移段 |
| 140 | 0.226 | 2.631 | 0 / −1.000 | 0 / −0.162 | 0.051 / −0.003 | 开始以转向为主；XY 首次达 0.25 m 是 sim 139.783 |
| 141 | 0.224 | 2.134 | 0 / −1.000 | 0 / −1.000 | −0.001 / −0.553 | 机器人实际在转 |
| 142 | 0.253 | 1.690 | 0.068 / +1.000 | 0.068 / +0.280 | 0.067 / +0.070 | 短暂越出 XY 容差，角速度反号 |
| 143 | 0.255 | 2.027 | 0.041 / +0.684 | 0.041 / +0.684 | 0.020 / −0.042 | 角速度反号使 yaw 回摆 |
| 144 | 0.243 | 2.232 | 0 / −1.000 | 0 / −1.000 | 0.003 / −0.360 | 再次向 goal yaw 旋转 |
| 145 | 0.201 | 1.722 | 0 / −1.000 | 0 / −1.000 | −0.001 / −0.426 | 转向继续 |
| 146 | 0.172 | 1.247 | 0 / −0.895 | 0 / −0.895 | 0.026 / −0.335 | 转向继续 |
| 147 | 0.179 | 0.956 | 0 / −0.579 | 0 / −0.579 | −0.008 / −0.334 | 转向继续 |
| 148 | 0.184 | 0.720 | 0 / −0.474 | 0 / −0.474 | −0.005 / −0.337 | 转向继续 |
| 149 | 0.183 | 0.566 | 0 / 0 | 0 / −0.208 | 0.021 / +0.102 | wall `0876.035877` progress 失败，随后发零速 |

在终态前 10 仿真秒，`cmd_vel_nav.angular.z` 21 帧中 20 帧非零，中位绝对值 0.895、最大 1.0 rad/s；最终 `cmd_vel.angular.z` 421 帧中 413 帧非零，中位 0.789 rad/s；odom angular 900 帧中位绝对值 0.336、最大 0.844 rad/s。相同窗口 `cmd_vel_nav.linear.x` 中位为零，XY 净位移 0.291 m，yaw 净变化 −2.055 rad。local costmap 185/185 帧有占据，未出现持续空图；正式 log 没有 `No valid trajectories`、Oscillation critic 或 RotateToGoal critic 错误。短暂的角速度反号说明转向并非严格单调，但不是“DWB 完全不输出”或“机器人不执行 angular command”。没有碰撞事件的逐帧直接证据，故不以此证明绝对无接触。

最后 20 仿真秒内，bag 的 `follow_path/_action/status` 有 82 个新 UUID，均经历 `EXECUTING→ABORTED`；高层 NavigateToPose 主 UUID 则一直 `EXECUTING` 到 sim `149.183` 才 `ABORTED`。这些重复的低层 UUID 与持续重规划/目标更新同时出现，不能把 82 次低层状态逐个当作独立导航 episode 失败。最后一个 FollowPath UUID `60d42823…` 在 sim `148.933` 开始、约 sim `148.950` abort，正与 controller_server 的 progress 异常同刻。[ControllerServer 1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_controller/src/controller_server.cpp) 在初次 `computeControl()` reset progress，后续 `updateGlobalPath()` 接纳 pending goal 并更新 path，并不重置 progress baseline；因此频繁的 FollowPath UUID 变化不破坏下面 10 s baseline 推断。

按每次 `cmd_vel_nav` 对应的最近 odom 近似 progress check 采样，最后一个 `>0.5 m` baseline 重置在 wall `1790870854.039692`、sim `138.433341`、位姿 `(6.45910,21.64738,2.61491)`。progress 错误 wall `1790870876.035877` 对应约 sim `148.950008`；即约 10.52 仿真秒，XY 从该 baseline 至终态只有 0.449 m，yaw 净变化约 −2.049 rad。终态 status wall `0876.533757`、sim `149.183341`。这是基于发布命令的**代理重建**，不能声称读到了插件内部 `baseline_time_`；但源码阈值、时间、轨迹以及精确的 `Failed to make progress` 异常共同支持 N1。

### Nav2 1.1.20 的实际判断语义和顺序

[SimpleProgressChecker 1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_controller/plugins/simple_progress_checker.cpp) 的 `check()` 在首次检查或与 baseline 的 `hypot(dx,dy) > required_movement_radius` 时重置 baseline，其他情况只允许 `now - baseline_time <= movement_time_allowance`；`pose_distance()` 只用 XY，**不计 yaw**。effective controller YAML 为 `required_movement_radius=0.5`、`movement_time_allowance=10.0`、`controller_frequency=1.0`。配置文件和实际加载库的版本已在正式 gate 中固定。

[ControllerServer 1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_controller/src/controller_server.cpp) 在 FollowPath 开始时 reset progress；每圈先等 local costmap current，再取 robot pose → `progress_checker_->check()` → DWB `computeVelocityCommands()` → 发布速度 → `isGoalReached()`。若 progress false，直接抛 `Failed to make progress`、发零速并终止 FollowPath；该圈不会进入 goal checker。`SimpleGoalChecker` 的 [1.1.20 源码](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_controller/plugins/simple_goal_checker.cpp)先查 XY，再查 yaw；`stateful=true` 只在 XY 首次满足后停止重复 XY 检查，不会使 yaw 0.566 rad 被 0.25 rad 容差接受。这里 XY 虽满足或短暂越界，yaw 始终未到容差；progress 仍要求距 baseline 超过 0.5 m 的平移。这是原配置对“终点以旋转为主”的语义冲突，属于有效 baseline 失败，而非本轮 TF/local costmap 或 wheel angular 控制失效。

## 修复需求与正式 baseline 决策

1. 本轮**不改** TF、costmap、LiDAR、NavFn、DWB critics、HuNav、scenario、goal/progress 参数，也不运行 Arena 1–5。
2. 启动链需要隔离的 infrastructure patch：当前真实 robot pose + global costmap 边界 + Nav2 active/readiness gate；由 Isaac 侧直接持有 action goal handle/UUID，回调仅接受 active UUID，terminal 清理；timer 只对明确 startup failure 重试。尤其不能让旧终态或其他客户端 goal 控制本 episode 的 `_is_goal_reached`。
3. patch 后先用有界 short/default 启动回归验证第一 UUID 不再因占位位姿或初始规划竞态退出、无多余 UUID、状态归属正确；重点观察 short 首次第三次 NavFn 重规划失败是否仍出现。default 接近终点的 N1 abort 不作为 patch 失败判据，也不准通过调参改为成功。通过后才能把 `INFRASTRUCTURE_BASELINE_READY` 升为 YES，并安排原始 Arena 1–5 DWB baseline。

## 请求的最终字段

```text
STARTUP_CLASS: S3 (default S1 observed; S2 implementation defect; short third-replan subcause not fully logged)
STARTUP_ROOT_CAUSE: default first goal preceded real robot TF by 10.8 ms and NavFn planned from placeholder (-10,-10); short first action planned and commanded but its third startup replan failed
STARTUP_PATCH_REQUIRED: YES
STARTUP_MINIMAL_PATCH: gate real pose/global bounds and Nav2 active, send Isaac goal with an owned action client, bind callbacks to active UUID, clear on terminal, retry only a classified startup failure

NEARGOAL_CLASS: N1 — PROGRESS_CHECKER_SEMANTIC_ABORT
NEARGOAL_ROOT_CAUSE: SimpleProgressChecker requires >0.5 m XY movement within 10 sim seconds while terminal yaw alignment mainly rotates; goal yaw still outside 0.25 rad
DWB_ROTATION_COMMAND_PRESENT: YES
ROBOT_ROTATION_RESPONSE_PRESENT: YES
PROGRESS_CHECKER_CAUSAL: YES (exact error and code; baseline pose/time reconstructed from nearest odom, not internally logged)
NAVIGATION_FAILURE_VALID_BASELINE_RESULT: YES for the default main action under current frozen navigation parameters

TF_COSTMAP_INFRASTRUCTURE: VERIFIED by existing gates; default startup real-pose handoff remains a lifecycle bug
GOAL_LIFECYCLE_TRUSTWORTHY: NO for automated episodes until UUID ownership is fixed; these two bag status arrays themselves were ordered correctly
INFRASTRUCTURE_BASELINE_READY: NO

REQUIRES_PARAMETER_TUNING: NO
READY_FOR_ARENA_1_TO_5: NO
NEXT_SINGLE_ACTION: implement and review the scoped startup readiness/UUID lifecycle patch, then run bounded startup regression
```
