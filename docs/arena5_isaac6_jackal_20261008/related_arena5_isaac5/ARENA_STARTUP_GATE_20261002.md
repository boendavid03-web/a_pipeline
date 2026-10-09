# Arena Isaac 5.1 startup 与 NavigateToPose ownership gate（2026-10-02）

## 判定

**INFRASTRUCTURE_BASELINE_READY = YES；READY_FOR_ARENA_1_TO_5 = YES。** 最终源码的 short no-human 和原始 default＋3 HuNav 各发送一个高层 NavigateToPose goal，没有经 `/goal_pose` 再发目标。两次均在 Nav2 active、机器人真实位姿和 global costmap 边界验证后进入 EXECUTING。short 的单一 action SUCCEEDED；default 的单一 action 在正常行驶 28.17 m 后因已知 N1 `Failed to make progress` ABORTED，这是当前冻结参数下的有效导航结果。此判定只表示自动 benchmark 的 startup 与 action 归属基础设施可用；原始 Arena 1–5 的 DWB 结果仍待采集。

本轮保留 Isaac Sim 5.1、当前 compatibility adapter、TF throttle=10、Nav2 1.1.20、NavFn、DWB、1 Hz controller、0.5 m/10 s progress checker、0.25 m/0.25 rad goal tolerance，以及原场景和 HuNav 行为。没有迁移官方 adapter、调 Nav2/HuNav 参数、运行 Arena 1–5 或搬迁目录。[机器可读判定](ARENA_STARTUP_GATE_20261002.yaml)记录了精确哈希和运行路径。2026-10-01 的[历史冻结清单](ARENA_ISAAC_FROZEN_BASELINE_MANIFEST_20261001.yaml)仍保留当时的 `NOT_READY` 状态，本文件是其后的 startup gate。

## 实施

[RobotManager 源码](isaac_sim/arena_ws/src/arena/arena-rosnav/task_generator/task_generator/manager/robot_manager/robot_manager.py)只在 Isaac 分支使用 Nav2 `ActionClient`，移除该分支的 `goal_pose` publisher、3 秒重发 timer 和 `GoalStatusArray.status_list[-1]` 决策。发送前等待本 episode reset 后的新 odom、新真实 map TF、新 global costmap 样本、三个 Nav2 lifecycle ACTIVE、action server ready，并确认机器人半径位于地图边界内。发送后保存 handle、UUID、episode generation；只由匹配的 result 回调更新结果，terminal 后清理归属，同一 episode 不重发。reset 时对旧 action 发 cancel，并等待旧 action terminal；迟到的旧回调不能设置新 episode 的成功状态。生命周期回调与 reset 共用默认互斥 callback group。非 Isaac 后端保留原 topic/status 路径。

源码原 SHA-256 `2e204448906afe192c9e3640fc337911311c51021015d0cae183a107b2e18c99`，最终 SHA-256 `1b57d1ee62c0a647ebdb1271784cb51ccab8d44a24c347a842f3fc5a7a2969a3`；59 个 Python 文件的内容 pin 从 `bf2e3ef2...` 变为 `779b9a91...`。[可逆补丁](ARENA_STARTUP_PATCH_20261002.patch)通过 `git apply --reverse --check`；原文件备份在 `[LOCAL_PATH]`。Task Generator 的 build/install 为此源码的可见 symlink/egg-link，实际运行日志含新增 `ISAAC_GOAL_SEND/ACCEPTED/TERMINAL` 标记。

## 测试和运行证据

[确定性测试](scripts/validation/arena_startup/test_robot_manager_startup.py) 9/9 通过：拒绝占位/过期位姿和旧 costmap、等待 Nav2 active、单 goal、ABORT 后不重发、reset cancel、迟到 action/lifecycle 回调隔离、非 Isaac 路径。`py_compile` 通过。reset 中旧 action 的 cancel/迟到回调是 mock 证据；本轮两个实跑各为单 episode，没有执行实时多 episode reset。

| Gate | high-level UUID/状态 | 首发 readiness | terminal 与控制 | TF/感知回归 |
|---|---|---|---|---|
| short no-human，ROS domain 219 | 仅 `0b77b461…`，日志 ACCEPTED，bag EXECUTING→SUCCEEDED，`goal_pose` 0 条 | 真实 TF 首次出现早于 SEND 约 112 ms；Nav2 ACTIVE；实际起点 `(25.294,1.246)` 在 global costmap 内 | XY 误差 0.128 m；终态后 8 条 `cmd_vel` 均为零 | 640 束 scan；local grid 343/414 非零；voxel 385/477 非零 |
| original default＋3 HuNav，ROS domain 220 | 仅 `044365d0…`，日志 ACCEPTED，bag EXECUTING→ABORTED，`goal_pose` 0 条 | SEND 前已有真实 TF，最近 TF `(25.300,1.250)`；Nav2 ACTIVE；起点在 global costmap 内 | 行驶 28.17 m；N1 `Failed to make progress`；XY 误差 0.102 m、yaw 误差 0.378 rad；终态后 10 条 `cmd_vel` 均为零 | 640 束 scan；local grid 791/1548 非零；voxel 1699/3244 非零；HuNav 3 个 agent 和首个服务调用已记录 |

- short：[启动时序与 UUID](scripts/validation/arena_startup/short_final_analysis_20261002.json)、[导航/感知汇总](<[LOCAL_PATH]>)、[原始 bag](<[LOCAL_PATH]>)、[运行日志](<[LOCAL_PATH]>)。
- default：[启动时序与 UUID](scripts/validation/arena_startup/default_final_analysis_20261002.json)、[导航/感知汇总](<[LOCAL_PATH]>)、[原始 bag](<[LOCAL_PATH]>)、[运行日志](<[LOCAL_PATH]>)。

两个运行的 `preflight.txt` 固定同一 adapter commit `a4beefe`、Nav2 1.1.20、Nav2 config SHA-256 `2c7eea09...`、隔离 TF throttle=10 文件 SHA-256 `c8f014b0...`。default 场景 SHA-256 `4970601d...` 和 HuNav default behavior SHA-256 `1b12b19b...` 与历史清单一致。运行后 domain 219/220 无 ROS 节点，相关进程已退出。`Failed to make progress` 不触发第二个高层 goal；本轮不调整 N1。

## 下一步和环境路径

下一步只运行原始 `1.json` 的 DWB baseline；每个原始场景先核对场景几何和碰撞资产，再逐个推进 `2.json` 至 `5.json`。官方 `arena5-isaac5.1.0` adapter 在这五个 baseline 建立后另案评估。

当前权威路径：Task Generator source 为上述嵌套源码；运行 Nav2 config 为 `[LOCAL_PATH]`；Isaac Python 3.11 install 为 `[LOCAL_PATH]`；本次统一 gate 入口为 [run_gate.bash](scripts/validation/arena_tf_throttle/run_gate.bash)。本轮没有搬迁或删除任何目录。
