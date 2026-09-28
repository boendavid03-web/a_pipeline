# Isaac Sim 5.1 Basic Navigation Interface

审计日期：2026-09-11  
被测 experience：`isaac_sim/backends/isaac5/generated/minimal_core_physx_no_rtx.kit`

## 结论

**手动目标接口 smoke PASS。** `ManualGoalController` 接收 goal 和 odom pose，输出 body-frame `(vx, vy, omega)`；运行时同时将 TF 和 360-beam scene-query LaserScan 放入统一 `NavigationObservation` envelope，然后由 Mecanum controller 驱动机器人。

本阶段没有接 SemanticCNN、DRL-VO、行人、Social Force、Arena 或 Nav2，也没有实现 obstacle avoidance；LaserScan 目前作为稳定输入边界传递，手动目标控制器只使用 odom pose 和 goal 计算命令。

## 实测

原始 runtime 证据：[`navigation_interface_run_2.log`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/navigation_interface_run_2.log:104)

测试 goal：`(x=0.5 m, y=0.0 m, yaw=0.0 rad)`；运行 `240` steps、`4.0333 s`。

结果：

- command 从约 `0.35 m/s` 随距离缩小到 zero；
- robot `Δx=+0.42060 m`；
- `goal_reached=true`；
- `navigation_inputs = {odom: true, tf: true, laser_scan: true}`；
- scan 每 0.1 s 记录一次，360 beams；
- teardown `world_stop=true`、`app_close=true`，见日志第 180 行。

## 模块边界

- [`navigation_interface.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/navigation_interface.py)：`NavigationObservation`、`ManualGoalController`
- [`robot_controller.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/robot_controller.py)：Mecanum wheel mapping 和运动适配
- [`lidar_sensor.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/lidar_sensor.py)：PhysX scene-query LaserScan
- [`ros_bridge.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/ros_bridge.py)：bundled Humble `rclpy` topic boundary
- [`basic_navigation.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/basic_navigation.py)：custom experience orchestrator

## 已通过与未覆盖

| 能力 | 状态 |
|---|---|
| Isaac5 custom Core/PhysX runtime | 已通过 |
| Mecanum730 USD/articulation | 已通过 |
| body velocity / wheel command | 已通过 |
| ROS `/cmd_vel`, `/clock`, `/odom`, `/tf`, `/tf_static` | 已通过 |
| 非 RTX `/scan` | 已通过 |
| manual goal -> cmd_vel -> robot motion | 已通过 |
| pedestrians / BehaviorAgent / Social Force | 未实现 |
| DR-SPAAM / DRL-VO | 未实现 |
| SemanticCNN / S3-Net | 未实现 |
| Arena / rosnav / Nav2 | 未接入 |
| obstacle-aware planner | 未实现 |

Gate3、Gate4、Gate5 和本 manual-goal smoke 均未修改 Isaac6 主链。
