# Isaac Sim 5.1 ROS 2 Control Gate 4

审计日期：2026-09-11  
被测 experience：`isaac_sim/backends/isaac5/generated/minimal_core_physx_no_rtx.kit`  
执行边界：未修改 Isaac6、现有 Isaac5 backend 或系统 driver；未加载行人、Arena、rosnav、LiDAR producer 或 DRL。

## 结论

**Gate 4 PASS。** 外部 ROS 2 Humble CLI 发布 `geometry_msgs/msg/Twist` 到 `/cmd_vel` 后，Isaac5 custom runtime 收到命令并驱动 Mecanum730 前进；同时发布 `/clock`、`/odom`、`/tf`、`/tf_static`。

## ROS 方案边界

没有启用 5.1 的 monolithic `isaacsim.ros2.bridge` extension。该 extension 的本机 manifest 声明了 `isaacsim.sensors.rtx` 依赖；在当前 RTX5090/595.84 环境下会把 ROS topic 验证重新带入不需要的 RTX/viewport 依赖闭包。

本 Gate 改用 Isaac5 安装内 bundled Humble Python 3.11 包，直接使用 `rclpy`、标准 message packages 和 bridge Humble libraries；custom Kit 仍只提供 Core/PhysX/Python API。这满足本阶段明确的 ROS topic contract，同时避免 RTX LiDAR 和 RTX Hydra startup。

## 实测证据

原始日志：[`ros_control_gate4_external_run_7.log`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/ros_control_gate4_external_run_7.log:104)

外部发布命令为：

```text
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

结果：

- ROS callback received count：`24`
- simulation：`240` steps / `4.0333 s`
- robot `Δx=+0.42001 m`
- command samples 在开始的零速后变为 `[0.15, 0, 0]`
- wheel joints 在运动期间累计约 `5.385 rad`
- `ros_close=true`、`world_stop=true`、`app_close=true`

ROS CLI 在运行期间发现的 topic：

```text
/clock
/cmd_vel
/odom
/parameter_events
/rosout
/tf
/tf_static
```

外部 echo 已读取 `/clock`、`/odom` 和 `/tf_static`；odom 使用 `odom -> base_link`，静态 TF 使用 `base_link -> base_scan`。最终 teardown 证据见日志第 180 行。

## 验收边界

| 项目 | 结果 |
|---|---|
| `/cmd_vel` subscribe | PASS |
| `/clock` publish | PASS |
| `/odom` publish | PASS |
| `/tf` publish | PASS |
| `/tf_static` publish | PASS |
| ROS 驱动机器人运动 | PASS |
| `isaacsim.exp.base.python.kit` | 未使用 |
| `isaacsim.ros2.bridge` extension | 未启用 |
| RTX SceneDB | 成功路径未加载 |

这个 Gate 证明的是 bundled `rclpy` topic boundary，不是 custom message、Nav2、Arena service 或 ROS 2 control hardware interface。
