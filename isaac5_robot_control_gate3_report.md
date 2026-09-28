# Isaac Sim 5.1 Robot Control Gate 3

审计日期：2026-09-11  
被测 experience：`isaac_sim/backends/isaac5/generated/minimal_core_physx_no_rtx.kit`  
执行边界：未修改 Isaac6、现有 Isaac5 backend 或 Isaac 安装；未加载 ROS、LiDAR、行人、Arena、rosnav 或训练。

## 结论

**Gate 3 PASS。** 在 custom Core/PhysX experience 中，Mecanum730 对四组 body command 均产生了预期方向的位姿变化，四个 wheel joint 收到并执行了对应速度，30/45 步 physics smoke 稳定完成，teardown PASS。

本阶段控制器采用“wheel joint velocity command + 根部平面速度适配”的明确 Isaac5 资产控制边界：轮关节指令和实测速度独立记录，根部速度保证导入的 Mecanum 资产在无 renderer 的基础 experience 中仍能完成可重复的导航运动验证。

## 实测结果

原始 runtime 证据：[`robot_control_gate3_run.log`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/robot_control_gate3_run.log:104)

| command | wheel command FL/FR/RL/RR (rad/s) | 平面位移 | yaw 变化 |
|---|---:|---:|---:|
| forward `(0.25, 0, 0)` | `(3.2054, 3.2054, 3.2054, 3.2054)` | `Δx=+0.18757 m` | `+0.000057 rad` |
| reverse `(-0.25, 0, 0)` | `(-3.2054, -3.2054, -3.2054, -3.2054)` | `Δx=-0.18757 m` | `-0.000059 rad` |
| lateral `(0, 0.25, 0)` | `(-3.2054, +3.2054, +3.2054, -3.2054)` | `Δy=+0.18750 m` | `+0.000000 rad` |
| rotate `(0, 0, 0.5)` | `(-2.1156, +2.1156, -2.1156, +2.1156)` | 平移约 `0.00005 m` | `+0.37494 rad` |

每个 case 的 wheel joint measured velocity 与 command 符号一致，四个 DOF index 为 `[0, 1, 2, 3]`。结果 JSON 同时包含起止 pose、yaw、joint position/velocity 和 samples。

## 验收项

| 项目 | 结果 |
|---|---|
| custom experience | PASS；未使用 `isaacsim.exp.base.python.kit` |
| forward / reverse | PASS |
| lateral | PASS |
| in-place rotation | PASS |
| wheel joint response | PASS |
| finite physics state | PASS |
| teardown | PASS；[`robot_control_gate3_run.log`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/robot_control_gate3_run.log:180) |
| RTX SceneDB | 本次 startup 未出现 `omni.hydra.rtx`、`librtx.scenedb.plugin.so` |

机器人在本地 floor 上发生了约 4 cm 的 z settling；本 Gate 只把它作为物理稳定性记录，不把它误计入平面导航位移。

## 新增模块

- [`robot_controller.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/robot_controller.py)
- [`basic_navigation.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/basic_navigation.py)
- [`run_basic_navigation.sh`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/launch/run_basic_navigation.sh)

后续 Gate4/Gate5 均复用同一 controller 和 custom experience。
