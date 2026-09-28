# Isaac Sim 5.1 Gate 2：Mecanum730 robot loading validation

审计日期：2026-09-11  
被测 experience：`isaac_sim/backends/isaac5/generated/minimal_core_physx_no_rtx.kit`  
执行边界：不修改 Isaac 6、不加载 ROS 2、不创建 LiDAR runtime、不加载 Arena；未修改现有 Isaac 5 navigation runtime、Isaac 5 安装或系统 driver。

## 1. 最终结论

**Gate 2 PASS。**

在已通过的 Isaac 5.1 custom Core/PhysX experience 中，当前 Mecanum730 robot USD 成功完成：

- USD reference 和 stage composition；
- `/World/Robot/base_footprint` articulation root 发现；
- 38 DOF articulation 初始化；
- 四个 wheel joint 发现和索引映射；
- 30 次 `render=False` PhysX stepping；
- robot pose、joint position/velocity 读取；
- `world.stop()` 和 `SimulationApp.close()` teardown。

最终进程 exit code 为 `0`，probe 结果和 teardown 均为 `PASS`。

本次没有发现 robot USD 对 `omni.hydra.rtx`、`librtx.scenedb.plugin.so` 或 RTX renderer extension 的运行时依赖，也没有发现缺失 extension 导致的失败。

## 2. 被测 robot USD

Canonical USD：

```text
/home/user/navigation_project/robot_related/robots/chassis_arm/
motion_wheel_arm_simple_sphere_usd/mecanum730_xms5_default.usd
```

SHA-256：

```text
f9dec5e8554d6504c52dc91622338024e035f82697ff331dde0f5c53c1d15f4c
```

probe 将该 USD reference 到：

```text
/World/Robot
```

预期 articulation root：

```text
/World/Robot/base_footprint
```

## 3. Probe 范围

临时 probe：[`robot_gate2_custom_core_physx_probe.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/robot_gate2_custom_core_physx_probe.py:1)

它只执行：

1. 启动 `minimal_core_physx_no_rtx.kit`；
2. 创建空 stage 和本地 ground plane；
3. reference 当前 Mecanum730 USD；
4. 使用 `isaacsim.core.api.robots.Robot` 创建 articulation wrapper；
5. 调用 `world.reset()`；
6. 读取 articulation root、DOF、joint names、pose 和 joint state；
7. 执行 30 次 `world.step(render=False, step_sim=True)`；
8. 正常停止 world 和关闭 Kit。

没有导入或创建：

- `rclpy`、ROS 2 bridge、UDP relay；
- RTX/PhysX LiDAR sensor API；
- Arena、`ros2isaacsim` 或 task generator；
- Isaac 6 runtime 或其业务脚本。

注意：canonical robot USD 自身会组合 `mecanum730_xms5_default_sensor.usd` layer，这是资产闭包的一部分；本 probe 没有加载 LiDAR extension、创建 LiDAR producer、读取 sensor buffer 或运行任何 LiDAR pipeline。这里的“no LiDAR”指没有 sensor runtime。

## 4. Stage composition 结果

最终日志：[`robot_gate2_run_clean.log`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/robot_gate2_run_clean.log:108)

结果：

| 检查 | 结果 |
|---|---|
| robot USD file exists | PASS |
| `/World/Robot` valid | PASS |
| `/World/Robot/base_footprint` valid | PASS |
| `UsdPhysics.ArticulationRootAPI` | PASS |
| composed prim count before Robot view | 973 |
| used layers | 7 |
| final stage root | `anon:...:World1.usd` |

已观察到的 used layers 包括：

```text
/home/user/navigation_project/robot_related/robots/chassis_arm/
motion_wheel_arm_simple_sphere_usd/mecanum730_xms5_default.usd

/home/user/navigation_project/robot_related/robots/chassis_arm/
motion_wheel_arm_simple_sphere_usd/configuration/mecanum730_xms5_default_base.usd

/home/user/navigation_project/robot_related/robots/chassis_arm/
motion_wheel_arm_simple_sphere_usd/configuration/mecanum730_xms5_default_physics.usd

/home/user/navigation_project/robot_related/robots/chassis_arm/
motion_wheel_arm_simple_sphere_usd/configuration/mecanum730_xms5_default_sensor.usd
```

另有 ground-plane helper 使用的 Isaac 5.1 `default_environment.usd` URL，以及两个 anonymous/session layer；它们不是 robot USD 的 RTX 或 Arena 依赖。

Isaac Sim 5.1 当前 `Usd.Stage` 没有 `GetCompositionErrors()` 方法，probe 已明确记录：

```text
GetCompositionErrors unavailable:
AttributeError("'Stage' object has no attribute 'GetCompositionErrors'")
```

因此 compose 判定采用实际 composed prim、articulation root 和 used-layer 结果；没有以该 API 不存在误报失败。

## 5. Articulation root 和 wheel joints

Articulation root：

```text
/World/Robot/base_footprint
```

DOF count：`38`

四个 wheel joint 均存在：

| joint | DOF index |
|---|---:|
| `wheel_fl_joint` | 0 |
| `wheel_fr_joint` | 1 |
| `wheel_rl_joint` | 2 |
| `wheel_rr_joint` | 3 |

完整 joint name 顺序：

```text
wheel_fl_joint
wheel_fr_joint
wheel_rl_joint
wheel_rr_joint
joint1
wheel_fl_roller_1_joint
wheel_fl_roller_2_joint
wheel_fl_roller_3_joint
wheel_fl_roller_4_joint
wheel_fl_roller_5_joint
wheel_fl_roller_6_joint
wheel_fl_roller_7_joint
wheel_fr_roller_1_joint
wheel_fr_roller_2_joint
wheel_fr_roller_3_joint
wheel_fr_roller_4_joint
wheel_fr_roller_5_joint
wheel_fr_roller_6_joint
wheel_fr_roller_7_joint
wheel_rl_roller_1_joint
wheel_rl_roller_2_joint
wheel_rl_roller_3_joint
wheel_rl_roller_4_joint
wheel_rl_roller_5_joint
wheel_rl_roller_6_joint
wheel_rl_roller_7_joint
wheel_rr_roller_1_joint
wheel_rr_roller_2_joint
wheel_rr_roller_3_joint
wheel_rr_roller_4_joint
wheel_rr_roller_5_joint
wheel_rr_roller_6_joint
wheel_rr_roller_7_joint
joint2
joint3
joint4
joint5
joint6
```

缺失 wheel joints：

```text
[]
```

## 6. Articulation state 和 pose

### 6.1 初始状态

初始 world pose：

```text
position xyz       = [-4.88e-11, -2.43e-11, 0.0118251]
orientation wxyz   = [1.0, -6.05e-11, 2.95e-10, -4.52e-11]
```

初始 38 个 joint positions 和 velocities 均为 `0.0`。

### 6.2 30 步后的状态

最终 world pose：

```text
position xyz       = [7.98896e-06, 2.03094e-04, -0.1840841]
orientation wxyz   = [0.99999988, 5.52185e-04, -6.89215e-06, 1.22207e-07]
```

最终主要运动状态：

```text
linear velocity    = [-0.00190754, 0.00040041, 0.00079166]
angular velocity   = [0.00183423, 0.01008515, -0.00029785]
physics steps      = 30
```

四个 wheel joint 的最终位置/速度：

| joint | position | velocity |
|---|---:|---:|
| `wheel_fl_joint` | `4.438e-08` | `0.002214` |
| `wheel_fr_joint` | `2.537e-08` | `0.002213` |
| `wheel_rl_joint` | `5.224e-09` | `0.002212` |
| `wheel_rr_joint` | `-1.391e-08` | `0.002211` |

完整 38-DOF positions、velocities 和 3 个 step samples 保存在原始 JSON 行：

[`robot_gate2_run_clean.log:108`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/robot_gate2_run_clean.log:108)

这 30 步包含正常重力 settling；本 Gate 只判断 articulation 是否能初始化、读取有限状态并完成 physics stepping，不把当前 pose settling 高度当作导航或碰撞验收。

## 7. Teardown

最终日志记录：

```text
ROBOT_GATE2_TEARDOWN={
  "world_stop": true,
  "app_close": true,
  "errors": [],
  "status": "PASS"
}
```

没有观察到 Vulkan/RTX/PhysX teardown crash。

## 8. RTX、缺失 extension 和非致命日志

对最终 probe stdout、Kit log 和 `LD_DEBUG=libs,files` 文件检查结果：

```text
omni.hydra.rtx                    not started
librtx.scenedb.plugin.so          not dynamically loaded
libcarb.scenerenderer-rtx.plugin  not dynamically loaded
missing extension failure          none
```

成功 runtime 使用了 Core/PhysX 相关 extension，包括 `omni.physx`、`omni.physx.tensors`、`omni.usd` 和 `omni.hydra.usdrt_delegate`，但没有启动 `omni.hydra.rtx`。

仍存在一个与上一个 Gate 相同的非致命日志：

```text
[omni.gpu_foundation_factory.plugin] Start up failed.
The default graphics plugin cannot be set!
```

它没有阻止 app ready、USD compose、articulation reset、30 次 stepping 或 teardown；本次没有把它误判为 robot USD 缺失 extension。

## 9. 过程中的 probe-only 问题

第一次 Gate 2 probe 使用了错误的路径推导，指向 `/home/user/robot_related/...`，在 Kit 启动前失败；修正为实际 canonical path 后进入 runtime。

第二次 probe 将 Isaac 5.1 不存在的 `Stage.GetCompositionErrors()` API 当作 composition failure；修正为记录 API 不可用，并用 composed prim、articulation root 和 used layers 验证 stage composition。

最终 clean probe 没有修改 USD 文件，也没有向 robot asset 写入 solver 属性。

## 10. 最终判定

| Gate 2 项目 | 判定 |
|---|---|
| custom Core/PhysX experience 启动 | PASS |
| Mecanum730 USD 读取 | PASS |
| stage compose | PASS（composed prim/layer 证据；5.1 无 `GetCompositionErrors()`） |
| articulation root | PASS |
| 4 wheel joints | PASS，index `0..3` |
| 38 DOF state | PASS，状态有限且可读取 |
| physics stepping | PASS，30 steps |
| teardown | PASS |
| RTX SceneDB dependency | 未发现 |
| missing extension failure | 未发现 |
| ROS/LiDAR/Arena | 未加载 |

**Gate 2 已通过，但这不等价于机器人运动控制、里程计/TF、ROS 2、LiDAR 或导航功能通过。**
