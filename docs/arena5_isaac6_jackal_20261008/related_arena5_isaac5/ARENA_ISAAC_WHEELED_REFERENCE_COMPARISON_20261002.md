# Isaac Sim 5.1 轮式机器人结构对照：Arena Jackal 接地转向

日期：2026-10-02。范围：只读调查；没有修改运行中的 adapter、机器人、installed package、Nav2 或 HuNav，没有运行 Arena。唯一新增文件是本报告。

## 结论及证据等级

```yaml
REFERENCE_1: NVIDIA Jetbot (Isaac Sim 5.1 官方示例和 test_spin)
WHY_KNOWN_GOOD: 5.1 自带 test_jetbot.py 对接地原地旋转的 odom yaw rate 作断言；本轮未重跑测试
KEY_PHYSICS_MODEL: 两个驱动球形碰撞轮，轮物理材质 static/dynamic friction=1.0，另有低摩擦 caster 球；不是四轮 skid-steer

REFERENCE_2: NVIDIA Nova Carter (Isaac Sim 5.1 自带 test_spin)
WHY_KNOWN_GOOD: 5.1 自带 test_carter_v2.py 对接地原地旋转的 odom yaw rate 作断言；本轮未重跑测试
KEY_PHYSICS_MODEL: 两个驱动轮和有转向自由度的 caster，非四个固定方位驱动轮；预制 USD 使用 force 型轮关节驱动

REFERENCE_3_CLOSEST_STATIC_MODEL: Isaac Sim 5.1 官方 Clearpath Jackal USD
WHY_RELEVANT: 官方 5.1 资产目录列为 4 joints/4 DOFs；本地 USD 的四轮位置、圆柱半径和宽度与当前 Jackal URDF 相同
KNOWN_GOOD_LIMIT: 本轮没有找到或运行针对该 USD 原地转向的官方数值断言，因此它是最贴近的物理结构参考，不把资产上架当作已测通过
KEY_PHYSICS_MODEL: 4 个 0.098 m 半径、0.04 m 宽的圆柱 collider，全部显式绑定 static/dynamic friction=0.2 的 wheel material；TGS，articulation solver 32 position/16 velocity iterations，force 型速度 drive

OFFICIAL_ARENA_5_1_CONTROL_ARCHITECTURE: URDF import -> 外部 ros2_control controller_manager/JointStateTopicSystem 按 URDF command_interface 输出关节命令 -> ROS2SubscribeJointState -> IsaacArticulationController；没有旧的 pairwise DifferentialController 图

CURRENT_VS_REFERENCE_MAJOR_DIFFERENCES: 最贴近的官方 Jackal USD 保留相同四轮圆柱拓扑，但显式低摩擦轮材质、force 型高 damping drive 和显式 solver iterations；当前导入后轮材质/接触参数尚未测量

DIFFERENCE_1: 四个接地轮的物理材质与横向滑移阻力
CAUSAL_RELEVANCE: 四轮固定朝向原地转向要求前后轮在地面横向滑移；接触摩擦过高会抵抗 yaw，并反向加载轮关节
CONFIDENCE: 高（结构差异和物理约束）；中（它是否是 229 的主因仍待 A/B）

DIFFERENCE_2: 轮关节 velocity drive 的 type/damping
CAUSAL_RELEVANCE: 当前 acceleration/1000 与官方 Jackal USD force/1e7 的接触载荷下目标跟踪能力可能不同；229 正好显示仅接地时关节跟踪变差
CONFIDENCE: 高（配置差异）；中低（因果方向和适当数值未验证）

DIFFERENCE_3: 双驱动轮加 caster 与四个固定方位驱动轮的接地约束数
CAUSAL_RELEVANCE: Jetbot/Nova Carter 的原地旋转测试没有验证四个前后轮必须横向 scrub 的条件；官方 Jackal 是更有效的四轮物理 A/B 参照
CONFIDENCE: 高（拓扑事实与运动学要求）；中（229 中约束阻力的实际大小尚待接触数据）

BEST_COMPONENT_TO_BACKPORT_OR_EMULATE: 先借鉴官方 Jackal USD 的轮 collider 物理材质绑定与接触属性审计方法；只在独立 probe stage 上验证，不直接写入当前模型
BEST_A_B_TEST_FOR_MAIN_EXECUTOR: 在 229 同源独立 probe 中先记录四轮 collider 的实际 USD 路径、有效 physics material/friction/combine、contact/rest offsets、接触点和 solver；随后只把四轮 collider 的有效材质改为官方 Jackal 的 0.2/0.2，保持几何、drive、地面和命令不变，重放 -0.2/0/-0.5/0 窗口，并与原始 229 比较轮速误差、yaw 增量和侧滑
DOES_THIS_REQUIRE_OFFICIAL_ADAPTER_MIGRATION: NO
WHY: 已定位的是接地后的 articulation/contact 响应，官方 Arena 改变命令传输却仍以 IsaacArticulationController 驱动导入轮关节；没有证据表明迁移 ros2_control 会修复接地物理
FILES_OR_UPSTREAM_REFERENCES: 本机 Isaac 5.1 Jetbot/Nova Carter 测试与 Clearpath Jackal USD、229 probe.json、官方 Arena 5.1 SpawnUrdf.py/control/topic_bridge.py；具体路径及上游链接见文末
HANDOFF_TO_MAIN_EXECUTOR: 先完成接地属性快照，再做单变量材质 A/B；若轮速仍失真，单独比较 force 型 drive；只有残余误差仍与 solver 相关时再测 iterations。每次保留原始 229 的窗口和指标，禁止据此宣布 DWB baseline 通过
```

## 当前 Jackal reference

数据源为 [A5 阻断报告](ARENA_CONTROL_CHAIN_BLOCKER_20261002.md)、[probe](scripts/validation/arena_control_chain/probe.py)、installed [Jackal URDF]([LOCAL_PATH])、[control.yaml]([LOCAL_PATH])，以及 229 运行对应的 [UrdfToUsd]([LOCAL_PATH])、[control parser]([LOCAL_PATH]) 和 [differential graph]([LOCAL_PATH])。这三份控制文件的 SHA-256 分别为 `5258725cc075d2879abb248629fd3a59a6404823ffbd516d7d1c38a39c3ae38b`、`634beda0130ce4596dfcbd6f275daf8a2c76c5e0fda767e386b302fedafae028`、`e209e26656bd63532335ec19a9c579d56d12567998a2b5eb6af1f1f0138da974`；仓内旧源码哈希不同，不能替代运行副本。

架构：URDF importer → 4 个 revolute wheel joints → 2 个 DifferentialController 图（前左/前右，后左/后右）→ 2 个 IsaacArticulationController → 地面上的四个刚体轮。半径 `0.098 m`，碰撞圆柱宽 `0.04 m`，URDF 轮轴 `Y`，前后轮中心 x=`±0.131 m`，左右中心 y=`±0.187795 m`。`control.yaml` 轮距 `0.36 m`、multiplier `1.5`；229 图实际用有效轮距 `0.54 m`。URDF 的 `<gazebo>` 为四轮写 `mu1=mu2=0.5`、`fdir1=1 0 0`、`kp=1e7`、`kd=1`，这些是 Gazebo/ODE 标签，**不能据此宣称当前 USD/PhysX 轮材质就是 0.5 或存在方向性摩擦**。旧 importer 没有显式读取这些标签或绑定物理材质。轮质量各 `0.477 kg`，chassis URDF 质量 `16.523 kg`，chassis 碰撞为 box；self collision 关闭。旧 importer 设 `import_inertia_tensor=False`、`default_drive_type=2`、`fix_base=False`。229 实测关节 USD 为 `acceleration` drive、stiffness `0`、damping `1000`、maxForce `3.402823e38`，轴 `Y`；articulation root `/World/jackal/base_link`。当前 probe 用 `World(physics_dt=1/60, rendering_dt=1/60)`、刚性 `FixedCuboid` 地面、每 playback tick 更新两个图；未记录真实 scene solver、有效材质绑定、接触点、contact/rest offsets、横向 slip 或带载轮力矩。

229 原始 [`probe.json`]([LOCAL_PATH]) SHA-256 `44178cd1ea0522c6033414452a178e846fd69dc107d27bc7597b02d3d739491d`。`-0.2 rad/s` 的四轮目标为 `(+0.551,-0.551,+0.551,-0.551) rad/s`，接地 2.9 s 轮角增量约 `(+0.243,-0.400,+0.328,-0.297) rad`，车体 yaw 仅 `-0.00272 rad`；`-0.5 rad/s` 时 yaw 为 `-0.25003 rad`。无近地面的 228 中四轮对相同目标跟踪约到 `1e-4 rad/s`。因此目标换算本身不是首个可重复故障层；地面接触/关节耦合是。无地面的车体 yaw 不是运动学验收。

## 物理语义矩阵

`未证实` 表示代码/探针未给出有效运行时值，绝不以 SDK 默认猜值。Jetbot 和 Nova Carter 的 `test_spin` 是发行包中的测试设计，本轮未执行；Clearpath Jackal 是官方资产，不假装已在本机跑过旋转。

| 项 | 当前 compatibility Jackal | Jetbot 5.1 | Nova Carter 5.1 | Clearpath Jackal 5.1 | Arena `arena5-isaac5.1.0` |
|---|---|---|---|---|---|
| 来源/几何 | installed URDF 导入；4 驱动圆柱轮，r=.098、宽=.04 m | 预制 USD；2 驱动球形碰撞轮，有 caster 球 | 预制 USD；2 驱动轮和有 swivel 的 caster | 预制 USD；4 驱动圆柱轮，r=.098、宽=.04 m | URDF 导入；实际轮几何取传入 URDF，分支无独立 Jackal/Husky 资产 |
| 轮碰撞轴/关节轴 | URDF cylinder 绕 X 旋转 90° 后轴沿 Y；229 关节 `Y` | 球形轮无 cylinder 轴；轮关节 `X` 加旋转 frame | 轮关节 `Z` 加 localRot frame；碰撞由复合 USD 决定 | cylinder local orient 绕 X 90°；轮关节 `X` 加 localRot，世界轮轴与 Y 一致 | 由 URDF importer 生成，需看实际 stage |
| 轮固定方位/侧滑 | 前后同侧四轮均不转向；yaw 必须横向 scrub | 只有两个驱动接地点，caster 球可滑/滚 | 两驱动轮，caster 有 swivel 与轮轴自由度 | 四轮固定方位；没有专用 tire/slip API，靠 rigid contact 的有限切向摩擦允许 scrub | 未定义 skid-steer tire 模型；仍是 URDF 刚体轮接触 |
| 轮材质 static/dynamic、combine | **有效绑定未证实**；URDF Gazebo `0.5/0.5` 不等于 USD 绑定 | 驱动轮 `1/1`，combine `max`；caster `0/0`、`min` | `wheel_material` `1/1`；实际 collider 绑定需逐级确认 | 4 个 collider 显式绑定 `/jackal/PhysicsMaterials/wheels`，`0.2/0.2`，combine 未显式设置 | 代码提取 URDF Gazebo `mu1/mu2` 并拟绑平均值到 collider；地面 `1/1`、combine `min`；绑定结果需核实 |
| 接触/rest offset | 229 未采集；不可从 URDF 推出 | 部分 collider 显式 offset；轮有效值需查 stage | 未证实 | 轮 collider 未显式 author；采用 PhysX 有效默认值，未在本轮量化 | 未显式配置轮 offset |
| 轮质量/惯量 | URDF 各 .477 kg；旧 importer `import_inertia_tensor=False`，实际惯量未采集 | 各 .25 kg；USD 预制 | 驱动轮各 2.22 kg；USD 预制 | 各约 .477 kg；chassis collision mass 17 kg；USD 已 author | `import_inertia_tensor=True`，实际结果仍应核实 |
| articulation root/层级 | `/World/jackal/base_link`；四轮 child rigid body | `/jetbot`；chassis + 两轮 | `/nova_carter/chassis_link`；两轮 + caster 子系统 | `/jackal`；base_link + 四轮 | 解析导入 articulation root 给 controller |
| wheel drive | `acceleration`；stiffness 0，damping 1000，maxForce 3.4e38 | `force`；stiffness 0，damping 174.53，maxForce inf | `force`；stiffness 0，damping 1e6，maxForce inf | `force`；stiffness 0，damping 1e7，maxForce inf | import `default_drive_type=2`；velocity drive 设 stiffness 0、damping 1e4；仍需查 USD 实际 type/maxForce |
| solver/scene | probe 构建 `World`、`1/60 s`；solver iterations 和 friction type 未采集 | articulation velocity iteration 8；scene TGS/patch | scene 和 solver 未证实 | articulation 32 position/16 velocity；scene TGS、stabilization on | `World()`/ground plane；有效 timestep/iterations 需运行时取证 |
| 控制目标/频率 | 两图各控制一对轮，playback tick；229 目标正确 | 官方 Python `apply_wheel_actions` 示例；测试 OmniGraph 两轮图，per update | 测试 OmniGraph DifferentialController → ArticulationController，两轮，per update | USD 只有驱动关节，**没有现成 cmd_vel 控制证明** | 外部 controller_manager 算关节目标，Isaac topic bridge 每 tick 应用；ROS CM update rate 要看外部配置 |

Jetbot [官方 5.1 移动控制器示例](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_simulation/mobile_robot_controllers.html) 与本机 `test_jetbot.py` 说明标准两轮 DifferentialController/Articulation 路径能被设计为原地旋转测试；Nova Carter 的本机 `test_carter_v2.py` 在接地平面上断言 yaw rate 与命令相差不超过 0.1 rad/s。它们的 caster/两驱动轮拓扑降低横向 scrub 约束，不能用来证明四轮模型只需换 controller。官方 5.1 [资产目录](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/assets/usd_assets_robots.html)列出 Clearpath Jackal 的四关节 USD；上述物理值直接从本机 5.1 USD 只读解析。

## 官方 Arena 5.1 adapter 的控制边界

本机独立 [官方分支快照]([LOCAL_PATH]) 与 [Arena-Rosnav/arena-isaac](https://github.com/Arena-Rosnav/arena-isaac/tree/arena5-isaac5.1.0) 对应。`SpawnUrdf.py` 在进入 Isaac 时 sanitize URDF，使用 `URDFParseAndImportFile`；与旧版一样保留非固定 base、关闭 self collision、`default_drive_type=2`，但开启 `import_inertia_tensor=True`。它读 URDF 中 `<gazebo reference=...><mu1>/<mu2>`，将两者平均为 **各向同性** static/dynamic friction，然后尝试给轮 collider 绑 PhysicsMaterial。它无法表达 URDF `fdir1` 指定的方向，也不等于解决 skid-steer 横向滑移。材质绑定代码查的是绝对路径 `/colliders/{link_name}`，只有该 prim 存在才绑定；在导入后移动到 `/World/{name}` 的执行顺序下，**必须查 stage 才能确认命中**，本轮不把该代码视为已验证的物理修复。

Isaac 5.1 自带 URDF importer 测试 `exts/isaacsim.asset.importer.urdf/isaacsim/asset/importer/urdf/tests/test_urdf.py` 检查的典型碰撞路径是 `/robot/link/collisions`；官方 Arena 分支使用的 `/colliders/{link_name}` 与之不一致。这使材质绑定 **高度可疑**：该代码会在 prim 不存在时直接 `continue`，并不报错。本轮没有官方 Arena stage 快照，因此结论仍是“很可能跳过”，不是“已经证实所有场景都跳过”。不能把该分支的材质代码当作已经验证的 Jackal 接地修复。

`Control.parse()` 扫描传入 URDF 的 `<ros2_control>` command interface。外部 controller_manager/`JointStateTopicSystem` 把 `cmd_vel` 变成有 name[] 和 velocity[] 的关节命令；Isaac 的 `topic_bridge.py` 用 `ROS2SubscribeJointState` 接该数组，交给 `IsaacArticulationController`。分支没有旧的两套 `DifferentialController` pairwise 图；`cmd_vel_topic` 仅用于派生 topic namespace，Isaac 侧不直接订阅 Twist。轮接地物理仍由导入的 USD/PhysX articulation、collision、material 和 scene 负责。分支本身没有可证实已转向正常的 Jackal/Husky 运行数据，也没有现成专用 lateral-slip 轮胎模型。

可独立借鉴的内容：`_extract_gazebo_physics` 的 URDF 摩擦意图提取、`Material.physics(...).bind_to(...)` 的物理材质机制、驱动 USD 属性与关节名逐项检查；其中 collider 路径必须以 **实际 stage** 为准。分类：轮材质绑定概念 `CAN_BACKPORT_ONE_PHYSICS_FIX`（先 A/B）；force/damping drive 对照 `CAN_BACKPORT_ONE_PHYSICS_FIX`（后续独立验证）；topic bridge/外部 controller_manager `CAN_BACKPORT_CONTROL_COMPONENT`，但不解释 229；整套迁移当前为 `NOT_RELEVANT` 于首个接地故障，不能推成 `REQUIRES_ARCHITECTURE_MIGRATION`。

## 最多三项有因果价值的差异及最小实验

1. **四轮 collider 的有效摩擦和材质绑定。** 官方 5.1 Jackal USD 的四个 collider 是与当前 URDF 同半径、同宽度、同四轮布局的刚性圆柱，却显式绑定 `0.2/0.2` 物理材质；当前旧 importer 无该绑定，229 也没采集有效值。四轮原地 yaw 需要前后接地点横向滑移，过大接触摩擦能同时解释 yaw 受抑和关节受载。最小实验：先读取每轮 USD collider、material binding 及有效 friction/combine/contact；若当前确实不同，仅在独立 probe stage 对四轮轮材质做 `0.2/0.2` 单变量 A/B。须同时检查 yaw 与 wheel-target error；仅 yaw 变大但轮速/轨迹不稳定不算通过。NVIDIA [Robot Setup Troubleshooting](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup/troubleshooting.html) 也明确提示差速机器人摩擦过低会打滑、过高会造成异常运动。
2. **接地负载下的关节 drive。** 当前 `acceleration/1000`，官方 Jackal `force/1e7`，两者不能简单按数值倍数比较。先保留原材质/几何，只复制 **drive type 语义**（必要时配套官方 drive 属性）到临时 stage 做第二组 A/B，观察四轮速度残差和 yaw。若低摩擦 alone 改善 yaw 但仍跟踪差，该实验更有针对性。不能直接把 1e7 抄进 live adapter；需要排除不稳定、大力矩或轮地穿透。
3. **四轮固定方位接地约束与双轮加 caster 的差别。** Jetbot/Nova Carter 的旋转断言是两驱动轮加可滑动或可转向 caster 的正例，不能外推到四个圆柱轮全部接地的 Jackal。若材质 A/B 仍无法分辨 lateral scrub，可在独立诊断 stage 暂时让一轴轮不接地，并保持另外两轮目标和地面不变；比较两轮接地与四轮接地的 yaw/轮速变化。这只用于证实约束来源，不是可回移的机器人结构修改。官方 Jackal 的 32/16 solver iterations 可作为后续低优先级读数对照；当前 229 未采集，不能声称低于它。

不列为主要差异：轮轴表面 API 名称 `X`/`Y` 不同，但 USD localRot 映射到同一世界轮轴；官方 Jackal 的轮半径、宽度和前后/左右位置均与当前模型一致。Gazebo `mu1/mu2/fdir1` 标签和 PhysX tire lateral slip API 不是同一种物理模型。官方 Jackal USD **没有**特殊 skid-steer tire API，靠轮接触材料允许有限侧滑；Jetbot/Nova Carter 用两驱动轮加 caster，不能作为四轮高摩擦刚性轮转向的正例。

## 文件与复核入口

- 当前：`ARENA_CONTROL_CHAIN_BLOCKER_20261002.md`；`scripts/validation/arena_control_chain/probe.py`；229 `probe.json`；installed Jackal URDF/control.yaml（路径见上）。
- Isaac 5.1 本机：`[LOCAL_PATH]`；`[LOCAL_PATH]`、`test_carter_v2.py`；`[LOCAL_PATH]`、`NVIDIA/NovaCarter/nova_carter.usd`、`Clearpath/Jackal/jackal.usd`（Jackal SHA-256 `be499d8ed3c83deff8a1ce43dc2976ba2b1a8cc66eae80360a19f6d8bb8720ec`）。
- 官方 Arena 5.1：`arena_isaac/arena_isaac/services/SpawnUrdf.py`；`arena_isaac/isaac_utils/graphs/control/__init__.py`、`topic_bridge.py`；`arena_isaac/arena_isaac/run_isaacsim.py`，均位于上述独立快照内。
- 上游：NVIDIA [Robot Assets 5.1](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/assets/usd_assets_robots.html)、[Mobile Robot Controllers 5.1](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_simulation/mobile_robot_controllers.html)、[Robot Setup Troubleshooting 5.1](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup/troubleshooting.html)、[Physics Materials 5.1](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.core.experimental.materials/docs/index.html)、[Clearpath Jackal 上游](https://github.com/jackal/jackal)。

`OFFICIAL_ADAPTER_MIGRATION_NOW = NO`。只有单项物理修正无法移植、并有同场景运行证据证明官方控制架构解决接地故障时，才值得另议迁移；目前条件不成立。
