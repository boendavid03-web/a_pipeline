# Arena 5 原始场景与行人路径核对（2026-09-29）

## 结论与范围

以当前 Arena 入口仓库的 `.repos/arena.repos` 为版本基准：场景来源是 `voshch/arena-simulation-setup` 的 `humble-fix@3f142b2`，而 Isaac 适配器来源是 `Arena-Rosnav/arena-isaac`。对 `factory`、`generated`、`hospital`、`house17`、`ignc`、`map_empty` 六个正式世界执行 Git blob 哈希核对，本机源码 87/87 个文件与指定上游提交完全一致。因此先前转换的 `factory.world`、`hospital.world` 和 `ignc.dae` 确实源于**这个固定版本的 Arena 场景**；转换所得 USD 是本机派生物，不是上游预置的“原版 Isaac USD”。

此处“原版”采用入口所固定的代码和数据版本，不能把 Arena-Rosnav 官网当前 `master` 或 `.old/` 历史场景混入验收集。Arena 5 论文介绍了跨仿真器的生成与定制场景、HuNavSim / 社会力模型，但论文展示本身不等于每个场景都已在本机 Isaac 5.1 完整运行。[Arena 5 论文与项目页](https://5.arena-rosnav.org/)，[固定场景源码](https://github.com/voshch/arena-simulation-setup/tree/3f142b25d88ce962c803b57cf20f38985d376dea/worlds)。

## 场景来源与当前门禁

| 场景 | 固定上游源码哈希 | 当前 Isaac 场景证据 | 尚缺的验收 |
|---|---:|---|---|
| `factory` | 9/9 | 原 `factory.world` 导出；66 模型、175 visual、209 collision，GUI stage 加载；独立 PhysX 落体抽样通过 | Nav2 到达与全部碰撞体接触测试 |
| `hospital` | 15/15 | 原 `hospital.world` 导出；178 模型、178 visual、178 collision，GUI stage 加载；独立 PhysX 落体抽样通过 | 两张原 MTL 所指 TGA 法线贴图缺失，完整视觉未通过；原文件 3 个极深地下模型的语义；Nav2 到达；全部碰撞体接触测试 |
| `ignc` | 8/8 | 原 `ignc.dae` 主建筑和默认 3 书架进入 stage；修复后无行人复测无嵌套刚体错误；建筑落体抽样通过 | 全部碰撞体接触与 Nav2 到达未验收 |
| `house17` | 18/18 | 原地图 62 墙、默认 3 书架进入 stage；墙体阻挡及 3/3 书架 PhysX 射线抽样通过 | 无单一原版 world USD；全部碰撞体接触未验收 |
| `map_empty` | 20/20 | 原地图 4 墙、恢复后的原静态书架进入 stage；墙体阻挡及 1/1 书架 PhysX 射线抽样通过 | 全部碰撞体接触未验收 |
| `generated` | 17/17 | 原 55 墙与 scenario 存在；完整障碍加载在首个家具处失败 | 7 种原名家具没有可导入模型，故完整场景未通过 |

`generated/map/obstacles.yaml` 的 12 种模型名中，本机固定源码只有 `Bookshelf`、`Chair`、`Desk`、`Sofa`、`Table` 的精确同名 SDF。缺 `CoffeTable`、`KidsBed`、`L-ShapedSofa`、`LazySofa`、`LoveseatSofa`、`Multi-SeatSofa`、`Shelf`。`CoffeeTable`、`SquareShelf` 等相似模型存在，但直接替换会改变原场景，不能算原版加载。查过固定上游提交的 `entities/` 与 `gazebo_models/` 文件树、本机源码与 Downloads 中对应 ZIP，均无这 7 种精确命名的模型资产。上游另有独立模型库的下载脚本，但它针对另一仓库版本，包含安装、删除操作，不能直接执行并当作这个固定场景的资产闭包。[上游模型下载脚本](https://github.com/Arena-Rosnav/arena-simulation-setup/blob/master/down_db_asset.sh)。

先前的“几何通过”只代表 stage 中的期望节点和 `CollisionAPI` 数量相符。修复前无行人日志明确报书架根与其 `link` 同时启用 `RigidBodyAPI`；修复后同三场景的日志中该错误为零，stage 节点与碰撞 API 计数保持不变。新增独立 PhysX 抽样已覆盖三份主场景的落体接触，以及两个地图场景的墙体阻挡和四个书架的射线命中；它仍未覆盖每个模型的动态接触。原始日志在 `/home/user/arena_isaac5_py311_factory/scene_geometry_20260929/`。没有因人数或位移给任何场景升级几何结论。

复验源码身份与缺失模型的命令：

```bash
python3 isaac_sim/backends/isaac5/runtime/verify_arena5_original_scenes.py \
  --upstream /tmp/arena5_simsetup_upstream_20260929 \
  --local isaac_sim/arena_ws/src/arena/simulation-setup
```

脚本要求上游 checkout 的 HEAD 为 `3f142b2`，输出每个世界的哈希核对和 `generated` 的精确模型名。它是来源检查，不能替代 Isaac 运行与物理检查。

## Arena 5 行人控制的三个版本边界

1. **Arena 5 研究设计**：论文描述 HuNavSim 和社会力模型经 Entity Manager 管理行为，Isaac 负责人物显示、动画和环境。它没有把一份论文图示等同于本机可运行的 Isaac 5.1 实现。[Arena 5 论文](https://5.arena-rosnav.org/arena5.pdf)。
2. **本机 Arena 入口默认值**：`arena.launch.py` 对 `sim:=isaac` 的 `human` 默认值是 `hunav`；`human.launch.py` 为 HuNav 启动管理器。本机 `/home/user/arena_isaac5_host_runtime/run_gui.bash` 明确覆盖为 `human:=isaac`，改由 `task_generator/.../human/isaac.py` 发 Isaac People/NavMesh 的位置目标。这解释了已有 10/17 人运动，但**那些记录不能作为原版 HuNav / LightSFM 控制验收**。
3. **官方 `arena5-isaac5.1.0` 适配分支**：2026-06-20 的 `16b8e341` 分支在 Isaac 端使用 `arena_people_msgs` 的 Spawn / Move / Update 服务；`UpdatePedestrians` 接收外部 pose + velocity，`Person.update_command` 通过 Animation Graph 执行更新。适配器是接受控制命令的执行端，不独自证明 HuNav/SFM 已接入。本机缺 `arena_people_msgs` 的消息/服务定义及 ROS 类型支持，原分支尚不能直接替换当前运行链。[官方 Isaac 适配仓库](https://github.com/Arena-Rosnav/arena-isaac)。

后续行人验收采用单一位姿权威：先找回/确认 Arena 5 原版 HuNav 计算链及其消息定义，再将外部 pose + velocity 送入 Isaac 执行与动画；不能同时让 Isaac People NavMesh 自主寻路和 HuNav 更新同一人物。原版 Gazebo 插件说明了 `arena_people_msgs` 可承载外部行人状态，但它本身不是 Isaac 适配器。[Arena 人物消息与 Gazebo 插件](https://github.com/Arena-Rosnav/gz_arena_human_plugin)。

## 优先顺序与最新复验

**2026-09-30 补充：**已从同一固定上游提交核对 `hospital` 原模型目录中现存的两张 PNG 法线图，制作独立的来源可追溯 USD 派生版，原 MTL 与原 USD 不变。派生版依赖缺失数为 0，独立 PhysX 落体通过，Arena GUI 无行人场景加载为 `178/178/178`，所属进程已回收；见 [完整场景门禁](ARENA_ISAAC_FULL_SCENE_GATE_20260929.md)。原 MTL 指向的 TGA 仍缺失，PNG 与 TGA 的视觉等价未知；GUI 日志中另有 PhysX tensor view 和 RTX LiDAR 错误，不能升级为完整动态物理或机器人传感器验收。`generated` 的 7 种原名资产缺口未变。

1. 追溯 `generated` 的 **7 种原名家具**与 `hospital` 的 **2 张源 MTL 贴图**，在来源明确后补齐并复验六场景的 visual、collision、PhysX 和 GUI 导入门禁；找不到则保留各自未通过状态。
2. 书架 USD 的嵌套刚体和材质作用域已在隔离 overlay 修复；后续只需扩大真实碰撞面与视觉抽样，保留原场景位置与几何。
3. 完整场景门禁后，再接 Arena 原版行人计算与 Isaac pose/animation 边界，采集每人连续轨迹和控制来源；Nav2/social benchmark 独立验收。

已完成的隔离修复：`shelf.usd` 的根 `/bookshelf` 移除 `RigidBodyAPI`，子节点 `/bookshelf/link` 的刚体与碰撞保留；并将原 `/Looks` 下的 14 个材质及内部连接移入 `/bookshelf/Looks`，使整套书架作为引用进入场景时可解析材质。原始文件备份在 `/home/user/arena_isaac5_py311_factory/scene_geometry_20260929/physics_fix/shelf.original.usd`，当前生成物为同目录的 `shelf.physics_materials_fixed.usd`，可由 [repair_arena_shelf_usd.py](isaac_sim/backends/isaac5/runtime/repair_arena_shelf_usd.py) 重现。独立 USD 引用探针确认 7/7 可视部件绑定到本体内材质。

修复后用独立 ROS domain 203–205 顺序复跑三个无行人场景：`map_empty` 为墙 `4/4`、书架 `1/1`、碰撞 API `14`；`house17` 为墙 `62/62`、书架 `3/3`、障碍碰撞 API `42`；`ignc` 为书架 `3/3`、障碍碰撞 API `42`。三份日志的嵌套刚体错误和材质引用越界警告均为 `0`，回调异常 `0`，进程组均正常停止。结构化证据为 [/home/user/arena_isaac5_py311_factory/scene_geometry_20260929/shelf_material_final/summary.json](/home/user/arena_isaac5_py311_factory/scene_geometry_20260929/shelf_material_final/summary.json)，同目录有各场景原始日志。这仍不等于动态接触/阻挡或逐像素视觉验收。

随后以独立 Isaac 5.1 PhysX stage 执行无行人接触抽样：`factory`、`hospital`、`ignc` 的方块分别被原场景衍生 USD 的实际碰撞面托住；`map_empty`、`house17` 的方块被按原墙线生成的墙体挡住，其书架分别 1/1、3/3 被 PhysX 射线命中。五份**物理抽样**均通过，但依赖扫描发现 `hospital` 的 `BedTable_Normal_Normal_Bump.tga`、`BP_Normal_Normal_Bump.tga` 缺失，故含视觉依赖的总结果为 4 PASS、1 FAIL。两张 TGA 是固定源码中的原 MTL 明确引用，源码和本地目录均没有文件；虽然目录有不同名字的 PNG 法线图，尚未证明可以等价替换。结构化 JSON 在 `/home/user/arena_isaac5_py311_factory/scene_geometry_20260929/physics_probe/`，可复现脚本为 [probe_arena_original_scene_physics.py](isaac_sim/backends/isaac5/runtime/probe_arena_original_scene_physics.py)。该探针复用相同 USD 或原墙线/书架位置，但并非从 Arena GUI 进程内采样，因此只证明上述选定碰撞位置有效；逐一碰撞体、完整 GUI 动态物理和视觉一致性仍需验收。

`generated` 仍未通过：固定 Arena5 提交与本机归档中缺少 7 种原名模型，禁止用相似家具替换后宣称原版完整。官方原版 HuNav/`arena_people_msgs` 仍排在完整场景门禁之后。

补充资产核查：已只读枚举 `/home/user/Downloads` 中可打开 ZIP 的文件目录，未找到上述 7 种精确模型目录。上游 `down_db_asset.sh` 指向的 Google Drive 文件夹目前从本机访问返回 HTTP 404；脚本自身还会安装工具和删除相邻目录，因此没有执行。当前缺口应保留为“原资产未取得”，不能由在线脚本的存在推断可用。

## 最近工作状态核对

2026-09-29 在线核对 GitHub 分支头：官方 `Arena-Rosnav/arena-isaac` 的 `arena5-isaac5.1.0` 仍为 `16b8e341`（2026-06-20），与本机归档一致；`isaac6.0.0` 已到 `1445c751`（2026-09-21），属于另一条版本线。Arena 入口固定的 `voshch/arena-simulation-setup` `humble-fix` 仍为 `3f142b2`；`Arena-Rosnav/arena-simulation-setup` 的 `master` 为 `55fb308`（2025-01-29）。这些近期分支状态没有提供 `generated` 七种原名资产或本机缺失的 `arena_people_msgs` 闭环证据。[Isaac 5.1 分支](https://github.com/Arena-Rosnav/arena-isaac/tree/arena5-isaac5.1.0)、[Isaac 6 分支](https://github.com/Arena-Rosnav/arena-isaac/tree/isaac6.0.0)、[固定场景分支](https://github.com/voshch/arena-simulation-setup/tree/humble-fix)。

本机最新记录增加了五个无行人的独立 PhysX 抽样通过，但 `hospital` 的源贴图依赖缺失使其视觉资产门禁失败；此前 Arena GUI 三场景复测均为 `GEOMETRY_COUNT_PASS`、`people_created=0`、`teardown=stopped`。前一轮 `map_empty` 矩阵内一次记录因脚本在 scenario-ready 标记写出前停止而记为 `INCOMPLETE`，不能混作最新结果。受保护的 `arena-isaac` checkout 仍有 3 个既有修改文件，本次没有切换或清理。
