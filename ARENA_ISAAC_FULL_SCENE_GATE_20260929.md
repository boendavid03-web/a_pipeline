# Arena + Isaac 5.1：先验完整场景的几何门禁

日期：2026-09-29。范围是当前 `arena-simulation-setup/worlds/` 下的六个正式场景：`factory`、`generated`、`hospital`、`house17`、`ignc`、`map_empty`。`map_highly_social` 是本机 overlay 变体；`.old/` 中的历史地图另列为未验范围。按用户要求，本门禁只核对原场景的模型、墙体、静态障碍和碰撞，不用行人运动替代场景完成状态。

**来源和物理状态补充：**六个世界的 87 个源码文件与 Arena 入口 `.repos` 固定的 `voshch/arena-simulation-setup@3f142b2` 完全一致，见 [Arena 5 原版核对](ARENA5_ORIGINAL_SCENES_AND_PEOPLE_20260929.md)。下表的“几何通过”仅代表节点和碰撞 API 计数，不代表完整 PhysX 动态接触验收。此前 `map_empty`、`house17`、`ignc` 的书架嵌套刚体错误已通过隔离 overlay 修复，最新无行人日志中该错误为零。

## 当前状态

**2026-09-30 `hospital` 来源贴图派生版：**固定上游提交的 `BedTable/meshes/BedTable_Normal.png` 与 `BPCart/meshes/BP_BP_LP_Normal.png` 均有可核对的 Git blob 和 SHA-256；它们是原目录中现存的法线图，但**无法证明**与原 MTL 所写、实际缺失的两张 TGA 像素等价。脚本 [repair_arena_hospital_source_normals.py](isaac_sim/backends/isaac5/runtime/repair_arena_hospital_source_normals.py) 将两张 PNG 复制到独立派生资产，只改两个材质的法线贴图引用，保留原 `hospital_full_scene.usda` 和原 Arena 源文件。派生版 `hospital_full_scene_source_png.usda` 的 USD 依赖扫描为 `layers=114`、`files=99`、缺失 0；独立 PhysX 落体与射线通过。Arena GUI 无行人加载确认使用该派生版，`models/visuals/collisions=178/178/178`、缺失 TGA 告警 0，测试进程组已停止。结构化证据见 [派生版清单](isaac_sim/backends/isaac5/generated/arena_worlds/hospital/hospital_source_png_manifest.json)、`/home/user/arena_isaac5_py311_factory/scene_geometry_20260929/physics_probe/hospital_source_png.json` 和 `/home/user/arena_isaac5_py311_factory/scene_geometry_20260929/hospital_source_png_gui/result.json`。GUI 日志持续报 PhysX tensor view invalidated 与 RTX LiDAR render product 未附着；因此该次 **GUI 几何加载通过，GUI 内动态物理和传感器仍未通过**。原 USD 的两张 TGA 缺失结论仍有效；派生版不能冒称原 MTL 的完整视觉等价。

**修复更新（22:08–22:10）**：隔离 overlay 的 shelf USD 已去掉根 `/bookshelf` 的 `RigidBodyAPI`，保留 `/bookshelf/link` 刚体及其碰撞。修复后的无行人复验显示 `map_empty` 的 `4/4 + 1/1 + 14` 计数通过、无嵌套刚体错误；`house17` 和 `ignc` 也分别通过 `62/62 + 3/3`、`3/3` 计数并正常回收。原始文件和修复文件均保留在 `/home/user/arena_isaac5_py311_factory/scene_geometry_20260929/physics_fix/`。

**后续材质门禁：**检查上述运行日志时发现原书架 USD 的材质位于 `/Looks`，超出 `/bookshelf` 引用范围，Isaac 忽略了绑定。隔离修复将 14 个材质与其连接纳入 `/bookshelf/Looks`；独立引用测试确认 7/7 可视部件可解析材质。三个场景随后在 ROS domain 203–205 重跑，均为 `GEOMETRY_COUNT_PASS`、无行人、所属进程组已停止；嵌套刚体错误和材质越界警告均为 0。最新证据在 `/home/user/arena_isaac5_py311_factory/scene_geometry_20260929/shelf_material_final/`；修复脚本见 [repair_arena_shelf_usd.py](isaac_sim/backends/isaac5/runtime/repair_arena_shelf_usd.py)。逐像素视觉仍未验收。

**物理抽样与依赖续测：**以 Isaac 5.1 的独立无行人 PhysX stage 重新引用原场景衍生 USD，`factory`、`hospital`、`ignc` 均从实际碰撞面上方释放 0.2 m 方块，120 个物理步后方块分别停在碰撞面以上约 0.10 m。按原墙体端点和原书架位置构造的 `map_empty`、`house17` stage 中，方块以 2 m/s 撞向墙体，最大前进约 0.275 m，小于射线测得的 0.375 m 墙距；两场景的 1/1、3/3 个原书架也被 PhysX 射线命中。五项**物理抽样**均通过。USD 依赖检查额外发现 `hospital` 的两张 TGA 法线贴图缺失，因此结合视觉资产门禁的总结果为 **4 PASS、1 FAIL**。结构化结果见 `/home/user/arena_isaac5_py311_factory/scene_geometry_20260929/physics_probe/*.json`，探针脚本见 [probe_arena_original_scene_physics.py](isaac_sim/backends/isaac5/runtime/probe_arena_original_scene_physics.py)。这是代表性接触与碰撞体可查询性验证；尚未逐一覆盖全部碰撞体，也不是 Arena GUI 内的动态接触验收。

`hospital` 的缺失依赖是 `BedTable_Normal_Normal_Bump.tga` 与 `BP_Normal_Normal_Bump.tga`。它们分别写在固定 Arena 源文件 `gazebo_models/BedTable/meshes/BedTable.mtl` 与 `gazebo_models/BPCart/meshes/Cart_BP.mtl` 中，但固定源码、本机模型目录和已生成 USD 引用路径都没有对应 TGA。原目录另有不同名字的 PNG 法线图，未经来源对应核对，不能悄悄改绑后称原版视觉通过。Isaac 安装内的 `OmniPBR.mdl`、`OmniPBR_Opacity.mdl` 虽被 USD 通用依赖扫描器列为 unresolved module，实际文件位于 Isaac 5.1 的 `kit/mdl/core/Base/`，探针单独标记为 Kit 内置模块，不计入缺失场景资产。

独立探针使用无 RTX 的 headless Kit 配置，原始日志有 `omni.gpu_foundation_factory.plugin` 默认图形插件启动报错；其后 PhysX 射线和 120 步接触仍完成。因而这里的物理结果不构成该探针的 GUI/渲染通过证据，GUI 加载结论沿用前述 Arena 入口复测。


| 原 Arena 场景 | 完整场景来源与期望 | Isaac 加载证据 | 结论 |
|---|---|---|---|
| `factory` | `factory.world`：66 顶层模型、175 visual、209 带几何的 collision | 生成日志和 GUI stage 均为 `66/175/209`；静态 USD 组合可打开 | **几何通过**；详见 [工厂验证](ARENA_ISAAC_FACTORY_10P_VALIDATION.md) |
| `hospital` | `hospital.world`：178 顶层模型、178 visual、178 collision | 生成日志和 GUI stage 均为 `178/178/178`；静态组合有 511 Mesh、251 碰撞 API；落体接触抽样通过 | **几何通过、完整视觉未通过**：两张源 MTL 引用的 TGA 缺失；另有 3 个原 SDF 模型埋于 `z=-7.57769e+08`，详见 [续测](ARENA_ISAAC_SCENE_GEOMETRY_CONTINUATION_20260929.md) |
| `ignc` | 原 `ignc.dae` 的主建筑 visual/collision；原 world 还 include `ground_plane`；默认 scenario 有 3 个静态书架 | 建筑 stage `1/1/1`；修复后无行人检查中书架 `3/3`、碰撞 API 42，嵌套刚体错误 0 | **当前默认场景几何通过** |
| `house17` | 原目录没有完整 `.world`；场景由 2D 地图、62 段 `walls.yaml` 墙体及默认 scenario 的 3 个书架构成 | 修复后无行人运行：墙体 `62/62`、墙碰撞 `62`；书架 `3/3`、障碍碰撞 API 42，嵌套刚体错误 0 | **当前默认场景几何通过**；它本来就是地图描述，不是整栋预制 USD |
| `map_empty` | 4 段墙、`obstacles.yaml` 中 1 个书架 | 修复后最终无行人复测为墙 `4/4`、墙碰撞 4、书架 `1/1`、障碍碰撞 API 14，场景参数生效，嵌套刚体错误 0 | **当前默认场景几何通过** |
| `generated` | 55 段墙、`obstacles.yaml` 的 15 个静态家具；默认 scenario 另有 5 个书架 | 之前的 Arena world callback 在第一件 `CoffeTable` 上报找不到 Isaac USD；未形成全部 20 个静态障碍的 stage 证据 | **未通过**，不可称完整场景已加载 |

`generated` 的 15 个 world 障碍共使用 12 种模型名。当前本地 `gazebo_models` 中只有 `Bookshelf`、`Chair`、`Desk`、`Sofa`、`Table` 这 5 种精确同名的 `model.sdf`；`CoffeTable`、`KidsBed`、`L-ShapedSofa`、`LazySofa`、`LoveseatSofa`、`Multi-SeatSofa`、`Shelf` 没有精确同名 SDF。运行 overlay 的静态实体仅有小写 `shelf`。本机相关 Arena ZIP 的文件目录也未找到前述六个特殊拼写的家具名。把 `CoffeeTable` 当成 `CoffeTable` 或用任意家具替代，只能生成近似场景，不能作为原场景完整导入的证据。

## 本轮场景专用检查

场景检查用独立 ROS domain 198、199、200、201、202 顺序运行，只停止本轮创建的进程组。外置 cp311 `run_isaacsim.py` 在设置 `ARENA_GEOMETRY_EXPECT_WALLS` 和 `ARENA_GEOMETRY_EXPECT_OBSTACLES` 时输出 stage 子节点与碰撞 API 计数。场景专用 JSON 在外置测试目录中将 `obstacles.dynamic` 设为空；三次通过运行均记录 `people_created=0`。原 Arena world、scenario 文件没有修改。

- 结构化结果与原始日志：`/home/user/arena_isaac5_py311_factory/scene_geometry_20260929/` 下的 `house17/`、`ignc/`、`map_empty_restored_final/map_empty/`。各有 `result.json` 与 `console.log`，结果均为 `GEOMETRY_COUNT_PASS`、`teardown=stopped`。
- `map_empty` 初测的 `0/1` 和恢复后第一次过早收尾的结果仍原样保留；最终复测同时确认了场景参数与 `4/4 + 1/1` 几何计数。
- 被恢复的文件是隔离 host overlay 的 `worlds/map_empty/map/obstacles.yaml`。修改前版本备份为 `/home/user/arena_isaac5_py311_factory/scene_geometry_20260929/map_empty_restore/obstacles_overlay_before.yaml`；当前内容与原 Arena 源文件相同。
- `factory`、`hospital`、`ignc` 的整场景 USD 是本机生成资产，位于被 Git 忽略的 `isaac_sim/backends/isaac5/generated/arena_worlds/`，迁移机器时须连同引用网格一起保留。

本表是**场景几何门禁**，物理续测仅覆盖代表性碰撞面和墙体，不包含 Nav2 到达、逐像素视觉、全部碰撞体的动态接触、人物行为或社会导航验收。`generated` 的 7 种原家具和 `hospital` 的两张源贴图均需明确来源；其余场景可继续更完整的物理和 GUI 验证。历史 `.old/` 地图尚未纳入本轮逐场加载，不能算作全部场景已完成。
