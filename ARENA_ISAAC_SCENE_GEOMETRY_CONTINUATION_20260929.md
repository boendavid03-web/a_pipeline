# Arena 原始三维场景继续验证：ignc 与 Hospital

日期：2026-09-29。延续 [基础场景矩阵](ARENA_ISAAC_BASIC_SCENE_MATRIX_20260929.md)，使用原 Arena launch、现有 Host/Isaac 3.10/3.11 overlay 和 Isaac Sim 5.1。只停止本轮启动的进程组；未切换或清理活动 `arena-isaac` checkout。

## 实测结果

| Arena 场景 | 原始三维来源及 Isaac stage | Arena/机器人/人物短测 | 尚未通过 |
|---|---|---|---|
| `ignc/default` | 原 `ignc.dae` 转为 USD；1 模型、1 visual、1 collision。静态 stage 有 2 个 Mesh 和 1 个碰撞 API；建筑边界约 `x=-12.55…12.45 m, y=-12.45…12.55 m, z=-1…4.19 m` | Arena GUI 加载，Jackal 里程计收到；3/3 人创建，5.31 s 内每人净位移 0.288–0.393 m | Nav2 目标到达 |
| `hospital/default`，楼板及墙体隔离试验 | 原 `hospital.world` 中的楼板、墙体 2 个模型；2 visual、2 collision | Arena GUI 加载，Jackal 里程计收到；17/17 人创建，8.26 s 内每人净位移 0.312–1.878 m | 家具未在该隔离试验加载；Nav2 目标到达 |
| `hospital/default`，完整原场景 | 原 `hospital.world` 的全部 **178 个顶层模型**及 113 个唯一 mesh URI；stage 计数为 **178 visual、178 collision**。静态组合 stage 有 511 个 Mesh、251 个碰撞 API | Arena GUI 加载，Jackal 里程计收到；17/17 人创建并发出路线批次，8.33 s 内每人净位移 **0.360–1.957 m**。基础链 `BASIC_PASS` | 日志有 `Goal failed`；无 Nav2 到达、碰撞距离或长期社会导航验收 |

完整 Hospital USD 的边界最低约 `-7.58e8 m`，原因来自**原 Arena `hospital.world`**：`StorageRack_1`、`WhiteChipChair_1` 和 `TrolleyBed_1` 的原 include pose 的 z 坐标就是 `-7.57769e+08`。这三个模型按源文件保持在地下，并非转换器把可见家具移走；因此 178 是完整源模型计数，不能解释为 178 个模型都在建筑可见区域。静态边界异常可在 `hospital_full_bound_audit.log` 与原 world 第 161、217、224 行复核。

这些位移来自 `/isaac/get_prim_attributes` 的两次服务采样，按父 prim 加 SkeletonRoot 局部位姿计算世界坐标净位移。它证明短时运动，不证明无碰撞、全程不冻结或导航任务完成。Hospital 完整场景与外壳场景都记录到 Nav2 `Goal failed`；原 Arena Hospital 在 Gazebo 的导航结果不能代替这里的 Isaac 结果。

## 产物和复现

- 转换器：[build_arena_factory_scene.py](isaac_sim/backends/isaac5/runtime/build_arena_factory_scene.py) 扩展了原 factory 路线，支持直接 SDF model、`model://` include 和 `--include-name`。未修改原 Arena world/SDF。
- `ignc`：[ignc_scene.usda](isaac_sim/backends/isaac5/generated/arena_worlds/ignc/ignc_scene.usda)，引用转换后的 `ignc_mesh.usd`。
- Hospital：[hospital_full_scene.usda](isaac_sim/backends/isaac5/generated/arena_worlds/hospital/hospital_full_scene.usda) 与用于隔离试验的 `hospital_shell_scene.usda`；网格资产在同目录 `meshes/`。
- 外置 `/home/user/arena_isaac5_host_runtime/run_gui.bash` 与 cp311 `run_isaacsim.py` 已配置按 `ARENA_WORLD=ignc|hospital|factory` 自动选择对应 USD。**运行验证时 Hospital 显式传入了同一 `ARENA_WORLD_USD` 路径**；自动选择分支已做语法检查和源代码检查，尚未单独重启验证。
- 原始运行证据：`/home/user/arena_isaac5_py311_factory/ignc_original_scene_20260929/`、`hospital_shell_20260929/`、`hospital_full_20260929/`。后两者各有 `result.json`、`people_positions.json`、`odom_probe.txt` 和 `console.log`。完整 Hospital 运行使用独立 ROS domain 197，PGID 486834，报告 `teardown=stopped`。
- 生成及静态检查日志：`/home/user/arena_isaac5_py311_factory/hospital_full_build.log`、`hospital_full_static_check.log`、`hospital_full_bound_audit.log`。完整 Hospital 构建报告 `ARENA_SDF_WORLD_PASS models=178 visuals=178 collisions=178`。
- 这些 USD 是本机生成产物，位于 `isaac_sim/backends/isaac5/generated/`，该目录被 `.gitignore` 排除；源代码和本报告本身不能在另一台机器上直接替代约 71 MB Hospital、24 MB ignc 的资产。

可复跑完整医院的入口参数：

```bash
ARENA_WORLD=hospital \
ARENA_TM_OBSTACLES=scenario \
ARENA_TM_ROBOTS=scenario \
ARENA_SCENARIO_FILE=/home/user/navigation_project/a_pipeline/isaac_sim/arena_ws/src/arena/simulation-setup/worlds/hospital/scenarios/default.json \
ARENA_WORLD_USD=/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/arena_worlds/hospital/hospital_full_scene.usda \
/home/user/arena_isaac5_host_runtime/run_gui.bash
```

## 切换场景时发现的下一处缺口

`generated` 的 world callback 首先因 `CoffeTable` USD 缺失而失败。原障碍配置共引用 **12 种**家具名；其中只有 `Bookshelf`、`Chair`、`Desk`、`Sofa`、`Table` 在本地 Gazebo 模型目录有精确同名的 `model.sdf`，当前最小 Host overlay 的静态实体目录仅有小写 `shelf`。Gazebo 模型中有 `CoffeeTable`，但与场景中的 `CoffeTable` 拼写不同。补一个别名仍不足以证明 15 个静态障碍全部导入，因此本轮记录缺口后转到其他原场景，没有把 `generated` 标成完整三维通过。
