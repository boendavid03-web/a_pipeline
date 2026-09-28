# Isaac Sim 5.1 最小 custom Kit experience 验证报告

审计日期：2026-09-11  
审计范围：Isaac Sim 5.1 本机安装、`isaacsim.exp.base.python.kit` 依赖、临时 custom `.kit`、空 stage Core/PhysX probe  
执行边界：未修改现有 Isaac 5 backend runtime、未修改 Isaac 安装、未修改系统 NVIDIA driver、未删除 Isaac 6；未加载 robot USD、ROS 2 或传感器。本报告和 `isaac_sim/backends/isaac5/generated/` 下的临时 probe/日志是本次新增实验产物。

## 1. 结论

**Gate 通过：Isaac Sim 5.1 的 Isaac Core + PhysX 可以在不启动 `omni.hydra.rtx` 的 custom experience 中启动空 stage，并完成 PhysX 初始化和 stepping。**

结果摘要：

| 项目 | 结果 |
|---|---|
| custom `.kit` 是否继承 `isaacsim.exp.base` | **否** |
| `omni.hydra.rtx` 是否作为本次 experience 的 startup extension | **否** |
| `librtx.scenedb.plugin.so` 是否被动态加载 | **否；动态链接器日志无匹配记录** |
| Kit app 是否启动到 ready | **是**，`app ready` |
| Isaac Core `World` 是否创建 | **是** |
| PhysX 是否初始化 | **是** |
| 空 stage physics-only stepping | **是，3 次通过** |
| 进程退出码 | **0** |
| 是否加载 robot / ROS / sensor | **否** |
| 是否完全不初始化 GPU/Vulkan | **否**；仍有 GPU Foundation/USDRT delegate，系统信息仍报告 Vulkan |

因此，当前 Gate 1 的新证据把问题边界收窄为：**RTX SceneDB 不是 Isaac Core/PhysX 空 stage 的必需启动条件；原 Gate 1 崩溃属于默认 base experience 的 RTX startup 路径，而不是 Core/PhysX 本身无法启动。**

这不是 robot USD 兼容性通过，也不是 ROS、LiDAR、renderer 或完整导航 backend 通过。

## 2. 被测文件和运行证据

### 2.1 临时 custom experience

文件：[`minimal_core_physx_no_rtx.kit`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/minimal_core_physx_no_rtx.kit:1)

关键设置：

- 只声明 `isaacsim.simulation_app` 和 `isaacsim.core.api` 两个直接依赖（第 8–14 行）。
- 不声明 `isaacsim.exp.base`、`omni.hydra.rtx`、`omni.kit.viewport.bundle` 或 RTX sensor extension。
- 设置 `vulkan = false`、关闭 window，并将 renderer active/enabled 置空（第 16–37 行）。
- 显式使用 Isaac 5.1 安装的 `exts` 与 `extscache`（第 25–32 行），避免把 custom 文件所在实验目录误当成 Isaac extension 根目录。

### 2.2 空 stage probe

文件：[`minimal_core_physx_no_rtx_probe.py`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/minimal_core_physx_no_rtx_probe.py:1)

probe 仅执行以下操作：

1. 用 custom `.kit` 创建 `SimulationApp`，`headless=True`、`multi_gpu=False`。
2. 导入 `isaacsim.core.api.World`，创建空 stage。
3. 调用 `world.initialize_physics()`。
4. 调用 3 次 `world.step(render=False, step_sim=True)`。
5. 正常 `app.close()`。

没有导入当前 `isaac5` backend、robot USD、ROS 2、LiDAR 或任意业务控制代码。

### 2.3 成功运行日志

- Probe stdout/stderr：[`minimal_core_physx_no_rtx_run_retry.log`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/minimal_core_physx_no_rtx_run_retry.log:1)
- Kit 日志：`/home/user/isaacsim/5.1.0/kit/logs/Kit/Isaac 5.1 Minimal Core PhysX No RTX Probe/5.1/kit_20260911_130441.log`
- 动态链接器日志：`isaac_sim/backends/isaac5/generated/minimal_core_physx_no_rtx_lddebug_retry.*`

成功标记位于 probe 日志第 101–110 行：

```text
[2.417s] app ready
MINIMAL_PROBE_APP_STARTED=1
MINIMAL_PROBE_EMPTY_WORLD_CREATED=1
MINIMAL_PROBE_PHYSICS_INITIALIZED=1
MINIMAL_PROBE_STEP=1
MINIMAL_PROBE_STEP=2
MINIMAL_PROBE_STEP=3
MINIMAL_PROBE_RESULT=PASS
```

Kit 随后正常执行 `Simulation App Shutting Down`，并以 shell exit code `0` 结束。

## 3. `isaacsim.exp.base.python.kit` 依赖分析

### 3.1 默认 Python experience 的入口

本机文件 `/home/user/isaacsim/5.1.0/apps/isaacsim.exp.base.python.kit` 第 9–10 行只有一个直接 app 依赖：

```toml
[dependencies]
"isaacsim.exp.base" = {}
```

它同时在第 17 行显式设置 `vulkan = true`，并把 `${app}/../exts` 和 `${app}/../extscache` 作为默认 extension 搜索路径。

真正扩大依赖图的是 `/home/user/isaacsim/5.1.0/apps/isaacsim.exp.base.kit`。该文件同时声明 Isaac Core、sensors、replicator、UI、viewport、physics 和大量 editor extensions；其中第 102–104 行附近明确包含：

```toml
"omni.hydra.engine.stats" = {}
"omni.hydra.rtx" = {}
"omni.kit.mainwindow" = {}
```

因此默认 Python experience 不是“只启动 Python API”，而是继承了带 RTX Hydra 的完整 base app。

### 3.2 RTX extension 为什么会带入 SceneDB

本机 `/home/user/isaacsim/5.1.0/extscache/omni.hydra.rtx-1.0.0+69cbf6ad.lx64.r/config/extension.toml` 记录：

- 第 10–12 行：`reloadable = false`，并注明不能在加载后禁用该 extension。
- 第 14–24 行：声明 `omni.gpu_foundation`、`omni.usd.core`、`omni.hydra.scene_delegate` 等依赖。
- 第 47–70 行：声明 RTX native plugins，其中包括 `carb.scenerenderer-rtx.plugin` 和 `rtx.scenedb.plugin`。

所以原始 Gate 1 使用 default base experience 时，SceneDB 是 RTX Hydra native plugin startup 的一部分；它不是 robot script 创建的可选对象。

### 3.3 Core/PhysX 最小闭包

对本机 extension manifest 的静态依赖闭包检查结果：

```text
root: isaacsim.core.api
resolved hard-dependency closure: 48 extensions
contains omni.hydra.rtx: no
contains omni.hydra.scene_delegate: no
contains omni.physx: yes
contains omni.usd: yes
contains omni.hydra.usdrt_delegate: yes
```

`isaacsim.core.api` 的闭包会带入 `omni.hydra.usdrt_delegate`、`omni.gpu_foundation`、USD 和 PhysX 组件；`omni.hydra.usdrt_delegate` 不等于 `omni.hydra.rtx`，也没有把 `rtx.scenedb.plugin` 引入本次 startup。

custom `.kit` 选择 `isaacsim.core.api`，因此保留了 Core/World/Python API 和 PhysX，而没有继承 base 的 RTX/UI/传感器集合。

## 4. 是否仍加载 RTX SceneDB

### 4.1 Extension startup 记录

成功运行日志的 startup 列表包含：

- `omni.physx.foundation`
- `omni.hydra.usdrt_delegate`
- `omni.physx.cooking`
- `omni.physx`
- `omni.physx.tensors`
- `isaacsim.core.api`

对应证据见 [`minimal_core_physx_no_rtx_run_retry.log`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/minimal_core_physx_no_rtx_run_retry.log:12) 至第 77 行。

该 startup 列表没有：

```text
omni.hydra.rtx
librtx.scenedb.plugin.so
libcarb.scenerenderer-rtx.plugin.so
```

### 4.2 动态链接器记录

本次通过 `LD_DEBUG=libs,files` 保存动态库加载记录，并对所有 retry 文件搜索：

```text
librtx.scenedb.plugin.so       0 matches
libcarb.scenerenderer-rtx.plugin.so  0 matches
libomni.hydra.rtx             0 matches
```

同时可以看到 PhysX native libraries 被加载，例如：

```text
libomni.physx.foundation.plugin.so
libomni.physx.cooking.plugin.so
libomni.physx.plugin.so
libomni.physx.tensors.plugin.so
```

这说明 probe 确实进入了 PhysX native runtime，而不是只停留在 Python import 层。

### 4.3 “registered” 与 “loaded” 的区别

Kit 启动日志在扫描 `/home/user/isaacsim/5.1.0/extscache` 时会登记可发现的 extension，因此能看到 `omni.hydra.rtx-1.0.0` 的 `registered` 行。这只是 extension catalog registration，不是 startup，也不是 native library load。

本次判断以三类证据为准：

1. startup extension 列表没有 `omni.hydra.rtx`；
2. dynamic linker 没有 `librtx.scenedb.plugin.so`；
3. app、Core World、PhysX 和 stepping 均成功。

因此“仍被扫描/登记”不能被误读成“SceneDB 已加载”。

## 5. Vulkan/GPU Foundation 的边界

虽然 custom `.kit` 设置了 `vulkan = false`、空 renderer 和无窗口，本次日志仍记录：

```text
Driver Version: 595.84        | Graphics API: Vulkan
GPU: NVIDIA GeForce RTX 5090
```

并启动了 `omni.gpu_foundation`、`omni.gpucompute.plugins` 和 `omni.hydra.usdrt_delegate`。日志还出现：

```text
[omni.gpu_foundation_factory.plugin] Start up failed. The default graphics plugin cannot be set!
```

该错误没有阻止 app ready、PhysX 初始化或 3 次 physics-only stepping；它是当前“无 renderer delegate”配置下的非致命 graphics setup 报告。

因此本实验的准确结论是：

- **已证明可以绕过 RTX SceneDB native plugin。**
- **未证明可以绕过所有 GPU/Vulkan 初始化。**
- **未证明 PXR/Storm raster 或完全 no-render 兼容。**

## 6. 首次实验配置问题及修正

第一次运行 custom `.kit` 时使用了相对于实验文件的 `${app}/../exts` 和 `${app}/../extscache`。由于 custom 文件位于项目目录，这些路径没有指向 Isaac 5.1 安装，Kit 进入 extension registry fallback 并在运行日志中出现下载动作；该次运行在依赖同步阶段超时，没有进入 Core/PhysX probe。

随后修正为显式的 Isaac 5.1 安装路径，并将：

```text
--/exts/omni.kit.registry.nucleus/enable=false
```

加入 probe，防止缺失本地依赖时悄悄转为网络下载。第二次运行在约 2.5 秒内通过。

这次修正没有改写 `/home/user/isaacsim/5.1.0` 内的文件，也没有修改系统 driver；第一次 registry fallback 产生的外部 extension cache 不属于 Isaac 安装目录，未在本次范围内删除。

## 7. 对 Gate 1 的影响

当前证据支持下面的分层判断：

| 层级 | 结论 |
|---|---|
| Kit custom experience | **通过**：可以创建不继承 base/RTX 的最小 experience |
| Isaac Core Python API | **通过**：`World` 创建成功 |
| USD 空 stage | **通过**：空 stage 创建成功 |
| PhysX native startup | **通过**：PhysX extensions/native libraries 启动，`initialize_physics()` 成功 |
| physics-only stepping | **通过**：3 次 `render=False` step 成功 |
| robot USD compose/articulation | **未测** |
| robot controller/odom/TF | **未测** |
| ROS 2 bridge/UDP | **未测** |
| LiDAR/RTX sensor | **未测，且当前 probe 明确不加载 sensor** |
| RTX5090 driver stability under default renderer | **未解决**：默认 base experience 的 `librtx.scenedb.plugin.so` 崩溃仍需另行处理 |

## 8. 最终判定

**Gate 1 最小 custom experience：PASS。**

在当前 RTX5090 / NVIDIA 595.84 / Isaac Sim 5.1 环境下，已经获得了新的 runtime 证据：

```text
Isaac Core + USD + PhysX + Python API
    └── custom experience（不继承 isaacsim.exp.base）
        └── 空 stage + PhysX initialize + 3 次 physics-only step：PASS
        └── omni.hydra.rtx startup：未发生
        └── librtx.scenedb.plugin.so 动态加载：未发生
```

这证明后续可以把“机器人/控制验证”作为下一层单独实验接到该 custom experience 上；本报告不改变既有迁移架构，也不把 robot Gate 1 宣称为已通过。
