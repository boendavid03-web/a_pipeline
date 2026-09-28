# Isaac Sim 5.1 Runtime Gate 1 崩溃分析

审计日期：2026-09-11  
审计范围：Isaac Sim 5.1 本机安装、当前 Isaac 5 backend 启动文件、Kit experience/extension 配置和最新 Gate 1 日志  
执行边界：本次只读检查；未启动新的 Isaac Sim 进程，未修改系统 driver，未删除 Isaac 6，也未修改迁移架构。本文件是本次唯一新增文件。

## 1. 结论

当前 Gate 1 的失败发生在机器人 USD、PhysX、ROS 或控制代码运行之前。最新崩溃栈明确落在：

```text
librtx.scenedb.plugin.so
  carbOnPluginStartup
libcarb.scenerenderer-rtx.plugin.so
libomni.hydra.rtx.plugin.so
```

本机 5.1 的默认 `SimulationApp`/`isaacsim.exp.base.python.kit` 路径没有发现一个可以通过单个 `--headless`、`--no-window`、`renderer=pxr` 或 `Wireframe` 参数可靠关闭 RTX SceneDB 初始化的受支持开关。

判断如下：

| 问题 | 判断 |
|---|---|
| 仅用 `--headless` 是否绕过 SceneDB？ | **否**。它只关闭窗口；最新命令仍初始化 Vulkan、GPU Foundation 和 RTX 依赖。 |
| 是否发现 `/rtx/scenedb/enabled=false` 一类总开关？ | **否**。本机 extension 配置、文档和 SceneDB 二进制字符串中没有发现可用的总禁用开关。 |
| `renderer=pxr` 是否足以绕过 SceneDB？ | **不能确认，按当前 base experience 判断为否**。`omni.hydra.rtx` 仍是 base 的必需依赖并在 active renderer 选择前加载其 native plugins。 |
| `Wireframe` 是否是非 RTX 基本模式？ | **否**。在 `SimulationApp` 中它只是写入 `/rtx/rendermode` 的字符串，仍属于 RTX 路径。 |
| 是否存在 raster/Storm 组件？ | **有静态组件证据**：本机有 `hdStorm.so`，也有 `omni.app.hydra.kit` 的 PXR/Storm experience 配置；但它不是当前 Isaac 5 robot backend 的已验证启动路径。 |
| 是否已有可直接用于机器人验证的 no-render experience？ | **未发现**。`omni.app.empty.kit` 没有 Isaac/PhysX 依赖，`omni.app.viewport.kit` 只是空 viewport，`omni.app.hydra.kit` 也不是 Isaac Core/PhysX backend。 |

因此，当前最高结论是：**Gate 1 需要先解决 Kit/RTX SceneDB 启动层，不能把 headless 失败解释为 robot USD 或 PhysX 兼容性失败。**

## 2. 最新 Gate 1 事实

最新 Kit 日志：

```text
/home/user/isaacsim/5.1.0/kit/logs/Kit/Isaac-Sim Python/5.1/kit_20260911_120558.log
```

崩溃报告对应的 command line 是：

```text
/home/user/isaacsim/5.1.0/kit/python/bin/python3 \
  /home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/runtime/validate_robot.py \
  --headless --steps 120
```

Kit 实际记录的启动参数包含：

```text
isaacsim.exp.base.python.kit
--renderer/multiGpu/enabled=True
--no-window
--app/window/hideUi=1
--headless
```

同时日志报告：

```text
Graphics API: Vulkan
Driver Version: 595.84
GPU: NVIDIA GeForce RTX 5090
```

最新崩溃时间为 `2026-09-11T04:06:04Z`。关键栈位于 `gate1_validate_robot.log` 末尾以及 Kit crash report 的对应 backtrace：

```text
001-004: librtx.scenedb.plugin.so ... carbOnPluginStartup
005-008: libcarb.scenerenderer-rtx.plugin.so ... carbOnPluginShutdown
009:     libomni.hydra.rtx.plugin.so
```

这说明进程在 `SimulationApp` 启动阶段即崩溃，尚未到达 `validate_robot.py` 中的 `World`、robot USD compose、articulation 或 physics stepping 逻辑。

## 3. 为什么 headless 没有绕过 RTX

### 3.1 当前 experience 的依赖闭包

`isaacsim.exp.base.python.kit` 只声明依赖 `isaacsim.exp.base`，而 `isaacsim.exp.base.kit` 明确包含：

```toml
"omni.hydra.rtx" = {}
```

同时 base app 设置：

```toml
vulkan = true
renderer.asyncInit = true
useFabricSceneDelegate = true
```

`omni.hydra.rtx` 的本机 `extension.toml` 进一步声明：

```toml
[core]
reloadable = false

[[native.plugin]]
path = "bin/deps/rtx.scenedb.plugin"
```

也就是说，在当前 base experience 中，SceneDB 是 RTX Hydra extension 的 native plugin，而不是由机器人脚本创建的可选传感器。它在场景和 robot USD 处理前就可能进入启动流程。

### 3.2 `headless` 的实际语义

Isaac 5.1 `SimulationApp` 的实现把 `headless=True` 转换为 `--no-window` 和隐藏 UI 参数；它仍然保留：

- Vulkan graphics API；
- GPU Foundation；
- base experience 的 RTX Hydra 依赖；
- renderer 初始化和 extension startup。

因此：

```text
headless != no-render
--no-window != no-Vulkan
```

当前日志已经实证了这一点：命令带有 `--headless --no-window`，但崩溃栈仍为 `librtx.scenedb.plugin.so`。

### 3.3 当前 Gate 1 还启用了 multi-GPU 默认值

`SimulationApp.DEFAULT_LAUNCHER_CONFIG` 的默认值是：

```python
"multi_gpu": True
```

`validate_robot.py` 只传入 `headless`、`renderer`、分辨率，没有覆盖 `multi_gpu`，所以最新 Gate 1 command line 为：

```text
--/renderer/multiGpu/enabled=True
```

这可能增加 RTX/SceneDB 初始化的变量，但它不是当前唯一根因：即使将 multi-GPU 关闭，`omni.hydra.rtx` 和 `rtx.scenedb.plugin` 仍会被当前 base experience 装载。现有 `run_navigation.py` 将 `multi_gpu=False`，这是一个值得单独验证的变量，但不能事先视为 SceneDB 问题已解决。

## 4. 是否存在禁用 RTX SceneDB 的启动参数

### 4.1 已检查的参数层

本机 `kit --help` 支持通用的 setting override：

```text
--</path/to/key>=<value>
```

也支持 `--enable EXT_ID` 和 `--disable-ext-startup`。但静态检查得到的边界是：

1. `--disable-ext-startup` 会阻止所有 extension startup，不能作为 Isaac Core/PhysX 机器人验证路径。
2. `--renderer/enabled=pxr`、`--renderer/active=pxr` 是 renderer delegate 选择，不是从 extension dependency graph 中移除 `omni.hydra.rtx`。
3. `--/exts/omni.kit.renderer.core/autostartRenderer=false` 只控制 renderer core 的自动启动，不等价于卸载已由 base experience 声明的 `omni.hydra.rtx` native plugins。
4. `--/exts/omni.kit.renderer.core/compatibilityMode=true` 是 renderer core 兼容模式，不是 SceneDB 禁用开关。
5. `--/renderer/multiGpu/enabled=false` 只改变 GPU 数量，不移除 SceneDB。
6. `--headless`、`--no-window` 和 `present.enabled=false` 都只影响窗口/present/UI 或 swapchain 行为，不构成 RTX SceneDB off switch。

### 4.2 SceneDB 二进制字符串检查

在本机：

```text
/home/user/isaacsim/5.1.0/extscache/omni.hydra.rtx-1.0.0+69cbf6ad.lx64.r/bin/deps/librtx.scenedb.plugin.so
```

可以看到内部使用的设置包括：

```text
/rtx-transient/scenedb/instancing/...
/rtx/scenedb/accelStructPriority
/rtx-transient/debugwindows/enable
syncFabricSceneDb
```

这些是 SceneDB 的内部功能、调度或调试设置；没有发现类似：

```text
/rtx/scenedb/enabled=false
/rtx/scenedb/disable=true
```

的总禁用合同。不能把 `syncFabricSceneDb`、instancing threshold 或 debug-window 参数误认为关闭 SceneDB。

### 4.3 能否从当前 base experience 排除 extension

理论上可以另建一个不引入 `omni.hydra.rtx` 的自定义 Kit experience，再逐步补入运行机器人所需的 Isaac Core、PhysX 和 USD extension；但这不是当前 `isaacsim.exp.base.python.kit` 的参数开关，也没有在本机完成依赖闭包和 runtime 验证。

此外，`omni.hydra.rtx` 自己标注为 `reloadable = false`，因此不能先启动后在 Python 中安全地 disable。若要真正避开 SceneDB，必须在 Kit extension resolution/startup 之前改变 experience 依赖图。

结论：**当前启动链没有可直接确认的 SceneDB 禁用参数；真正的绕过路径只能是另一个未验证的 custom experience/extension closure。**

## 5. headless、renderer 和 experience 对比

| 路径 | 本机静态事实 | 能否直接做 robot Gate 1 |
|---|---|---|
| `isaacsim.exp.base.python.kit` | Isaac Python 入口；继承 `isaacsim.exp.base`，包含 `omni.hydra.rtx`，Vulkan=true | **当前会触发 SceneDB，不能作为绕过路径** |
| `isaacsim.exp.base.kit` | 同一 base 依赖闭包，包含 UI、RTX Hydra、PhysX 等基础组件 | **同样不能绕过 SceneDB** |
| `omni.app.empty.kit` | 没有依赖，属于空 Kit | **不能直接运行 Isaac Core/World/Robot** |
| `omni.app.viewport.kit` | `renderer.enabled=""`、`renderer.active=""`，不主动选择 renderer | **只是空 viewport app，不是 Isaac robot app** |
| `omni.app.hydra.kit` | 明确依赖 `omni.kit.viewport.pxr` 和 `omni.hydra.pxr.settings`，设置 `renderer.enabled='pxr'`、`renderer.active='pxr'` | **可作为 Storm/PXR 独立渲染探针候选；不能直接证明 Isaac PhysX robot 可运行** |
| `renderer=RaytracedLighting` | `SimulationApp` 写入 `/rtx/rendermode=RaytracedLighting` | **仍为 RTX 路径** |
| `renderer=PathTracing` | `SimulationApp` 写入 `/rtx/rendermode=PathTracing` | **仍为 RTX 路径** |
| `renderer=Wireframe` | 作为未知 renderer 字符串写入 `/rtx/rendermode=Wireframe` | **不是 no-render，也不是 Storm** |

本机虽然存在：

```text
/home/user/isaacsim/5.1.0/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311/bin/usd/hdStorm.so
```

但本次没有证据证明：

- 当前 `SimulationApp` base experience 可以在 SceneDB 初始化之前切换到 Storm；
- PXR/Storm 与本机 5.1 Isaac Core/PhysX robot backend 的完整依赖闭包成立；
- headless/no-window 下 PXR 所需的 OpenGL interop 合同成立；
- 机器人加载、articulation stepping、控制和 teardown 在该 experience 中通过。

## 6. 对三种可能路径的判断

### 6.1 真正 no-render

**理论可行，当前没有现成可直接执行的证据。**

需要一个在 extension resolution 阶段不加载 `omni.hydra.rtx` 的自定义 experience，并保留运行 `isaacsim.core.api.World`、`Robot`、USD/PhysX 所需的最小依赖。`omni.app.empty.kit` 本身过于空，不能直接替代 Isaac 5 Python experience。

这属于 Kit 启动依赖图的独立诊断工作，不等于修改迁移架构；但在获得新的 runtime 证据前，不能称为 Gate 1 已有 no-render fallback。

### 6.2 PXR/Storm raster

**有候选组件，但当前 backend 不能直接宣称可用。**

最接近的本机参考是 `omni.app.hydra.kit`：它显式选择 PXR/Storm。可是直接把它作为 `SimulationApp` experience 使用，可能缺失当前 robot script 所需的 Isaac Core/PhysX extension；而继续使用 `isaacsim.exp.base.python.kit` 又会保留 `omni.hydra.rtx` SceneDB startup。

因此 PXR/Storm 只能作为后续单变量实验：先验证“Kit/PXR 能启动且不加载 `librtx.scenedb.plugin.so`”，再验证“Isaac Core/PhysX robot 能在同一 experience 中启动”。两者不能用一个启动成功的 USD viewer 结果互相替代。

### 6.3 当前 base experience 加 renderer override

**不建议把它当作 SceneDB 修复。**

即使追加：

```text
--/renderer/enabled=pxr
--/renderer/active=pxr
--/renderer/multiGpu/enabled=false
--/renderer/multiGpu/autoEnable=false
```

它仍然没有改变 `isaacsim.exp.base.kit` 对 `omni.hydra.rtx` 的依赖，也没有从 extension graph 删除 `rtx.scenedb.plugin`。这是 renderer selection / multi-GPU 的实验组合，不是 SceneDB disable 证明。

## 7. 只读排查后的推荐 Gate 顺序

不修改当前迁移架构的前提下，后续如获授权执行 runtime 实验，应保持单变量顺序：

1. **Gate R0：Kit extension 证据**  
   使用最小 custom experience 或现有 PXR experience，仅观察启动日志是否出现 `omni.hydra.rtx`、`rtx.scenedb.plugin` 和 Vulkan graphics initialization。此步不加载 robot USD。

2. **Gate R1：PXR/Storm standalone**  
   只证明 PXR/Storm app 能启动、创建空 stage、退出；记录是否仍加载 `librtx.scenedb.plugin.so`。这不是 robot Gate 1。

3. **Gate R2：Isaac Core/PhysX robot**  
   在确认 SceneDB 未加载的同一 experience 中，再加入 `World`、canonical robot USD、articulation 和有限 stepping。

4. **Gate R3：控制/ROS/LiDAR**  
   只有 R2 通过后，才恢复 command、ROS、scene-query LiDAR 和 teardown 检查。

当前不应把 `renderer=Wireframe`、`--headless` 或“没有窗口”当成 R0/R1 通过证据。

## 8. 最终判断

1. **当前 Isaac Sim 5.1 默认 runtime gate1 不能通过启动参数直接禁用 RTX SceneDB。** 失败点位于 base experience 的 RTX native plugin startup，而不是机器人代码。
2. **headless/no-window 不足以绕过 RTX。** 当前最新日志已在 headless 条件下报告 Vulkan 并于 `librtx.scenedb.plugin.so` 崩溃。
3. **Storm/PXR 有静态组件，但不是当前 robot backend 的现成 fallback。** 需要先建立并验证不加载 RTX SceneDB 的 Kit experience，再判断是否能承载 Isaac Core/PhysX。
4. **没有发现可直接使用的 no-render 模式。** `empty`/`viewport` experience 缺少 Isaac robot 依赖；`hydra` experience 是 PXR viewer 候选，不是现成 robot validator。
5. **RTX5090、NVIDIA 595.84、Isaac 5.1 的环境事实已由最新 crash report 记录。** 本分析不建议修改系统 driver，也不要求删除或替换 Isaac 6。

本报告结论仍停留在：**Gate 1 native RTX startup crash confirmed；SceneDB bypass requires a separately constructed and verified Kit experience; robot runtime compatibility remains untested.**

