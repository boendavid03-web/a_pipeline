# Isaac Sim 5.1 Renderer / Viewport Analysis

调查日期：2026-09-12（Asia/Shanghai）

调查范围：`/home/user/navigation_project/a_pipeline`、外置 Isaac Sim 5.1 安装 `/home/user/isaacsim/5.1.0`，以及项目内 Isaac Sim 6.0.1 安装。本文把静态发现、历史运行证据、本轮运行证据和用户给定事实分开陈述。

## Executive conclusion

**本轮最终判定：Result C — RTX PATH BLOCKED。**

当前机器上的 Isaac Sim 5.1 确实包含原生 OpenUSD/Hydra Storm renderer：`HdStormRendererPlugin`、`hdStorm.so`、`libusd_usdImagingGL.so` 和 Python `pxr.UsdImagingGL` 均实际存在，Hydra registry 也动态返回了 `HdStormRendererPlugin`（显示名 `GL`）。因此不能说“Isaac 5.1 完全没有 non-RTX renderer”。

但是，这个安装没有把 Storm 接入 Kit viewport 所需的两个本地 extension：`omni.kit.viewport.pxr` 和 `omni.hydra.pxr`。在关闭 registry、显式列出所有本地 extension 目录后，最小 experience 仍在 12 ms 处报告 `omni.kit.viewport.pxr ... Available versions: (none found)`。这排除了单纯 search path 配置错误。原生 `UsdImagingGL.Engine()` 在没有 Kit/GL context 时也因 OpenGL 4.5 context 不存在而无法创建 delegate；它不是可替代真实 Kit viewport 的独立窗口方案。

默认 Isaac 5.1 GUI 的 RTX 路径则在 RTX 5090 + driver 595.84 上重复失败。本轮唯一一次隔离 RTX 复现已经同时设置 `multi_gpu=False`、`renderer/multiGpu/enabled=false` 和 `renderer/multiGpu/autoEnable=false`，仍在 `rtx.scenedb.plugin` 初始化附近原生段错误。现有带符号回溯把失败点进一步限定到 `librtx.scenedb.plugin.so!carbOnPluginStartup+0x3b4de`。所以 multi-GPU 自动启用不是必要触发条件。

结论边界如下：

- Core/PhysX/no-RTX backend：有当前运行 PASS 证据。
- 原生 non-RTX Hydra delegate：存在且可被 registry 发现。
- non-RTX **真实 Kit 3D viewport**：本机安装闭包不完整，当前不能启动。
- RTX 真实 Kit 3D viewport：SceneDB native startup 被当前运行时组合阻塞。
- Mecanum730 在 Isaac 5.1 真实 viewport 中可见并运动：**当前没有实现，也没有伪造为 PASS**。空 cube viewport 未过门，按约束没有加载机器人。

## Current known working stack

当前已确认可工作的路径是：

```text
Isaac Sim 5.1 Kit/Core/USD/PhysX
  -> minimal_core_physx_no_rtx.kit
  -> 不依赖 omni.hydra.rtx
  -> 不加载 librtx.scenedb.plugin.so
  -> physics/backend benchmark 可运行并正常 teardown
  -> 没有真实 3D viewport
```

本地证据 `isaac_sim/backends/isaac5/generated/stage8_benchmark_run.log` 显示：

- `app ready` 和 `Simulation App Startup Complete`；
- 三个 episode 全部 `SUCCESS`，success rate 1.0；
- 0 collision、0 timeout；
- `world_stop=true`、`app_close=true`；
- 日志中没有 `omni.hydra.rtx`、`rtx.scenedb` 或 `librtx.scenedb.plugin.so`。

这证明 physics/backend 路径可用，不证明 renderer 或 3D GUI 可用。背景中列出的导航、ROS、LiDAR、DRL-VO、SemanticCNN 等既有 PASS 本轮没有重跑，也没有修改。

## Isaac5 vs Isaac6 renderer comparison

| Component | Isaac5 | Isaac6 | Key difference |
| --- | --- | --- | --- |
| Product build | `5.1.0-rc.19+release.26219.9c81211b.gl` | `6.0.1-rc.7+release.42383.32955d8d.gl` | 不同产品代际 |
| Kit / Kernel | Kit `107.3.3+production.229672.69cbf6ad.gl`; Kernel `206.6+release.9587.07f17b1b.gl` | Kit `110.1.2+production.326809.f9bf0dda.gl`; Kernel `210.1.11+release.13746.80f6dc72.gl` | 完整 Kit/Carbonite 代际变化 |
| Embedded Python | 3.11 | 3.12 | native ABI 不同 |
| OpenUSD evidence | USD 24.05 family; `omni.usd.libs-1.0.1` | USD 25.11 family; `omni.usd.libs-1.0.3` | USD/Hydra 版本不同 |
| Default renderer path | RTX/Vulkan | RTX/Vulkan | 两者 Full app 都不是用 Storm |
| `omni.hydra.rtx` | `1.0.0+69cbf6ad.lx64.r` | `1.0.4+f9bf0dda.lx64.r` | Isaac6 为更新 binary stack |
| SceneDB interfaces | `FabricSceneDb v0.1`, `SceneDb v4.4` | `FabricSceneDb v0.2`, `SceneDb v6.1` | 首个有直接运行差异的核心组件 |
| `omni.kit.renderer.core` | `1.1.0+69cbf6ad.lx64.r.cp311` | `1.2.2+f9bf0dda.lx64.r.cp312` | 更新 |
| `omni.kit.renderer.init` | `0.0.0+69cbf6ad.lx64.r` | `1.0.1+f9bf0dda.lx64.r` | 更新 |
| `omni.gpu_foundation` | `0.0.0+69cbf6ad.lx64.r.cp311` | `0.0.0+f9bf0dda.lx64.r.cp312` | 名义版本相同，build/ABI 不同 |
| Vulkan shader cache | `omni.gpu_foundation.shadercache.vulkan-1.0.0+69cbf6ad` | `...-1.0.0+f9bf0dda` | build 不同 |
| `omni.hydra.usdrt_delegate` | `7.5.1+69cbf6ad.lx64.r.cp311` | `7.5.2+f9bf0dda.lx64.r.cp312` | 更新 |
| Viewport window | `omni.kit.viewport.window-107.2.0+69cbf6ad` | `...-109.0.2+f9bf0dda` | 更新 |
| Viewport bundle | `104.0.1+69cbf6ad` | `107.0.0+f9bf0dda` | 更新 |
| Viewport RTX | `104.0.1+69cbf6ad` | `107.0.0+f9bf0dda` | 更新 |
| Local PXR Kit bridge | `omni.kit.viewport.pxr` / `omni.hydra.pxr` 均未安装 | 两者也未随当前 Full install 本地提供 | Isaac6 正常 GUI 使用 RTX，不依赖 PXR bridge |
| Native Storm | `hdStorm.so`, `UsdImagingGL`, `HdStormRendererPlugin` 存在 | 对应 native 文件也存在 | native delegate 存在不等于 Kit viewport bridge 存在 |
| Observed GUI evidence | 同一 5.1 build 在 driver 580.173.02 历史日志中成功加载 Full app；595.84 当前失败 | 本地可读成功日志使用 580.173.02；“6.0.1 + 595.84 GUI PASS”是用户给定的当前事实 | 本地日志不能把 Isaac6/595.84 当成本轮重新验证；但 newer SceneDB stack 与当前用户事实一致 |

回答“Isaac 5.1 本身是否支持真正 GUI”：**支持。** `/home/user/.nvidia-omniverse/logs/Kit/Isaac-Sim Full/5.1/kit_20260808_200257.log` 是同一个 Isaac 5.1 / Kit hash，在 RTX 5090、driver 580.173.02 下运行的历史证据：SceneDB v4.4 初始化，RTX viewport 被分配到 device 0，73.876 s 输出 `Isaac Sim Full App is loaded`，进程运行到约 142 s 后由用户中断。这不是本轮的视觉复验，但足以证明产品和该安装并非 physics-only。

回答“为什么 Isaac6 能显示而 Isaac5 当前不能”：可防御的解释是 **Isaac 5.1 的旧 RTX/SceneDB binary stack 与当前 RTX 5090 + driver 595.84 组合发生 native runtime incompatibility，而 Isaac 6 使用更新的 Kit/GPU Foundation/Hydra RTX/SceneDB v6.1 stack并能越过同一阶段**。现有 stripped binary 和日志不能把原因再精确到某一行 NVIDIA 内部源码，因此不把它描述成已证明的单一 driver bug。

## Installed renderer capability audit

Isaac 5.1 本地实际存在：

```text
/home/user/isaacsim/5.1.0/extscache/
  omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311/
    pxr/UsdImagingGL/_usdImagingGL.so
    bin/libusd_usdImagingGL.so
    bin/usd/hdStorm.so
    bin/usd/hdStorm/resources/plugInfo.json
```

`plugInfo.json` 把 `HdStormRendererPlugin` 注册为 `HdRendererPlugin`，library path 为 `../hdStorm.so`。

对 `/home/user/isaacsim/5.1.0/{kit,exts,extscache,apps}` 的 extension/native/plugin metadata 搜索结果：

- 找到的 OpenUSD renderer plugin：`HdStormRendererPlugin`；
- 未找到 `HdPrmanLoaderRendererPlugin`、`HdEmbreeRendererPlugin` 或其他可用 non-RTX delegate；
- 找到 `omni.kit.viewport.window`、`omni.kit.viewport.bundle`、`omni.kit.viewport.rtx`；
- 没有本地 `omni.kit.viewport.pxr` 目录；
- 没有本地 `omni.hydra.pxr` 目录；
- registry index 仅有远端包 metadata，不是已安装 extension。

因此核心问题 3 的答案是 **有 `HdStormRendererPlugin`**；核心问题 4 的答案必须分层：**native USD/Hydra 层有可发现的 non-RTX delegate，但本机没有可供 Kit viewport 使用的完整 non-RTX extension 闭包。**

## Hydra render delegate discovery

使用 Isaac 5.1 embedded Python 3.11，并显式设置本地 `PYTHONPATH`、USD native library path 和 `PXR_PLUGINPATH_NAME` 后，实际输出为：

```text
PLUGIN_COUNT 36
HDSTORM_PLUGINS [('hdStorm', '.../bin/usd/hdStorm.so', False)]
RENDERER_IDS ['HdStormRendererPlugin']
RENDERER_NAMES [('HdStormRendererPlugin', 'GL')]
```

这是真实 Hydra registry 动态发现证据，不是仅凭文件名推断。

继续直接构造 `UsdImagingGL.Engine()` 时输出：

```text
HgiGL minimum OpenGL requirements not met.
Please ensure that OpenGL is initialized and supports version 4.5.
...
UsdImagingGLEngine ... No renderer plugins found!
```

含义是：registry 可以在没有窗口的 Python 进程中枚举 Storm；真正创建 render delegate 需要由窗口/graphics integration 提供有效 OpenGL 4.5 context。`omni.hydra.pxr` / `omni.kit.viewport.pxr` 正是 Kit 与这层能力之间缺失的桥。直接调用 USD API 不能自动产生符合验收条件的 Isaac/Kit 3D viewport。

## Viewport dependency graph

默认 Isaac 5.1 Full/Python 路径：

```text
isaacsim.exp.full
  -> isaacsim.exp.base
       -> omni.kit.viewport.window
       -> omni.hydra.rtx
            -> native plugin bin/deps/rtx.scenedb.plugin
               -> librtx.scenedb.plugin.so
  -> omni.kit.viewport.bundle
  -> omni.kit.viewport.rtx
       -> omni.hydra.rtx
```

关键静态证据：

- `apps/isaacsim.exp.base.kit` 直接硬依赖 `omni.hydra.rtx`；
- `apps/isaacsim.exp.full.kit` 加入 `omni.kit.viewport.bundle` 和 `omni.kit.viewport.rtx`；
- `omni.kit.viewport.rtx/config/extension.toml` 直接依赖 `omni.hydra.rtx`；
- `omni.hydra.rtx/config/extension.toml` 的 native plugin 列表明确包含 `bin/deps/rtx.scenedb.plugin`。

预期 non-RTX 路径：

```text
omni.app.hydra
  -> omni.kit.viewport.bundle / omni.kit.viewport.window
  -> omni.kit.viewport.pxr             [本地缺失]
       -> omni.hydra.pxr               [本地缺失]
       -> omni.gpu_foundation
       -> native OpenUSD UsdImagingGL
       -> HdStormRendererPlugin / GL
```

`omni.hydra.pxr.settings` 对 registry 中的 `omni.kit.viewport.pxr` 是 optional，但 `omni.app.hydra.kit` 自己把它列为硬依赖。最小 probe 已去掉该 settings 依赖，仍因核心 `omni.kit.viewport.pxr` 缺失而失败。

其他 experience 的意义：

- `omni.app.viewport.kit` 只提供 viewport shell，并明确要求通过 `--enable omni.hydra.xxx` 选择 renderer；其 `renderer.enabled/active` 为空。
- `omni.app.empty.kit` / `omni.app.mini.kit` 不提供完整 renderer delegate。
- `omni.kit.viewport.window` 只是窗口/viewport orchestration，不等于 render delegate。

## SceneDB crash chain

当前 595.84 下的已证实链条：

```text
GPU Foundation / Vulkan enumeration PASS
  -> RTX 5090 被识别为 device 0
  -> omni.kit.renderer.init startup PASS
  -> omni.hydra.rtx-1.0.0 extension startup PASS
  -> rtx.scenedb.plugin 被注册为 SceneDb v4.4
  -> UsdContext::createViewportImpl(engine='rtx', device=0)
  -> app ready（仅表示 Kit loop ready，不表示 viewport 可用）
  -> Initializing plugin: rtx.scenedb.plugin
  -> native SIGSEGV
```

现有 crash backtrace 的顶部有效帧为：

```text
librtx.scenedb.plugin.so!...
librtx.scenedb.plugin.so!carbOnPluginStartup+0x3b4de
libcarb.scenerenderer-rtx.plugin.so!...
libomni.hydra.rtx.plugin.so!...
```

因此不能把失败说成 Vulkan instance/device creation 失败：Vulkan 已经枚举 GPU，RTX extension 也已加载。也不能说场景同步已经开始或完成：日志没有越过成功 SceneDB 初始化的明确证据。

## Non-RTX viewport experiment

本轮建立的隔离 probe：

- `isaac_sim/backends/isaac5/generated/minimal_core_physx_viewport_no_rtx.kit`
- `isaac_sim/backends/isaac5/generated/minimal_core_physx_viewport_no_rtx.py`
- `isaac_sim/backends/isaac5/generated/minimal_core_physx_viewport_no_rtx_run.log`

设计边界：

- `renderer.enabled/active = pxr`；
- 禁止 registry：`app.extensions.registryEnabled=false`；
- 显式列出 Kit、Isaac apps/exts/extscache/extsUser/extsDeprecated 的本地目录；
- 没有声明 `omni.hydra.rtx`、RTX sensors、synthetic data、replicator 或 ROS bridge；
- 若依赖能解析，脚本只创建 floor、cube、camera、distant light，运行至少 60 秒，并从 `/proc/self/maps` 检查 `librtx.scenedb.plugin.so`。

实际结果：

```text
Failed to resolve extension dependencies
dependency: 'omni.kit.viewport.pxr' ... can't be satisfied
Available versions: (none found)
Synced registries: (none)
exit_code=1
```

紧随其后的 `ModuleNotFoundError: omni.kit.usd` 是 SimulationApp 在应用依赖解析失败后继续执行 Python helper 导入产生的级联错误，不是首个失败点。

| Required PASS condition | Result |
| --- | --- |
| 软件窗口出现 | NOT REACHED |
| 真正 3D viewport 出现 | NOT REACHED |
| floor/cube 可见 | NOT REACHED |
| camera 正常 | NOT REACHED |
| 连续运行至少 60 秒 | NOT REACHED |
| 动态证明 SceneDB 未加载 | NOT REACHED；进程在 extension resolution 前退出 |
| 正常 teardown | NOT APPLICABLE；Kit app 未完成创建 |

核心问题 5 的答案：**就当前磁盘上的安装闭包，不能构建 `Core + PhysX + real 3D viewport` 且避开 SceneDB。** 不是 Storm 不存在，而是 Kit-PXR bridge 未安装。若以后取得与 Kit hash `69cbf6ad`、Linux release、cp311 精确匹配的 bridge 及其依赖闭包，理论上存在可验证路径；目前尚未运行证明，不能标为可用。

`omni.app.hydra.kit` 触发 registry sync 的原因归类为：

- **A：本地真的缺 extension（主因）**；
- **D：Kit 默认 registry fallback（伴随行为）**；
- 不是 B/E：本轮已经使用完整显式本地 search folders，仍是 `(none found)`；
- 没有证据支持 C：registry metadata 能解析到匹配 Kit hash 的版本，问题不是已安装版本冲突。

历史 registry 日志中选择过：

```text
omni.kit.viewport.pxr-104.0.2+69cbf6ad
omni.hydra.pxr-1.2.4+69cbf6ad.lx64.r.cp311
omni.hydra.pxr.settings-1.0.8+69cbf6ad
```

但这些条目只存在于 registry index metadata；本轮没有继续下载，也没有把它们当作已安装文件。

## RTX differential analysis

SceneDB binary：

| Property | Isaac5 | Isaac6 |
| --- | --- | --- |
| Path | `/home/user/isaacsim/5.1.0/extscache/omni.hydra.rtx-1.0.0+69cbf6ad.lx64.r/bin/deps/librtx.scenedb.plugin.so` | `isaac_sim/isaacsim-6.0.1/extscache/omni.hydra.rtx-1.0.4+f9bf0dda.lx64.r/bin/deps/librtx.scenedb.plugin.so` |
| SHA256 | `70066ea3da0f67d2d32fe6ab66bae1cb34ec62b9ddc8647b8915515204b99b90` | `3fc6bb790d164decaf40aa52c5c76a6401e67e210f395e40cea9ea20ac6c57f0` |
| ELF Build ID | `c5d7fe60e2f44bf876651f7055ccf201b7127315` | `c69f93567740418057f0739d469eec000d59d9fe` |
| SceneDB interface | v4.4 / Fabric v0.1 | v6.1 / Fabric v0.2 |
| Python/TBB ABI | `libboost_python311`, `libpython3.11`, `libtbb.so.2` | `libusd_python`, `libpython3.12`, `libtbb.so.12` |
| RPATH | `$ORIGIN:/builds/omniverse/kit/rendering/_build/target-deps/python/lib` | 相同形态 |

两份 SceneDB ELF 都没有直接 `DT_NEEDED` 到 `libvulkan` 或 NVIDIA driver library，也没有可见的直接 Vulkan import。相关 Vulkan/device 功能由外围 GPU Foundation/scene renderer plugin 间接提供。普通 shell 下 `ldd` 出现找不到某些 Kit private library，不能据此判定运行时缺库，因为 Kit extension loader 会追加相应搜索路径；两代 binary 都具有这种布局。

历史运行矩阵提供了强相关但不是源码级因果证明：

| Stack | Evidence | Result |
| --- | --- | --- |
| Isaac5/Kit 107.3.3 + RTX5090 + driver 580.173.02 | 多个 Full 5.1 日志；`kit_20260808_200257.log` 最清晰 | SceneDB v4.4 PASS，view device 0，Full app loaded，运行 > 60 s |
| Isaac5/Kit 107.3.3 + RTX5090 + driver 595.84 | 多个当前/历史日志 + 本轮唯一隔离复现 | SceneDB startup 附近 native crash；无 Full app loaded |
| Isaac6/Kit 110.1.2 + RTX5090 + driver 580.173.02 | `kit_20260809_200928.log` | SceneDB v6.1 PASS，后续 rasterizing/materialdb/denoising/postprocessing 启动，Full app loaded |
| Isaac6 + RTX5090 + driver 595.84 | 用户给定当前事实 | 真正 GUI/3D viewport PASS；本轮未重启复验 |

**Last common successful initialization point：** GPU Foundation/Vulkan device enumeration、Hydra RTX extension/native libraries 注册、RTX viewport creation request。

**First diverging component：** `rtx.scenedb.plugin` native initialization。Isaac5 SceneDB v4.4 在 `carbOnPluginStartup` 内崩溃；Isaac6 SceneDB v6.1 越过该点并继续初始化 rasterizing、MaterialDB、denoising 和 postprocessing。

回答核心问题 9：Isaac6 相同阶段成功的直接证据是它加载的是不同 SHA/Build ID、不同接口版本、不同 Kit/Python/TBB ABI 的 SceneDB/renderer stack。当前证据能证明“新版 stack 越过旧版失败点”，不能仅凭 stripped binaries 精确指出 NVIDIA 在 v4.4 到 v6.1 之间修了哪一段内部代码。

## Exact failure point

最窄且不越过证据的定位是：

> **Isaac 5.1 的 RTX SceneDB native plugin 在首次 RTX viewport 激活后的 `rtx.scenedb.plugin` startup 阶段发生 SIGSEGV，符号化到 `librtx.scenedb.plugin.so!carbOnPluginStartup+0x3b4de`。**

阶段分类：

- SceneDB plugin load/register：PASS；
- GPU/Vulkan enumeration：PASS；
- Hydra RTX extension startup：PASS；
- RTX viewport create request：已发生；
- SceneDB native plugin initialization：**FAIL**；
- 能否更具体归为 SceneDB device creation：证据不足；
- 场景同步：没有成功到达证据；
- 可用 viewport/render frame：没有成功证据。

设置审计结果：

- 未发现 `disableSceneDB`、`useSceneDB=false` 或其他能静态证明阻止 plugin startup 的设置；
- 找到 `/rtx/scenedb/...` 运行参数，但都是 SceneDB 行为/质量设置，不是启动旁路；
- 找到 renderer compatibility、raytracing、multi-GPU 相关设置，但 manifest 仍会因 `omni.hydra.rtx` 加载 SceneDB；没有证据表明单独切换这些设置能绕开 native plugin；
- 本轮 multi-GPU-off 复现仍失败，排除了“必须启用 multi-GPU 才崩溃”；虽然 `rtx.multigpumanager.plugin` 仍会作为 RTX stack 的 native component 初始化，但 device table 只有 GPU 0。

## Experiments performed

1. **只读 renderer/viewport inventory**：比较 Isaac5 与 Isaac6 实际 extension 目录、manifest、VERSION、Kit log。
2. **原生 Storm 文件和 metadata 搜索**：覆盖 `kit/`、`exts/`、`extscache/`、`apps/`、Python/native library、`plugInfo.json`。
3. **Hydra registry 动态枚举**：确认唯一返回的 OpenUSD delegate 是 `HdStormRendererPlugin`，显示名 `GL`。
4. **直接 `UsdImagingGL.Engine()` construction**：在无 GL context 的受限进程中失败，确认“可枚举”不等于“可直接显示”。
5. **离线最小 PXR viewport dependency gate**：registry 完全关闭、完整本地 folders、最小依赖；12 ms 在 `omni.kit.viewport.pxr` 缺失处停止，无下载、无 GUI、无机器人。
6. **SceneDB ELF differential**：`sha256sum`、`readelf`、`strings`、`ldd` 只读检查；没有 patch binary。
7. **既有 Isaac5/Isaac6 log 对照**：找出 last common point 与 first divergence。
8. **唯一一次隔离 RTX reproduction**：headless、单 GPU、multi-GPU/autoEnable 均 false、无机器人/ROS/传感器、20 s 上限；4.773 s `app ready` 后仍 native segfault，Kit log 最后一组关键事件包含 SceneDB v4.4 initialization。运行后 `nvidia-smi` 正常显示 RTX 5090/595.84；未观察到 Xid 输出。

没有执行第二次 RTX hard-start；没有执行 Isaac6 GUI；没有启动机器人；没有继续 registry 下载。

最终 probe 源文件另经只读语法检查：Python AST `PASS`，`.kit` 标准 TOML parse `PASS`。这只证明文件可解析，不提升 renderer/runtime 状态。

## Files created

最终报告：

- `isaac5_renderer_viewport_analysis.md`

仅在允许的 generated 目录新增的隔离实验文件：

- `isaac_sim/backends/isaac5/generated/minimal_core_physx_viewport_no_rtx.kit`
- `isaac_sim/backends/isaac5/generated/minimal_core_physx_viewport_no_rtx.py`
- `isaac_sim/backends/isaac5/generated/minimal_core_physx_viewport_no_rtx_run.log`
- `isaac_sim/backends/isaac5/generated/isaac5_rtx_multigpu_off_repro.log`

Kit 自己还在既有安装日志目录生成了本次运行日志：

- `/home/user/isaacsim/5.1.0/kit/logs/Kit/Isaac 5.1 Minimal Core PhysX PXR Viewport Probe/5.1/kit_20260912_122530.log`
- `/home/user/isaacsim/5.1.0/kit/logs/Kit/Isaac-Sim Python/5.1/kit_20260912_122954.log`

没有生成新的 crash dump；RTX reproduction 明确关闭了 crash reporter。

## Files NOT modified

本轮没有修改：

- Isaac Sim 6.0.1 安装或其项目脚本；
- `/home/user/isaacsim/5.1.0` 安装文件；
- 既有 `minimal_core_physx_no_rtx.kit` 和已 PASS backend；
- DRL-VO、SemanticCNN、checkpoint；
- ROS、LiDAR、pedestrian、Social Force、benchmark contract；
- canonical Mecanum730 robot USD；
- NVIDIA driver、Vulkan 系统配置、`/usr`；
- 用户 cache、Omniverse cache、shader cache；
- 工作树中原先存在的其他 modified/untracked 文件。

## Final status

**Result C — RTX PATH BLOCKED**

不是 Result A：本轮没有任何真实 Kit 3D viewport，也没有 cube 或机器人可见证据。

不选择 Result B：本机确实包含并动态发现 `HdStormRendererPlugin`，所以“non-RTX renderer 完全不存在/永远不可能”过强。准确说法是：**当前安装缺少 Kit-PXR bridge，因此当前磁盘闭包无法实现 non-RTX 真 viewport。**

选择 Result C：默认/已安装完整 viewport 路径只有 RTX，且 Isaac 5.1 SceneDB v4.4 在当前 595.84 环境中的 native startup 仍被阻塞；no-RTX Core/PhysX backend 不受该阻塞影响。

十个核心问题的简答：

1. Isaac Sim 5.1 是否支持真正 GUI？**支持；历史同安装 Full log 有 >60 s RTX viewport 运行证据。**
2. 为什么 Isaac6 可显示、Isaac5 当前不能？**首个差异在旧 SceneDB v4.4 与新版 v6.1；旧 stack 在当前组合 native crash，新 stack 能越过。内部具体修复点未知。**
3. Isaac5 是否有 `HdStormRendererPlugin`？**有，文件和动态 registry 均证实。**
4. 是否有可用 non-RTX Hydra delegate？**native 层可发现；Kit real viewport 层当前不可用，因为 bridge 缺失。**
5. 能否不加载 SceneDB 构建 Core+PhysX+real viewport？**按当前已安装文件不能；补齐精确匹配 bridge 后才值得重新验证。**
6. `omni.app.hydra.kit` 为什么 registry sync？**它硬依赖本地没有的 PXR extensions，Kit 执行默认 fallback。**
7. 缺 extension 还是 path 问题？**缺 extension；完整本地 folders + offline solver 已排除 path 主因。**
8. SceneDB crash 在哪一阶段？**`rtx.scenedb.plugin` native startup，`carbOnPluginStartup+0x3b4de`。**
9. Isaac6 为什么同阶段成功？**使用不同且更新的 SceneDB v6.1/renderer binary stack；日志显示继续进入后续 RTX plugins。**
10. 当前能否看到 Mecanum730 在 Isaac5 viewport 中运动？**不能；空 cube viewport 尚未通过，因此没有进入机器人阶段。**

## Recommended next step

最小风险下一步不是改 driver、patch binary 或继续试 renderer flags，而是在获得用户明确授权后，执行一个新的、仍然隔离的 **PXR bridge closure gate**：

1. 只取得与 Kit hash `69cbf6ad`、Linux release、cp311 精确匹配的 `omni.kit.viewport.pxr-104.0.2`、`omni.hydra.pxr-1.2.4` 及 dependency closure；`omni.hydra.pxr.settings-1.0.8` 仅在需要 settings UI 时加入。
2. 放入独立实验 extension 目录，不覆盖 `/home/user/isaacsim/5.1.0`，不删除或复用不明 cache；记录来源、SHA256 和完整文件清单。
3. registry 再次关闭，先运行本报告中的 floor/cube/camera/light probe 60 秒。
4. 必须同时取得：真实 Kit viewport 可见、实际 renderer ID 为 `HdStormRendererPlugin`、`/proc/self/maps` 无 `librtx.scenedb.plugin.so`、正常 teardown。
5. 只有上述全部 PASS，才建立 robot-only probe：Mecanum730 + PhysX + viewport，不接 ROS/LiDAR/DRL-VO/SemanticCNN/pedestrian。

如果不能合法、可重复地取得这一精确 bridge closure，则工程上应继续把 Isaac 5.1 定位为 no-RTX backend，并用已能显示的 Isaac 6.0.1 承担 GUI；不应把状态窗口、离屏截图或 USD 成功打开宣称为 Isaac 5.1 3D GUI PASS。
