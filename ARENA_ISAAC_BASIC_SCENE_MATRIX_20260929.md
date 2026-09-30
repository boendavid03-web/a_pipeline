# Arena + Isaac Sim 5.1 基础场景实测

日期：2026-09-29。使用本机 Arena 原始 launch，经 `/home/user/arena_isaac5_host_runtime/run_gui.bash` 顺序测试。每次使用独立 ROS domain；只向本轮持有的进程组发送停止信号，既有 Gazebo/HuNav 进程未处理。原始日志和结构化采样在 `/home/user/arena_isaac5_py311_factory/basic_world_tests_20260929/`。

**同日后续：**本矩阵记录首次基础测试时的三维资产状态。后来已将原 `ignc.dae` 和原 `hospital.world` 的全部 178 个顶层模型导入 Isaac，并分别复跑 3 人、17 人的 Arena GUI 基础链，均观察到全员短时位移；Nav2 仍未到达。结果、地下异常源模型和日志见 [三维场景继续验证](ARENA_ISAAC_SCENE_GEOMETRY_CONTINUATION_20260929.md)。下表中“未导入”仅描述首次测试时的状态。

## 验收口径

- **基础链通过**：Arena 请求加载指定地图、场景参数生效、Isaac child 启动、Jackal 里程计收到消息、预期人数创建、人物路线批次发出，且每个人在两个服务位姿采样之间有净位移。这是短时启动/运动检查。
- **完整三维场景呈现**：另需确认原 Arena 三维模型进入 Isaac stage。上面的基础链通过本身不证明这一点。
- **Nav2 到达**：需独立看到目标成功；本轮没有任何场景达到这个门槛。若日志出现 `Goal failed` 或机器人出界，直接列为导航问题。没有报错也不代表到达。

`/isaac/get_prim_attributes` 返回人物父 prim 与 SkeletonRoot 位姿；下表的位移是两次采样后计算的世界坐标**净位移**，不是累计路径或碰撞判定。所有场景均为原有 Arena world/scenario JSON，`map_highly_social` 是此前已有的本机 overlay 变体。

## 原有地图的默认场景

| Arena world / scenario | 人数 | 基础链结果 | 行人净位移范围 | 原三维场景 | 主要限制 |
|---|---:|---|---|---|---|
| `map_empty/default` | 3 | 人物创建、路线、运动通过 | 0.937–1.306 m / 5.28 s | 2D 地图与运行时构造；未做完整 stage 几何计数 | 首轮未单独采 Jackal 里程计；另一次补测遭服务发现故障，已停止，未覆盖首轮人物运动证据 |
| `factory/default` | 2 | 基础链通过 | 1.003–1.332 m / 5.15 s | **完整 factory USD 加载**：66 模型、175 visual、209 collision | Nav2 未验收 |
| `generated/default` | 3 | 人物与里程计通过，场景几何**部分失败** | 1.042–1.297 m / 5.27 s | 55 个墙体配置、15 个静态障碍配置；未验 stage 全量 | world callback 缺 `CoffeTable` 的 Isaac USD，且机器人出界、目标失败 |
| `house17/default` | 0 | 首轮过早结束；独立复测确认场景参数与 Jackal 里程计 | 不适用 | 62 个墙体配置；未验 stage 全量 | Jackal 里程计约 `(23.99,19.99)`，Nav2 报出界和目标失败 |
| `ignc/default` | 3 | 地图、人物、里程计通过 | 1.208–1.399 m / 5.21 s | **原 `ignc.dae` 未导入**；仅运行时基础链 | Nav2 目标失败 |
| `hospital/default` | 17 | 地图、人物、里程计通过 | 0.755–2.169 m / 7.64 s | **原 Hospital 建筑模型未导入**；不能当作 Hospital 三维可视化成功 | Gazebo Hospital 的 178 个模型使用本地 `gazebo_models` 的 include；本轮未转换这些资产，也未验 Nav2 到达 |
| `map_highly_social/default` | 10 | 基础链通过 | 0.630–1.178 m / 6.48 s | 基于 `map_empty` 的本机 overlay；未验 stage 全量 | 机器人出界、目标失败 |

## 代表性多人变体

| Arena world / scenario | 人数 | 基础链结果 | 行人净位移范围 | 备注 |
|---|---:|---|---|---|
| `factory/default1` | 5 | 通过 | 1.090–1.253 m / 5.48 s | 同一完整工厂 USD；本轮 Nav2 有 `Goal failed` |
| `generated/blocked_corridors` | 7 | **部分通过** | 未能完成全员采样 | 7 人创建；位姿服务对 `/_20` 未返回，路线批次只确认 `20`；同样缺 `CoffeTable` USD，已停止并切换场景 |
| `generated/evacuation` | 10 | 人物与里程计通过，场景几何**部分失败** | 0.911–1.282 m / 6.38 s | world callback 缺 `CoffeTable` USD；机器人出界、目标失败 |
| `house17/1` | 3 | 人物与里程计通过 | 1.069–1.085 m / 5.23 s | 机器人出界、目标失败 |
| `house17/evacuation` | 10 | 人物与里程计通过 | 0.642–1.378 m / 6.36 s | 机器人出界、目标失败 |

## 需要保留的边界

1. **只有 `factory` 已验证原 Arena 三维静态场景的完整导入。** 其余地图的短时人物运动不等于 Gazebo 原场景已在 Isaac 呈现。`hospital` 和 `ignc` 尤其只有 2D 地图/任务/人物链通过。
2. `generated` 的缺失资源是精确的 world callback 报错：`no model CoffeTable among [USD] ... could not be converted`。Arena 源目录有 Gazebo `CoffeeTable` 模型，但当前 host overlay 没有对应 Isaac USD。该场景不能标为完整地图通过。
3. `generated`、`house17`、`map_highly_social` 出现机器人超出 Nav2 costmap 的警告。人物运动证据不清除该机器人导航缺口。
4. 本轮每张只做基础短测；未做长时间、碰撞、最小间距、任务成功率、Jackal 到达或社会导航 benchmark 验收。服务发现故障和 `generated/blocked_corridors` 的位姿服务超时均按场景记录并停止，没有在同一故障上连续重启。

结构化证据：每个测试目录的 `result.json`、`people_positions.json`、`odom_probe.txt` 和 `console.log`；`house17/default` 独立复测日志位于 `house17_retry_210/console.log`。本轮所有新启动的 Isaac/Arena PGID 已停止。
