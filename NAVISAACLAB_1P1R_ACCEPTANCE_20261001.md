# NavIsaacLab 1 人 + 1 车验收报告（2026-10-01）

## 结论

本轮目标已完成：现有 NavIsaacLab 2.0 链路可在同一 warehouse 场景中完成 1 个 MaskedMimic 人体与 1 台 Nova Carter 的启动、受控近距离交互、RGB/depth 取证、完整 episode 终止、完整 reset、checkpoint 加载/续训，并可正常退出。没有重新制作资产、重装环境、修改安装版 Isaac 源码、扩大人数或做长训练。

本轮是**系统生命周期与数据链验收**，不是 PPO 导航质量验收。交互 run 中确实执行了 step=128 checkpoint 的策略推理，但为获得可重复的人体穿越证据，明确将送给车辆的 action 覆盖为零；候选策略 action、requested action 和 executed motion 分开记录。

## Gate 总表

| Gate | Result | Evidence |
|---|---|---|
| MotionLib | PASS — RUNTIME VERIFIED | `amass_smpl_validation.pt` 在实际 `build_env()` 链中装载；678,007,942 B，SHA-256 `651e...c68`，既有 345-motion/finite/interpolation 校验保持有效。 |
| MaskedMimic load | PASS — RUNTIME VERIFIED | 匹配 `57f98a9` 的 config/checkpoint 经实际 `agent.setup()`、`agent.load()` 并连续产生人体 action。 |
| build_env | PASS — RUNTIME VERIFIED | 最终验收 19.311 s；三次独立 bring-up 分别 21.422/19.461/19.403 s。 |
| clean shutdown | PASS — RUNTIME VERIFIED | 正常 `SimulationApp.close(wait_for_replicator=False, skip_cleanup=False)`，exit 0；未使用 SIGKILL 或 `skip_cleanup=True`。 |
| 3-run repeatability | PASS — RUNTIME VERIFIED | 3/3 exit 0；关闭 0.817/0.756/0.508 s；无本次残留进程。 |
| human movement | PASS — RUNTIME VERIFIED | 120 步位移 3.309 m；速度 0.032–1.569 m/s，均值 0.812 m/s。 |
| human within neighbor radius | PASS — RUNTIME VERIFIED | 4.5 m 起步后进入 4.0 m radius；最近 1.634 m。 |
| neighbor_mask active | PASS — RUNTIME VERIFIED | step 37 首次从 0 变 1；之后 84 个 step 有效。 |
| neighbor values correct | PASS — RUNTIME VERIFIED | 独立 world-state 对照：距离最大误差 0 m，bearing 最大误差 `5.06e-8` rad，本地相对速度最大误差 `1.27e-7` m/s。 |
| RGB valid | PASS — RUNTIME VERIFIED | 原始帧 480×640×3，非全零；保存首帧、mask 激活、3 m、terminal 样本。 |
| human visible in RGB | PASS — RUNTIME VERIFIED | 3 m 原始/标注帧中可清晰看到 articulated skeleton；人体根节点投影 `(316.01, 202.29)` 落在人体躯干。 |
| depth valid | PASS — RUNTIME VERIFIED | 原始 depth 480×640；3 m 帧 finite ratio 0.9456，tensor 与 16-bit PNG 均保存。 |
| human represented in depth | PASS — RUNTIME VERIFIED | 3 m depth 有清晰同形人体轮廓；投影根节点附近最近 depth 与几何深度误差 `3.50e-5` m。 |
| full episode terminal | PASS — RUNTIME VERIFIED | step 120 / 4.8 sim s 触发 `timeout`，terminal reward breakdown 完整。该短 timeout 是诊断配置，不表示策略失败。 |
| episode reset | PASS — RUNTIME VERIFIED | 完整 reset 后 human `[4,-3]`、robot `[-0.5,-3]`；goal、step counter、previous action、mask、depth、map 均重置/有效。 |
| PPO checkpoint load | PASS — RUNTIME VERIFIED | step=128 checkpoint 被真实 `RobotPPOTrainer.load()` 加载，120 步逐步推理。 |
| PPO resume | PASS — RUNTIME VERIFIED | 从 step 128 续到 192，完成一轮 64-step update 并保存新 checkpoint；39/39 model tensors finite 且均发生更新，optimizer 117/117 tensor fields finite。 |

## A. Shutdown — PASS

原入口保存后只调用 `post_quit()`，当时 ProtoMotions/Isaac Lab 的 live `SimulationContext`、callbacks 与 `SimulationApp` 所有权没有按顺序释放，进程停在 Kit cleanup。现在 `AppLauncher`/`SimulationApp` 所有权保存在 `ProtoMotionsRuntime`，训练入口通过 `finally` 调用统一 cleanup：

1. 关闭导航文件句柄并标记 simulator stopped；
2. 禁用 app-control-on-stop handle；
3. `SimulationContext.clear_all_callbacks()`；
4. `SimulationContext.clear_instance()`；
5. `SimulationApp.close(wait_for_replicator=False, skip_cleanup=False)`。

最后成功执行的关闭函数是第 5 步。Isaac Sim 5.1 在该调用内部结束进程，因此同进程不会打印 `close done`；父进程捕获的 exit code 0 和 0.508–0.817 s shutdown duration 是权威完成证据。

三次数据：

| Run | Build | First reset | Total | Shutdown | Max RSS | Max VRAM | Exit |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 21.422 s | 0.076 s | 24.706 s | 0.817 s | 9055.9 MiB | 10539 MiB | 0 |
| 2 | 19.461 s | 0.076 s | 22.550 s | 0.756 s | 9069.5 MiB | 10513 MiB | 0 |
| 3 | 19.403 s | 0.076 s | 22.244 s | 0.508 s | 9024.9 MiB | 10507 MiB | 0 |

## B. 1+1 interaction — PASS

- minimum human-robot distance：`1.63441694 m`
- `neighbor_mask`：已激活，step 37 从 0→1
- RGB：已确认看到 human；当前 `human_mesh=false`，所以看到的是白色 articulated skeleton，不是 SMPL 表面网格
- depth：已确认对应同一个 human；3 m 投影/depth 误差仅 `3.50e-5 m`
- 人体位移：`3.3086 m`
- 观测数值：distance/bearing/local relative velocity 均与独立 world-state 计算一致

受控动作边界：checkpoint 候选 action 约为 linear `0.50143–0.50148`、angular `0.00161–0.00168`；本次实际 requested action 固定 `[0,0]`，controller reported executed velocity 为 0。物理 pose 每步最大数值漂移 `0.000314 m`，120 步累计 `0.01466 m`。这不是策略性能结果，但明确证明了 checkpoint inference 与人车邻域/传感器链同时工作，且没有混淆 requested 与 executed motion。

## C. Full episode — PASS

诊断配置在 step 120（4.8 sim s）以 `timeout` 正常结束；episode return `-51.1267`，终止步 `reward_timeout=-50`，无 collision/reached/stuck。随后先执行 robot episode reset，再执行完整 env reset：

- robot start 恢复为 `[-0.5,-3.0]`
- human start 恢复为 `[4.0,-3.0]`
- robot goal 保持固定评测 goal `[5.025,-3.025]`
- episode step=0、previous action=0、neighbor mask 回到 0
- depth/map 均 finite
- reset 后 camera mount error `1.33e-7 m`

首轮诊断曾暴露 reset 后第一帧 camera pose 仍是 teleport 前缓存；现已在 robot teleport 后显式 render/reset/force-update camera，并由最终 run 03 重新验证，不使用旧帧冒充通过。

## D. Checkpoint reload/resume — PASS

- 输入：`robot_ppo_latest.pt`，step 128，3,347,065 B，SHA-256 `b5fb...fd6`
- 真实续训：128 → 192 robot steps，64-step rollout、1 epoch、1 minibatch
- 输出：`output/crowdsim_robot_ppo_acceptance_resume_20261001/20261001_035610/robot_ppo_latest.pt`
- 输出大小：3,347,577 B
- 输出 SHA-256：`2fcad365696c44725c15e9c1f73b2494715dd9d858c10c4a8c86f0e74d777e12`
- 39/39 model tensors finite，39/39 与输入不同；39 optimizer param states、117 tensor fields 全 finite；optimizer step 为 3
- 保存后同样走正常 shutdown 并 exit 0

## E. 当前唯一下一步

已经适合进入下一阶段：**原版 PPO 正式训练 + 固定评测**。

本轮到此停止。下一阶段应先冻结训练配置、随机种子、固定 evaluation routes/metrics 与 checkpoint cadence，再决定正式训练步数；现在不扩大人数、不接 world model、不做百万步训练。

## 产物与可复现入口

- 最终结构化摘要：`../Assets/repro_logs/acceptance_1p1r/acceptance_summary_03.json`
- 120-step JSONL：`../Assets/repro_logs/acceptance_1p1r/interaction_03.jsonl`
- 最终 stdout/stderr：`../Assets/repro_logs/acceptance_1p1r/interaction_03.log`
- episode 派生指标：`../Assets/repro_logs/acceptance_1p1r/episode_03.log`
- 3 m RGB：`../Assets/repro_logs/acceptance_1p1r/sensor_samples_03/rgb_0062_interaction_3m.png`
- 3 m 投影标注 RGB：`../Assets/repro_logs/acceptance_1p1r/sensor_samples_03/rgb_0062_interaction_3m_projected.png`
- 3 m depth：`../Assets/repro_logs/acceptance_1p1r/sensor_samples_03/depth_0062_interaction_3m.png`
- 续训日志：`../Assets/repro_logs/acceptance_1p1r/resume_01.log`
- 三次退出日志：`../Assets/repro_logs/acceptance_1p1r/shutdown_repeat_01.log` 至 `shutdown_repeat_03.log`
- 完整命令：`../Assets/repro_logs/acceptance_1p1r/commands.txt`
- 环境/哈希：`../Assets/repro_logs/acceptance_1p1r/environment.txt`

## 实现与回归

项目内新增/修改集中在显式 runtime ownership/cleanup、固定诊断路线、reset 后相机刷新，以及独立验收脚本；没有改 PPO network 或 reward 设计。`CrowdSim/config/env.yaml` 与 `env_smoke.yaml` 的 SHA-256 保持为 `5eb3...679b` 与 `f7ec...0cdf`。

回归结果：`PYTHONPATH=.:src ... pytest -q CrowdSim/tests tests` 为 **38 passed**；全部相关 Python 文件通过 `py_compile`。验收结束后没有对应的 Isaac/acceptance/training 残留进程。
