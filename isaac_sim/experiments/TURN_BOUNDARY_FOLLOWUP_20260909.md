# 非零转弯与边界约束跟进 · 2026-09-09

**NOT_CONVERGED_STOPPED：两次有效 180 秒验证均未满足保留条件，已精确恢复到此前保留的 boundary guard 第二轮。未执行第三轮。**

| 版本 | 越界采样 / 持续事件 | 估算越界人时 | 最长冻结 |
|---|---:|---:|---:|
| 原保留版 | 204 / 8 | 7.981 s | 0.605 s |
| 补充朝向预测 + 等价几何索引 | 220 / 6 | 6.778 s | **1.417 s** |
| 原预测 + 等价几何索引 | **666 / 9** | **19.413 s** | 0.490 s |

两轮均为同一 lobby、15 人、seed 7、base speed 1.0、gazebo_social、avoidance off、CPU PhysX、headless/no-ROS；自然结束，15/15 人移动，无 runtime reset、follow restart 或 Motion Matching non-finite warning。实际观察交付率分别约 32.46、34.31 Hz，控制约 15 Hz。不能将自然退出 PASS 当作零越界验收。

离线空间索引在 15,186 条冻结查询上与原 guard 完全一致，查询速度约 3.365 倍，但在线轨迹和越界指标仍变化。因此未把这项加速留在正式路径，也未把时间变化认定为已完成因果隔离的根因。朝向预测的冻结发生时 follow 正常、速度命令非零、latch 已释放；它不能简单归因为显式 yield。

正式 social、steering、launcher、地图生成器、几何、现有测试和共享 kernel 均与本轮开始前内容一致，保留用户原有修改。恢复后 99 项相关测试通过。候选源码、失败初始化日志、两次运行、几何等价性、冻结窗口和还原哈希全部保留。

下一步需要先做固定指令、固定 simulation/animation dt 的独立重放，再只改变计算负载，确认执行轨迹对时序的敏感性；有执行误差界后才能继续设计根节点约束。本轮没有继续调预测参数或追加人群长跑。

完整证据和只读复现命令：[RESULTS.md](../../runs/pedestrian_turn_boundary/20260908_225639/RESULTS.md)。
