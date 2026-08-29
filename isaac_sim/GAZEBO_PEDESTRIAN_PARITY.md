# Isaac Sim 行人迁移方案（Gazebo eng_hall_15）

## 已实现的数据与运动语义

`run_custom_people_drlvo_demo.sh` 默认使用：

- `ISAAC_PEDESTRIAN_COUNT=19`
- `ISAAC_PEDESTRIAN_SEED=7`
- `ISAAC_PEDESTRIAN_SPEED=1.0`
- `ISAAC_PEDESTRIAN_AVOIDANCE_MODE=gentle`

启动器读取 Gazebo 的 `eng_hall_15.xml`，跳过 `type=2` 的机器人占位项，保留六个行人簇的路线拓扑和原始权重 `[5, 5, 2, 1, 1, 1]`。指定总人数后使用与 Gazebo 控制器相同的最大余数法；19 人得到 `[7, 6, 3, 1, 1, 1]`。

每个 Isaac 角色被展开成独立的 IRA group，因此可以从同一个 seed 复现：

- 簇内初始散布位置；
- `max(0.1, N(base_speed, 0.26))` 个体速度；
- 角色配置顺序和 ID；
- 每个人从初始点进入本簇路线后的闭环巡逻。

逐人出生点不是独立“最近栅格”投影：生成器在同一连通自由区内全局分配，默认要求中心距
至少 `1.0 m`（IRA 导航半径为 `0.5 m`），空间不足会直接失败。validator 会在启动前再次
计算两两距离，避免两个角色从重合/相交状态开始后再把分离责任错误交给局部避障器。

所有 XML waypoint 和初始点先投影到膨胀后的已知自由栅格，再逐段 A* 路由；最后显式回到起点，避免 IRA 的隐式闭环边穿墙。生成后会再次对 Gazebo 静态碰撞盒执行线段验证。运行时若实际加载人数与生成配置不一致，场景直接报错，不发布一个“看起来成功”的残缺人群。

## Gazebo 与 Isaac 的职责映射

| Gazebo 行为 | Isaac 实现 |
|---|---|
| 六簇人数比例和循环 waypoint | XML 驱动的逐人 IRA patrol |
| seed 控制初始位置和 vmax | 生成器中的 Python `random.Random(seed)` |
| 静态障碍物排斥、沿墙走 | 膨胀自由栅格 + A* + NavMesh |
| 人–人社会力的局部避碰职责 | BehaviorAgent auto/obstacle avoidance + 0.90/1.10 m 非传送式速度礼让 |
| 机器人个人空间排斥的局部避碰职责 | `gentle` 默认让机器人参加 object avoidance，并在持续接近时低速侧闪 |
| 15 Hz 行人真值 | 原有 UDP/ROS 桥发布 `/pedestrian_ground_truth` |

## 有意保留的差异

Isaac 当前不逐帧重算 Gazebo 的显式社会力、随机扰动和组群 gaze/coherence/repulsion 项，
所以这里是职责映射，不宣称动力学或逐点轨迹等价。直接在 15 Hz 写 SkelRoot pose 会与
BehaviorAgent 的 NavMesh、避障和骨骼运动控制争夺同一个 transform，容易产生脚滑、穿墙
和动画跳变。因此当前实现选择“Gazebo 的人口/路线/seed 语义 + Isaac 的连续局部避障”。

自由空间的路线生成净空默认是 `0.55 m`，独立运行时侵入观察边界是 `0.20 m`。守卫以
10 Hz 观察，连续 3 次越界才累计一次持续侵入，但绝不在运行中调用 `reset()`；启动时
确定出生点的一次复位保留。严格社交验收要求持续侵入和运行时复位都为零。普通运行和
`--test-pedestrian-social` 都累计人–人最小中心距、视觉重叠、个人空间违规比例以及
人–机器人净距。Gazebo parity 应比较这些分布和路线完成率，而不是宣称两边轨迹逐帧相同。

如果实验要求力学轨迹逐点可比，下一阶段应新增独立的 crowd solver，并只把求解速度作为 BehaviorAgent 的局部目标/速度约束输入；在确认 Isaac 6.0.1 提供稳定的运行时 steering 接口前，不应通过每帧 `reset()` 模拟社会力。

## 使用

```bash
bash isaac_sim/scripts/run_custom_people_drlvo_demo.sh
```

覆盖人数、seed 或基准速度：

```bash
ISAAC_PEDESTRIAN_COUNT=25 \
ISAAC_PEDESTRIAN_SEED=15 \
ISAAC_PEDESTRIAN_SPEED=0.9 \
bash isaac_sim/scripts/run_custom_people_drlvo_demo.sh
```

`ISAAC_PEDESTRIAN_COUNT=0` 会关闭角色流水线，但仍由桥发布空的行人真值数组。演示启动器接受的正人数范围是 `1..50`；上限用于防止一次误配置生成过量骨骼角色。实际容量仍受角色动画、RTX LiDAR 和目标实时因子限制。19 人是默认对齐工况，超过 30 人应先用短时 headless 运行测量实时因子和真值发布频率。

`ISAAC_PEDESTRIAN_COUNT=-1` 保留 XML 原始 15 人；`0` 表示无人；正整数按六簇比例分配。
