# Isaac Sim 6.0.1 动态行人、DRL-VO Demo 与 rosbag 采集

当前入口把以下组件放在同一个 Isaac Sim 6.0.1 场景中：

- 官方离线 IRA 仓库和原有正常行走的动态行人；
- `robot_related` 中的 Mecanum 730 + XMS5 自有机器人 USD；
- 全向 `/cmd_vel` 键盘遥控；
- 与 Gazebo/V7 数据链兼容的双雷达、里程计、TF、时钟、控制标签和行人真值。

锁定的机器人资产是：

```text
/home/user/navigation_project/robot_related/robots/chassis_arm/
motion_wheel_arm_simple_sphere_usd/mecanum730_xms5_default.usd
```

不要修改源 USD，也不要把 Gazebo 的 SDF 代理当作 Isaac 正式机器人。

## 当前稳定性前提

2026-08-13 11:31:49 当前开机周期由错误的 Nav2 costmap 参数触发了
`controller_server` 和 `planner_server` 的 general protection fault；参数类型现已修复，
`nvidia-smi` 也能正常读取 RTX 5090，但启动器的保守安全门不会区分 Nav2 进程异常与
GPU/内核异常。因此本次开机仍需先正常重启 Ubuntu，再继续 Isaac/RTX runtime 测试。
这不表示 Ubuntu 已损坏，而是为了清除按启动周期记录的安全锁。启动器检测到 Xid、
general protection fault、内核 stall 或 GPU bus loss 等致命签名时会拒绝再次启动 GPU
仿真，不要绕过检查。
此前故障证据见：

```text
isaac_sim/scripts/INCIDENT_2026-08-09_GPU_KERNEL_WIFI.md
```

## 运行结构

Isaac 6.0.1 的嵌入式 Python 与 Ubuntu 22.04 ROS 2 Humble 的 Python ABI 不同。当前实现
不在 Kit 内导入系统 `rclpy`：

- Isaac 进程负责 IRA、机器人控制和默认的原生双 RTX LiDAR；
- 系统 ROS 进程负责 ROS 消息、双雷达合并和 rosbag；
- 两者只通过 `127.0.0.1` UDP 交换控制与遥测；
- 启动器固定使用 Fast DDS 和 localhost discovery；直接入口默认 domain 0，
  DRL-VO 一键 Demo 默认使用与 Gazebo smoke 命令一致的 domain 78。

启动器会自动管理 ROS/UDP 桥和 `/scan_merged` 节点，不再需要单独启动旧的
`isaac_collection_support.launch.py`。

## 终端 1：启动 Isaac 6.0.1

```bash
cd /home/user/navigation_project/a_pipeline
ISAAC_LIDAR_RATE_HZ=10 \
ISAAC_LIDAR_SAMPLE_COUNT=360 \
  bash isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh --deterministic
```

`ISAAC_LIDAR_RATE_HZ` 是双雷达唯一的目标频率入口，允许 `1` 到 `30` 的整数 Hz。它会同时
设置 RTX `scanRateBaseHz`、sensor `tickRate`、采集调度、ROS `LaserScan.scan_time` 和
rosbag 验收目标。`--deterministic` 按仿真时间推进：渲染较慢时墙钟运行也会变慢，但不会
通过重复旧帧伪造目标频率。正式录包推荐使用该模式。

`ISAAC_LIDAR_SAMPLE_COUNT` 独立控制每路 ROS `LaserScan` 的角度槽数，允许 90--4096，
默认 360；producer、`/scan_01`、`/scan_02`、`/scan`、双雷达 merger 的
`/scan_merged`、sensor config 与验包器使用同一个值。它表示输出数组长度，不保证每个槽
都有有限回波，也不是 `PointCloud2` 点数。物理发射/回报能力仍由 RTX profile 决定。
360 槽的旧遥测仍使用单个纯 JSON 数据报；高槽数消息会在 localhost 上自动使用 zlib，
必要时按带 message id/index/count 的有界 UDP 片段传输，relay 只有收到完整一组才发布。
每个片段不超过 UDP payload 上限，缺片不会产生半帧 LaserScan。

RTX 雷达有两个官方 profile 和一个工程自有导航 profile；频率、输出槽数与 profile 是三个
独立参数：

| `ISAAC_RTX_LIDAR_PROFILE` | 原生模型 | 用途 |
|---|---|---|
| `rplidar_s2e`（默认） | Slamtec RPLIDAR S2E，单通道二维旋转雷达 | 导航/SLAM；已通过本机短程 A/B 验收 |
| `navigation_2d_32k` | 工程自有单通道二维模型，32 kHz firing、30 Hz authored ceiling | 15 Hz/高角分辨率导航测试；不修改官方资产 |
| `example_dense` | NVIDIA `Example_Rotary_2D`，128 通道 | 历史兼容、RTX/GMO 诊断；计算量大 |

例如以 10 Hz 使用单通道二维模型：

```bash
ISAAC_RTX_LIDAR_PROFILE=rplidar_s2e \
ISAAC_LIDAR_RATE_HZ=10 \
  bash isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh \
  --headless --fast
```

运行日志和 `/data_collection/sensor_config` 会保存 profile 名、资产路径及资产 SHA256；不要
直接修改离线官方 USD 来换频率。目标 Hz 仍只由 `ISAAC_LIDAR_RATE_HZ` 覆盖。

15 Hz、每路 2000 个 ROS 角度槽的无窗口短测使用：

```bash
ISAAC_ENABLE_PEOPLE=0 \
ISAAC_PEDESTRIAN_AVOIDANCE_MODE=off \
ISAAC_RTX_LIDAR_PROFILE=navigation_2d_32k \
ISAAC_LIDAR_RATE_HZ=15 \
ISAAC_LIDAR_SAMPLE_COUNT=2000 \
  bash isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh \
  --headless --fast --duration 3
```

32 kHz / 15 Hz 约为每圈 2133 次理论发射机会，随后按最近回波投影到 2000 个 ROS 槽。
只有日志最终出现 `RESULT ... PASS`、`published_pairs` 接近 45、两路
`dropped_unpaired=0`，才说明该次仿真频率门通过；不能只看配置打印了 15 Hz。

2026-08-13 00:24 的首次 `navigation_2d_32k` 实测已经确认两路均输出 2000 槽、30 对完整
scan、无未配对丢帧，墙钟吞吐为 16.95 对/s；但原生 GMO 仿真时间仍只有 9.886 Hz，因此
该轮按设计判定 FAIL。随后源码按 Isaac Sim 6.0.1 官方 standalone RTX 初始化方式补上
`SimulationManager.setup_simulation(1/60)`，让多 tick renderer 使用由 PhysX 推进的
`/ExternalSimulationTime`，并在应用启动时显式启用 multi-tick/per-sensor TLAS。该修复已
通过 Python 语法检查，但因本开机周期随后出现上述 Xid 69，尚未进行 runtime 复测。

重启后先确认内核没有新 Xid，再原样复测：

```bash
cd /home/user/navigation_project/a_pipeline
ISAAC_ENABLE_PEOPLE=0 \
ISAAC_PEDESTRIAN_AVOIDANCE_MODE=off \
ISAAC_RTX_LIDAR_PROFILE=navigation_2d_32k \
ISAAC_LIDAR_RATE_HZ=15 \
ISAAC_LIDAR_SAMPLE_COUNT=2000 \
  bash isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh \
  --headless --fast --duration 3
```

2026-08-13 10:48 的重启后复测证明上述 multi-tick 初始化已生效，但尚未最终 PASS：原生
时间戳不再固定为约 100 ms，而是严格交替 `66.666666/133.333332 ms`，30 对扫描的平均值
仍为 9.886 Hz；两路保持 0 个未配对丢帧，墙钟吞吐达到 21.30 对/s。源码检查发现 native
render product 原先只在同为 15 Hz 的 ROS 发布门打开时才被轮询，Writer 回调晚一个 app
update 可见时会产生同频相位混叠。当前实现已将二者解耦：每次 app update 都提取并配对
所有新 native 帧，ROS telemetry 再按目标频率逐帧发布；不复制旧帧，队列溢出会直接失败。

该轮结束后 NVIDIA 驱动进入不可通信状态（`nvidia-smi` 失败，内核同时记录
`refcntRequestReference_IMPL ... status 0x56`），因此采集解耦修复目前只有静态语法验证，
不能在此开机周期继续复测。必须再次正常重启，并确认 `nvidia-smi` 与启动器内核安全门均
恢复后，才可执行上面的 3 秒命令。

例如改成 5 Hz：

```bash
ISAAC_LIDAR_RATE_HZ=5 \
  bash isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh --deterministic
```

无需 GUI 的批量采集可使用：

```bash
ISAAC_LIDAR_RATE_HZ=10 \
  bash isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh \
  --headless --fast
```

不加 `--deterministic` 时 GUI 按墙钟实时运行；当 viewport、两路 RTX 和动态行人的总渲染
负载超过机器的实时吞吐时，真实雷达频率可能低于目标。录包脚本会先测量两路雷达至少
1 秒仿真时间；达不到目标频率时拒绝开始录制。

### 三种频率不要混用

- `requested_hz`：`ISAAC_LIDAR_RATE_HZ` 写入 RTX sensor 的目标值。
- `sim_hz`：由 `LaserScan.header.stamp` 计算；该字段使用与 `/clock`、odom、TF 相同的 Isaac
  timeline 遥测仿真时间。这是 `use_sim_time` 下 SLAM、Nav2、训练和普通验包使用的频率。
- RTX GMO 的 `timestampNs` 只用于把前后雷达按同一原生采集帧配对和诊断，不直接写入 ROS
  header；GUI/render 丢失采集时，该内部计数器不保证与 USD timeline 同步。
- `wall_hz`：ROS 在真实墙钟一秒内实际收到多少帧；只有硬件在环、真人实时交互或外部
  非仿真时钟系统才必须让它也达到目标。

2026-08-12 的同场景、无人群、无窗口 A/B 解释了原先低频的主因：128 通道
`example_dense` 每雷达每圈约产生 3.6--4.2 万个原生回波，再压成 360 槽；它只达到约
16.2 app FPS 和 2.68 对/s 墙钟吞吐。单通道 `rplidar_s2e` 每雷达每圈约 470--570 个
原生回波，3.0167 仿真秒产生 30 对 scan，`sim_hz=10.0`、`wall_hz=16.18`、97.63 app FPS，
两路未配对丢帧为 0且 RESULT PASS。设备能够实时生成 10 Hz；此前主要是 profile 选得过重，
不是 RTX 5090 本身性能不足。

同一高效默认档又以 `ISAAC_LIDAR_RATE_HZ=7` 验收：3.0167 仿真秒产生 20 对，
`sim_hz=7.000000042`、`wall_hz=10.96`、95.76 app FPS、未配对丢帧为 0，RESULT PASS。
这证明 1--30 Hz 参数链并非只对默认 10 Hz 特判。短测在 RESULT 之后、Kit 销毁 RTX
render product 时仍可能打印 GMO invalid-magic warning；它是当前 Isaac 6.0.1 的退出阶段
噪声，不应被当成运行中雷达失效，但长期任务仍应以 RESULT、bag 合同和内核监控共同验收。

2026-08-13 以官方 `rplidar_s2e` 直接请求 15 Hz 的短测没有通过：5 仿真秒虽生成 50 对、
两路未配对丢帧均为 0且墙钟吞吐约 17.57 对/s，但仿真时间频率只有约 9.93 Hz。该结果说明
机器/UDP/pairing 吞吐不是当次主瓶颈，官方 profile 的 10 Hz authored 基准才是上限。因而
15 Hz 测试必须显式选择 `navigation_2d_32k`，不能把 `rplidar_s2e` 的目标数字强改为 15。

录包默认只要求 `sim_hz`。如果任务明确要求真实时间也达到目标，额外启用实时门槛：

```bash
ISAAC_REQUIRE_REALTIME_LIDAR=1 \
  bash isaac_sim/scripts/record_rosbag.sh 60
```

此模式会在录制前同时检查两路 `sim_hz` 和 `wall_hz`，不足即拒绝录制；验包时也可用同一
环境变量要求 bag 的 receipt-time rate 达标。实时门槛失败时，应先关闭 GUI/行人或降低
`ISAAC_LIDAR_RATE_HZ`，而不是复制旧帧或篡改时间戳。

等待终端输出：

```text
WAREHOUSE_PEOPLE_ROBOT_READY=
```

它表示 IRA 行人、自有机器人、碰撞感知麦克纳姆控制、双 RTX LiDAR 强度输出和 UDP
遥测端已经建立。若启动器
因当前开机已有 GPU/内核致命签名而拒绝运行，不要绕过检查。

## 一条命令运行自有场景 DRL-VO Demo

以下入口会依次启动 Engineering Lobby、带连续步行动画的 IRA 行人、Mecanum730、
与 Gazebo 策略输入合同一致的 15 Hz/每路 2000 束双 PhysX 距离雷达及合并器，等待 ROS
合同就绪后再启动 DRL-VO base 策略和 RViz。该模式不要求 `LaserScan.intensities`：

```bash
cd /home/user/navigation_project/a_pipeline
bash isaac_sim/scripts/run_custom_people_drlvo_demo.sh
```

默认控制模式是 `policy`，默认雷达模式是 `physx`。需要原生强度时仍可显式设置
`ISAAC_LIDAR_MODE=rtx`；未显式设置时两种模式都是每路 2000 束、15 Hz。两种模式都保留
`/scan_01`、`/scan_02` 和 `/scan_merged`，DRL-VO base 策略只消费距离，不消费强度。
策略模式默认在当次运行目录写入 `trajectory.csv` 和
`closed_loop_demo_summary.json`；可用 `ISAAC_DEMO_RECORD_TRACE=false` 关闭。

默认不会预先发送终点。ROS 节点就绪后会自动弹出完整地图窗口：蓝色圆点是机器人，点击
地图空闲区域会显示黄色目标，按“发布目标并隐藏窗口”后才开始导航；机器人到达并稳定停止
后，窗口会再次自动弹出选择下一终点。

若只想跳过弹窗并直接运行已验收的固定目标 `(6,4)`：

```bash
ISAAC_DEMO_AUTO_GOAL=true \
  bash isaac_sim/scripts/run_custom_people_drlvo_demo.sh
```

同一场景和传感器切换到纯遥控模式时，不会启动 DRL-VO `/cmd_vel` 发布者：

```bash
# 终端 1
ISAAC_DEMO_CONTROL_MODE=teleop \
ISAAC_PEDESTRIAN_COUNT=25 \
ISAAC_PEDESTRIAN_SEED=15 \
ISAAC_PEDESTRIAN_SPEED=0.9 \
  bash isaac_sim/scripts/run_custom_people_drlvo_demo.sh

# 终端 2：等待 ISAAC_CUSTOM_TELEOP_READY 后执行
ISAAC_ROS_DOMAIN_ID=78 bash isaac_sim/scripts/teleop_robot.sh
```

一次只能选择 `teleop` 或 `policy`。这保证 `/cmd_vel` 始终只有一个发布者，与 Gazebo
采集合同一致。需要同时验收 range-only 雷达频率、唯一控制发布者、非零速度命令和里程计
位移时，在第三个终端运行：

```bash
source /opt/ros/humble/setup.bash
source workspaces/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=78 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 isaac_sim/scripts/check_capture_ready.py \
  --timeout 60 --verify-lidar-rate --verify-motion
```

自动策略的固定目标闭环验收可直接在入口中开启；只有收到已接受目标、非空全局路径、
局部/最终目标、成功的 DRL-VO 推理、非零 `/cmd_vel` 以及至少 5 cm 里程计位移后才打印
`ISAAC_DRLVO_NAVIGATION_VERIFIED=PASS`：

```bash
ISAAC_DEMO_AUTO_GOAL=true \
ISAAC_DEMO_VERIFY_NAVIGATION=true \
ISAAC_DEMO_RVIZ=false \
ISAAC_PEDESTRIAN_COUNT=25 \
ISAAC_PEDESTRIAN_SEED=15 \
ISAAC_PEDESTRIAN_SPEED=0.9 \
  bash isaac_sim/scripts/run_custom_people_drlvo_demo.sh --headless
```

该验收命令打印 `ISAAC_DRLVO_SMOKE_TEST=PASS` 后会自动安全关闭 Isaac 和策略；如需验收
通过后继续观察场景，增加 `ISAAC_DEMO_EXIT_AFTER_VERIFY=false`。

无窗口 smoke test 可附加 `--headless`。日志按运行时间保存在
`runs/isaac_custom_drlvo_demo/<时间>/isaac.log` 和 `drlvo.log`。启动策略前会检查
`/scan_01`、`/scan_02`、`/odom` 和行人真值各自只有一个发布者；若 ROS domain 已由
另一套 Gazebo/Isaac 占用，脚本会明确退出，不会删除正在采集的任务。
`ISAAC_DEMO_CLEAN_STALE=1` 已被明确禁用并会直接报错；需由操作者只停止自己确认拥有的进程。
随后脚本还会用本次运行的
新鲜数据验证双雷达束数/仿真频率、时钟、TF、里程计、合并雷达和行人真值；Isaac 启动
失败时不会再误报 READY。

弹窗可用 `ISAAC_DEMO_GOAL_PICKER=false` 显式关闭。无人值守采集使用同一个入口时会自动
关闭弹窗，启动 rosbag 和基于静态地图 A* 可达性的短/中/长距离
分桶目标调度器；目标安全膨胀 0.5 m、路径膨胀 0.4 m，并按 seed 保存逐 episode 状态：

```bash
ISAAC_DEMO_AUTO_CAPTURE=1 \
ISAAC_DEMO_CAPTURE_DURATION_SEC=1800 \
ISAAC_DEMO_COLLECTION_SEED=7001 \
ISAAC_DEMO_RVIZ=false \
  bash isaac_sim/scripts/run_custom_people_drlvo_demo.sh --headless
```

bag、调度状态和两端日志位于同一个时间目录。只想录当前手工/单目标 Demo 时设置
`ISAAC_DEMO_RECORD_BAG=1`；采集合同包含双雷达、合并雷达、odom/TF/clock、控制标签、
唯一 ID 的 8 人真值、目标/局部子目标、episode 事件和传感器配置。

## 选择离线场景

启动器通过 `ISAAC_SCENE` 选择已有的本地 Isaac 6.0 资产；不设置时仍使用原来的
`warehouse`，因此现有录包流程和 IRA 三名行人的行为不变。

| `ISAAC_SCENE` | 本地 USD | 行人 |
|---|---|---|
| `warehouse`（默认） | `Isaac/Samples/BehaviorTree/IRA_OBT_Sample_Warehouse.usd` | 默认启用现有 IRA patrol |
| `simple_room` | `Isaac/Environments/Simple_Room/simple_room.usd` | 关闭 |
| `hospital` | `Isaac/Environments/Hospital/hospital.usd` | 关闭 |
| `digital_twin_warehouse` | `Isaac/Environments/Digital_Twin_Warehouse/small_warehouse_digital_twin.usd` | 关闭 |
| `custom` | `isaac_sim/scenes/a_pipeline_eng_lobby.usda`（默认，可覆盖） | 默认启用 8 人 Gazebo 路线巡逻 |

例如把同一台 Mecanum730/XMS5、双雷达和 ROS/UDP bridge 加载到 Hospital：

```bash
cd /home/user/navigation_project/a_pipeline
ISAAC_SCENE=hospital \
ISAAC_PEDESTRIAN_AVOIDANCE_MODE=off \
  bash isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh
```

其它场景同样只需把 `hospital` 改成 `simple_room`、`digital_twin_warehouse` 或 `custom`。
三个官方非 Warehouse 场景默认关闭行人；`custom` 使用独立的 Engineering Lobby IRA
配置，不会错误套用 Warehouse 路径。三个官方非 Warehouse 场景的机器人出生点为
`(0, 0, 0.30 m)`；项目 `custom` 场景为 `(2, 2, 0.01 m)`，并默认启用动态刚体、重力和
地面接触。第一次使用任一新场景时应在
GUI 确认机器人位于可通行地面，再进行录包或 SLAM 验收。

### 项目自有场景：Gazebo V7 Engineering Lobby

`custom` 的默认 USD 是本项目自己的资产，不依赖在线 Nucleus：

```text
isaac_sim/scenes/a_pipeline_eng_lobby.usda
```

它由 Gazebo V7 默认场景
`workspaces/ros2_ws/src/semantic_nav_gazebo/worlds/gazebo_eng_lobby.world` 确定性生成，
保留其中 79 个当前启用的静态盒体碰撞、米制 X/Y 坐标、Z 轴高度和 yaw，并另外提供：

- 40 m × 32 m 的可碰撞地面；
- PhysX 重力场景；
- 墙体和矮障碍的本地材质、灯光及语义标签；
- 与 Gazebo V7 相同的机器人出生平面坐标 `(2, 2)`，Isaac 初始根高度为 `0.01 m`；
- 116.189 kg 动态 PhysX 底盘、重力和地面/墙体接触，横滚和俯仰锁定以保持导航底盘直立；
- 可烘焙 NavMesh，以及从 `eng_hall_15.xml` 移植的书店、咖啡区、东/西大厅和电梯路线；
- 本机 `Isaac/People/Characters` 蒙皮人物和 HumanMotionLibrary `WalkForward` 连续步态；
- 场景源文件 SHA256 和迁移数量，便于以后确认 USD 是否过期。

直接使用这个自有场景：

```bash
cd /home/user/navigation_project/a_pipeline
ISAAC_SCENE=custom \
ISAAC_PEDESTRIAN_AVOIDANCE_MODE=gentle \
  bash isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh --deterministic
```

出现 `WAREHOUSE_PEOPLE_ROBOT_READY=` 后，后续仍使用同一套遥控、录包和 SLAM 脚本；
机器人、双 LiDAR、`/odom`、TF、`/clock` 和 13 个 ROS 话题合同不会因为换场景或在
range-only PhysX/原生 RTX 间切换而改变。

Gazebo 场景有修改时，可重新生成并检查 USD：

```bash
bash isaac_sim/scripts/build_custom_scene.sh
python3 isaac_sim/scripts/convert_gazebo_boxes_to_usda.py \
  --expected-boxes 79 --check
```

生成器只迁移面向 2D 导航的静态 box collision。Gazebo 插件、actor、`model://` include、
mesh、joint 和动态行人不会被错误近似。源 world 内 15 个藏在地下的 Gazebo 行人碰撞
代理因此被有意跳过；可见动态人物由独立的
`scripts/ira_people_demo/custom_eng_lobby_people.yaml` 作为 schema/资产模板创建。人数由
`ISAAC_PEDESTRIAN_COUNT` 决定（脚本默认 19；例如显式设为 8），按 Gazebo 六簇权重分配。
所有出生点在已知自由区内全局分配并要求至少 1.0 m 中心距，随后由启动前 validator 再次
检查；NavMesh 会绕过墙体。它们不是移动圆柱，也不是 Warehouse 路径的直接复制。

### 换成后续自己的 USD 或另一个 Gazebo 盒体场景

若已经在 Isaac Sim GUI 中搭好并保存了自己的 `.usd/.usda`，无需改 Python 代码：

```bash
ISAAC_SCENE=custom \
ISAAC_CUSTOM_SCENE_USD=/绝对路径/your_scene.usda \
ISAAC_CUSTOM_SPAWN_X_M=2.0 \
ISAAC_CUSTOM_SPAWN_Y_M=2.0 \
ISAAC_CUSTOM_SPAWN_Z_M=0.01 \
ISAAC_ROBOT_PHYSICS=1 \
  bash isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh
```

自定义 USD 应使用米制场景，up axis 为 Z 或 Y，并包含地面及静态碰撞体。若来源仍是以静态
box 为主的 Gazebo 世界，也可生成到新文件：

```bash
bash isaac_sim/scripts/build_custom_scene.sh \
  /绝对路径/your_world.world \
  /home/user/navigation_project/a_pipeline/isaac_sim/scenes/your_scene.usda
```

然后把该输出路径传给 `ISAAC_CUSTOM_SCENE_USD`。第一次必须先用 GUI 低速遥控，确认出生点、
单位、up axis、碰撞和雷达回波，再用于批量 rosbag、SLAM 或训练采集。
任意替换的 USD 默认关闭行人，因为现有 8 人坐标只匹配 Engineering Lobby；为新场景完成
NavMesh 和专用 IRA 路线配置前，不应强制设置 `ISAAC_ENABLE_PEOPLE=1`。

### 自有场景已完成的短程验收（2026-08-23）

- Isaac 6.0.1 正确打开本地 USDA，识别 `upAxis=Z`、`metersPerUnit=1.0`，机器人在
  `(2, 2, 0.30 m)` 直立加载；
- 机器人从测试出生点 `(2, 4, 0.30 m)` 向 `grey_wall_2` 横移 3.64 m 后，在
  `Y=0.36 m` 停止，碰撞保护命中 112 次且没有穿墙，结果 `PASS`；
- 原生双 RPLIDAR S2E RTX LiDAR 在 3.0167 秒仿真内产生 30 对扫描，`sim_hz=10.0`、
  `wall_hz=19.12`，前后雷达未配对丢帧均为 0；
- ROS/UDP bridge 和双雷达合并节点正常建立，ready 信息包含完整 13 个采集话题；退出后
  本次 bridge/merger 进程均被回收。
- 自有 NavMesh 成功烘焙，8/8 个本地蒙皮人物加载，长测 `people_moving=8/8`；
- `/pedestrian_ground_truth` 实读为 8 个唯一 ID，包含各自位置、朝向和二维速度；
- 一键 Demo 实际加载 163-weight base checkpoint，在 CUDA 上从 `(2,2)` 导航到测试目标
  `(6,4)`，规划 32 点路径，最终 odom 为 `(5.690,3.867)` 并由到达节点判定成功；
- 同轮以 15 Hz 为传感器请求值、每路 2000 槽；8 人负载下长测实际约 9.86 Hz，因此一键
  Demo 保守默认 10 Hz。`/scan_merged` 连续发布，结束后 Isaac、relay、merger 和策略进程
  全部回收。验收日志位于
  `runs/isaac_custom_drlvo_demo/20260823_165928/`。

以上证明场景加载、动态行人、DRL-VO 闭环控制、静态碰撞、双 RTX LiDAR 和 ROS 入口已经
连通。当前尚未为该场景保存一张完整 SLAM 地图，也没有完成 30 分钟自有场景 rosbag 严格
验包；正式训练采集前仍应按本文“使用 2D 双雷达遥控建图”和“严格验包”完成长时流程。

## 终端 2：启动键盘遥控

等待终端 1 ready 后运行：

```bash
cd /home/user/navigation_project/a_pipeline
bash isaac_sim/scripts/teleop_robot.sh
```

启动后先不要按方向键，先到终端 3 开始录包。一次只能有一个 `/cmd_vel` 发布者；脚本
发现其他遥控或导航节点时会拒绝启动。

常用按键：

```text
u / i / o：前左转 / 前进 / 前右转
j / l：左转 / 右转
m / , / .：后左转 / 后退 / 后右转
U / I / O：前左横移 / 前进 / 前右横移
J / L：左横移 / 右横移
M / < / >：后左横移 / 后退 / 后右横移
k 或空格：立即停车
q / z：同时增加 / 减小线速度和角速度
w / x：增加 / 减小线速度
e / c：增加 / 减小角速度
Ctrl-C：发送零速度并退出
```

方向指令以 20 Hz 持续发布。默认线速度为 0.5 m/s、角速度为 1.0 rad/s，可在启动前修改：

```bash
ISAAC_TELEOP_LINEAR_SPEED=0.3 \
ISAAC_TELEOP_ANGULAR_SPEED=0.6 \
  bash isaac_sim/scripts/teleop_robot.sh
```

Isaac 运行时的极端值保护线速度上限为 10.0 m/s，正常遥控速度不会再被旧的 0.6 m/s
上限截断。高速运动会缩短碰撞检测和人工停车的反应时间，实际速度仍应通过
`ISAAC_TELEOP_LINEAR_SPEED` 或 teleop 的 `w/x`、`q/z` 按键控制。

## 终端 3：录制遥控 rosbag

推荐先用手动结束模式：

```bash
cd /home/user/navigation_project/a_pipeline
bash isaac_sim/scripts/record_rosbag.sh
```

脚本在同一个 DDS 上下文中一次性检查 13 个话题的类型、发布者、实时样本和 `/clock`
推进情况；同时核对自描述传感器配置、测量 `/scan_01` 与 `/scan_02` 的真实频率，并默认
要求 `/scan_01`、`/scan_02`、`/scan_merged` 都具有与 ranges 等长且包含非零返回的强度
数组。看到下面一行后，才回到终端 2 按方向键：

```text
CAPTURE_READY: start driving now.
```

结束顺序：

1. 在遥控终端按 `k`；
2. 保持停车至少 1 秒仿真时间；
3. 在录包终端按 `Ctrl-C`；
4. 等待 `CAPTURE_COMPLETE` 和 `metadata.yaml` 写完；
5. 再退出遥控或关闭 Isaac。

也可以按仿真时间自动录制，例如 180 秒：

```bash
bash isaac_sim/scripts/record_rosbag.sh 180
```

参数按 `/clock` 计算，不是墙钟时间。自动录制临近结束时仍应先按 `k`，保留停车样本。

输出目录不会覆盖旧数据：

```text
isaac_sim/bags/<时间>_isaac_6_teleop/
isaac_sim/bags/logs/<时间>_isaac_6_teleop_record.log
isaac_sim/bags/logs/<时间>_isaac_6_teleop.env
```

## 录制话题

话题合同与 Gazebo/V7 采集保持一致：

```text
/scan                         sensor_msgs/msg/LaserScan
/scan_01                      sensor_msgs/msg/LaserScan
/scan_02                      sensor_msgs/msg/LaserScan
/scan_merged                  sensor_msgs/msg/LaserScan
/odom                         nav_msgs/msg/Odometry
/tf                           tf2_msgs/msg/TFMessage
/tf_static                    tf2_msgs/msg/TFMessage
/clock                        rosgraph_msgs/msg/Clock
/cmd_vel                      geometry_msgs/msg/Twist
/cmd_vel_stamped              geometry_msgs/msg/TwistStamped
/pedestrian_ground_truth      semantic_nav_gazebo/msg/PedestrianStateArray
/data_collection/episode_event std_msgs/msg/String
/data_collection/sensor_config std_msgs/msg/String
```

`/data_collection/sensor_config` 使用 transient-local QoS，记录雷达模式、目标频率、束数、
`lidar_rate_basis=simulation_time`、ROS header timestamp domain、RTX 配对 timestamp domain、
时间模式以及 producer、UDP bridge、launcher 的 SHA256。运行日志与 RESULT 另分开报告 sim-time pair rate、wall-time
delivery rate 和 RTF。验包脚本优先从 bag 内读取目标
频率，因此自定义频率的 bag 不需要事后凭记忆填写验收参数；旧 bag 没有该话题时仍按
历史默认 10 Hz 检查，也可显式传入 `ISAAC_LIDAR_RATE_HZ` 覆盖。

双雷达默认是 Isaac Sim 6.0.1 的 `rplidar_s2e` 原生 RTX LiDAR，启用 FULL GMO
辅助输出。默认频率是 10 Hz，也可通过 `ISAAC_LIDAR_RATE_HZ` 配置；每个原生回波按最近
回波投影为 `ISAAC_LIDAR_SAMPLE_COUNT` 指定的角度槽（默认 360）、0.5–50 m 的 ROS
`LaserScan`。`ranges` 的无回波槽为 `+inf`；
`intensities` 与 ranges 等长，保存 RTX
归一化 `scalar` 映射后的 0–255 强度，无回波槽为 0。该强度由 RTX 材质响应、入射方向和
传感器模型产生，不是固定值；`/scan_merged` 选择每个角度槽最近的回波并同步保留其强度。
系统桥把仓库 X-Z 平面转换为右手 ROS `odom` 的 X-Y 平面：ROS `+X=stage +X`，
ROS `+Y=stage -Z`。
因此 `j/u` 为 ROS 正角速度左转，`l/o` 为右转，`J/L` 分别为左/右横移。
`cmd_vel_stamped` 只有在 rosbag 已真正订阅
原始 `/cmd_vel` 后才开始发布，防止启动发现阶段出现无法配对的控制标签。第一次非零
指令产生手动遥控 episode `start`；停车持续 0.5 秒仿真时间后产生 `end`。该事件使用
`isaac_manual_teleop_episode/v1`，明确表示没有导航目标；不能冒充正式 V7 的 goal-conditioned
episode。若要直接生成目标条件训练数据，还需先接入 `/goal_pose` 和 online subgoal。

如需复现旧的距离-only PhysX 结果，可显式使用：

```bash
# 终端 1
ISAAC_LIDAR_MODE=physx \
  bash isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh

# 终端 3：只有 PhysX 回退模式才关闭强度门禁
ISAAC_REQUIRE_LIDAR_INTENSITY=0 \
  bash isaac_sim/scripts/record_rosbag.sh
```

## 严格验包

指定刚录制的 bag：

```bash
cd /home/user/navigation_project/a_pipeline
bash isaac_sim/scripts/check_rosbag.sh \
  isaac_sim/bags/<时间>_isaac_6_teleop
```

不传路径时检查最新一个 Isaac 6.0.1 teleop bag：

```bash
bash isaac_sim/scripts/check_rosbag.sh
```

检查内容包括：

- 13 个话题的类型和非零消息数，新 bag 还包含自描述的雷达模式、频率和束数；
- 双雷达束数、量程和仿真时间频率；
- 四个 LaserScan 话题的强度长度、非零返回和非恒定强度分布；
- `cmd_vel_stamped` 与 `/clock`、扫描的时间重叠；
- 原始和 stamped 控制分布一致；
- 至少有一条运动指令，并且最后一条 `/cmd_vel` 为零；
- 手动遥控 episode 的 start/end 成对且时间递增；
- 行人位置差分与 Twist 的速度、方向和时间连续性一致。

只有最后出现以下内容才算这包可用：

```text
teleop_command_semantics: PASS
manual_episode_intervals: PASS
pedestrian_ground_truth_kinematics: PASS
ISAAC_TELEOP_ROSBAG_CHECK=PASS
```

## 已完成的验收

2026-08-10 已完成真实 Isaac 6.0.1 GUI 联合验收：42 秒仿真约 92.23 FPS，自有机器人
遥控前进 1.7568 m，3/3 名 IRA 行人运动，RTX 5090 Vulkan/CUDA 正常。严格通过的
rosbag 为 `isaac_sim/bags/20260810_011319_isaac_6_teleop`，共 3207 条消息、12 个话题；
双雷达均为 360 束、约 10.0015 Hz，最终零速度、完整手动 episode、里程计、TF 和行人
运动学检查全部 PASS。这个历史包生成于 RTX 强度接入之前，`intensities` 为空；新的默认
RTX 录包会额外通过 `lidar_intensity_contract: PASS`。

2026-08-10 又完成转向坐标和碰撞代理验收：ROS 正角速度按左转方向应用；可视体尺寸约
`0.585 x 1.761 x 0.447 m`，带 2 cm 平面安全余量的碰撞代理约为
`0.625 x 1.721 x 0.487 m`。临时测试墙放在出生点前 0.8 m 时，机器人在前进约
0.397 m 后被 PhysX sweep 持续阻挡，没有穿透。碰撞代理只对场景中已有 PhysX collider
的物体生效，仍不等价于完整轮地接触和轮臂多刚体动力学。

同日又完成历史快速侧闪模式验收（当时为 `ISAAC_PEDESTRIAN_DODGE=1`，现等价于
`ISAAC_PEDESTRIAN_AVOIDANCE_MODE=legacy_dodge`）：3 名行人的
BehaviorAgent 均显式开启 obstacle/auto avoidance，
机器人碰撞代理作为 1000 kg 运动学刚体体积参与避让，并在行人继续靠近框外 1.2 m 时
暂停该行人的 native patrol、完整执行一次原生侧向 `dodge`，等待 0.5 秒动作切换间隔后
恢复巡逻。机器人固定在 Patrol Point A/B 之间运行 15.01 秒，3/3 行人各触发一次绕行动作；
105 个近距离采样的最小“行人中心到机器人碰撞框”间距为 0.4575 m，严格高于 0.15 m
人体外观余量，进入碰撞框的采样为 0。验收日志为
`isaac_sim/scripts/logs/warehouse_people_robot_6_0_20260810_110732.log`；另一轮独立通过的
最小净距为 0.4690 m，日志为
`isaac_sim/scripts/logs/warehouse_people_robot_6_0_20260810_110536.log`。

运行时 A/B 开关如下；启用行人时默认是已通过定向人–机器人验收的 `gentle`：

```bash
ISAAC_ROBOT_COLLISION_PROTECTION=1 ISAAC_PEDESTRIAN_AVOIDANCE_MODE=off \
  bash isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh

ISAAC_ROBOT_COLLISION_PROTECTION=1 ISAAC_PEDESTRIAN_AVOIDANCE_MODE=native \
  bash isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh

ISAAC_ROBOT_COLLISION_PROTECTION=1 ISAAC_PEDESTRIAN_AVOIDANCE_MODE=gentle \
  bash isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh

ISAAC_ROBOT_COLLISION_PROTECTION=1 ISAAC_PEDESTRIAN_AVOIDANCE_MODE=legacy_dodge \
  bash isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh
```

四个模式下机器人都只执行键盘 `/cmd_vel`，不会主动规划绕行。无论模式如何，行人之间的
BehaviorAgent obstacle/auto avoidance 都会被显式开启并使用统一质量参数；`off` 只让行人
忽略机器人，`native` 让机器人碰撞代理参加连续物体避障。实测 `native` 单独使用仍可能
让行人进入机器人碰撞框，因此默认 `gentle` 在
native 基础上仅于持续接近
0.25 秒且净距不超过 0.65 m 时，以 `motion_scale=0.75` 低速侧闪兜底，完成后 0.15 秒恢复
巡逻；`legacy_dodge` 完整保留历史 1.2 m、`motion_scale=2.0`、0.5 秒恢复延迟的快速侧闪。
旧变量 `ISAAC_PEDESTRIAN_DODGE=0/1` 继续兼容，分别等价于 `off/legacy_dodge`。

`WAREHOUSE_PEOPLE_ROBOT_RESULT` 现在在普通运行中也报告人–人最小中心距、视觉重叠、个人
空间违规比例、人–机器人最小净距、运行时复位次数和持续自由空间侵入。人–人距离低于
0.90 m 时，礼让层只把一人的当前 BehaviorAgent 速度临时降为零，超过 1.10 m 后恢复；
它不替换当前 `moveTo`、不跳过路线航点，也不写人物坐标。可用
`--test-pedestrian-social --duration 30 --headless --fast --no-ros` 做严格短测：所有人必须
移动、视觉重叠为零、个人空间违规比例不超过 5%，且不允许用瞬移恢复掩盖失败。

2026-08-24 的真实 Isaac 6.0.1 自有场景短测（8 人、seed 15、30 秒、native 基线）结果为：
8/8 移动，最小人间中心距 `0.4737 m`，视觉重叠 0，个人空间违规比例 `2.0813%`，
运行时复位 0，持续自由空间侵入 0，完整 `PASS`。仓库定向人–机器人测试中，`native`
最小净距为 `-0.219 m` 并失败；同配置 `gentle` 触发 3 次低速绕行，最小净距
`0.4440 m`、进入机器人 0 帧，完整 `PASS`。

## 使用 2D 双雷达遥控建图

Isaac 运行链路现在可以像 Gazebo V7 一样把 `/scan_01` 和 `/scan_02` 合成为
`/scan_merged`，再用 `slam_toolbox` 在线建立占据栅格地图。建图与 rosbag 录制相互独立，
可以同时运行；首次使用时建议先单独确认地图链路。

终端 1 启动场景并等待 `WAREHOUSE_PEOPLE_ROBOT_READY=`：

```bash
cd /home/user/navigation_project/a_pipeline
ISAAC_LIDAR_RATE_HZ=10 \
  bash isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh --deterministic
```

建图建议固定使用 `--deterministic`。这样 RTX 帧即使因渲染负载而降低墙钟吞吐，timeline
也会相应放慢，送给 SLAM 的扫描频率仍与配置一致。修改 bridge 或 producer 后必须完整停止并
重新启动终端 1；只重启 `slam_toolbox` 不会加载新的时间戳逻辑。

终端 2 启动 `slam_toolbox` 和 RViz，等待 `ISAAC_SLAM_READY`：

```bash
cd /home/user/navigation_project/a_pipeline
bash isaac_sim/scripts/start_slam_mapping.sh
```

无桌面或不需要 RViz 时使用：

```bash
ISAAC_SLAM_START_RVIZ=0 bash isaac_sim/scripts/start_slam_mapping.sh
```

终端 3 启动键盘遥控，缓慢覆盖仓库内可通行区域，并尽量在原路闭环处多停留几秒：

```bash
cd /home/user/navigation_project/a_pipeline
ISAAC_TELEOP_LINEAR_SPEED=0.3 ISAAC_TELEOP_ANGULAR_SPEED=0.6 \
  bash isaac_sim/scripts/teleop_robot.sh
```

终端 4 可随时检查建图链路：

```bash
cd /home/user/navigation_project/a_pipeline
bash isaac_sim/scripts/check_slam_mapping.sh
```

完成后先在遥控终端按 `k` 停车，再保存地图：

```bash
bash isaac_sim/scripts/save_slam_map.sh
```

也可以指定不会覆盖旧文件的地图名：

```bash
bash isaac_sim/scripts/save_slam_map.sh warehouse_round1
```

地图输出为 `isaac_sim/maps/slam/<名称>.yaml` 和 `.pgm`，运行日志位于
`isaac_sim/maps/logs/`。保存成功后再依次停止遥控、SLAM 和 Isaac。默认配置为 5 cm
分辨率，输入话题为 `/scan_merged`，坐标链为 `map -> odom -> base_link`。仓库动态行人
会被雷达短暂观测，可能产生局部残影；重复经过该区域、让动态观测被更新后再保存地图。

`isaac_sim/bags/20260808_205629_isaac_collection` 是旧 Isaac 运行链路的历史包，只能
作为消息合同参考，不能当作当前 6.0.1 IRA + 自有机器人入口已经完成真实联合验收的证据。

## Isaac 动作与实际速度评估合同

Isaac DRL-VO 评估把模型动作、控制命令和刚体实际状态分开记录。`/drl_vo/actuation_decision`
以同一 decision sequence 原子记录模型映射后的 raw physical action、最终 `/cmd_vel`、安全
gate 及原因；`/isaac/actuation_state` 记录 bridge 接收命令、Isaac 应用命令、命令年龄/watchdog
和实际 body twist。两者与 `/clock` 使用同一仿真时间轴，并由 evaluator 因果地以 15 Hz 重采样。

动态 PhysX 模式的 `actual_velocity_source=physx_rigid_body_api`，直接来自
`SingleRigidPrim.get_linear_velocity/get_angular_velocity`，转成 ROS `base_link` body frame；
线速度同时按 USD stage 的 `metersPerUnit` 从 stage units/s 转为 m/s（角速度保持 rad/s）；
非动态兼容模式仅使用严格正 dt 的固定 tick 世界位姿差分，首样本、重置和异常 dt 都标为无效。
因此 `/odom.pose` 仍是世界位姿，而 `/odom.twist` 是实际测得的速度，绝不再从 `/cmd_vel` 回填。

自动多 episode 重定位使用 Isaac 原生链路：scheduler 选择
`relocation_backend=isaac_pose_topic` 后发布 `/isaac/reset_pose`，ROS bridge 通过独立 UDP 端口转发；
simulator 仅在最终命令已停止、目标有限且目标碰撞查询通过时设置 PhysX pose，并将刚体线/角速度清零。
结果通过 `/isaac/reset_event` 回传，scheduler 仍须等到 fresh `/odom` 在位置、朝向和静止容差内才继续。
Gazebo 默认仍为 `gazebo_set_entity_pose`，不会改变原有 `/world/default/set_pose` 行为。

启用 `ISAAC_DEMO_RECORD_BAG=1` 时包会包含 decision/state、`/clock`、TF、目标、路径、episode、
行人 ground truth、推理指标和传感器配置。默认 policy demo 也会启用 episode evaluator，生成与
Gazebo 兼容的四个原始 CSV，并额外生成 `actuation_decisions.csv`、`simulator_actuation.csv`、
`actuation_alignment.csv` 和 episode/session JSON。tracking 中 bias 定义为 downstream - upstream；
线速度和角速度均报告 MAE、RMSE、bias、P50/P95/max、correlation、ratio 和 zero-command hold；
policy gate 与 simulator watchdog/collision protection 都参与分层。decision/state 分别使用因果 ZOH/最近样本，
二者 freshness 默认 0.20 s；最佳延迟在 0--0.50 s 内以 1/15 s 搜索，并让所有候选使用共同的下游
时间区间。样本不足、低方差命令、过期/重复时间戳、序列缺口或无新话题时输出 coverage/diagnostic
与 `null`/reason，而不是虚假的巨大数值。没有 raw action 的 stop decision 仍参与 final-to-actual 的
zero-hold 统计，但不会伪装成 raw model action。
`bias<=0.05 m/s`、`MAE<=0.08 m/s`、`P95<=0.15 m/s`、`correlation>=0.9`
仅作为 ungated、非零、稳定直行且无 watchdog/collision 样本的建议基准，不是拥挤转弯 episode 的硬门槛。

碰撞相关字段仍是 map/human overlap 或 Isaac 运动保护的 proxy；本合同不把它们宣称为 PhysX
接触传感器真值。运行前保持独占 ROS domain，绝不要设置清理旧进程的环境变量，也不要和 Gazebo
自动采集并行启动。
