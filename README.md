# a_pipeline：v7 双雷达语义导航数据流水线

这个目录是可复制交付包。支持在 Ubuntu 22.04 + NVIDIA 主机上依次完成：

1. Gazebo v7 双雷达 SLAM；
2. 保存占据栅格地图；
3. 使用 LabelMe 做人工语义标注；
4. 采集 ROS 2 bag；
5. 将每束雷达转换为语义标签；
6. 训练 S3-Net、SemanticCNN 和 DRL-VO；
7. 在 Gazebo 中运行 5 种感知或导航 Demo。
项目源码、Python 环境、地图、bag、数据集、日志和模型都位于本目录。ROS 2、Gazebo、系统库和 NVIDIA 驱动仍属于操作系统环境。

## 当前已验证基线（2026-07）

当前主要训练基线来自三个完整 bag/seed 隔离的数据集：

```text
任务根:
runs/20260717_042135_v7_dual/datasets/
20260727_three_bag_online_seed_split_v1/

train: 20260727_074611 / seed 17 / 9 episodes / 10065 samples
dev:   20260727_080451 / seed 27 / 8 episodes / 6679 samples
test:  20260727_084207 / seed 47 / 14 episodes / 11734 samples
total: 31 episodes / 28478 samples
```

该数据集使用 `/data_collection/episode_event` 的完整 start/end 区间、因果
`/cmd_vel_stamped` 和因果 online `/semantic_cnn/local_subgoal`。episode 事件的
rosbag storage time 通过 `/clock` 映射到 simulation time；命令和 subgoal 都只允许
匹配不晚于当前 scan 的消息。SemanticCNN 的 10 帧窗口不会跨 bag、episode 或 split。

聚合检查、逐包统计、CUDA smoke 和转换日志位于：

```text
runs/20260717_042135_v7_dual/datasets/
20260727_three_bag_online_seed_split_v1/control/
```

`aggregate_seed_split_check.json` 和 `cuda_training_smoke_final.json` 均为 PASS。
正式训练成果见第 11 节。这里的离线指标、shadow replay 和行为克隆结果都不能单独
证明闭环导航成功。

Isaac Sim 6.0.1 场景启动后，可以按与 Gazebo 相同的方式遥控并录制麦克纳姆机器人：

```bash
# 终端 1：IRA 动态行人仓库、自有机器人、ROS 采集桥
cd /home/user/navigation_project/a_pipeline
bash isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh

# 终端 2：等待 WAREHOUSE_PEOPLE_ROBOT_READY= 后启动遥控，先不要按方向键
cd /home/user/navigation_project/a_pipeline
bash isaac_sim/scripts/teleop_robot.sh

# 终端 3：出现 CAPTURE_READY 后回到终端 2 驾驶
cd /home/user/navigation_project/a_pipeline
bash isaac_sim/scripts/record_rosbag.sh
```

按键与原 Gazebo 遥控完全一致：`u/i/o` 前进与转向，`m/,/.` 后退与转向，
`J/L` 左右横移，`k` 或空格停车。
结束时先在遥控终端按 `k`，等待至少 1 秒仿真时间，再在录包终端按 `Ctrl-C`。
完整话题、自动定时和验包步骤见 `isaac_sim/README.md`。

Isaac Sim 中也已提供与 Gazebo V7 相同的“2D 双雷达 + 键盘遥控 + slam_toolbox”建图
操作。启动场景后分别运行：

```bash
bash isaac_sim/scripts/start_slam_mapping.sh
bash isaac_sim/scripts/teleop_robot.sh
bash isaac_sim/scripts/check_slam_mapping.sh
bash isaac_sim/scripts/save_slam_map.sh
```

地图保存在 `isaac_sim/maps/slam/`。终端顺序、RViz 开关和具名保存方式见
`isaac_sim/README.md` 的“使用 2D 双雷达遥控建图”。

项目也提供了从 Gazebo V7 默认 Engineering Lobby 迁移出的自有 Isaac USD 场景：

```bash
cd /home/user/navigation_project/a_pipeline
ISAAC_SCENE=custom \
  bash isaac_sim/scripts/run_isaac_6_0_warehouse_people_robot.sh --deterministic
```

该场景保留 79 个启用的静态盒体布局和碰撞，资产位于
`isaac_sim/scenes/a_pipeline_eng_lobby.usda`。后续可直接替换成自己的 USD，也可以从另一个
Gazebo 静态盒体世界重新生成；具体命令、边界和出生点配置见 `isaac_sim/README.md` 的
“项目自有场景”和“换成后续自己的 USD”小节。

要直接演示“自有场景 + 带步行动画的行人 + DRL-VO base 策略”，运行：

```bash
cd /home/user/navigation_project/a_pipeline
bash isaac_sim/scripts/run_custom_people_drlvo_demo.sh
```

该入口现在默认使用与 Gazebo 轻量测试配置一致的双 360 束、10 Hz PhysX 距离雷达，
不要求强度；显式设置 `ISAAC_LIDAR_MODE=rtx` 可切回原生 RTX。设置
`ISAAC_DEMO_CONTROL_MODE=teleop` 时只启动场景和传感器，随后在另一终端以
`ISAAC_ROS_DOMAIN_ID=78 bash isaac_sim/scripts/teleop_robot.sh` 遥控，避免与自动策略争用
`/cmd_vel`。自动闭环 smoke test 可同时设置 `ISAAC_DEMO_AUTO_GOAL=true`、
`ISAAC_DEMO_VERIFY_NAVIGATION=true` 和 `ISAAC_DEMO_RVIZ=false`；入口会验证目标接受、
全局/局部规划、DRL-VO 成功推理、非零控制以及里程计位移后再报告 PASS。若 domain 78
已有 Gazebo/Isaac 发布者，入口会退出并提示先结束任务或改用其他 domain，不会误用旧话题。

ROS 就绪后会自动弹出完整地图终点选择窗口；到达后再次弹窗。设置
`ISAAC_DEMO_AUTO_GOAL=true` 可跳过弹窗并使用已实测通过的 `(6,4)`；同一个入口可用
`ISAAC_DEMO_AUTO_CAPTURE=1` 开启自动目标调度和 rosbag 数据采集，完整参数见
`isaac_sim/README.md`。

当前采集模式会导入自有机器人 USD 的完整外观/底座层和原始尺寸，并用确定性的
麦克纳姆根位姿控制保证 IRA 行人、双雷达和 rosbag 的实时性能。机器人同时带有按可视
包围盒生成的不可见运动学碰撞盒，平移使用 PhysX box sweep、转动使用 overlap 检查，
遇到仓库中已有碰撞体的墙面或物体会停止，不再直接穿模。高开销的轮臂多刚体物理层
仍不参与运行，因此不模拟轮地打滑或车轮外观旋转。启动日志中的
`navigation_control=collision_stop_only_kinematic_mecanum_visual_proxy`、
`collision_aware_motion=true`、`wheel_velocity_control=false` 会明确标出这一运行模式。
默认 `ISAAC_ROBOT_COLLISION_PROTECTION=1`、
`ISAAC_PEDESTRIAN_AVOIDANCE_MODE=off`：机器人只在碰撞查询命中时硬停车，不会自动转向
或侧移，行人也不会针对机器人执行额外动作。行人/机器人 A/B 现提供 `off`、`native`、
`gentle`、`legacy_dodge` 四档；推荐 `gentle`，以 Isaac 原生连续物体避障为主，只在行人
持续接近 0.25 秒且距机器人碰撞框不超过 0.65 m 时执行 `motion_scale=0.75` 的低速侧闪。
`native` 完全不强制侧闪，`legacy_dodge` 保留历史 1.2 m、2.0 倍快速侧闪。
旧的 `ISAAC_PEDESTRIAN_DODGE=0/1` 仍分别映射为 `off/legacy_dodge`。所有模式使用完全
相同的实体碰撞盒和仓库。`ISAAC_ROBOT_COLLISION_PROTECTION=0` 只用于无碰撞保护的
基线测试。

底层通用场景入口默认仍支持 Isaac Sim 6.0.1 原生 RTX LiDAR；自有场景 DRL-VO/遥控一键
入口为了 25 人负载默认选择 PhysX range-only。PhysX 的强度数组为空；RTX 的
`LaserScan.intensities` 保存由材质、射线方向和传感器模型产生的强度（0–255）。两种模式
的 `/scan_01`、`/scan_02`、`/scan_merged`、TF 和时间戳合同保持不变。录制 PhysX 数据时
设置 `ISAAC_REQUIRE_LIDAR_INTENSITY=0`。

## 0. 空间与系统要求

- Ubuntu 22.04 x86_64；
- NVIDIA 显卡和可工作的驱动；
- 安装环境前至少预留 20 GiB；
- 安装 ROS 和 Python 依赖时需要网络；
- 不要把 .venvs/、workspaces/ros2_ws/build/ 或 install/ 从另一台机器复制过来。
以下命令假设已经进入收到的目录：

```bash
cd /home/user/navigation_project/a_pipeline
```

## 1. 第一次安装

以下命令只需执行一次。系统依赖安装会使用 sudo；如果检测到 NVIDIA 硬件但驱动未加载，
`01_install_system_deps.sh` 会安装系统推荐驱动。安装驱动后必须重启，再从第一个命令
重新执行；不要在未重启的情况下继续 Python/CUDA 验证。

```bash
bash environment/01_install_system_deps.sh
bash environment/00_check_host.sh
bash environment/02_create_python_envs.sh
bash environment/03_build_ros2_workspace.sh
bash environment/04_verify_install.sh
```

`03_build_ros2_workspace.sh` 会检查 ROS 工作区中是否残留另一台机器的 CMake
缓存；如果发现旧路径，会先把旧的 `build/`、`install/`、`log/` 移到带时间戳的
`.stale-*` 目录，再从当前项目路径重新构建。旧构建产物不会被删除。

如果第一个命令安装了 NVIDIA 驱动并输出 `ACTION REQUIRED`，先重启电脑；重启后从
`bash environment/00_check_host.sh` 继续，不要直接运行后面的 Python 或 ROS 命令。

如果安装过程中因系统已有旧版 Python ROS 包而中断，直接重新运行第一个命令即可；
脚本会自动移除冲突的旧包、修复未完成的 dpkg 状态并继续安装。

每次打开新终端先执行：

```bash
cd /home/user/navigation_project/a_pipeline
source environment/activate.sh
```

## 2. 创建一次独立实验并指定雷达参数

每次实验使用不同 RUN_ID，已有实验默认不会被覆盖。

```bash
source environment/activate.sh

export RUN_ID="$(date +%Y%m%d_%H%M%S)_v7_dual"
export LIDAR_SAMPLES_01=2000
export LIDAR_SAMPLES_02=2000
export LIDAR_UPDATE_RATE_01=15
export LIDAR_UPDATE_RATE_02=15
export LIDAR_RANGE_MIN_01=0.1
export LIDAR_RANGE_MIN_02=0.1
export LIDAR_RANGE_MAX_01=8.0
export LIDAR_RANGE_MAX_02=8.0

bash pipelines/v7_native_pipeline/scripts/00_create_run.sh
export RUN_MANIFEST="$A_PIPELINE_ROOT/runs/$RUN_ID/run_manifest.env"
```

后续每个新终端都执行下面三行，并把 RUN_ID 替换成这次实验的值：

```bash
cd /home/user/navigation_project/a_pipeline
source environment/activate.sh
export RUN_MANIFEST="$A_PIPELINE_ROOT/runs/20260717_042135_v7_dual/run_manifest.env"
```

这三行是在“切换到项目并加载本次实验配置”。

```bash
cd /home/user/navigation_project/a_pipeline
```

进入流水线项目根目录，保证后面的相对路径脚本能找到。

```bash
source environment/activate.sh
```

配置当前终端的项目环境变量，例如：
- A_PIPELINE_ROOT=/home/user/navigation_project/a_pipeline
- ROS 2 工作区路径、Python 虚拟环境路径
- ROS 日志、缓存目录
- runs/ 目录
它会输出类似：

```bash
A_PIPELINE_ROOT=/home/user/navigation_project/a_pipeline
export RUN_MANIFEST="$A_PIPELINE_ROOT/runs/你的RUN_ID/run_manifest.env"
```

指定“接下来所有步骤使用哪一次运行”的配置文件。把 你的RUN_ID 替换成之前实际创建的值，例如：

```bash
export RUN_MANIFEST="$A_PIPELINE_ROOT/runs/20260717_153000_v7_dual/run_manifest.env"
```
后续脚本通常会读取这个文件，从而知道 bag、地图、数据集、日志和模型该写到哪一个 runs/<RUN_ID>/ 隔离目录，避免不同实验互相混写。
所有输出都在：

```text
runs/<RUN_ID>/
```

## 3. 启动双雷达 SLAM

终端 1：

```bash
bash pipelines/v7_native_pipeline/scripts/01_start_v7_dual_slam.sh
```

保持终端 1 运行。终端 2 检查话题：

```bash
bash pipelines/v7_native_pipeline/scripts/02_check_topics.sh
```

应看到 /scan_01、/scan_02、/scan_merged、/odom、/tf、/clock 和 /map。

终端 3 遥控机器人建图：

```bash
bash pipelines/v7_native_pipeline/scripts/05b_teleop.sh
```

完成建图后，在终端 2 保存地图：

```bash
bash pipelines/v7_native_pipeline/scripts/03_save_slam_map.sh
```

地图输出在 runs/<RUN_ID>/maps/slam/。

## 4. 人工语义标注地图

准备 LabelMe 图片：

```bash
bash pipelines/v7_native_pipeline/scripts/04_prepare_labelme_map.sh
```

启动 LabelMe：

```bash
bash pipelines/v7_native_pipeline/scripts/04c_open_labelme.sh
```

允许类别固定为：Chair、Door、Elevator、Person、Pillar、Sofa、Table、Trash bin、Wall。

在 LabelMe 中保存为脚本提示的 map_labelme.json，然后导出像素语义图：

```bash
bash pipelines/v7_native_pipeline/scripts/04b_export_labelme_json.sh
```

语义图输出在 runs/<RUN_ID>/maps/semantic_label/，其中训练转换使用 map.yaml 和 label.png。
## 5. 采集带动态行人的 ROS 2 bag

修改 ROS 2 节点或拉取新版本后，先构建一次：

```bash
cd /home/user/navigation_project/a_pipeline/workspaces/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select semantic_nav_gazebo --symlink-install
```

当前 run：

```bash
cd /home/user/navigation_project/a_pipeline
source environment/activate.sh
export RUN_MANIFEST="$A_PIPELINE_ROOT/runs/20260717_042135_v7_dual/run_manifest.env"
```

### 终端 1：启动 Gazebo、双雷达与 SLAM

如果终端 1 已在运行仿真，但不是按本次行人参数启动的，先在该终端按 Ctrl-C 停止，再执行：

```bash
cd /home/user/navigation_project/a_pipeline
source environment/activate.sh
export RUN_MANIFEST="$A_PIPELINE_ROOT/runs/20260717_042135_v7_dual/run_manifest.env"

export SPAWN_SCENE_PEDESTRIANS=true
export PEDESTRIAN_COUNT=8
export PEDESTRIAN_SEED=7
export PEDESTRIAN_SPEED=1.0
export PEDESTRIAN_UPDATE_RATE=15
export PEDESTRIAN_SIMULATION_FACTOR=1.0
export START_RVIZ=false

bash pipelines/v7_native_pipeline/scripts/01_start_v7_dual_slam.sh
```

保持终端 1 运行，不要在录包期间停止它。

#### 行人数量规则

- 不设置 PEDESTRIAN_COUNT：保留场景 XML 中的默认人数；
- PEDESTRIAN_COUNT=0：不生成行人；
- PEDESTRIAN_COUNT=<正整数>：精确生成该人数，并按场景原有路线比例分配；
- PEDESTRIAN_SEED=7：固定随机种子，方便复现；
- PEDESTRIAN_SIMULATION_FACTOR 必须为 1.0，调速只修改 PEDESTRIAN_SPEED；
- 修改人数或种子后，必须重启终端 1 的仿真才会生效；
- 第一批正式数据推荐 8 人；通过多个 seed 扩充场景比单包使用 23 人更稳定。
  
### 终端 2：开始录制 rosbag

确认终端 1 的 Gazebo、SLAM、/scan_01 和 /scan_02 已正常启动。执行本节录制命令
之前，先按照下方“终端 3”启动一次 teleop，但先不要按方向键；`/cmd_vel`
必须恰好只有这一个发布者。然后回到终端 2 执行：

```bash
cd /home/user/navigation_project/a_pipeline
source environment/activate.sh
export RUN_MANIFEST="$A_PIPELINE_ROOT/runs/20260717_042135_v7_dual/run_manifest.env"

REQUIRE_PEDESTRIAN_GROUND_TRUTH=1 \
REQUIRE_CMD_VEL_PUBLISHER=1 \
CAPTURE_SIM_DURATION_SEC=180 \
  bash pipelines/v7_native_pipeline/scripts/05_record_rosbag.sh
```

出现以下信息后再开始驾驶：

```text
CAPTURE_READY: all requested topics are subscribed.
```

`CAPTURE_SIM_DURATION_SEC=180` 按 `/clock` 计时，表示 180 秒仿真数据，而不是
180 秒墙钟时间；脚本每 10 秒打印一次进度并在到时后正常结束 rosbag。设为 0
则保留手动 Ctrl-C 停止模式。

录包脚本会自动新建时间戳目录，不覆盖已有 raw bag，并记录：

/scan_01
/scan_02
/scan_merged
/odom
/tf
/tf_static
/cmd_vel
/cmd_vel_stamped
/pedestrian_ground_truth
/clock
/data_collection/episode_event

如果采集时已经启动在线规划器，还会自动附加
`/semantic_cnn/global_path`、`/semantic_cnn/local_subgoal` 和
`/semantic_cnn/final_goal`。用于 SemanticCNN、DRL-VO 或当前三包正式基线的数据，
必须记录 online `/semantic_cnn/local_subgoal`，并在 07b 中显式使用
`DUAL_SLOT_SUBGOAL_SOURCE=online`。转换器只做 episode 内 causal hold-last；无先验
或超过最大年龄的帧会被丢弃并统计原因。hindsight subgoal 只保留为明确标注的历史
兼容实验，不能与 online 数据混入同一正式训练集合。

其中 /cmd_vel_stamped 是后续控制标签对齐所需的话题。动态行人场景中的
/pedestrian_ground_truth 使用 odom 坐标系记录每个仿真人的 ID、Pose 和 Twist，
用于评估雷达行人检测、语义标签和速度估计；没有启动行人控制器时该话题不会产生消息。

### 终端 3：键盘控制机器人

打开第三个终端：

```bash
cd /home/user/navigation_project/a_pipeline
source environment/activate.sh
export RUN_MANIFEST="$A_PIPELINE_ROOT/runs/20260717_042135_v7_dual/run_manifest.env"

bash pipelines/v7_native_pipeline/scripts/05b_teleop.sh
```

按键后必须保持终端 3 焦点。方向指令会以 20 Hz 连续发布，按 k 或空格立即停车；Ctrl-C 退出时也会发送零速度。
不要再启动第二个 teleop；录包前可用 `ros2 topic info -v /cmd_vel` 确认发布者数量为 1。

#### 普通按键

```text
u / i / o：前左转 / 前进 / 前右转
j / l：左转 / 右转
m / , / .：后左转 / 后退 / 后右转
k 或空格：停车
```

速度调节：

```text
q / z：线速度和角速度同时 ×1.1 / ×0.9
w / x：仅线速度 ×1.1 / ×0.9
e / c：仅角速度 ×1.1 / ×0.9
```

Shift 大写组合与横移：

```text
U / I / O：前左横移 / 前进 / 前右横移
J / L：左横移 / 右横移
M / < / >：后左横移 / 后退 / 后右横移
t / b：线速度 z 正向 / 反向
```

当前底盘为平面移动底盘；若底盘不支持横移或 z 方向运动，对应指令可能不会产生可见位移。



默认初始线速度为 0.5 m/s，默认初始角速度为 1.0 rad/s；每次调速后终端会打印当前值。

### 停止录制并检查 bag

自动计时录制接近结束时：

1. 在遥控终端按 `k`，保留几秒零速度样本；
2. 等录包终端输出 `CAPTURE_COMPLETE`，再退出 teleop；
3. 终端 1 保持运行，在任意新终端检查刚刚这一个 bag：
  
```bash
cd /home/user/navigation_project/a_pipeline
source environment/activate.sh
export RUN_MANIFEST="$A_PIPELINE_ROOT/runs/20260717_042135_v7_dual/run_manifest.env"
source "$RUN_MANIFEST"

NEW_BAG="$LAST_BAG_DIR"
REQUIRE_PEDESTRIAN_GROUND_TRUTH=1 \
MIN_BAG_SIM_DURATION_SEC=170 \
  bash pipelines/v7_native_pipeline/scripts/06_check_bag.sh "$NEW_BAG"
```

手动录制模式先按 `k`，再回到录包终端按 Ctrl-C；脚本仍会正常结束并写完
`metadata.yaml`。若要检查其他指定 bag：

```bash
bash pipelines/v7_native_pipeline/scripts/06_check_bag.sh \
  "$A_PIPELINE_ROOT/runs/20260717_042135_v7_dual/bags/raw/<bag目录名>"
```

只有检查输出同时满足以下条件，才进入转换：

```text
PASS /scan_01
PASS /scan_02
PASS /odom
PASS /tf
PASS /tf_static
PASS /cmd_vel
PASS /cmd_vel_stamped
PASS /pedestrian_ground_truth
PASS /clock

cmd_vel_stamped_in_clock_range: True
cmd_vel_stamped_scan_overlap: True
pedestrian_ground_truth_kinematics: PASS
```

raw bag 位于：

```text
runs/<RUN_ID>/bags/raw/<时间戳>_v7_dual_teleop_bag/
```


---

## 6. 转换固定双雷达逐束语义训练数据（07b + 07c）

该流程保留两路原始雷达的束身份：

```text
/scan_01：前 N1 个固定槽位
/scan_02：后 N2 个固定槽位
```

不会进行重采样、跨雷达去重、角度分桶或 /scan_merged 融合。

当前 run 的配置为：

```text
/scan_01：2000 束，15 Hz，0.1–8.0 m
/scan_02：2000 束，15 Hz，0.1–8.0 m
07b 输出：4000 固定槽位
```

### 动态 Person 标签前提

dynamic 模式要求下面文件中存在名为 Person 的类别：

```text
runs/<RUN_ID>/maps/semantic_label/label_names.txt
```

类别 ID 不固定；07b 会按 label_names.txt 的实际行号读取 Person ID，绝不能假设 Person 是某个固定数字。

检查 Person 类是否存在：

```bash
cd /home/user/navigation_project/a_pipeline
source environment/activate.sh
export RUN_MANIFEST="$A_PIPELINE_ROOT/runs/20260717_042135_v7_dual/run_manifest.env"

grep -in '^Person$' \
  "$A_PIPELINE_ROOT/runs/20260717_042135_v7_dual/maps/semantic_label/label_names.txt"
```

若没有任何输出，先确认你确实需要 dynamic Person 标签；确认后才追加一次：

```bash
printf 'Person\n' >> \
  "$A_PIPELINE_ROOT/runs/20260717_042135_v7_dual/maps/semantic_label/label_names.txt"
```

这只追加新类别，不会改变既有静态类别的 ID。

dynamic 规则为：

```text
静态语义图没有类别，
且雷达端点落在 PGM 自由空间，
且该束有效并且不是 self-mask
→ 写入 label_names.txt 中实际的 Person ID。
```

这是一种动态自由空间端点规则，不是逐实例人工真值标注。

### 可选：按显式 allowlist 批量补齐 07b 与 07c

`07bc_convert_export_all_raw_bags.sh` 不再遍历整个 raw 目录。它要求
`DUAL_SLOT_BAG_ALLOWLIST_FILE`，且只接受位于当前 run 的 `bags/raw/` 下、带
`metadata.yaml` 的绝对路径；未提供 allowlist、重复路径或越界路径都会直接失败。
这样可以避免把旧 bag 或未经批准的 bag 混入训练集合。

当前三包基线可使用已经审计的 allowlist 和 overlay manifest：

```bash
cd /home/user/navigation_project/a_pipeline
source environment/activate.sh

export TASK_ROOT="$A_PIPELINE_ROOT/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1"
export RUN_MANIFEST="$TASK_ROOT/control/run_manifest.overlay.env"

DUAL_SLOT_BAG_ALLOWLIST_FILE="$TASK_ROOT/control/bag_allowlist.txt" \
DUAL_SLOT_OUTPUT_ROOT="$TASK_ROOT/fixed_slots" \
FIXED_DUAL_TRAINING_OUTPUT_ROOT="$TASK_ROOT/semantic2d" \
DUAL_SLOT_KEEP_INTERMEDIATE=0 \
DUAL_SLOT_PERSON_LABEL_MODE=ground-truth-legs \
DUAL_SLOT_SUBGOAL_SOURCE=online \
  bash pipelines/v7_native_pipeline/scripts/07bc_convert_export_all_raw_bags.sh
```

行为如下：

- 严格按 allowlist 的行序处理，不扫描其他 raw bag；
- 已完成 07b 和 07c：跳过；
- 缺少 07b：按当前 `DUAL_SLOT_PERSON_LABEL_MODE` 进行双雷达原始槽位转换；
- 缺少 07c：导出为训练目录；
- `DUAL_SLOT_KEEP_INTERMEDIATE=0` 时，仅在 07c 最终校验通过后删除本轮新生成的
  07b session；已有或失败任务的中间数据保留；
- 同名输出目录已存在：拒绝覆盖；
- 某 bag 的时间对齐、TF、频率或双雷达同步检查失败：该 bag 停止转换，但不影响其他 bag。
  
如需禁用自动 Person 标签：

```bash
cd /home/user/navigation_project/a_pipeline
source environment/activate.sh
export TASK_ROOT="$A_PIPELINE_ROOT/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1"
export RUN_MANIFEST="$TASK_ROOT/control/run_manifest.overlay.env"

DUAL_SLOT_BAG_ALLOWLIST_FILE="$TASK_ROOT/control/bag_allowlist.txt" \
DUAL_SLOT_OUTPUT_ROOT="$TASK_ROOT/fixed_slots" \
FIXED_DUAL_TRAINING_OUTPUT_ROOT="$TASK_ROOT/semantic2d" \
DUAL_SLOT_PERSON_LABEL_MODE=disabled \
DUAL_SLOT_SUBGOAL_SOURCE=online \
  bash pipelines/v7_native_pipeline/scripts/07bc_convert_export_all_raw_bags.sh
```

### 只转换一个刚录制的新 bag

先指定 bag：

```bash
cd /home/user/navigation_project/a_pipeline
source environment/activate.sh
export RUN_MANIFEST="$A_PIPELINE_ROOT/runs/20260717_042135_v7_dual/run_manifest.env"

NEW_BAG="$A_PIPELINE_ROOT/runs/20260717_042135_v7_dual/bags/raw/<新bag目录名>"
test -d "$NEW_BAG" || { echo "ERROR: bag 不存在: $NEW_BAG"; exit 1; }
```

对带行人真值的新包执行 ground-truth legs 07b：

```bash
export DUAL_SLOT_BAG_DIR="$NEW_BAG"
export DUAL_SLOT_PERSON_LABEL_MODE=ground-truth-legs
export DUAL_SLOT_SUBGOAL_SOURCE=online
export DUAL_SLOT_SUBGOAL_MAX_AGE_MS=300

bash pipelines/v7_native_pipeline/scripts/07b_convert_bag_to_fixed_dual_lidar.sh
```

如果该 bag 没有录到 online local subgoal，不要把它静默改成正式训练数据；应让
转换失败并单独判断是否只用于 S3-Net 或明确标注的历史兼容实验。

07b 会验证：

```text
双雷达束数
量程
实际仿真时间频率
双雷达同步
/cmd_vel_stamped 与 /clock 的时间对齐
map -> base_link TF
原始槽位数据与转换结果一致性
```

07b 成功后，在同一终端执行 07c：

```bash
bash pipelines/v7_native_pipeline/scripts/07c_export_fixed_dual_training_dataset.sh
```

### 输出位置

07b 的固定双雷达 NPZ 中间数据：

```text
runs/<RUN_ID>/datasets/fixed_dual_lidar_slots/
  <bag时间戳>-v7-fixed-dual-v3-<N1>x<N2>-converted-pedgt-v1-sgonline/
```

包含：

```text
samples/*.npz
train.txt
dev.txt
test.txt
metadata.json
projection_debug/*.png
```

07c 的训练目录：

```text
runs/<RUN_ID>/datasets/semantic2d_fixed_dual_native/
  <bag时间戳>-v7-fixed-dual-v3-<N1>x<N2>-training-pedgt-v1-sgonline/
```

根目录还包含：

```text
runs/<RUN_ID>/datasets/semantic2d_fixed_dual_native/dataset.txt
runs/<RUN_ID>/datasets/semantic2d_fixed_dual_native/label_names.txt
```

正式多包集合应使用显式 seed split manifest，把完整 bag 分配到 train/dev/test。
导出的文件名包含 bag timestamp、episode ID 和帧序号；SemanticCNN 按 metadata
顺序构造窗口，不按连续数字文件名猜测时序。当前三包集合的实际根目录是：

```text
runs/20260717_042135_v7_dual/datasets/
20260727_three_bag_online_seed_split_v1/{fixed_slots,semantic2d}/
```

07b 和 07c 的日志与检查报告位于：

```text
runs/<RUN_ID>/logs/07b_fixed_dual_lidar_*.log
runs/<RUN_ID>/logs/07b_fixed_dual_lidar_check_*.json
runs/<RUN_ID>/logs/07c_export_fixed_dual_training_*.log
runs/<RUN_ID>/logs/07c_export_fixed_dual_training_check_*.json
```

## 7. 训练 S3-Net

S3-Net 学习每一束固定槽位雷达的语义类别。固定双雷达数据会保留
/scan_01 在前、/scan_02 在后的束身份；类别数从训练根目录的
label_names.txt 自动读取，不能在命令或代码中假设某个类别 ID。

### 训练前检查

以下命令假定已完成第 6 节、dataset.txt 已列出所有要参与训练的 session，
并且当前终端已执行第 2 节的 `source environment/activate.sh` 和
`export RUN_MANIFEST=...`。先确认 GPU 可用：

```bash
"$TORCH_PY" -c 'import torch; print("cuda_available:", torch.cuda.is_available()); print("device:", torch.cuda.get_device_name(0) if torch.cuda.is_available() else "none")'
```

输出必须为 cuda_available: True。正式训练默认在没有 CUDA 时退出，不会悄悄改用长时间 CPU 训练。

### 固定双雷达 smoke

先运行一次只读数据索引检查和一批次 S3-Net + SemanticCNN 契约 smoke：

```bash
bash pipelines/v7_native_pipeline/scripts/08c_smoke_fixed_dual_training.sh
```

它不会转换、覆盖或删除已有训练 session；成功时会写入：

```text
runs/<RUN_ID>/training/fixed_dual_smoke_report.json
runs/<RUN_ID>/training/s3net/fixed_dual_range_incidence_smoke_stats.json
```

确认报告为 PASS 后再开始正式训练。正式训练使用 dataset.txt 中的全部 session，
先按 train split 计算归一化统计，再训练并自动评估 dev split。

### 正式 GPU 训练

前台运行：

```bash
S3NET_EPOCHS=301 \
S3NET_BATCH_SIZE=512 \
S3NET_CHECKPOINT_INTERVAL=10 \
  bash pipelines/v7_native_pipeline/scripts/09_train_s3net_native_stats.sh
```

没有 tmux 时，可在当前目录后台运行，并把终端关闭后仍保留日志：

```bash
nohup env \
  S3NET_EPOCHS=301 \
  S3NET_BATCH_SIZE=512 \
  S3NET_CHECKPOINT_INTERVAL=10 \
  bash pipelines/v7_native_pipeline/scripts/09_train_s3net_native_stats.sh \
  > "runs/<RUN_ID>/training/s3net/$(date +%Y%m%d_%H%M%S)_launcher.log" 2>&1 &

echo "PID: $!"
```

将 <RUN_ID> 替换为实际 run ID。查看后台进度：

```bash
tail -f runs/<RUN_ID>/training/s3net/*_launcher.log
```

每次正式训练都会新建：

```text
runs/<RUN_ID>/training/s3net/<时间戳>_s3net_native_stats_<epoch>epoch/
```

目录内包含训练日志、训练配置、代码快照、train/dev 曲线、周期 checkpoint、
s3net_native_stats_{latest,best_dev,final}.pth、dev 评估 JSON，以及
eval_reports/s3net_full_check_report.md、类别 IoU 和混淆矩阵图。`decode_best_dev/`
和 `decode_final/` 还会输出 `semantic_comparison_<id>.jpg`：左侧为 ground truth，
右侧为 S3-Net 推理结果，同时保留两张原有的独立图片。

下游测试优先使用本次时间戳目录中的 s3net_native_stats_best_dev.pth；它按最低 dev loss 选择。
默认不会覆盖旧结果或共享 alias。S3NET_WRITE_COMPAT_ALIASES=1 会尝试写共享 alias，通常不需要使用。

S3-Net 的内置完整检查是 dev 评估；若需要正式 held-out test，应在保留的 test split 上另行评估 best-dev 模型，不能把 dev 指标当作 test 指标。

## 8. 训练 SemanticCNN

SemanticCNN 使用固定双雷达语义输入和历史帧，目标是
cmd_velocities/[linear_x, angular_z]。只有每个参与训练的 session 都有有效
/cmd_vel_stamped 且 07c 已导出 cmd_velocities/ 时才运行；不支持的运动形式
（例如未采集的倒车或横移）不会凭训练自动获得。

正式训练也会先验证 dataset.txt，并且 CUDA 不可用时默认拒绝启动。建议先确认 GPU：

```bash
"$TORCH_PY" -c 'import torch; print("cuda_available:", torch.cuda.is_available()); print("device:", torch.cuda.get_device_name(0) if torch.cuda.is_available() else "none")'
```

### 正式 GPU 训练

```bash
SEMANTIC_CNN_EPOCHS=51 \
SEMANTIC_CNN_BATCH_SIZE=64 \
SEMANTIC_CNN_CHECKPOINT_INTERVAL=10 \
  bash pipelines/v7_native_pipeline/scripts/10_train_semantic_cnn.sh
```

默认 SEMANTIC_CNN_STOP_LOSS_WEIGHT=1.0；可显式记录该参数，但不要在没有对比评估时随意增大：

```bash
SEMANTIC_CNN_EPOCHS=51 \
SEMANTIC_CNN_BATCH_SIZE=64 \
SEMANTIC_CNN_CHECKPOINT_INTERVAL=10 \
SEMANTIC_CNN_STOP_LOSS_WEIGHT=1.0 \
  bash pipelines/v7_native_pipeline/scripts/10_train_semantic_cnn.sh
```

### 后台运行示例

```bash
nohup env \
  SEMANTIC_CNN_EPOCHS=51 \
  SEMANTIC_CNN_BATCH_SIZE=64 \
  SEMANTIC_CNN_CHECKPOINT_INTERVAL=10 \
  SEMANTIC_CNN_STOP_LOSS_WEIGHT=1.0 \
  bash pipelines/v7_native_pipeline/scripts/10_train_semantic_cnn.sh \
  > "runs/<RUN_ID>/training/semantic_cnn/$(date +%Y%m%d_%H%M%S)_launcher.log" 2>&1 &

echo "PID: $!"
```

查看后台进度：

```bash
tail -f runs/<RUN_ID>/training/semantic_cnn/*_launcher.log
```

每次训练写入独立目录：

```text
runs/<RUN_ID>/training/semantic_cnn/<时间戳>_semantic_cnn_native_cmd_<epoch>epoch/
```

其中包含日志、配置、代码快照、周期 checkpoint、
semantic_cnn_native_cmd_{latest,best_dev,final}.pth、训练/验证曲线和：

```text
eval_reports/semantic_cnn_full_check_report.md
eval_reports/semantic_cnn_best_dev_test_pred_vs_target.csv
eval_reports/semantic_cnn_loss_curve.png
```

训练结束后，检查完整报告中的最佳 dev epoch、held-out test 的线速度/角速度误差、
左右转向方向准确率、停车样本预测比例和塌缩检查。下游测试优先使用本次目录中的
semantic_cnn_native_cmd_best_dev.pth，不要使用 final 代替它，也不要把更长的从头训练称为 resume。
默认不会覆盖旧结果或共享 alias；SEMANTIC_CNN_WRITE_COMPAT_ALIASES=1 会尝试写共享 alias，通常不需要使用。

## 9. 训练 DRL-VO 离线行为克隆

当前 DRL-VO 结果是在已导出的 replay 上做离线行为克隆，不是在线 PPO 训练。基础模型冻结
预训练特征提取器，只微调 policy MLP 和 action head；语义模型额外训练 7 类语义
late-fusion 分支。下面复现本次两个训练，输出目录会自动增加时间戳，不覆盖已有结果。

先设置公共路径：

```bash
cd /home/user/navigation_project/a_pipeline
source environment/activate.sh

export TASK_ROOT="$A_PIPELINE_ROOT/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1"
export DRLVO_REPLAY="$TASK_ROOT/training/drl_vo/replay/20260727_110005"
export DRLVO_PRETRAINED="$A_PIPELINE_ROOT/github_src/drl_vo_nav-drl_vo/drl_vo/src/model/drl_vo.zip"

cd "$A_PIPELINE_ROOT/methods/experiments/drl_vo_ros2_offline"
```

基础 DRL-VO：

```bash
"$TORCH_PY" train_behavior_cloning.py \
  --replay-dir "$DRLVO_REPLAY" \
  --model "$DRLVO_PRETRAINED" \
  --output-root "$TASK_ROOT/training/drl_vo/base_bc" \
  --epochs 200 \
  --batch-size 64 \
  --feature-batch-size 16 \
  --learning-rate 3e-4 \
  --patience 30 \
  --seed 1337
```

语义 DRL-VO：

```bash
"$TORCH_PY" train_behavior_cloning.py \
  --replay-dir "$DRLVO_REPLAY" \
  --model "$DRLVO_PRETRAINED" \
  --output-root "$TASK_ROOT/training/drl_vo/semantic_bc" \
  --use-semantics \
  --semantic-num-classes 7 \
  --semantic-person-class 6 \
  --epochs 200 \
  --batch-size 64 \
  --feature-batch-size 16 \
  --learning-rate 3e-4 \
  --patience 30 \
  --seed 1337
```

每次输出包含 `checkpoints/best.pt`、`training_summary.json`、`report.md`、
`eval_predictions.csv` 和完整 `run_command.txt`。正式比较优先使用 `best.pt`。
当前 replay 的 `observations.npz` 已保存 whole-bag `split_labels`，训练器会直接映射
train/dev/test（内部把 dev 作为 validation），不会再对同一轨迹做 blocked frame
split。`--block-size` 和 `--purge-frames` 只用于没有 `split_labels` 的旧 replay
兼容路径。
训练报告的 PASS 只表示离线控制信号可学习，不等于 Gazebo 闭环导航成功。

## 10. 内置小样例验证

交付包包含 632×482 语义地图、35 帧×360 束转换结果和对应小型 rosbag：

```bash
source environment/activate.sh
"$TORCH_PY" scripts/validation/verify_smoke_example.py examples/smoke
python3 scripts/validation/verify_portable_bundle.py .
```

样例 bag 没有速度指令，因此只用于地图、TF、雷达与语义转换验证，不用于
SemanticCNN 或 DRL-VO 训练。

## 11. 最新训练成果与 5 个 Demo

以下结果均使用：

```text
runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/
```

### 11.1 最新训练成果

| 模型 | 训练结果 | 离线结果摘要 | Demo 用 checkpoint（相对 `$TASK_ROOT/training/`） |
| --- | --- | --- | --- |
| SemanticCNN Global | 51 epoch，`global_virtual_angle_80` | best epoch 24；held-out test MSE 0.046169；转向方向准确率 97.36% | `semantic_cnn/20260727_120116_semantic_cnn_native_cmd_51epoch/semantic_cnn_native_cmd_best_dev.pth` |
| SemanticCNN Sensor-split | 51 epoch，`sensor_split_40x2` | best epoch 47；held-out test MSE 0.060290；转向方向准确率 94.22% | `semantic_cnn/20260727_142358_semantic_cnn_native_cmd_51epoch/semantic_cnn_native_cmd_best_dev.pth` |
| 基础 DRL-VO BC | 41 epoch | PASS；best epoch 11；test MSE 0.318941 | `drl_vo/base_bc/20260727_114455/checkpoints/best.pt` |
| 语义 DRL-VO BC | 65 epoch，启用语义输入 | PASS；best epoch 35；test MSE 0.319000；语义投影训练检查通过 | `drl_vo/semantic_bc/20260727_115227/checkpoints/best.pt` |
| S3-Net | 301 epoch，`range_incidence`，7 类 | best epoch 27；dev beam accuracy 50.73%；present-class mean IoU 29.55% | `s3net/20260727_162813_s3net_native_stats_301epoch/s3net_native_stats_best_dev.pth` |

完整报告位于各训练目录的 `report.md` 或 `eval_reports*/`。这些是离线评估结果，
不能单独证明闭环导航成功。两个 DRL-VO Demo 使用 Gazebo 行人真值速度；
语义 DRL-VO 还使用 Person 真值语义，属于 oracle 输入实验。

### 11.2 公共环境

下面共 5 个 Demo。先执行一次公共环境，然后每次只运行一种；停止时在当前终端按
`Ctrl+C`。启动前应确认没有其他 Gazebo Demo、teleop 或控制器占用 `/cmd_vel`。
RViz 会在机器人上方实时显示 `/cmd_vel` 的线速度 `v`（m/s）和角速度 `omega`
（rad/s）；超过 0.5 秒没有新命令会标记为 `STALE`，纯感知 S3-Net Demo 显示 `NO DATA`。

```bash
cd /home/user/navigation_project/a_pipeline
source environment/activate.sh

set +u
source workspaces/ros2_ws/install/setup.bash

export TASK_ROOT=/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1
```

### 11.3 Demo 1：SemanticCNN Global

支持全局路径、黄色 subgoal、红色双雷达点云。启动后会弹出完整地图目标选择窗口；
点击并发布目标后窗口隐藏，机器人到达并停止后窗口会再次出现。也可使用 RViz 的
`2D Goal Pose` 更换目标。

```bash
ros2 launch semantic_nav_gazebo \
  semantic_cnn_fixed_dual_start_goal_demo.launch.py \
  semantic_cnn_model:="$TASK_ROOT/training/semantic_cnn/20260727_120116_semantic_cnn_native_cmd_51epoch/semantic_cnn_native_cmd_best_dev.pth" \
  semantic_cnn_model_code:="$TASK_ROOT/training/semantic_cnn/20260727_120116_semantic_cnn_native_cmd_51epoch/model_code_scripts" \
  semantic_cnn_pool_mode:=global_virtual_angle_80 \
  enable_goal_picker:=true auto_set_initial_goal:=false \
  robot_x:=2.0 robot_y:=2.0 robot_yaw:=0.0 \
  goal_x:=20.0 goal_y:=20.0 \
  spawn_scene_pedestrians:=true \
  scene_file:=scenarios/lobby/eng_hall_15.xml \
  pedestrian_count:=8 pedestrian_speed:=1.0 pedestrian_seed:=7 \
  pedestrian_use_actors:=false \
  max_linear:=0.5 max_angular:=1.0 \
  front_stop_distance:=0.5 stop_on_empty_front:=true \
  lookahead:=1.0 inflate_radius:=0.5 \
  visualize:=true publish_debug_images:=true \
  start_rviz:=true gui:=true \
  record_trace:=false
```

### 11.4 Demo 2：SemanticCNN Sensor-split

与上面相同，但使用双雷达分区训练模型。

```bash
ros2 launch semantic_nav_gazebo \
  semantic_cnn_fixed_dual_start_goal_demo.launch.py \
  semantic_cnn_model:="$TASK_ROOT/training/semantic_cnn/20260727_142358_semantic_cnn_native_cmd_51epoch/semantic_cnn_native_cmd_best_dev.pth" \
  semantic_cnn_model_code:="$TASK_ROOT/training/semantic_cnn/20260727_142358_semantic_cnn_native_cmd_51epoch/model_code_scripts" \
  semantic_cnn_pool_mode:=sensor_split_40x2 \
  enable_goal_picker:=true auto_set_initial_goal:=false \
  robot_x:=2.0 robot_y:=2.0 robot_yaw:=0.0 \
  goal_x:=20.0 goal_y:=20.0 \
  spawn_scene_pedestrians:=true \
  scene_file:=scenarios/lobby/eng_hall_15.xml \
  pedestrian_count:=8 pedestrian_speed:=1.0 pedestrian_seed:=7 \
  pedestrian_use_actors:=false \
  max_linear:=0.5 max_angular:=1.0 \
  front_stop_distance:=0.5 stop_on_empty_front:=true \
  lookahead:=1.0 inflate_radius:=0.5 \
  visualize:=true publish_debug_images:=true \
  start_rviz:=true gui:=true \
  record_trace:=false
```

### 11.5 Demo 3：基础 DRL-VO

启动后弹出地图目标选择窗口，不预设第一个目标。到达后会再次弹窗选择下一目标。

```bash
ros2 launch semantic_nav_gazebo \
  drl_vo_fixed_dual_start_goal_demo.launch.py \
  policy_mode:=base \
  drl_vo_model:="$TASK_ROOT/training/drl_vo/base_bc/20260727_114455/checkpoints/best.pt" \
  enable_goal_picker:=true auto_set_initial_goal:=false \
  robot_x:=2.0 robot_y:=2.0 robot_yaw:=0.0 \
  spawn_scene_pedestrians:=true \
  scene_file:=scenarios/lobby/eng_hall_15.xml \
  pedestrian_count:=8 pedestrian_speed:=1.0 pedestrian_seed:=7 \
  pedestrian_use_actors:=false \
  require_pedestrian_truth:=true \
  oracle_pedestrian_velocity:=true \
  max_linear:=0.5 max_angular:=1.0 \
  front_stop_distance:=0.5 stop_on_empty_front:=true \
  lookahead:=1.0 inflate_radius:=0.5 \
  start_rviz:=true gui:=true \
  record_trace:=false
```

### 11.6 DRL-VO 单 episode 自动评估

`evaluate_episode:=true` 会启动模型无关的旁路 evaluator。它在收到
`/data_collection/goal_accepted` 后开始，以 ROS simulation time 记录完整 odom、
`/cmd_vel`、行人真值和推理 telemetry，并在到点、仿真超时或 Ctrl-C 时写入：

```text
episode_summary.json
trajectory.csv
commands.csv
pedestrian_trace.csv
inference_trace.csv
```

原始 CSV 是后续重算 jerk、TTC 与 personal-space 指标的依据，应与 summary 一起保留。
第一版尚未接入 Gazebo contact sensor，因此 `collision` 始终是 unknown（JSON 中为
`null`）；`goal_reached: true` 只说明进入 goal tolerance，**不等于**无碰撞 success。
基础 DRL-VO 目前使用 oracle pedestrian velocity；该事实会写入 summary 的 method metadata。

需要在同一次 Gazebo 启动中连续手动选择多个目标时，增加
`evaluation_multi_episode:=true`。每个收到的 `/data_collection/goal_accepted`
都会独立写入 `episode_0001/`、`episode_0002/` 等子目录，根目录的
`session_summary.json` 提供 episode 索引。若导航途中发布了新目标，当前段先以
`superseded_by_new_goal` 结束并落盘，再开始下一段。默认值为 `false`，因此既有
单 episode 命令和扁平输出布局保持不变。

手动运行单 episode 评估（不要与其他 `/cmd_vel` 发布者并行）：

```bash
export TASK_ROOT=/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1
export DEMO_DIR=/home/user/navigation_project/a_pipeline/runs/20260808_gazebo_play/evaluations/drl_vo_base_$(date +%Y%m%d_%H%M%S)
mkdir -p "$DEMO_DIR"

ros2 launch semantic_nav_gazebo \
  drl_vo_fixed_dual_start_goal_demo.launch.py \
  policy_mode:=base \
  drl_vo_model:="$TASK_ROOT/training/drl_vo/base_bc/20260727_114455/checkpoints/best.pt" \
  enable_goal_picker:=false \
  auto_set_initial_goal:=true \
  robot_x:=2.0 robot_y:=2.0 robot_yaw:=0.0 \
  goal_x:=16.0 goal_y:=16.0 \
  spawn_scene_pedestrians:=true \
  pedestrian_count:=8 pedestrian_speed:=1.0 pedestrian_seed:=7 \
  require_pedestrian_truth:=true \
  oracle_pedestrian_velocity:=true \
  evaluate_episode:=true \
  evaluation_multi_episode:=false \
  evaluation_output_dir:="$DEMO_DIR" \
  evaluation_timeout_sec:=360.0 \
  start_rviz:=true gui:=true
```

### 11.7 Demo 4：语义 DRL-VO

使用加入语义输入的 DRL-VO BC 模型。目标选择方式与基础 DRL-VO 相同。

```bash
ros2 launch semantic_nav_gazebo \
  drl_vo_fixed_dual_start_goal_demo.launch.py \
  policy_mode:=semantic \
  drl_vo_model:="$TASK_ROOT/training/drl_vo/semantic_bc/20260727_115227/checkpoints/best.pt" \
  enable_goal_picker:=true auto_set_initial_goal:=false \
  robot_x:=2.0 robot_y:=2.0 robot_yaw:=0.0 \
  spawn_scene_pedestrians:=true \
  scene_file:=scenarios/lobby/eng_hall_15.xml \
  pedestrian_count:=8 pedestrian_speed:=1.0 pedestrian_seed:=7 \
  pedestrian_use_actors:=false \
  require_pedestrian_truth:=true \
  oracle_pedestrian_velocity:=true \
  oracle_person_semantics:=true \
  max_linear:=0.5 max_angular:=1.0 \
  front_stop_distance:=0.5 stop_on_empty_front:=true \
  lookahead:=1.0 inflate_radius:=0.5 \
  start_rviz:=true gui:=true \
  record_trace:=false
```

### 11.7 Demo 5：S3-Net 语义识别

这是纯感知 Demo，只显示语义识别和点云，不会发布 `/cmd_vel`，机器人不会自主导航。

```bash
ros2 launch semantic_nav_gazebo \
  s3net_fixed_dual_perception_demo.launch.py \
  s3net_model:="$TASK_ROOT/training/s3net/20260727_162813_s3net_native_stats_301epoch/s3net_native_stats_best_dev.pth" \
  s3net_model_code:="$TASK_ROOT/training/s3net/20260727_162813_s3net_native_stats_301epoch/model_code_scripts" \
  s3net_stats_json:="$TASK_ROOT/training/s3net/20260727_162813_s3net_native_stats_301epoch/s3net_native_lidar_train_stats.json" \
  sampling_strategy:=contract \
  robot_x:=2.0 robot_y:=2.0 robot_yaw:=0.0 \
  spawn_scene_pedestrians:=true \
  scene_file:=scenarios/lobby/eng_hall_15.xml \
  pedestrian_count:=8 pedestrian_speed:=1.0 pedestrian_seed:=7 \
  pedestrian_use_actors:=false \
  start_rviz:=true gui:=true
```

建议先用 `pedestrian_count:=8` 比较模型。需要压力测试时，再统一改成 `28`；
否则大量行人造成的物理阻塞会干扰对策略本身的判断。

## 12. SemanticCNN fixed-dual Gazebo 闭环验收

SemanticCNN 的离线 loss、held-out test 误差和方向准确率只能说明模型在已转换数据上的表现；
训练完成后还必须在 Gazebo 中做一次完整闭环验证，确认在线双雷达预处理、模型推理、
安全控制、全局路径和目标停车能够共同工作。闭环验证失败时，不能用一段中途轨迹或
RViz 截图代替到点结果。

本节使用独立的 fixed-dual 入口：

```text
workspaces/ros2_ws/src/semantic_nav_gazebo/launch/semantic_cnn_fixed_dual_start_goal_demo.launch.py
```

推理输入是同步的 /scan_01 + /scan_02，经 TF 转换到 base_link 后构造最近 10 帧的
80×80 scan_map 和 80×80 semantic_map。不要改用旧 /scan_merged 推理节点，也不要
同时启动旧 SemanticCNN demo。当前在线 semantic 标签来自静态 label.png 地图投影，
不是在线 S3-Net 感知闭环；S3-Net 的训练结果不会在这个 demo 中自动成为在线输入。

### 12.1 加载本次 run 和最佳模型

在一个新终端中执行，并把 <RUN_ID> 替换为实际 run：

```bash
cd /home/user/navigation_project/a_pipeline
source environment/activate.sh
export RUN_MANIFEST="$A_PIPELINE_ROOT/runs/<RUN_ID>/run_manifest.env"
source "$RUN_MANIFEST"

test -f "$LAST_SEMANTIC_CNN_BEST_DEV"
test -f "$LAST_SEMANTIC_CNN_RESULT_DIR/model_code_scripts/model.py"
printf 'checkpoint: %s\nmodel code: %s\n' \
  "$LAST_SEMANTIC_CNN_BEST_DEV" \
  "$LAST_SEMANTIC_CNN_RESULT_DIR/model_code_scripts"
```

pipelines/v7_native_pipeline/scripts/10_train_semantic_cnn.sh 会把
LAST_SEMANTIC_CNN_BEST_DEV、结果目录和报告路径写回
run_manifest.env。这里需要显式 source "$RUN_MANIFEST"，否则当前终端不会获得刚写入
manifest 的模型路径。demo 必须同时使用本次训练目录中的 best-dev checkpoint 和
model_code_scripts/ 快照，不能把新 checkpoint 与另一次训练的 model.py 混用。

### 12.2 构建运行包

只从当前 ROS 2 workspace 构建活动包，避免搜索到历史或重复 package：

```bash
cd "$ROS_WS"
colcon build \
  --base-paths src \
  --packages-select semantic_nav_gazebo \
  --symlink-install
source install/setup.bash
cd "$A_PIPELINE_ROOT"
```

构建必须成功。如果新增脚本后 ros2 run 找不到它，先检查
workspaces/ros2_ws/src/semantic_nav_gazebo/CMakeLists.txt 的 install(PROGRAMS ...)，
不要改用另一套 ROS workspace。

### 12.3 启动标准闭环验收

先确认没有其他 Gazebo demo、teleop 或控制器仍在运行；/cmd_vel 同时存在多个发布者会让
闭环结果无效。下面的命令使用起点 (2,2)、目标 (16,16)，并把轨迹和最终结果写入一个
新的时间戳目录，不覆盖旧 demo：

```bash
export DEMO_DIR="$RUN_ROOT/demos/semantic_cnn_fixed_dual_2_2_to_16_16_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$DEMO_DIR"

ros2 launch semantic_nav_gazebo \
  semantic_cnn_fixed_dual_start_goal_demo.launch.py \
  semantic_cnn_model:="$LAST_SEMANTIC_CNN_BEST_DEV" \
  semantic_cnn_model_code:="$LAST_SEMANTIC_CNN_RESULT_DIR/model_code_scripts" \
  robot_x:=2.0 robot_y:=2.0 robot_yaw:=0.0 \
  goal_x:=16.0 goal_y:=16.0 \
  max_linear:=0.11 \
  max_angular:=1.5 \
  front_stop_distance:=0.5 \
  lookahead:=1.0 \
  inflate_radius:=1.0 \
  visualize:=true \
  publish_debug_images:=true \
  start_rviz:=true \
  gui:=true \
  record_trace:=true \
  trace_path:="$DEMO_DIR/trajectory.csv" \
  trace_timeout_sec:=360.0
```

inflate_radius=1.0 是已有标准路线成功验收过的保守参数，不代表路径长度最优。当前机器人
碰撞体采用带 1 cm 余量的可视网格低层凸轮廓，边界约 0.58×0.47 m，任意朝向的外接圆
半径约 0.34 m；若希望减少绕路，可另开新的
demo 目录测试 inflate_radius:=0.6，但必须重新完成本节全部验收，不能沿用 1.0 m 的
成功结果。

启动后模型先积累 10 组同步 scan，此时机器人保持停车。序列就绪后才开始推理。标准路线
速度较低，完整运行可能需要数分钟；到达目标后机器人停车，但 Gazebo 和 RViz 会继续运行，
检查完结果后在 launch 终端按 Ctrl-C。

### 12.4 RViz 中应看到的内容

launch 默认加载 semantic_cnn_fixed_dual_debug.rviz，固定坐标系为 map：

- Map：当前 run 的 SLAM 占据栅格；
- Global Path：/semantic_cnn/global_path，青色 A* 路径；
- Actual Robot Trajectory：/semantic_cnn/actual_trajectory，蓝色实际轨迹；
- 最终目标：红色 Marker；
- local subgoal：黄色 Marker，随路径推进移动，并显示坐标和距最终目标距离；
- 双雷达有效点：两个原始 scan 经 TF、虚拟化和自车过滤后的当前有效点；
- CNN 面板：实际送入模型的 80×80 scan_map 和 semantic_map；
- 推理状态：序列进度、帧号、raw/最终命令、front_min、front_stop、
goal_stop 和 scan_timeout。
其中 debug 发布是控制旁路：它显示在线节点已经使用的数组和状态，不重新构造另一套模型
输入，也不修改 /cmd_vel。

### 12.5 第二个终端检查 topic 和唯一控制器

另开终端，重新加载同一个 run：

```bash
cd /home/user/navigation_project/a_pipeline
source environment/activate.sh
export RUN_MANIFEST="$A_PIPELINE_ROOT/runs/<RUN_ID>/run_manifest.env"
source "$RUN_MANIFEST"

ros2 topic list | sort | grep '^/semantic_cnn/'
ros2 topic info -v /cmd_vel
ros2 topic echo /semantic_cnn/global_path --once --no-arr
ros2 topic echo /semantic_cnn/local_subgoal --once --no-arr
ros2 topic echo /semantic_cnn/debug/scan_map --once --no-arr
ros2 topic echo /semantic_cnn/debug/semantic_map --once --no-arr
ros2 topic echo /semantic_cnn/debug/raw_cmd --once --no-arr
ros2 topic echo /semantic_cnn/debug/cmd_vel --once --no-arr
```

应至少存在：

| Topic | 含义 |
| --- | --- |
| `/semantic_cnn/global_path` | 静态地图上的 A* 全局路径 |
| `/semantic_cnn/actual_trajectory` | Gazebo 中机器人实际轨迹 |
| `/semantic_cnn/final_goal` | 最终目标点 |
| `/semantic_cnn/local_subgoal` | 机器人坐标系下的当前局部目标 |
| `/semantic_cnn/raw_model_cmd` | SemanticCNN 原始线速度/角速度输出 |
| `/semantic_cnn/debug/markers` | 有效点、目标、subgoal、CNN 面板和安全状态 |
| `/semantic_cnn/debug/scan_map` | 实际送入 CNN 的 80×80 float32 scan 输入 |
| `/semantic_cnn/debug/semantic_map` | 实际送入 CNN 的 80×80 float32 semantic 输入 |
| `/semantic_cnn/debug/raw_cmd` | 模型原始命令的 debug 镜像 |
| `/semantic_cnn/debug/cmd_vel` | 限速和防撞后实际命令的 debug 镜像 |

两个 Image topic 应显示：

```text
height: 80
width: 80
encoding: 32FC1
step: 320
```

`ros2 topic info -v /cmd_vel` 必须只有一个 ROS 发布者，即 fixed-dual inference 节点；
不应出现旧 /scan_merged SemanticCNN 控制器或 teleop 发布者。

### 12.6 判定是否真正到点

轨迹记录器会在 $DEMO_DIR 中写入：

```text
trajectory.csv
closed_loop_demo_summary.json
```

检查结果：

```bash
python3 -m json.tool "$DEMO_DIR/closed_loop_demo_summary.json"
```

只有同时满足以下条件才能报告闭环成功：

```text
reached: true
finish_reason: goal_tolerance_reached
minimum_goal_distance <= goal_tolerance
```

标准容差为 0.35 m。若结果为 wall_timeout、节点异常退出，或者 summary 不存在，应明确
报告失败原因；不能把中途截图、局部轨迹或“已经接近目标”写成成功。

一次完整验收应记录：

1. colcon build 成功；
2. 日志显示模型在 CUDA 或 CPU 上成功加载；
3. /scan_01、/scan_02 同步输入实际到达，序列达到 10/10；
4. global path、local subgoal 和所有 debug topic 有实际消息；
5. RViz 可见路径、轨迹、subgoal、目标、CNN 输入和安全状态；
6. /cmd_vel 只有一个发布者；
7. summary 明确给出到点或失败结果；
8. 使用的 checkpoint、模型代码、参数和输出目录均有记录。

### 12.7 后续新模型的接入原则

如果新训练结果保持完全相同的合同——同步 /scan_01 + /scan_02、TF 虚拟点、自车过滤、
10 帧、global_virtual_angle_80、80×80 双输入、相同 semantic 类别 ID、相同 local-subgoal
归一化、相同网络结构和 [linear_x, angular_z] 输出——通常不需要修改或重新构建 demo，
只需将 launch 的两个参数指向新训练目录：

```text
semantic_cnn_model:=/绝对路径/semantic_cnn_native_cmd_best_dev.pth
semantic_cnn_model_code:=/绝对路径/model_code_scripts
```

如果序列长度、输入尺寸、pooling、距离归一化、语义 ID、目标归一化、网络构造函数、
checkpoint 结构或输出含义有任意变化，就不能只换 .pth。应为新合同新增独立 inference
adapter/launch，并保留本 demo 的限速、防撞、目标停车和 scan-timeout 停车外壳。新模型先用
max_linear:=0.0 max_angular:=0.0 验证加载和 tensor，再低速短距离测试，最后才运行完整路线。

无论哪种情况，都应保留每次训练生成的 model_code_scripts/ 和训练配置，并优先使用
semantic_cnn_native_cmd_best_dev.pth，不要覆盖已经验收的模型、地图和 demo 结果。

### 12.8 可选动态行人测试

fixed-dual demo 默认不生成行人，以保持标准静态路线验收可重复。需要观察动态交互时，在
第 12.3 节命令中额外加入：

```bash
spawn_scene_pedestrians:=true \
scene_file:=scenarios/lobby/eng_hall_15.xml \
pedestrian_count:=8 \
pedestrian_speed:=1.0 \
pedestrian_seed:=7 \
pedestrian_use_actors:=false
```

eng_hall_15.xml 提供大厅中的行人起点、循环 waypoint、墙体和群组关系。普通行人模型配合
不可见碰撞代理，使 Gazebo GPU LiDAR 能收到移动行人的回波；固定 pedestrian_seed 便于
重复同一组实验。改变行人数、速度或 seed 后，应写入新的 demo 输出目录。

当前机器人没有独立的在线行人检测、轨迹预测或动态全局重规划。移动行人会作为双雷达
几何障碍进入 scan_map，SemanticCNN 可能根据训练中学到的几何模式产生绕行命令；但在线
semantic_map 仍由静态 label.png 投影，移动行人通常不会被实时标成 Person。最后一道硬保护
是 front_stop_distance：正前方障碍进入该距离后线速度清零，并保留或补充转向。此外，场景
行人控制器自身包含对机器人的个人空间力，行人主动让路不能单独证明机器人具有行人语义
避让能力。动态行人测试结果必须与无行人的标准闭环结果分开记录。

## 13. 打包发给别人

不要直接打包当前机器生成的 .venvs 和 ROS 构建目录。执行：

```bash
bash scripts/release/create_bundle.sh
```

压缩包和 SHA256 校验文件输出到 dist/。接收者解压后从第 1 节开始执行。

## 结果边界

- 原始包、地图和实验结果不会被自动删除；
- RUN_ID 隔离每次实验；
- 系统 apt 包、显卡驱动和少量系统缓存不可能放进项目目录；
- 其余项目运行产物均进入 .runtime/ 或 runs/<RUN_ID>/。
