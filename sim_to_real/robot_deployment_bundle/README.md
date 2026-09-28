# 固定双 LiDAR 真机 Shadow 部署包

这是一个可整体复制到真机计算机的 ROS 2 Humble 推理部署包。它只发布
shadow 速度建议，默认不会向底盘 `/cmd_vel` 发布消息。

支持两条互斥运行的链路：

- 方法 A：双 LiDAR → 几何扫描对齐 → 在线 S3-Net → SemanticCNN →
  `/sim_to_real/semantic_cnn/cmd_vel_shadow`
- 方法 B：双 LiDAR → TF-aware scan merger → DR-SPAAM → point tracker →
  base DRL-VO → `/sim_to_real/drl_vo/cmd_vel_shadow`

真机必须自行提供定位、TF、地图、全局规划、全局路径和最终目标。本包不包含
SLAM、map server、建图或全局规划。

## 1. 复制与安装

把整个 `robot_deployment_bundle/` 目录复制给部署人员，不要只复制
`ros2_ws/` 或 `checkpoints/`。

```bash
rsync -a robot_deployment_bundle/ robot@ROBOT:/opt/robot_deployment_bundle/
ssh robot@ROBOT
cd /opt/robot_deployment_bundle
sha256sum -c SHA256SUMS
python3 -m pip install --user -r requirements-runtime.txt
./build.sh
```

`build.sh` 不使用 `sudo`，也不修改 shell 启动文件。PyTorch 应选择与真机
CPU/CUDA 环境匹配的发行版本；不要为了部署重新训练或转换这里的 checkpoint。

## 2. 真机输入合同

默认输入如下，实际名称可通过 launch 参数修改：

| 输入 | ROS 类型 | 默认值/要求 |
| --- | --- | --- |
| LiDAR 1 | `sensor_msgs/msg/LaserScan` | `/scan_01`，约 2000 束、15 Hz |
| LiDAR 2 | `sensor_msgs/msg/LaserScan` | `/scan_02`，约 2000 束、15 Hz |
| 里程计 | `nav_msgs/msg/Odometry` | `/odom` |
| 全局路径 | `nav_msgs/msg/Path` | `/plan` |
| 最终目标 | `geometry_msgs/msg/PoseStamped` | `/goal_pose` |
| TF | TF2 | `map/odom → base_link`，以及两个 LiDAR frame → `base_link` |

默认 frame 为 `map`、`odom`、`base_link`。硬件侧名称集中记录在
`config/robot_topics.yaml`；启动时用同名 launch 参数覆盖，例如：

```bash
./run_shadow_experiment.sh semantic_cnn \
  scan_01_topic:=/lidar_front/scan \
  scan_02_topic:=/lidar_rear/scan \
  odom_topic:=/localization/odom \
  path_topic:=/planner/plan \
  goal_topic:=/goal_pose \
  base_frame:=base_footprint
```

双 LiDAR 适配器会检查 beam 数、角度/距离元数据、frame、时间戳和两路时间差；
S3-Net 路径会把 ranges 几何重采样到每路严格 2000 束，并把有效范围限制为
0.1–8.0 m。它不是只改 LaserScan 元数据，也不使用 beam crop。

## 3. 构建与只读预检

先启动真机原有的定位和全局规划栈，再执行：

```bash
cd /opt/robot_deployment_bundle
./preflight.sh semantic
./preflight.sh drlvo
./tools/inspect_topics.sh
./tools/inspect_tf.sh base_link laser_01_frame laser_02_frame
python3 ./tools/model_load_smoke.py
```

若真机 topic/frame 不是默认名，可给 preflight 设置环境变量：

```bash
SCAN_01_TOPIC=/lidar_front/scan \
SCAN_02_TOPIC=/lidar_rear/scan \
ODOM_TOPIC=/localization/odom \
PATH_TOPIC=/planner/plan \
GOAL_TOPIC=/goal_pose \
MAP_FRAME=map ODOM_FRAME=odom BASE_FRAME=base_footprint \
./preflight.sh semantic
```

preflight 和 `tools/` 下的检查脚本只读取 ROS 图，不创建控制 publisher，也不发送
运动命令。`model_load_smoke.py` 只在 CPU 上反序列化四个 checkpoint。

## 4. 启动方法 A

```bash
cd /opt/robot_deployment_bundle
./run_shadow_experiment.sh semantic_cnn \
  scan_01_topic:=/scan_01 scan_02_topic:=/scan_02 \
  odom_topic:=/odom path_topic:=/plan goal_topic:=/goal_pose \
  map_frame:=map odom_frame:=odom base_frame:=base_link
```

观察输出：

```bash
source /opt/ros/humble/setup.bash
source /opt/robot_deployment_bundle/ros2_ws/install/setup.bash
ros2 topic echo /s3net/labels --once
python3 /opt/robot_deployment_bundle/tools/shadow_cmd_monitor.py \
  /sim_to_real/semantic_cnn/cmd_vel_shadow
```

该 launch 固定启用在线 S3-Net，不读取静态 `label.png` 或仿真地图。

## 5. 启动方法 B

```bash
cd /opt/robot_deployment_bundle
./run_shadow_experiment.sh drl_vo \
  scan_01_topic:=/scan_01 scan_02_topic:=/scan_02 \
  odom_topic:=/odom path_topic:=/plan goal_topic:=/goal_pose \
  map_frame:=map odom_frame:=odom base_frame:=base_link
```

观察输出：

```bash
ros2 topic echo /scan_merged --once
ros2 topic echo /dr_spaam_detections_scored --once
ros2 topic echo /pedestrian_tracks --once
python3 /opt/robot_deployment_bundle/tools/shadow_cmd_monitor.py \
  /sim_to_real/drl_vo/cmd_vel_shadow
```

该 launch 固定使用 `mode=base`、`pedestrian_source=dr_spaam`、
`require_pedestrian_truth=false` 和 `/pedestrian_tracks`。部署版 DRL-VO 不创建
`/pedestrian_ground_truth` 订阅，也不读取仿真语义地图。

## 6. 目录与文件职责

```text
robot_deployment_bundle/
├── README.md                         # 本部署、运行与安全说明
├── VERSION                           # 部署包版本
├── MANIFEST.md                       # 打包范围与排除项
├── MODEL_PROVENANCE.md               # 四个模型的来源、大小、用途与哈希
├── SHA256SUMS                        # 除自身外的全包文件完整性清单
├── requirements-runtime.txt          # Python 运行依赖
├── build.sh                          # 安装本地 DR-SPAAM 并构建三个 ROS 包
├── preflight.sh                      # 只读依赖、哈希、topic、TF 检查
├── run_semantic_cnn_shadow.sh        # 方法 A 的直接入口
├── run_drl_vo_drspaam_shadow.sh      # 方法 B 的直接入口
├── run_shadow_experiment.sh          # 推荐入口；一次只选择 A 或 B
├── config/                           # 真机接口与两条链路的默认合同
├── checkpoints/                      # 网络结构、权重与归一化统计
├── ros2_ws/src/
│   ├── semantic_nav_runtime/
│   │   ├── launch/                   # 两条 shadow launch
│   │   ├── scripts/
│   │   │   ├── fixed_dual_scan_adapter.py       # 2×2000 束几何对齐
│   │   │   ├── v7_dual_laser_scan_merger.py     # TF-aware 融合与自滤波
│   │   │   ├── global_path_to_local_subgoal.py  # 全局路径转前视点
│   │   │   ├── path_subgoal_core.py             # 纯路径几何函数
│   │   │   ├── s3net_fixed_dual_inference_node.py
│   │   │   ├── semantic_cnn_fixed_dual_inference_node.py
│   │   │   ├── pedestrian_point_tracker.py
│   │   │   ├── pedestrian_point_tracker_core.py
│   │   │   ├── drl_vo_fixed_dual_inference_node.py
│   │   │   └── drl_vo_control_contract.py
│   │   ├── msg/                      # tracker 与 DRL-VO 自定义消息
│   │   └── test/                     # colcon/pytest 测试入口
│   ├── dr_spaam_ros2/                # DR-SPAAM ROS 2 包装节点
│   └── navigation_evaluation_msgs/   # shadow 门控和推理遥测消息
├── runtime_code/                     # DRL-VO 观测与策略运行闭包
├── third_party/dr_spaam/             # DR-SPAAM 最小推理代码
├── tools/                             # topic、TF、模型和 shadow 输出检查
└── tests/                             # 纯几何、模型、哈希与安全合同测试
```

各配置文件和辅助文件的细分职责：

- `config/robot_topics.yaml`：真机 topic/frame 名称。
- `config/lidar_contract.yaml`：2×2000 束、角度、距离、同步和本体滤波合同。
- `config/semantic_cnn.yaml`：在线 S3-Net 与 SemanticCNN shadow 输出合同。
- `config/dr_spaam.yaml`：merged scan、检测、score 和 track topic。
- `config/drl_vo.yaml`：base + DR-SPAAM、禁用 ground truth 的合同。
- `MODEL_PROVENANCE.md`：四个 checkpoint 的来源和硬件验证边界。
- `MANIFEST.md`：明确未复制的训练、仿真和历史数据内容。

## 7. 常见失败

| 现象 | 优先检查 |
| --- | --- |
| beam 数错误 | 两路原始 scan 是否确为 2000 束；不要通过裁切适配模型 |
| scan 不同步 | 两路硬件时间戳差值是否超过 0.05 s |
| 缺少 TF | path/goal frame 与 base 是否连通；LiDAR frame 是否可转到 base |
| local subgoal stale | `/plan`、`/odom`、TF 是否新鲜；默认发布率为 10 Hz |
| S3-Net 无标签 | aligned scans、模型路径、PyTorch device 与同步 |
| DR-SPAAM 无检测 | `/scan_merged`、权重、device、检测阈值和订阅者 |
| DRL-VO 无输出 | tracks、双 scan、odom、local/final goal 的 freshness gate |
| 文件校验失败 | 重新完整复制部署包；不要混用旧版文件 |

## 8. 安全边界

shadow Twist 只表示模型建议，不代表真机闭环安全。连接执行器前必须另行完成：

- 速度和加速度限制；
- 独立 watchdog；
- 物理急停与远程停止；
- 地理围栏；
- 碰撞与失联验证；
- 现场安全人员审核。

本包不提供自动连接 `/cmd_vel` 的脚本。若以后需要闭环，必须由部署人员在独立、
可审计的 mux/remap 与安全链路中显式完成，不能把当前 shadow 验证当作闭环验收。
