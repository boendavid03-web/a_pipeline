# 迁移包验证记录

验证日期：2026-07-17。

验证主机：Ubuntu 22.04 x86_64、ROS 2 Humble、Gazebo Fortress 6.16、NVIDIA GPU。

## 已通过

- 所有 Shell 脚本通过 `bash -n`。
- Python 源码通过 `compileall`。
- 活跃源码无原开发目录绝对依赖，交付源码无外部符号链接。
- `semantic_nav_gazebo` 从空的 `build/install/log` 执行 `colcon build --symlink-install` 成功。
- 无界面 Gazebo 成功加载包内 v7 机器人模型和 130 MB 网格。
- `/scan_01`、`/scan_02`、`/scan_merged`、`/odom`、`/tf`、`/tf_static`、`/clock`、`/cmd_vel`、`/map` 全部存在。
- `/scan_merged` 实测约 9.8 Hz，配置目标为 10 Hz、360 束。
- 内置 rosbag 的 TF 对齐状态为 `safe`。
- 从交付包工具重新转换得到 35 帧×360 束数据，语义直方图与预期一致。
- 使用迁移后的 S3-Net 源码完成 1 epoch 冒烟训练、dev 评估并生成模型。
- 交付压缩包解压后通过路径检查和 `RELEASE_MANIFEST.sha256` 全量校验。

## 需要在接收机器确认

- 系统依赖和 `.venvs` 安装脚本未在一台全新机器上执行；当前主机已有 ROS 2 和训练环境。
- LabelMe 图形界面的人工绘制与保存需要接收者实际操作。
- SemanticCNN 未使用内置样例训练，因为样例 bag 没有 `/cmd_vel_stamped`；正式采集脚本已经包含该话题。
- 长时间正式训练和真实机器人硬件接入不属于本次迁移验证。

## 2026-08-07 路径迁移复核

本次复核针对当前工作区 `/home/user/navigation_project/a_pipeline`：

- 所有项目 Shell 脚本通过 `bash -n`；Python 源码通过 `compileall`。
- 内置 smoke 样例验证通过：632×482 地图、35 帧×360 束、语义直方图一致。
- `verify_portable_bundle.py` 通过，活动源码不再包含原机器的绝对路径。
- README 中的操作路径已更新；DRL-VO 和行人 BEV 训练/测试脚本改为按源码位置
  自动定位项目根目录。
- ROS 构建脚本会自动识别并隔离跨机器的旧 CMake `build/install/log` 产物，避免
  旧绝对路径污染当前构建。
- 主机预检现在会实际调用 `nvidia-smi` 查询 GPU，不再把命令存在误判为驱动可用；
  当前主机已通过检查，识别到 NVIDIA GeForce RTX 5090、驱动 595.84、32607 MiB 显存。
- 系统依赖安装会自动停用已知失效的区域 ROS 镜像，并在检测到未加载 NVIDIA 驱动时
  安装 `ubuntu-drivers` 推荐版本；驱动安装后要求重启。
- ROS apt-source 版本解析已避免 `curl` 管道提前关闭造成的错误 23；旧 apt 备份会
  保存到 `/var/backups/a_pipeline/`，不再污染 `sources.list.d/`。
- 安装脚本会处理 Ubuntu 旧版 `python3-catkin-pkg`、`python3-rospkg`、
  `python3-rosdistro` 与 ROS 拆分包的文件覆盖冲突，并可从中断的 dpkg 状态继续恢复。

随后已在当前主机完成安装闭环：`00_check_host.sh`、`02_create_python_envs.sh`、
`03_build_ros2_workspace.sh` 和 `04_verify_install.sh` 均通过；CUDA 可用，
`semantic_nav_gazebo` 从当前路径重新构建成功，最终输出为
`PASS: a_pipeline installation verified`。旧的跨机器 ROS 构建产物已移动到
`build.stale-*`、`install.stale-*`、`log.stale-*` 目录并被验证/发布流程忽略。
