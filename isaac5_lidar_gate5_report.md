# Isaac Sim 5.1 2D LiDAR Gate 5

审计日期：2026-09-11  
被测 experience：`isaac_sim/backends/isaac5/generated/minimal_core_physx_no_rtx.kit`  
传感器实现：PhysX scene-query `raycast_closest`；未创建 RTX LiDAR prim、未加载 RTX LiDAR extension。

## 结论

**Gate 5 PASS。** 单雷达 `/scan` 在机器人移动期间持续发布 360-beam `sensor_msgs/LaserScan`；实测 simulation-time 约 10 Hz，frame 为 `base_scan`，scan time 为 0.1 s，ROS 外部 echo 成功读取数据。

## 实测结果

原始 runtime 证据：[`lidar_gate5_ros_run_1.log`](/home/user/navigation_project/a_pipeline/isaac_sim/backends/isaac5/generated/lidar_gate5_ros_run_1.log:104)

配置与观测：

| 项目 | 结果 |
|---|---|
| backend | PhysX scene query，360 条水平 ray |
| topic | `/scan` |
| frame_id | `base_scan` |
| beam count | `360`，每个 sample 均为 360 |
| angle increment | `0.01745329 rad`（约 1°） |
| range | `1.0–20.0 m` |
| observed simulation rate | `30 samples / 3.0 s ≈ 10 Hz` |
| timestamp | 从 `0.100000005 s` 连续到 `3.000000156 s` |
| robot motion | `Δx=+0.28254 m`，由外部 `/cmd_vel` 驱动 |
| teardown | PASS |

外部 echo 读到的首帧证据包含：`frame_id: base_scan`、`angle_min=-π`、`angle_max≈π`、`scan_time=0.100000001`、360 个 ranges。运行期间 scan sample 随机器人位移发生变化；例如原始 JSON 中 1.1 s 后的 sample ranges 与初始帧不同。

## 防自命中处理

射线从 sensor mount 沿 ray direction 前移 `range_min=1.0 m` 后开始查询，避免 Mecanum730 自身 chassis/arm 成为所有 beam 的近距离命中。该 near-range 是当前 Isaac5 scene-query 后端的验证合同，不是 RTX sensor 参数。

## 验收边界

| 项目 | 结果 |
|---|---|
| 普通 2D LaserScan | PASS |
| 360 beams | PASS |
| 10 Hz simulation-time cadence | PASS |
| timestamp / frame_id | PASS |
| robot moving while scan updates | PASS |
| ROS `/scan` external echo | PASS |
| RTX LiDAR | 未使用 |
| `librtx.scenedb.plugin.so` | 成功路径未加载 |

该 Gate 不证明 intensity、dual-LiDAR、2000-beam、RTX GMO 或传感器自过滤等后续合同。
