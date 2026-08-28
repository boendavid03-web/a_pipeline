# 双 2D 雷达行人感知离线基线

入口：

```bash
python3 pipelines/v7_native_pipeline/scripts/evaluate_lidar_pedestrian_tracking.py \
  --dataset runs/20260717_042135_v7_dual/datasets/semantic2d_fixed_dual_native/20260724_063336-v7-fixed-dual-v3-2000x2000-training-pedgt-v1 \
  --output-root runs/20260717_042135_v7_dual/evaluations/lidar_pedestrian_tracking \
  --run-name <new-run-name>
```

估计器只接收 `EstimatorFrame` 中的雷达、机器人 map 位姿和双雷达时间戳。语义标签及
`pedestrian_*` 真值只在外层评估器中读取，不会传给检测或跟踪模块。

每次运行拒绝覆盖已有目录，并输出：

```text
data_audit.json
config.json
clusters.jsonl
measurements.jsonl
associations.jsonl
tracks.jsonl
velocity_references.jsonl
metrics.json
trajectory_overview.png
```

测试：

```bash
python3 -m unittest -v \
  pipelines/v7_native_pipeline/tests/test_lidar_pedestrian_tracking.py
```

## 当前开发集结果

首个完整结果位于：

```text
runs/20260717_042135_v7_dual/evaluations/lidar_pedestrian_tracking/
20260724_geometry_baseline_v3
```

该 bag 是 development/evaluation bag，不是独立测试集。当前几何基线达到了完整
Person 点保留和可见行人检测召回，但测量重复、假确认及身份切换仍较多，不能作为导航
上线结果。

数据审计还发现 17 个扫描帧复用了行人真值时间戳，其中一个重复时间戳对应了不同的
map 位置。因此速度拟合使用唯一且与点云对齐的扫描时间作为拟合自变量，同时继续使用
行人真值时间戳检查数据新鲜度、长间隔和冲突；包含冲突时间戳的参考会标记无效。原始
`pedestrian_velocities` 只用于一致性诊断，不作为速度验收真值。
