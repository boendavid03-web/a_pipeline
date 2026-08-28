# 冒烟样例

该样例来自一次 35 帧 `/scan_merged` ROS 2 bag，用于验证：

- 632×482 地图和语义图尺寸一致；
- 语义类别 ID 合法；
- 每帧 360 束的距离、角度、有效掩码和语义标签逐束对应；
- TF 投影采用 `tf-map-scan`，对齐状态为 `safe`。

样例没有 `/cmd_vel` 或 `/cmd_vel_stamped` 消息，因此 `cmd_velocities/` 为 0 个文件是正常现象，不能用它验证 SemanticCNN 控制训练。

执行：

```bash
source environment/activate.sh
"$TORCH_PY" scripts/validation/verify_smoke_example.py examples/smoke
```
