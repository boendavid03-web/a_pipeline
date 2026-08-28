# 固定双雷达槽位转换（07b + 07c）

主交付流程使用 native 360 束转换，见项目根目录 `README.md`。此可选路径保留两个原始雷达的固定槽位和来源索引：不重采样、不融合、不跨雷达去重。

先设置当前 run：

```bash
cd /home/user/navigation_project/a_pipeline
source environment/activate.sh
export RUN_MANIFEST="$A_PIPELINE_ROOT/runs/<RUN_ID>/run_manifest.env"
```

然后按顺序执行：

```bash
bash pipelines/v7_native_pipeline/scripts/07b_convert_bag_to_fixed_dual_lidar.sh
bash pipelines/v7_native_pipeline/scripts/07c_export_fixed_dual_training_dataset.sh
bash pipelines/v7_native_pipeline/scripts/08c_smoke_fixed_dual_training.sh
```

批量脚本 `07bc_convert_export_all_raw_bags.sh` 默认保留 07b 中间 session。若只需要
最终训练数据，可设置 `DUAL_SLOT_KEEP_INTERMEDIATE=0`。脚本只会在 07c 导出和最终
校验成功后，删除本轮新生成的 07b session；已有 session 或失败任务的中间数据不会
被删除。

## 自身回波处理

默认模式是 `first-synchronized-pair-fixed-beam-identity`：07b 的第一个成功同步输出帧，用 `base_link` 下的机器人 footprint 识别自身回波束；两部雷达的原始 beam index 随后固定，整段 bag 都使用同一组 mask。

被固定为自身的束始终满足：

- `self_mask=true`
- `valid_mask=false`
- `virtual_ranges/virtual_angles=NaN`
- `semantic_label=-1`

因此后来靠近机器人的人或物体不会因为落进 footprint 而被再次错误遮掉。首帧本身仍有物理歧义：开始录包/转换时应让机器人静止，footprint 周围不要有人或障碍物；校准束 runs、时间戳和首帧审计都会写入 `metadata.json` 并由检查器复核。

## 当前 run 参数与兼容性

07b 从 manifest 读取每部雷达实际 beam 数、量程和更新率。07c 从 07b 元数据读取真实量程，并从 bag 时间戳推导实际帧周期；例如当前 `2000 + 2000`、`8m`、`15Hz` 会导出 `4000` slots、`pool_range_max=8.0` 和约 `66ms` 的周期，不会使用旧的 `50m/100ms` 假设。

07c 会拒绝把不同 beam 数、量程、池化配置、语义标签或时序契约的 session 混入同一训练根目录，也会验证实测双雷达频率与 manifest 一致。若将来两个雷达使用不同 beam 数，07b 原始槽位转换仍可用；当前 S3-Net 共用 batch loader 不能训练异长的两个 stream，因此 07c 会明确拒绝，而不会悄悄生成错误数据。

输出位于当前 `runs/<RUN_ID>/`，目标 session 已存在时默认拒绝覆盖：

- 07b：`datasets/fixed_dual_lidar_slots/<bag>-v7-fixed-dual-v3-<N1>x<N2>-converted/`
- 07c：`datasets/semantic2d_fixed_dual_native/<bag>-v7-fixed-dual-v3-<N1>x<N2>-training/`
