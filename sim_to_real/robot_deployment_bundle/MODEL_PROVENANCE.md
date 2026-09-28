# Model provenance and validation boundary

| Model | Original source | Bundle target | Bytes | SHA-256 | Purpose and input contract | Hardware verified |
| --- | --- | --- | ---: | --- | --- | --- |
| `s3net_native_stats_best_dev.pth` | `sim_to_real/cnn_sim_to_real/models/s3net/` | `checkpoints/s3net/` | 750591 | `d7ae12c45a7d2a44ffd6ec07ebc99f70514ad46632c0f5a8f345d98d040d2330` | Online S3-Net semantic labels; two independently aligned 2000-ray scans on the recorded angular grid and 0.1–8.0 m valid range. | No |
| `semantic_cnn_native_cmd_best_dev.pth` | `sim_to_real/cnn_sim_to_real/models/semantic_cnn/` | `checkpoints/semantic_cnn/` | 29010693 | `175ee162e1f7ccd23651efdcfc657c512685c439bf98e026a6bfc9c811b9ac26` | SemanticCNN policy; aligned dual LiDAR, online S3-Net labels, `/odom`, base-frame local subgoal, map-frame final goal. | No |
| `ckpt_jrdb_ann_ft_dr_spaam_e20.pth` | `github_src/drl_vo_nav-drl_vo/GenSafeNav-ROS2-main/dr_spaam_ros2/model_weight/` | `checkpoints/dr_spaam/` | 31741699 | `861ca286ab68c0ab227529435fa62f11a89ce4b20e53cbbdb788c1c545a85dd9` | DR-SPAAM pedestrian detector; TF-aware merged panoramic `LaserScan`. | No |
| `base_bc_best.pt` | `sim_to_real/drlvo_drspaam_sim_to_real/models/drl_vo/` | `checkpoints/drl_vo/` | 10624766 | `57cfd10e6f528f96f721480420c1d6f873b0ff53ad4bbd3e997da5d2fcd6c4e2` | Base DRL-VO policy; fixed dual 2000-ray inputs, odometry, fresh subgoal/final goal, and DR-SPAAM tracker velocity map. | No |

No model was retrained, architecture-modified, parameter-tuned, or claimed to have passed sim-to-real or hardware-safety validation by this packaging work. `SHA256SUMS` is the authoritative integrity manifest for the entire bundle.
