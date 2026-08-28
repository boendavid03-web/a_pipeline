# Native LiDAR TF Alignment Check

- bag: `examples/smoke/test_rosbag`
- has /tf: True
- has /tf_static: True
- map frame: `map`
- base frame: `base_link`
- /scan_merged frame_id: `base_link`
- can resolve map->base_link: True
- can resolve map->scan_frame: True
- can resolve odom->base_link: True
- map resolution: 0.05
- map origin: [-0.11, 0.0204, 0.0]

## Scan Geometry
### /scan_01
- count: 35
- frame_ids: `{'mecanum730_xms5_v7_teacher_dual_scan/base_scan_01/lidar_2d_01': 35}`
- angle_min: -3.141590118408203 .. -3.141590118408203
- angle_max: 3.141590118408203 .. 3.141590118408203
- angle_increment: 0.007863804697990417 .. 0.007863804697990417
- beam_count_unique: [800]
- range_min: 0.10000000149011612 .. 0.10000000149011612
- range_max: 50.0 .. 50.0

### /scan_02
- count: 35
- frame_ids: `{'mecanum730_xms5_v7_teacher_dual_scan/base_scan_02/lidar_2d_02': 35}`
- angle_min: -3.141590118408203 .. -3.141590118408203
- angle_max: 3.141590118408203 .. 3.141590118408203
- angle_increment: 0.007863804697990417 .. 0.007863804697990417
- beam_count_unique: [800]
- range_min: 0.10000000149011612 .. 0.10000000149011612
- range_max: 50.0 .. 50.0

### /scan_merged
- count: 35
- frame_ids: `{'base_link': 35}`
- angle_min: -3.1415927410125732 .. -3.1415927410125732
- angle_max: 3.1415927410125732 .. 3.1415927410125732
- angle_increment: 0.017501909285783768 .. 0.017501909285783768
- beam_count_unique: [360]
- range_min: 0.10000000149011612 .. 0.10000000149011612
- range_max: 50.0 .. 50.0

## TF Edges
- dynamic: `[['map', 'odom'], ['odom', 'base_link']]`
- static: `[['base_link', 'mecanum730_xms5_v7_teacher_dual_scan/base_scan_01/lidar_2d_01'], ['base_link', 'mecanum730_xms5_v7_teacher_dual_scan/base_scan_02/lidar_2d_02']]`

## Assessment
- No obvious coordinate-frame risk detected by this check.

FINAL_STATUS=safe
