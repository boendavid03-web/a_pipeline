# ROS 2 Bag to Native-LiDAR Semantic2D Conversion

## Inputs
- bag: `examples/smoke/test_rosbag`
- map yaml: `semantic_map/map.yaml`
- semantic label: `semantic_map/label.png`
- scan topic: `/scan_merged`
- odom topic: `/odom`
- cmd_vel topic: `/cmd_vel`

## TF / Projection
- pose source used: `tf-map-scan`
- scan frame: `base_link`
- base frame: `base_link`
- map frame: `map`
- tf alignment status: `safe`
- used map->scan TF: True
- used map->base TF: False
- fallback to odom: False
- projection debug dir: `converted_dataset/20260716-085329-semantic-converted/projection_debug`
- warnings: []

## Output
- dataset root: `converted_dataset`
- session: `20260716-085329-semantic-converted`
- samples: 35
- native lidar: True
- interpolated to baseline 1081: False
- beam count unique: [360]
- ignore label: -1
- train samples: 24
- dev samples: 4
- test samples: 7
- split ratios: 0.7/0.1/0.2
- split seed: 0

## Person Labeling
- person label mode: `disabled`
- rule: `static neighborhood majority; every unlabeled, non-free, unknown, or out-of-map endpoint -> ignore=-1; Person=4 is disabled`
- static label filter radius: 2
- occupancy image: `semantic_map/occupancy.pgm`
- occupancy free threshold: 0.25
- occupancy occupied threshold: 0.65

## Velocity
- velocities/ = odom_twist，表示实际速度状态
- cmd_velocities/ = 对齐后的控制命令，表示模型未来要输出的动作标签
- velocity source used: `odom_twist`
- cmd_velocities generated: False
- cmd_velocity_dim: 3
- cmd_velocities source: `None`
- cmd_vel alignment method: `none`
- cmd_vel match policy: `hold-last`
- cmd_vel alignment status: `unavailable`
- cmd_vel stamped available: False
- cmd_vel stamped count: 0
- cmd_vel clock mapping status: `unavailable`
- cmd_vel clock mapping monotonic: True
- cmd_vel time basis mismatch: False
- cmd_vel alignment warnings: ['no cmd_vel or cmd_vel_stamped messages available']
- scan time range sec: 21.0..27.8
- odom time range sec: 20.905..27.909
- cmd_vel time range sec: None..None
- cmd_vel mapped time range sec: None..None
- clock time range sec: 20.889..27.932
- raw cmd_vel angular.z: {'count': 0, 'min': None, 'mean': None, 'max': None, 'unique_count': 0, 'unique_preview': []}
- cmd_velocities angular.z: {'count': 0, 'min': None, 'mean': None, 'max': None, 'unique_count': 0, 'unique_preview': []}
- cmd_velocities nonzero count: 0
- cmd_velocities no prior count: 0
- cmd_velocities hold-last after final count: 0
- raw odom twist angular.z: {'count': 207, 'min': 0.0, 'mean': 0.0, 'max': 0.0, 'unique_count': 1, 'unique_preview': [0.0]}
- converted velocity angular_z: {'count': 35, 'min': 0.0, 'mean': 0.0, 'max': 0.0, 'unique_count': 1, 'unique_preview': [0.0]}

## Label Histogram
- label names source: `semantic_map/label_names.txt`
- -1 ignore: 402
- 1 Chair: 111
- 5 Pillar: 204
- 6 Sofa: 325
- 7 Table: 197
- 9 Wall: 11361
