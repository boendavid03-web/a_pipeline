# Deployment manifest

This bundle contains only fixed-dual-LiDAR inference, tracking, global-path-to-local-subgoal conversion, ROS messages, model files, and deployment tools. It deliberately excludes SLAM, mapping, maps, global planning, training data, training scripts, simulator assets, historical runs, and DR-SPAAM dataset/plot/training code.

Runtime package responsibilities:

- `semantic_nav_runtime`: scan adaptation/merging, path subgoal conversion, S3-Net, SemanticCNN, tracker, and DRL-VO.
- `dr_spaam_ros2`: ROS adapter for retained DR-SPAAM inference library.
- `navigation_evaluation_msgs`: typed shadow telemetry messages.
- `third_party/dr_spaam`: retained detector, model, and utility imports only.
