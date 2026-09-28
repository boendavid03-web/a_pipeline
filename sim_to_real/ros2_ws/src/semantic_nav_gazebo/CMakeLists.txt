cmake_minimum_required(VERSION 3.8)
project(semantic_nav_gazebo)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DrlVoTrainingState.msg"
  "msg/PedestrianState.msg"
  "msg/PedestrianStateArray.msg"
  "msg/TrackedPedestrian.msg"
  "msg/TrackedPedestrianArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/drl_vo_control_contract.py
    scripts/drl_vo_fixed_dual_inference_node.py
    scripts/fixed_dual_scan_adapter.py
    scripts/navigation_evaluation_core.py
    scripts/odom_path_node.py
    scripts/pedestrian_point_tracker.py
    scripts/pedestrian_point_tracker_core.py
    scripts/s3net_fixed_dual_inference_node.py
    scripts/semantic_cnn_fixed_dual_inference_node.py
    scripts/semantic_start_goal_path_node.py
    scripts/v7_dual_laser_scan_merger.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_fixed_dual_scan_adapter
    test/test_fixed_dual_scan_adapter.py
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
