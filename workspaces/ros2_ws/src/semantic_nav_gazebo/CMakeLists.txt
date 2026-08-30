cmake_minimum_required(VERSION 3.8)
project(semantic_nav_gazebo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(navigation_evaluation_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-gazebo6 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DrlVoTrainingState.msg"
  "msg/PedestrianState.msg"
  "msg/PedestrianStateArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(cmd_vel_ign_relay src/cmd_vel_ign_relay.cpp)
ament_target_dependencies(cmd_vel_ign_relay rclcpp geometry_msgs)
target_link_libraries(
  cmd_vel_ign_relay
  ignition-transport11::ignition-transport11
  ignition-msgs8::ignition-msgs8
)

add_executable(pedestrian_actor_pose_bridge src/pedestrian_actor_pose_bridge.cpp)
ament_target_dependencies(pedestrian_actor_pose_bridge rclcpp geometry_msgs)
target_link_libraries(
  pedestrian_actor_pose_bridge
  ignition-transport11::ignition-transport11
  ignition-msgs8::ignition-msgs8
)

add_library(pedestrian_actor_motion_system SHARED
  src/pedestrian_actor_motion_system.cpp
)
target_link_libraries(
  pedestrian_actor_motion_system
  ignition-gazebo6::ignition-gazebo6
  ignition-transport11::ignition-transport11
  ignition-msgs8::ignition-msgs8
)

install(
  DIRECTORY launch config maps worlds docs models scenarios rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN "*.bak" EXCLUDE
  PATTERN "*.bak.*" EXCLUDE
)

install(
  PROGRAMS
    scripts/continuous_teleop.py
    scripts/closed_loop_demo_recorder.py
    scripts/auto_goal_rosbag_scheduler.py
    scripts/calf_policy_node.py
    scripts/demo_pedestrian_controller.py
    scripts/demo_goal_arrival_node.py
    scripts/demo_velocity_display_node.py
    scripts/drl_vo_fixed_dual_inference_node.py
    scripts/drl_vo_control_contract.py
    scripts/navigation_episode_evaluator.py
    scripts/navigation_evaluation_core.py
    scripts/drl_vo_online_ppo_training_node.py
    scripts/episode_goal_picker.py
    scripts/fixed_goal_sequence.py
    scripts/fixed_four_video_capture.py
    scripts/isaac_pedestrian_ground_truth_adapter.py
    scripts/scenario_pedestrian_controller.py
    scripts/odom_path_node.py
    scripts/run_v7_dual_slam.sh
    scripts/s3net_fixed_dual_inference_node.py
    scripts/semantic_cnn_fixed_dual_inference_node.py
    scripts/semantic_cnn_v7_inference_node.py
    scripts/semantic_start_goal_path_node.py
    scripts/teleop_episode_recorder_controller.py
    scripts/tokennav_v7_inference_node.py
    scripts/v7_dual_laser_scan_merger.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    cmd_vel_ign_relay
    pedestrian_actor_pose_bridge
    pedestrian_actor_motion_system
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_pytest_test(
    test_scenario_pedestrian_time
    test/test_scenario_pedestrian_time.py
  )
  ament_add_pytest_test(
    test_navigation_evaluation_core
    test/test_navigation_evaluation_core.py
  )
  ament_add_pytest_test(
    test_navigation_episode_evaluator_multi
    test/test_navigation_episode_evaluator_multi.py
  )
  ament_add_pytest_test(
    test_odom_path_node
    test/test_odom_path_node.py
  )
  ament_add_pytest_test(
    test_auto_goal_rosbag_scheduler_helpers
    test/test_auto_goal_rosbag_scheduler_helpers.py
  )
  ament_add_pytest_test(
    test_drl_vo_control_contract
    test/test_drl_vo_control_contract.py
  )
  ament_add_pytest_test(
    test_fixed_goal_sequence_config
    test/test_fixed_goal_sequence_config.py
  )
  ament_add_pytest_test(
    test_fixed_four_video_contract
    test/test_fixed_four_video_contract.py
  )
  set(semantic_cnn_test_site_packages
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../..//.venvs/train/lib/python3.10/site-packages"
  )
  if(EXISTS "${semantic_cnn_test_site_packages}")
    ament_add_pytest_test(
      test_semantic_cnn_fixed_dual_helpers
      test/test_semantic_cnn_fixed_dual_helpers.py
      PYTHON_EXECUTABLE "${Python3_EXECUTABLE}"
      APPEND_ENV "PYTHONPATH=${semantic_cnn_test_site_packages}"
    )
  else()
    message(STATUS
      "Skipping test_semantic_cnn_fixed_dual_helpers: "
      "project .venvs/train site-packages are not available"
    )
  endif()
  set(calf_test_site_packages
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../..//.venvs/calf_ros2/lib/python3.10/site-packages"
  )
  if(EXISTS "${calf_test_site_packages}")
    ament_add_pytest_test(
      test_calf_policy_helpers
      test/test_calf_policy_helpers.py
      PYTHON_EXECUTABLE "${Python3_EXECUTABLE}"
      APPEND_ENV
        "PYTHONPATH=${calf_test_site_packages}:${CMAKE_CURRENT_SOURCE_DIR}/../../../../github_src/drl_vo_nav-drl_vo/LegNav-Sim-master"
    )
  else()
    message(STATUS
      "Skipping test_calf_policy_helpers: project .venvs/calf_ros2 site-packages are not available"
    )
  endif()
endif()

ament_package()
