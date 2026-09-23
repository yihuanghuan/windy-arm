cmake_minimum_required(VERSION 3.8)
project(manipulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==============================
# ROS2 dependencies
# ==============================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(dummy_interface REQUIRED)
find_package(serial REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

# ==============================
# Arm Platform Core Library
# ==============================
add_library(manipulator_core
  src/robotics/arm_platform.cc
  src/robotics/arm/abs_arm.cc
  src/robotics/arm/a_l1.cc
  src/robotics/arm/a_l1_gamma.cc
  src/robotics/arm/sim_arm.cc
  src/robotics/arm/torque_sim_arm.cc
  src/robotics/arm/arm_factory.cc
  src/robotics/bus/abs_bus.cc
  src/robotics/bus/serial_bus.cc
  src/robotics/motor/dm_motor.cc
  src/robotics/protocol/abs_protocol.cc
  src/robotics/protocol/protocol_factory.cc
  src/robotics/protocol/protocol_v1.cc
  src/robotics/config/arm_config.cc
)

add_library(controller_lib
  src/controller/smooth_position_controller.cc
  src/controller/gravity_controller.cc
)

add_library(planner_lib
  src/planning/abs_motion_planner.cc
  src/planning/reset_motion_planner.cc
)

add_library(collision_lib
  src/collision/collision_avoidance.cc
)

ament_target_dependencies(planner_lib
  rclcpp
  std_msgs
  sensor_msgs
  dummy_interface
)

ament_target_dependencies(controller_lib
  rclcpp
  geometry_msgs
  dummy_interface
)

target_link_libraries(planner_lib
  Eigen3::Eigen
)

target_link_libraries(controller_lib
  Eigen3::Eigen
  pinocchio::pinocchio
)

target_link_libraries(collision_lib
  Eigen3::Eigen
  pinocchio::pinocchio
)

target_include_directories(collision_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(manipulator_core
  Eigen3::Eigen
  pinocchio::pinocchio
  yaml-cpp
  collision_lib
)

target_include_directories(manipulator_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(planner_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ==============================
# Robot Arm Node (ROS2 Node)
# ==============================
add_executable(arm_hardware_node
  src/ros_nodes/arm_hardware_node.cc
)

target_link_libraries(arm_hardware_node
  manipulator_core
  controller_lib
  planner_lib
)

add_executable(master_arm_node
  src/ros_nodes/master_arm_node.cc
)

target_link_libraries(master_arm_node
  manipulator_core
  controller_lib
  planner_lib
)

ament_target_dependencies(manipulator_core
  rclcpp
  std_msgs
  sensor_msgs
  dummy_interface
  serial
)

ament_target_dependencies(master_arm_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  dummy_interface
)

add_executable(slave_arm_node
  src/ros_nodes/slave_arm_node.cc
)

add_executable(student_arm_node
  src/ros_nodes/student_arm_node.cc
)

target_link_libraries(student_arm_node
  manipulator_core
  controller_lib
  planner_lib
)

ament_target_dependencies(student_arm_node
  std_srvs
  rclcpp
  sensor_msgs
  std_msgs
  dummy_interface
)

add_executable(robot_health_aggregator
  src/ros_nodes/robot_health_aggregator.cc
)

add_executable(flight_status_bridge
  src/ros_nodes/flight_status_bridge.cc
)

target_link_libraries(slave_arm_node
  manipulator_core
  controller_lib
  planner_lib
)

ament_target_dependencies(slave_arm_node
  rclcpp
  sensor_msgs
  std_msgs
  dummy_interface
  visualization_msgs
)

ament_target_dependencies(robot_health_aggregator
  rclcpp
  std_msgs
)

ament_target_dependencies(flight_status_bridge
  rclcpp
  std_msgs
  sensor_msgs
  mavros_msgs
)

target_include_directories(robot_health_aggregator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ==============================
# Install
# ==============================
install(TARGETS
  manipulator_core
  collision_lib
  controller_lib
  planner_lib
  arm_hardware_node
  master_arm_node
  slave_arm_node
  student_arm_node
  robot_health_aggregator
  flight_status_bridge
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

install(PROGRAMS
  demo/base_trust_estimator.py
  demo/dart_noise.py
  demo/online_path_time_scaler.py
  demo/pinocchio_ik.py
  demo/mpc_pinocchio_ik.py
  demo/second_order_mpc_ik.py
  demo/move_arm_mpc_ik_demo.py
  demo/move_arm_horizontal_circle_mpc_ik_demo.py
  demo/move_ll100_lh420_nmpc_demo.py
  demo/ll100_traj_pub.py
  demo/ll100_trajectory_visualizer_demo.py
  demo/moving_base_trajectory_config.py
  demo/move_base_circle_visual_demo.py
  demo/move_base_circle_mpc_ik_demo.py
  demo/plot_moving_base_report.py
  demo/eval_uncertainty_b4.py
  demo/ll100_lh420_zero_joint_state_publisher.py
  demo/ll100_lh420_rotor_joint_state_publisher.py
  demo/tune_unified_mpc_params.py
  demo/tune_ll100_nmpc.py
  demo/trajectory_visualizer_demo.py
  task_space_smc/sim/offline_simulation.py
  task_space_smc/hw/torque_interface_probe.py
  task_space_smc/hw/whole_task_smc_ros_runner.py
  task_space_smc/hw/vicon_floating_base_bridge.py
  task_space_smc/sim/plot_whole_task_smc.py
  task_space_smc/sim/tune_whole_task_smc.py
  task_space_smc/hw/fk_gravity_verify.py
  task_space_smc/hw/phase2_torque_probe.py
  task_space_smc/hw/phase3_gravity_calibration.py
  task_space_smc/hw/phase4_gravity_compensation.py
  task_space_smc/hw/manual_guidance.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY task_space_smc/
  DESTINATION lib/${PROJECT_NAME}/task_space_smc
  FILES_MATCHING
    PATTERN "*.py"
    PATTERN "*.yaml"
    PATTERN "*.md"
)

install(PROGRAMS
  scripts/set_camera_ip.sh
  scripts/get_camera_ip.sh
  scripts/set_livox_lidar_ip.sh
  scripts/get_livox_lidar_ip.sh
  scripts/start.sh
  scripts/stop.sh
  scripts/clean.sh
  scripts/install_acados.sh
  scripts/setup_acados_env.sh
  scripts/check_nmpc_env.sh
  scripts/run_task_space_smc_whole.sh
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  demo/setup_fastdds_udp_only.sh
  DESTINATION share/${PROJECT_NAME}/demo
)

install(FILES
  demo/fastdds_udp_only.xml
  DESTINATION share/${PROJECT_NAME}/demo
)

install(FILES
  ../../7DOF_ARM_NMPC_README.md
  ../../LL100_LH420_MPC_README.md
  ../../MPC_TUNING_README.md
  DESTINATION share/${PROJECT_NAME}/demo
)

install(FILES
  scripts/robot.service
  DESTINATION lib/systemd/system
)

# ==============================
# Export
# ==============================
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs std_msgs mavros_msgs dummy_interface visualization_msgs serial)

# ==============================
# Tests & Lint
# ==============================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_torque_sim_arm
    test/test_torque_sim_arm.cc
  )
  if(TARGET test_torque_sim_arm)
    target_link_libraries(test_torque_sim_arm manipulator_core)
    target_compile_definitions(test_torque_sim_arm PRIVATE
      MANIPULATOR_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )
  endif()

  ament_add_gtest(test_arm6_config
    test/test_arm6_config.cc
  )
  if(TARGET test_arm6_config)
    target_link_libraries(test_arm6_config manipulator_core ${YAML_CPP_LIBRARIES})
    target_compile_definitions(test_arm6_config PRIVATE
      MANIPULATOR_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )
  endif()

  ament_add_gtest(test_protocol_heartbeat
    test/test_protocol_heartbeat.cc
  )
  if(TARGET test_protocol_heartbeat)
    target_link_libraries(test_protocol_heartbeat manipulator_core)
  endif()

  ament_add_gtest(test_effort_safety
    test/test_effort_safety.cc
  )
  if(TARGET test_effort_safety)
    target_link_libraries(test_effort_safety manipulator_core)
    ament_target_dependencies(test_effort_safety std_srvs)
  endif()
endif()

ament_package()
