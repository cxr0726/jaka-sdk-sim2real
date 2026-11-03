cmake_minimum_required(VERSION 3.8)
project(kpi_deploy_real)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Default Release when not specified (single-configuration generators).
if(NOT CMAKE_CONFIGURATION_TYPES)
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose Debug or Release" FORCE)
  endif()
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-missing-field-initializers)
endif()

# Bundled third-party deps (synced into ext_libs via robohub_ros_ws sync script).
set(_kpi_lwc_root "./ext_libs/legged_waist_controller")
set(_kpi_lwc_include "${_kpi_lwc_root}/include")

message(STATUS "${PROJECT_NAME}: CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")
option(KPI_BUILD_REAL "Build the real RobotIntf deployment executable" ON)

message(STATUS "${PROJECT_NAME}: _kpi_lwc_root=${_kpi_lwc_root}")
message(STATUS "${PROJECT_NAME}: KPI_BUILD_REAL=${KPI_BUILD_REAL}")


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Threads REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(YAML-CPP REQUIRED)
if(NOT YAML-CPP_FOUND)
  message(FATAL_ERROR "YAML-CPP library not found! Please install libyaml-cpp-dev.")
endif()

set(_kpi_build_rpath "runtime_lib:/opt/ros/humble/lib:${CMAKE_CURRENT_SOURCE_DIR}/ext_libs/zmq/lib")
set(_kpi_install_rpath "runtime_lib:/opt/ros/humble/lib")

file(GLOB KPI_COMMON_SOURCES src/*.cpp)

if(KPI_BUILD_REAL)
  add_executable(KpiDeployReal
    ${KPI_COMMON_SOURCES}
  )

  ament_target_dependencies(KpiDeployReal
    rclcpp
    sensor_msgs
    trajectory_msgs
  )

  target_include_directories(KpiDeployReal PRIVATE
    include
    ${_kpi_lwc_include}/legged_waist_controller
    ${_kpi_lwc_include}/rt_thread
    ${CMAKE_CURRENT_SOURCE_DIR}/ext_libs/onnxruntime-linux-x64-1.24.4/include
    ${CMAKE_CURRENT_SOURCE_DIR}/ext_libs/zmq/include
    ${CMAKE_CURRENT_SOURCE_DIR}/ext_libs
  )

  target_link_directories(KpiDeployReal PRIVATE
    "runtime_lib"
    "${CMAKE_CURRENT_SOURCE_DIR}/ext_libs/zmq/lib"
  )

  target_link_options(KpiDeployReal PRIVATE
    "LINKER:--disable-new-dtags"
  )

  set_target_properties(KpiDeployReal PROPERTIES
    BUILD_RPATH "${_kpi_build_rpath}"
    INSTALL_RPATH "${_kpi_install_rpath}"
    INSTALL_RPATH_USE_LINK_PATH FALSE
  )

  target_link_libraries(KpiDeployReal
    Eigen3::Eigen
    Threads::Threads
    legged_waist_robot_intf
    onnxruntime
    yaml-cpp
    zmq
  )

endif()

include(CTest)
if(BUILD_TESTING)
  # These targets never construct RobotIntf or start ROS/controller threads.
  add_executable(RealSuperMimicDeploymentTest
    tests/test_supermimic_deployment.cpp src/SuperMimicDeployment.cpp)
  target_include_directories(RealSuperMimicDeploymentTest PRIVATE include
    ext_libs/onnxruntime-linux-x64-1.24.4/include)
  target_link_libraries(RealSuperMimicDeploymentTest PRIVATE Eigen3::Eigen yaml-cpp
    "${CMAKE_CURRENT_SOURCE_DIR}/runtime_lib/libonnxruntime.so")
  add_test(NAME real_supermimic_yaml_contract COMMAND RealSuperMimicDeploymentTest
    "${CMAKE_CURRENT_SOURCE_DIR}/config/policy_mimic/ckpt_0100000_new_supermimic.onnx"
    "${CMAKE_CURRENT_SOURCE_DIR}/config/KpiConfig.yaml")
  set_tests_properties(real_supermimic_yaml_contract PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

  add_executable(RealSuperMimicSafetyTest tests/test_supermimic_safety.cpp
    src/SuperMimicSafety.cpp src/ParallelAnkleSolver.cpp)
  target_include_directories(RealSuperMimicSafetyTest BEFORE PRIVATE tests/stubs include)
  add_test(NAME real_supermimic_ankle_safety COMMAND RealSuperMimicSafetyTest)

  add_executable(RealSourceTimelineTest tests/test_source_timeline.cpp)
  target_include_directories(RealSourceTimelineTest PRIVATE include)
  add_test(NAME real_supermimic_source_timeline COMMAND RealSourceTimelineTest)

  add_executable(RealSuperMotionBufferTest
    tests/test_super_realtime_motion_buffer.cpp src/SuperRealtimeMotionBuffer.cpp)
  target_include_directories(RealSuperMotionBufferTest PRIVATE include ext_libs ext_libs/zmq/include)
  target_link_libraries(RealSuperMotionBufferTest PRIVATE Eigen3::Eigen Threads::Threads
    "${CMAKE_CURRENT_SOURCE_DIR}/runtime_lib/libzmq.so")
  add_test(NAME real_supermimic_motion_buffer COMMAND RealSuperMotionBufferTest)

  add_executable(RealSuperMimicPolicyTest tests/test_supermimic_policy.cpp
    src/GlobalVars.cpp src/Util.cpp src/ParKine.cpp src/FSM.cpp
    src/SuperMimicDeployment.cpp src/SuperMimicSafety.cpp src/ParallelAnkleSolver.cpp
    src/SuperRealtimeMotionBuffer.cpp src/StatePublisher.cpp)
  target_include_directories(RealSuperMimicPolicyTest PRIVATE include
    ${_kpi_lwc_include}/legged_waist_controller ${_kpi_lwc_include}/rt_thread
    ext_libs/onnxruntime-linux-x64-1.24.4/include ext_libs/zmq/include ext_libs)
  ament_target_dependencies(RealSuperMimicPolicyTest rclcpp sensor_msgs trajectory_msgs)
  target_link_directories(RealSuperMimicPolicyTest PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/runtime_lib")
  target_link_libraries(RealSuperMimicPolicyTest Eigen3::Eigen Threads::Threads
    legged_waist_robot_intf onnxruntime yaml-cpp zmq)
  set_target_properties(RealSuperMimicPolicyTest PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/runtime_lib:/opt/ros/humble/lib")
  add_test(NAME real_supermimic_policy_math COMMAND RealSuperMimicPolicyTest
    "${CMAKE_CURRENT_SOURCE_DIR}/config/KpiConfig.yaml"
    "${CMAKE_CURRENT_SOURCE_DIR}/config/policy_mimic/ckpt_0100000_new_supermimic.onnx")
  set_tests_properties(real_supermimic_policy_math PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

  add_executable(RealSuperMimicConfigTest tests/test_supermimic_config.cpp
    src/GlobalVars.cpp src/ParKine.cpp src/SuperMimicDeployment.cpp)
  target_include_directories(RealSuperMimicConfigTest PRIVATE include
    ${_kpi_lwc_include}/legged_waist_controller ${_kpi_lwc_include}/rt_thread
    ext_libs/onnxruntime-linux-x64-1.24.4/include ext_libs)
  ament_target_dependencies(RealSuperMimicConfigTest rclcpp sensor_msgs trajectory_msgs)
  target_link_directories(RealSuperMimicConfigTest PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/runtime_lib")
  target_link_libraries(RealSuperMimicConfigTest Eigen3::Eigen Threads::Threads
    legged_waist_robot_intf onnxruntime yaml-cpp)
  set_target_properties(RealSuperMimicConfigTest PROPERTIES BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/runtime_lib:/opt/ros/humble/lib")
  add_test(NAME real_supermimic_actual_config_init COMMAND RealSuperMimicConfigTest
    "${CMAKE_CURRENT_SOURCE_DIR}/config/KpiConfig.yaml")
  set_tests_properties(real_supermimic_actual_config_init PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
endif()

ament_package()
