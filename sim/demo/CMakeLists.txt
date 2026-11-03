cmake_minimum_required(VERSION 3.16)
project(kpi_mujoco_sim_sdk_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose Debug or Release" FORCE)
endif()

if(NOT Python3_EXECUTABLE)
  set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "Python used by ament_cmake")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-missing-field-initializers)
endif()

get_filename_component(_demo_root "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)
set(KPI_MUJOCO_SIM_SDK_ROOT "${_demo_root}/../sdk" CACHE PATH
  "Path to the packaged kpi_mujoco_sim SDK root")
get_filename_component(KPI_MUJOCO_SIM_SDK_ROOT "${KPI_MUJOCO_SIM_SDK_ROOT}" ABSOLUTE)

set(_sdk_include "${KPI_MUJOCO_SIM_SDK_ROOT}/include")
set(_sdk_kpi_include "${KPI_MUJOCO_SIM_SDK_ROOT}/include/kpi_mujoco_sim")
set(_sdk_release_lib "${KPI_MUJOCO_SIM_SDK_ROOT}/lib/release/libkpi_mujoco_sim.so")
set(_sdk_mujoco_include "${KPI_MUJOCO_SIM_SDK_ROOT}/third_party/mujoco/include")
set(_sdk_mujoco_lib "${KPI_MUJOCO_SIM_SDK_ROOT}/third_party/mujoco/lib/libmujoco.so")
set(_sdk_glfw_lib "${KPI_MUJOCO_SIM_SDK_ROOT}/third_party/glfw/lib/libglfw.so")

foreach(_required_path
    "${_sdk_include}"
    "${_sdk_kpi_include}"
    "${_sdk_release_lib}"
    "${_sdk_mujoco_include}"
    "${_sdk_mujoco_lib}"
    "${_sdk_glfw_lib}")
  if(NOT EXISTS "${_required_path}")
    message(FATAL_ERROR "Required SDK file or directory not found: ${_required_path}")
  endif()
endforeach()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(YAML-CPP REQUIRED)
find_package(OpenGL REQUIRED)

set(_demo_onnx_root "${CMAKE_CURRENT_SOURCE_DIR}/ext_libs/onnxruntime-linux-x64-1.24.4")
set(_demo_onnx_include "${_demo_onnx_root}/include")
set(_demo_onnx_libdir "${_demo_onnx_root}/lib")

foreach(_required_path
    "${_demo_onnx_include}"
    "${_demo_onnx_libdir}/libonnxruntime.so")
  if(NOT EXISTS "${_required_path}")
    message(FATAL_ERROR "Required demo dependency not found: ${_required_path}")
  endif()
endforeach()

set(KPI_DEMO_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/KpiSimMain.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/FSM.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/FSMInit.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/FSMIdle.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/FSMLoco.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/FSMFault.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/FSMMimic.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/FSMMimicJakaMiniZmq.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/FSMMimicJakaMiniZmqFixwrist.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/FSMMimicJakaMiniSuperZmq.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/SuperMimicDeployment.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/RealtimeMotionBuffer.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/StatePublisher.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/GlobalVars.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/ParKine.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/ParallelAnkleSolver.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/CheckedMujocoPhysics.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/Util.cpp"
)

add_executable(KpiMujocoSimDemo
  ${KPI_DEMO_SOURCES}
)

ament_target_dependencies(KpiMujocoSimDemo
  rclcpp
  sensor_msgs
  trajectory_msgs
)

target_include_directories(KpiMujocoSimDemo PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${_sdk_include}"
  "${_sdk_kpi_include}"
  "${_sdk_mujoco_include}"
  "${_demo_onnx_include}"
  "${CMAKE_CURRENT_SOURCE_DIR}/ext_libs/zmq/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/ext_libs"
)

target_compile_definitions(KpiMujocoSimDemo PRIVATE
  KPI_CONFIG_BASE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
  KPI_LOG_BASE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/log"
  KPI_MUJOCO_HAS_VIEWER=1
)

target_link_directories(KpiMujocoSimDemo PRIVATE
  "${_demo_onnx_libdir}"
  "${CMAKE_CURRENT_SOURCE_DIR}/ext_libs/zmq/lib"
)

target_link_libraries(KpiMujocoSimDemo
  Eigen3::Eigen
  Threads::Threads
  "${_sdk_release_lib}"
  "${_sdk_mujoco_lib}"
  "${_sdk_glfw_lib}"
  OpenGL::GL
  onnxruntime
  yaml-cpp
  zmq
)

# The packaged SDK calls these public C symbols through its PLT. Export the
# checked implementations explicitly so SDK PD and policy feedback share the
# same geometry, rather than resolving the SDK's legacy Newton implementation.
if(UNIX AND NOT APPLE)
  target_link_options(KpiMujocoSimDemo PRIVATE
    "-Wl,--export-dynamic-symbol=AB2PitchRoll"
    "-Wl,--export-dynamic-symbol=PitchRoll2AB")
endif()

set(_demo_rpath
  "${KPI_MUJOCO_SIM_SDK_ROOT}/lib/release"
  "${KPI_MUJOCO_SIM_SDK_ROOT}/third_party/mujoco/lib"
  "${KPI_MUJOCO_SIM_SDK_ROOT}/third_party/glfw/lib"
  "${_demo_onnx_libdir}"
  "${CMAKE_CURRENT_SOURCE_DIR}/ext_libs/zmq/lib"
  "/opt/ros/humble/lib"
)
list(JOIN _demo_rpath ":" _demo_rpath_joined)

set_target_properties(KpiMujocoSimDemo PROPERTIES
  BUILD_RPATH "${_demo_rpath_joined}"
  INSTALL_RPATH "${_demo_rpath_joined}"
  INSTALL_RPATH_USE_LINK_PATH FALSE
)

install(TARGETS KpiMujocoSimDemo
  RUNTIME DESTINATION bin
)

include(CTest)
if(BUILD_TESTING)
  add_executable(ParallelAnkleSolverTest tests/test_parallel_ankle_solver.cpp
    src/ParallelAnkleSolver.cpp src/ParKine.cpp)
  target_include_directories(ParallelAnkleSolverTest PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
  add_test(NAME parallel_ankle_solver COMMAND ParallelAnkleSolverTest)

  add_executable(CheckedMujocoPhysicsTest tests/test_checked_mujoco_physics.cpp
    src/CheckedMujocoPhysics.cpp src/ParallelAnkleSolver.cpp src/ParKine.cpp)
  ament_target_dependencies(CheckedMujocoPhysicsTest rclcpp sensor_msgs trajectory_msgs)
  target_include_directories(CheckedMujocoPhysicsTest PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${_sdk_include}" "${_sdk_kpi_include}" "${_sdk_mujoco_include}")
  target_link_libraries(CheckedMujocoPhysicsTest "${_sdk_release_lib}" "${_sdk_mujoco_lib}")
  target_link_options(CheckedMujocoPhysicsTest PRIVATE
    "-Wl,--export-dynamic-symbol=AB2PitchRoll" "-Wl,--export-dynamic-symbol=PitchRoll2AB")
  set_target_properties(CheckedMujocoPhysicsTest PROPERTIES BUILD_RPATH "${_demo_rpath_joined}")
  add_test(NAME checked_mujoco_physics COMMAND CheckedMujocoPhysicsTest
    "${KPI_MUJOCO_SIM_SDK_ROOT}/assets/khan_mini/Khan_mini_sym.xml")

  add_executable(AnkleFaultHandlingTest tests/test_ankle_fault_handling.cpp
    src/Util.cpp src/ParallelAnkleSolver.cpp src/ParKine.cpp)
  ament_target_dependencies(AnkleFaultHandlingTest rclcpp sensor_msgs trajectory_msgs)
  target_include_directories(AnkleFaultHandlingTest PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${_sdk_include}" "${_sdk_kpi_include}" "${_sdk_mujoco_include}" "${_demo_onnx_include}")
  target_link_libraries(AnkleFaultHandlingTest Eigen3::Eigen "${_sdk_release_lib}" onnxruntime yaml-cpp)
  target_link_directories(AnkleFaultHandlingTest PRIVATE "${_demo_onnx_libdir}")
  set_target_properties(AnkleFaultHandlingTest PROPERTIES BUILD_RPATH "${_demo_rpath_joined}")
  add_test(NAME ankle_fault_handling COMMAND AnkleFaultHandlingTest)

  add_executable(ContactModelTest tests/test_contact_model.cpp)
  target_include_directories(ContactModelTest PRIVATE "${_sdk_mujoco_include}")
  target_link_libraries(ContactModelTest "${_sdk_mujoco_lib}")
  set_target_properties(ContactModelTest PROPERTIES BUILD_RPATH "${KPI_MUJOCO_SIM_SDK_ROOT}/third_party/mujoco/lib")
  add_test(NAME contact_model COMMAND ContactModelTest
    "${KPI_MUJOCO_SIM_SDK_ROOT}/assets/khan_mini/Khan_mini_sym.xml"
    "${KPI_MUJOCO_SIM_SDK_ROOT}/assets/khan_mini/Khan_mini_contact.xml")

  add_executable(SourceTimelineTest tests/test_source_timeline.cpp)
  target_include_directories(SourceTimelineTest PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")
  add_test(NAME source_timeline COMMAND SourceTimelineTest)

  add_executable(RealtimeMotionBufferTest
    tests/test_realtime_motion_buffer.cpp src/RealtimeMotionBuffer.cpp)
  target_include_directories(RealtimeMotionBufferTest PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/ext_libs"
    "${CMAKE_CURRENT_SOURCE_DIR}/ext_libs/zmq/include")
  target_link_directories(RealtimeMotionBufferTest PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/ext_libs/zmq/lib")
  target_link_libraries(RealtimeMotionBufferTest Eigen3::Eigen Threads::Threads zmq)
  set_target_properties(RealtimeMotionBufferTest PROPERTIES
    BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/ext_libs/zmq/lib")
  add_test(NAME realtime_motion_buffer COMMAND RealtimeMotionBufferTest)
  add_test(NAME pico_publisher_timestamps
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_pico_publisher_timestamps.py")
  add_test(NAME super_npz_publisher
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_super_npz_publisher.py")

  add_executable(SuperMimicDeploymentTest
    tests/test_supermimic_deployment.cpp src/SuperMimicDeployment.cpp)
  target_include_directories(SuperMimicDeploymentTest PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/ext_libs"
    "${_demo_onnx_include}")
  target_link_directories(SuperMimicDeploymentTest PRIVATE "${_demo_onnx_libdir}")
  target_link_libraries(SuperMimicDeploymentTest Eigen3::Eigen onnxruntime)
  set_target_properties(SuperMimicDeploymentTest PROPERTIES BUILD_RPATH "${_demo_onnx_libdir}")
  add_test(NAME supermimic_deployment COMMAND SuperMimicDeploymentTest
    "${CMAKE_CURRENT_SOURCE_DIR}/export/khan_mini/ckpt_0100000.onnx"
    "${CMAKE_CURRENT_SOURCE_DIR}/config/policy_mimic/latest_m62k_pico_base.onnx")
endif()

ament_package()
