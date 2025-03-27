cmake_minimum_required(VERSION 3.8)
project(event_detector)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})
add_compile_definitions(ROS_VERSION=$ENV{ROS_VERSION})
add_compile_definitions(ROS_DISTRO=$ENV{ROS_DISTRO})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(etsi_its_cam_msgs REQUIRED)
find_package(etsi_its_cam_ts_msgs REQUIRED)
find_package(etsi_its_cpm_ts_msgs REQUIRED)
find_package(etsi_its_denm_msgs REQUIRED)
find_package(etsi_its_mapem_ts_msgs REQUIRED)
find_package(etsi_its_spatem_ts_msgs REQUIRED)
find_package(etsi_its_vam_ts_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(${PROJECT_NAME}_lib SHARED
  src/EventDetector.cpp
  src/AnalysisManager.cpp
  src/AnalysisRule.cpp
  src/BufferManager.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "event_detector::EventDetector"
  EXECUTABLE ${PROJECT_NAME})

target_include_directories(
  ${PROJECT_NAME}_lib PUBLIC
                        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                        $<INSTALL_INTERFACE:include>
                        ${rclcpp_INCLUDE_DIRS}
                        ${rclcpp_lifecycle_INCLUDE_DIRS}
                        ${lifecycle_msgs_INCLUDE_DIRS})
target_compile_features(${PROJECT_NAME}_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(${PROJECT_NAME}_lib
  builtin_interfaces
  diagnostic_msgs
  etsi_its_cam_msgs
  etsi_its_cam_ts_msgs
  etsi_its_cpm_ts_msgs
  etsi_its_denm_msgs
  etsi_its_mapem_ts_msgs
  etsi_its_spatem_ts_msgs
  etsi_its_vam_ts_msgs
  geometry_msgs
  lifecycle_msgs
  nav_msgs
  perception_msgs
  pluginlib
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  rosbag2_cpp
  sensor_msgs
  shape_msgs
  std_msgs
  std_srvs
  stereo_msgs
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
  trajectory_msgs
  vision_msgs
  visualization_msgs
)

ament_export_targets(${PROJECT_NAME}_libTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  builtin_interfaces
  diagnostic_msgs
  etsi_its_cam_msgs
  etsi_its_cam_ts_msgs
  etsi_its_cpm_ts_msgs
  etsi_its_denm_msgs
  etsi_its_mapem_ts_msgs
  etsi_its_spatem_ts_msgs
  etsi_its_vam_ts_msgs
  geometry_msgs
  lifecycle_msgs
  nav_msgs
  perception_msgs
  pluginlib
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  rosbag2_cpp
  sensor_msgs
  shape_msgs
  std_msgs
  std_srvs
  stereo_msgs
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
  trajectory_msgs
  vision_msgs
  visualization_msgs
)

# add git commit hash as preprocessor variable
execute_process(
  COMMAND bash -c "echo \"$(git rev-parse HEAD)$(git describe --always --dirty --match '' | grep -o -- -dirty)\""
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE GIT_COMMIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE)
target_compile_definitions(${PROJECT_NAME}_lib PRIVATE
  "-DGIT_COMMIT_HASH=\"${GIT_COMMIT_HASH}\"")

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}_libTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_clang_format_CONFIG_FILE $ENV{WORKSPACE}/src/target/.vscode/format/.clang-format)
  set(ament_cmake_clang_tidy_CONFIG_FILE $ENV{WORKSPACE}/src/target/.vscode/lint/.clang-tidy)
  set(ament_cmake_flake8_CONFIG_FILE $ENV{WORKSPACE}/src/target/.vscode/lint/ament_flake8.ini)
  ament_lint_auto_find_test_dependencies()

  # unit tests
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_buffer_test test/buffer_test.cpp)
  target_link_libraries(${PROJECT_NAME}_buffer_test ${PROJECT_NAME}_lib)
  install(TARGETS ${PROJECT_NAME}_buffer_test DESTINATION lib/${PROJECT_NAME})

  # perf tests
  set(AMENT_RUN_PERFORMANCE_TESTS 1)
  find_package(ament_cmake_google_benchmark REQUIRED)
  ament_add_google_benchmark(${PROJECT_NAME}_buffer_benchmark test/buffer_benchmark.cpp)
  ament_target_dependencies(${PROJECT_NAME}_buffer_benchmark rclcpp rosbag2_cpp)
  target_link_libraries(${PROJECT_NAME}_buffer_benchmark ${PROJECT_NAME}_lib)
  install(TARGETS ${PROJECT_NAME}_buffer_benchmark DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
