cmake_minimum_required(VERSION 3.8)
project(kau_object_detection_lane)

# CMake 의 기본 빌드 타입은 "빈 문자열" 이고, 그러면 GCC 에 -O 가
# 아예 안 붙는다 (= -O0).
# 프레임당 도는 픽셀/클러스터 루프가 -O0 에서 몇 배로 느려진다.
# 2026-08-25: build/ 를 훑어보니 이 패키지가 실제로 그 상태였다
# (kau_control/kau_lane_detection 만 Release, 나머지는 빈 값).
# --cmake-args 없이 `colcon build` 해도 안전하도록 기본값을 박아 둔다.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# This package is deliberately self-contained. It must never gain a
# find_package on kau_object_detection, nor link against that package's
# targets: the whole point is that the Lane-only stack builds and runs with
# that package absent. The detection logic was forked in on 2026-08-25;
# see README.md "Source synchronisation".
# test/test_config_and_launch.py enforces this on every build.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(kau_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# RViz visualisation of the published Object List.
find_package(visualization_msgs REQUIRED)
# tf2 supplies the base_link <- lidar_link transform the Object List is
# published in. No Gazebo Transport dependency exists in this package: the
# world pose diagnostics were not forked in.
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# Start signal detection only needs core OpenCV plus JPEG decoding and colour
# conversion. No cv_bridge: the compressed payload goes straight to cv::imdecode.
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# --- Detection libraries --------------------------------------------------
# Dependency order, none of them reaching outside this package:
#
#   scan_validation      (sensor_msgs)
#   laser_clusterer      (sensor_msgs)
#   cluster_geometry     -> laser_clusterer
#   cone_occupancy       -> cluster_geometry
#   object_transform     -> cone_occupancy
#   object_list          -> object_transform, kau_msgs
#   obstacle_markers     (kau_msgs, visualization_msgs)
#   start_signal_logic   (standalone)
#   green_lamp_detector  (OpenCV)

add_library(scan_validation
  src/scan_validation.cpp
)
target_include_directories(scan_validation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(scan_validation sensor_msgs)

add_library(laser_clusterer
  src/laser_clusterer.cpp
)
target_include_directories(laser_clusterer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(laser_clusterer sensor_msgs)

add_library(cluster_geometry
  src/cluster_geometry.cpp
)
target_include_directories(cluster_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(cluster_geometry sensor_msgs)

add_library(cone_occupancy
  src/cone_occupancy.cpp
)
target_include_directories(cone_occupancy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(cone_occupancy cluster_geometry)
ament_target_dependencies(cone_occupancy sensor_msgs)

# Final coordinate stage: tf2 planar transform plus the circle carrier. No
# track ring and no mount offset arithmetic; see the header for why.
add_library(object_transform
  src/object_transform.cpp
)
target_include_directories(object_transform PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(object_transform cone_occupancy)
ament_target_dependencies(object_transform sensor_msgs)

add_library(object_list
  src/object_list.cpp
)
target_include_directories(object_list PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(object_list object_transform)
ament_target_dependencies(object_list kau_msgs builtin_interfaces)

# Visualisation-only builder of the RViz markers of one published Object List
# frame. Depends on the Object List message alone, never on the pipeline that
# produced it.
add_library(obstacle_markers
  src/obstacle_markers.cpp
)
target_include_directories(obstacle_markers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(obstacle_markers kau_msgs visualization_msgs)

add_library(start_signal_logic
  src/start_signal_logic.cpp
)
target_include_directories(start_signal_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(green_lamp_detector
  src/green_lamp_detector.cpp
)
target_include_directories(green_lamp_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(green_lamp_detector
  opencv_core
  opencv_imgproc
)

# --- Executables ----------------------------------------------------------

add_executable(lane_laser_object_detector_node
  src/lane_laser_object_detector_node.cpp
)
target_link_libraries(
  lane_laser_object_detector_node
  scan_validation
  laser_clusterer
  cluster_geometry
  cone_occupancy
  object_transform
  object_list
  obstacle_markers
)
ament_target_dependencies(
  lane_laser_object_detector_node
  rclcpp
  sensor_msgs
  kau_msgs
  builtin_interfaces
  geometry_msgs
  tf2
  tf2_ros
  visualization_msgs
)

add_executable(start_signal_detector_node
  src/start_signal_detector_node.cpp
)
target_link_libraries(
  start_signal_detector_node
  start_signal_logic
  green_lamp_detector
  opencv_core
  opencv_imgcodecs
)
ament_target_dependencies(start_signal_detector_node rclcpp sensor_msgs std_msgs)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS
    scan_validation
    laser_clusterer
    cluster_geometry
    cone_occupancy
    object_transform
    object_list
    obstacle_markers
    start_signal_logic
    green_lamp_detector
    lane_laser_object_detector_node
    start_signal_detector_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_scan_validation
    test/test_scan_validation.cpp
  )
  if(TARGET test_scan_validation)
    target_link_libraries(test_scan_validation scan_validation)
    ament_target_dependencies(test_scan_validation sensor_msgs)
  endif()

  ament_add_gtest(test_laser_clusterer
    test/test_laser_clusterer.cpp
  )
  if(TARGET test_laser_clusterer)
    target_link_libraries(test_laser_clusterer laser_clusterer)
    ament_target_dependencies(test_laser_clusterer sensor_msgs)
  endif()

  ament_add_gtest(test_cluster_geometry
    test/test_cluster_geometry.cpp
  )
  if(TARGET test_cluster_geometry)
    target_link_libraries(test_cluster_geometry cluster_geometry laser_clusterer)
    ament_target_dependencies(test_cluster_geometry sensor_msgs)
  endif()

  ament_add_gtest(test_cone_occupancy
    test/test_cone_occupancy.cpp
  )
  if(TARGET test_cone_occupancy)
    target_link_libraries(
      test_cone_occupancy
      cone_occupancy
      cluster_geometry
      laser_clusterer
    )
    ament_target_dependencies(test_cone_occupancy sensor_msgs)
  endif()

  ament_add_gtest(test_object_transform
    test/test_object_transform.cpp
  )
  if(TARGET test_object_transform)
    target_link_libraries(
      test_object_transform
      object_transform
      cone_occupancy
      cluster_geometry
    )
    ament_target_dependencies(test_object_transform sensor_msgs)
  endif()

  ament_add_gtest(test_object_list
    test/test_object_list.cpp
  )
  if(TARGET test_object_list)
    target_link_libraries(
      test_object_list
      object_list
      object_transform
      cone_occupancy
      cluster_geometry
    )
    ament_target_dependencies(test_object_list kau_msgs builtin_interfaces sensor_msgs)
  endif()

  # Proves the pipeline keeps no state between scans: no tracker, no EMA, no
  # carry-over of an earlier frame's obstacles.
  ament_add_gtest(test_no_temporal_persistence
    test/test_no_temporal_persistence.cpp
  )
  if(TARGET test_no_temporal_persistence)
    target_link_libraries(
      test_no_temporal_persistence
      object_list
      object_transform
      cone_occupancy
      cluster_geometry
      laser_clusterer
      scan_validation
    )
    ament_target_dependencies(
      test_no_temporal_persistence
      kau_msgs
      builtin_interfaces
      sensor_msgs
    )
  endif()

  ament_add_gtest(test_obstacle_markers
    test/test_obstacle_markers.cpp
  )
  if(TARGET test_obstacle_markers)
    target_link_libraries(
      test_obstacle_markers
      obstacle_markers
      object_list
      object_transform
      cone_occupancy
      cluster_geometry
    )
    ament_target_dependencies(
      test_obstacle_markers
      kau_msgs
      builtin_interfaces
      sensor_msgs
      visualization_msgs
    )
  endif()

  ament_add_gtest(test_start_signal_logic
    test/test_start_signal_logic.cpp
  )
  if(TARGET test_start_signal_logic)
    target_link_libraries(test_start_signal_logic start_signal_logic)
  endif()

  ament_add_gtest(test_green_lamp_detector
    test/test_green_lamp_detector.cpp
  )
  if(TARGET test_green_lamp_detector)
    target_link_libraries(
      test_green_lamp_detector
      green_lamp_detector
      opencv_core
      opencv_imgproc
    )
  endif()

  # Static checks of the shipped YAML and launch files: valid syntax, and no
  # parameter or path that would reintroduce a map/tracker/kau_object_detection
  # dependency.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_config_and_launch test/test_config_and_launch.py)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  sensor_msgs kau_msgs builtin_interfaces std_msgs OpenCV
  geometry_msgs tf2 tf2_ros visualization_msgs)

ament_package()
