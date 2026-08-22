cmake_minimum_required(VERSION 3.8)
project(kau_object_detection)

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
# tf2 supplies the map <- scan frame transform the Object List is published in.
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# Start signal detection only needs core OpenCV plus JPEG decoding and colour
# conversion. No cv_bridge: the compressed payload goes straight to cv::imdecode.
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

# Gazebo Transport is optional. The CMake config packages that ship with Gazebo
# Harmonic are preferred because their imported targets already carry the
# transitive gz-msgs and gz-math dependencies. pkg-config is the fallback for
# installations that expose only the .pc files. When neither is present the
# package still builds and the world pose diagnostics simply stay unavailable.
set(KAU_GZ_AVAILABLE FALSE)
find_package(gz-transport13 QUIET)
if(gz-transport13_FOUND)
  set(KAU_GZ_AVAILABLE TRUE)
  set(KAU_GZ_LINK_TARGET gz-transport13::core)
  message(STATUS "kau_object_detection: gz-transport13 CMake config package found")
else()
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(KAU_GZ_PKG QUIET IMPORTED_TARGET gz-transport13)
    if(KAU_GZ_PKG_FOUND)
      set(KAU_GZ_AVAILABLE TRUE)
      set(KAU_GZ_LINK_TARGET PkgConfig::KAU_GZ_PKG)
      message(STATUS "kau_object_detection: gz-transport13 pkg-config module found")
    endif()
  endif()
endif()
if(NOT KAU_GZ_AVAILABLE)
  message(WARNING
    "kau_object_detection: gz-transport13 not found; "
    "world pose diagnostics are disabled in this build")
endif()

add_library(laser_scan_validator
  src/laser_scan_validator.cpp
)
target_include_directories(laser_scan_validator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(laser_scan_validator sensor_msgs)

add_library(laser_scan_clusterer
  src/laser_scan_clusterer.cpp
)
target_include_directories(laser_scan_clusterer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(laser_scan_clusterer sensor_msgs)

add_library(cluster_geometry
  src/cluster_geometry.cpp
)
target_include_directories(cluster_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(cluster_geometry sensor_msgs)

add_library(world_pose
  src/world_pose.cpp
)
target_include_directories(world_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(track_geometry
  src/track_geometry.cpp
)
target_include_directories(track_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(cone_occupancy
  src/cone_occupancy.cpp
)
target_include_directories(cone_occupancy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(cone_occupancy cluster_geometry)
ament_target_dependencies(cone_occupancy sensor_msgs)

add_library(track_roi
  src/track_roi.cpp
)
target_include_directories(track_roi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(track_roi track_geometry cone_occupancy)
ament_target_dependencies(track_roi sensor_msgs)

add_library(obstacle_tracker
  src/obstacle_tracker.cpp
)
target_include_directories(obstacle_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(object_list_transform
  src/object_list_transform.cpp
)
target_include_directories(object_list_transform PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(object_list_transform track_roi)
ament_target_dependencies(object_list_transform sensor_msgs)

add_library(object_list
  src/object_list.cpp
)
target_include_directories(object_list PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(object_list track_roi)
ament_target_dependencies(object_list kau_msgs builtin_interfaces)

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

add_executable(laser_scan_validator_node
  src/laser_scan_validator_node.cpp
)
target_link_libraries(laser_scan_validator_node laser_scan_validator)
ament_target_dependencies(laser_scan_validator_node rclcpp sensor_msgs)

add_executable(laser_scan_clusterer_node
  src/laser_scan_clusterer_node.cpp
)
target_link_libraries(
  laser_scan_clusterer_node
  laser_scan_validator
  laser_scan_clusterer
  cluster_geometry
  cone_occupancy
  track_geometry
  track_roi
  world_pose
  object_list
  object_list_transform
  obstacle_tracker
)
ament_target_dependencies(
  laser_scan_clusterer_node
  rclcpp
  sensor_msgs
  kau_msgs
  builtin_interfaces
  geometry_msgs
  tf2
  tf2_ros
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

if(KAU_GZ_AVAILABLE)
  target_link_libraries(laser_scan_clusterer_node ${KAU_GZ_LINK_TARGET})
  target_compile_definitions(laser_scan_clusterer_node
    PRIVATE KAU_OBJECT_DETECTION_WITH_GZ_TRANSPORT=1
  )
endif()

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
    laser_scan_validator
    laser_scan_clusterer
    cluster_geometry
    cone_occupancy
    track_geometry
    track_roi
    world_pose
    object_list
    object_list_transform
    obstacle_tracker
    start_signal_logic
    green_lamp_detector
    laser_scan_validator_node
    laser_scan_clusterer_node
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

  ament_add_gtest(test_laser_scan_validator
    test/test_laser_scan_validator.cpp
  )
  if(TARGET test_laser_scan_validator)
    target_link_libraries(test_laser_scan_validator laser_scan_validator)
    ament_target_dependencies(test_laser_scan_validator sensor_msgs)
  endif()

  ament_add_gtest(test_laser_scan_clusterer
    test/test_laser_scan_clusterer.cpp
  )
  if(TARGET test_laser_scan_clusterer)
    target_link_libraries(test_laser_scan_clusterer laser_scan_clusterer)
    ament_target_dependencies(test_laser_scan_clusterer sensor_msgs)
  endif()

  ament_add_gtest(test_cluster_geometry
    test/test_cluster_geometry.cpp
  )
  if(TARGET test_cluster_geometry)
    target_link_libraries(test_cluster_geometry cluster_geometry laser_scan_clusterer)
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
      laser_scan_clusterer
    )
    ament_target_dependencies(test_cone_occupancy sensor_msgs)
  endif()

  ament_add_gtest(test_world_pose
    test/test_world_pose.cpp
  )
  if(TARGET test_world_pose)
    target_link_libraries(test_world_pose world_pose)
  endif()

  ament_add_gtest(test_track_geometry
    test/test_track_geometry.cpp
  )
  if(TARGET test_track_geometry)
    target_link_libraries(test_track_geometry track_geometry)
  endif()

  ament_add_gtest(test_track_roi
    test/test_track_roi.cpp
  )
  if(TARGET test_track_roi)
    target_link_libraries(
      test_track_roi
      track_roi
      track_geometry
      cone_occupancy
      cluster_geometry
      world_pose
    )
    ament_target_dependencies(test_track_roi sensor_msgs)
  endif()

  ament_add_gtest(test_object_list
    test/test_object_list.cpp
  )
  if(TARGET test_object_list)
    target_link_libraries(
      test_object_list
      object_list
      track_roi
      track_geometry
      cone_occupancy
      cluster_geometry
    )
    ament_target_dependencies(test_object_list kau_msgs builtin_interfaces sensor_msgs)
  endif()

  ament_add_gtest(test_object_list_transform
    test/test_object_list_transform.cpp
  )
  if(TARGET test_object_list_transform)
    target_link_libraries(
      test_object_list_transform
      object_list_transform
      object_list
      track_roi
      track_geometry
      cone_occupancy
      cluster_geometry
    )
    ament_target_dependencies(
      test_object_list_transform
      kau_msgs
      builtin_interfaces
      sensor_msgs
    )
  endif()

  ament_add_gtest(test_obstacle_tracker
    test/test_obstacle_tracker.cpp
  )
  if(TARGET test_obstacle_tracker)
    target_link_libraries(test_obstacle_tracker obstacle_tracker)
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
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  sensor_msgs kau_msgs builtin_interfaces std_msgs OpenCV
  geometry_msgs tf2 tf2_ros)

ament_package()
