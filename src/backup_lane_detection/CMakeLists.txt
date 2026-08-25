cmake_minimum_required(VERSION 3.8)
project(backup_lane_detection)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(backup_msgs REQUIRED)
find_package(backup_common REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(lane_detector_node
  src/main.cpp
  src/lane_detector_node.cpp
)
target_include_directories(lane_detector_node PRIVATE include ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(lane_detector_node
  rclcpp std_msgs sensor_msgs backup_msgs backup_common cv_bridge)
target_link_libraries(lane_detector_node ${OpenCV_LIBS})

install(TARGETS lane_detector_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
