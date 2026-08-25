cmake_minimum_required(VERSION 3.8)
project(backup_object_detection)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(backup_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(obstacle_detector_node
  src/main_obstacle.cpp
  src/obstacle_detector_node.cpp
)
target_include_directories(obstacle_detector_node PRIVATE include)
ament_target_dependencies(obstacle_detector_node
  rclcpp std_msgs sensor_msgs backup_msgs)

add_executable(start_signal_node
  src/main_start_signal.cpp
  src/start_signal_node.cpp
)
target_include_directories(start_signal_node PRIVATE include ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(start_signal_node
  rclcpp std_msgs sensor_msgs cv_bridge)
target_link_libraries(start_signal_node ${OpenCV_LIBS})

install(TARGETS obstacle_detector_node start_signal_node
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
