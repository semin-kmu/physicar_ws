cmake_minimum_required(VERSION 3.8)
project(backup_path_planner)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(backup_msgs REQUIRED)
find_package(backup_common REQUIRED)

add_executable(path_planner_node
  src/main.cpp
  src/path_planner_node.cpp
)
target_include_directories(path_planner_node PRIVATE include)
ament_target_dependencies(path_planner_node rclcpp std_msgs backup_msgs backup_common)

install(TARGETS path_planner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
