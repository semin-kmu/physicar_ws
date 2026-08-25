cmake_minimum_required(VERSION 3.8)
project(backup_steer_controller)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(backup_msgs REQUIRED)
find_package(backup_common REQUIRED)

add_executable(steer_controller_node
  src/main.cpp
  src/steer_controller_node.cpp
)
target_include_directories(steer_controller_node PRIVATE include)
ament_target_dependencies(steer_controller_node rclcpp std_msgs backup_msgs backup_common)

install(TARGETS steer_controller_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
